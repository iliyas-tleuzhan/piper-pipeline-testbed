from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from piper_on_bunker.control.piper_joint_phase_executor import PiperJointPhaseExecutor
from piper_on_bunker.openclaw.semantic_plan import Phase
from piper_on_bunker.openclaw.semantic_plan import plan_manipulation_task
from piper_on_bunker.openclaw.semantic_plan import validate_phase_sequence
from piper_on_bunker.policies.openpi_piper_policy import ShadowOpenPIPiperClient
from piper_on_bunker.policies.openpi_piper_policy import make_observation
from piper_on_bunker.policies.openpi_piper_policy import validate_openpi_response


class PhaseStatus(str, Enum):
    READY = "READY"
    INFERENCING = "INFERENCING"
    EXECUTING = "EXECUTING"
    VERIFYING = "VERIFYING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"


@dataclass
class OrchestratorStatus:
    task_id: str
    active_phase: str | None
    phase_status: PhaseStatus
    robot_state_fresh: bool
    camera_state_fresh: bool
    policy_checkpoint: str | None
    piper_compatible: bool
    safety_status: str
    failure_reason: str | None = None

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "active_phase": self.active_phase,
            "phase_status": self.phase_status.value,
            "robot_state_fresh": self.robot_state_fresh,
            "camera_state_fresh": self.camera_state_fresh,
            "policy_checkpoint": self.policy_checkpoint,
            "piper_compatible": self.piper_compatible,
            "safety_status": self.safety_status,
            "failure_reason": self.failure_reason,
        }


class PhaseOrchestrator:
    def __init__(self, policy_client=None, executor: PiperJointPhaseExecutor | None = None, *, frequency_hz: float = 20.0):
        self.policy_client = policy_client or ShadowOpenPIPiperClient(frequency_hz=frequency_hz)
        self.executor = executor or PiperJointPhaseExecutor()
        self.frequency_hz = float(frequency_hz)

    def plan_task(self, instruction: str):
        plan = plan_manipulation_task(instruction)
        validate_phase_sequence(plan.phases)
        return plan

    def run_phase_shadow(self, phase: Phase, *, exterior_image=None, wrist_image=None, state=None) -> dict:
        exterior = exterior_image if exterior_image is not None else np.zeros((224, 224, 3), dtype=np.uint8)
        wrist = wrist_image if wrist_image is not None else np.zeros((224, 224, 3), dtype=np.uint8)
        current_state = np.asarray(state if state is not None else np.zeros(7), dtype=np.float64)
        observation = make_observation(exterior, wrist, current_state, phase.policy_prompt)
        payload = self.policy_client.infer_phase(observation)
        response = validate_openpi_response(
            payload,
            require_piper_compatible=False,
            expected_frequency_hz=self.frequency_hz,
        )
        result = self.executor.execute_response(
            response,
            current_state=current_state,
            state_age_s=0.0,
            camera_age_s=0.0,
            execute=False,
        )
        return {
            "phase": phase.to_dict(),
            "policy_checkpoint": response.metadata.checkpoint,
            "piper_compatible": response.metadata.piper_compatible,
            "execution": result.__dict__,
        }

