from __future__ import annotations

from dataclasses import asdict
from dataclasses import dataclass
from enum import Enum
from time import time
from typing import Iterable
from uuid import uuid4


PLAN_SCHEMA_VERSION = "openclaw.manipulation_plan.v1"


class SemanticSkill(str, Enum):
    OBSERVE_SCENE = "OBSERVE_SCENE"
    APPROACH_OBJECT = "APPROACH_OBJECT"
    GRASP_OBJECT = "GRASP_OBJECT"
    LIFT_OBJECT = "LIFT_OBJECT"
    TRANSPORT_OBJECT = "TRANSPORT_OBJECT"
    PLACE_OBJECT = "PLACE_OBJECT"
    RELEASE_OBJECT = "RELEASE_OBJECT"
    RETRACT_ARM = "RETRACT_ARM"
    VERIFY_GRASP = "VERIFY_GRASP"
    VERIFY_PLACEMENT = "VERIFY_PLACEMENT"
    RECOVER_TO_SAFE_POSE = "RECOVER_TO_SAFE_POSE"
    NAVIGATE_TO_WORKSPACE = "NAVIGATE_TO_WORKSPACE"
    OBSERVE_WORKSPACE = "OBSERVE_WORKSPACE"
    SIDE_DOCK = "SIDE_DOCK"
    LOCK_BASE = "LOCK_BASE"
    UNLOCK_BASE = "UNLOCK_BASE"
    DEPART_WORKSPACE = "DEPART_WORKSPACE"


MANIPULATION_SKILLS = {
    SemanticSkill.OBSERVE_SCENE,
    SemanticSkill.APPROACH_OBJECT,
    SemanticSkill.GRASP_OBJECT,
    SemanticSkill.LIFT_OBJECT,
    SemanticSkill.TRANSPORT_OBJECT,
    SemanticSkill.PLACE_OBJECT,
    SemanticSkill.RELEASE_OBJECT,
    SemanticSkill.RETRACT_ARM,
    SemanticSkill.VERIFY_GRASP,
    SemanticSkill.VERIFY_PLACEMENT,
    SemanticSkill.RECOVER_TO_SAFE_POSE,
}


@dataclass(frozen=True)
class Phase:
    phase_id: str
    skill: SemanticSkill
    policy_prompt: str
    target: str | None = None
    destination: str | None = None
    preconditions: tuple[str, ...] = ()
    completion_checks: tuple[str, ...] = ()

    def to_dict(self) -> dict:
        payload = asdict(self)
        payload["skill"] = self.skill.value
        payload["preconditions"] = list(self.preconditions)
        payload["completion_checks"] = list(self.completion_checks)
        return payload


@dataclass(frozen=True)
class ManipulationPlan:
    task_id: str
    instruction: str
    schema_version: str
    created_unix_s: float
    phases: tuple[Phase, ...]

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "instruction": self.instruction,
            "schema_version": self.schema_version,
            "created_unix_s": self.created_unix_s,
            "phases": [phase.to_dict() for phase in self.phases],
        }


def plan_manipulation_task(instruction: str, *, task_id: str | None = None) -> ManipulationPlan:
    """Create a restricted semantic phase plan.

    This first version intentionally recognizes the tabletop pick/place shape
    instead of generating arbitrary low-level motion requests.
    """

    normalized = " ".join(instruction.strip().split()).lower()
    target = "red cup" if "red cup" in normalized else _extract_after(normalized, "move the ") or "object"
    destination = "paper" if "paper" in normalized else _extract_after(normalized, "onto the ") or "destination"
    phases = (
        Phase(
            phase_id="approach",
            skill=SemanticSkill.APPROACH_OBJECT,
            policy_prompt=f"Approach the {target} and finish in a grasp-ready pose.",
            target=target,
            preconditions=("base_locked", "target_visible"),
            completion_checks=("arm_motion_complete", "cup_visible", "gripper_near_target"),
        ),
        Phase(
            phase_id="grasp",
            skill=SemanticSkill.GRASP_OBJECT,
            policy_prompt=f"Align with and securely grasp the {target}.",
            target=target,
            preconditions=("approach_complete",),
            completion_checks=("gripper_closed", "object_acquired"),
        ),
        Phase(
            phase_id="transport",
            skill=SemanticSkill.TRANSPORT_OBJECT,
            policy_prompt=f"Lift the {target} and move it above the {destination}.",
            target=target,
            destination=destination,
            preconditions=("grasp_confirmed",),
            completion_checks=("object_above_destination",),
        ),
        Phase(
            phase_id="release",
            skill=SemanticSkill.RELEASE_OBJECT,
            policy_prompt=f"Place the {target} on the {destination}, release it, and retract.",
            target=target,
            destination=destination,
            preconditions=("transport_complete",),
            completion_checks=("gripper_open", "object_on_destination"),
        ),
    )
    return ManipulationPlan(
        task_id=task_id or str(uuid4()),
        instruction=instruction,
        schema_version=PLAN_SCHEMA_VERSION,
        created_unix_s=time(),
        phases=phases,
    )


def validate_phase_sequence(phases: Iterable[Phase]) -> None:
    phase_ids = [phase.phase_id for phase in phases]
    required_order = ["approach", "grasp", "transport", "release"]
    if phase_ids[:4] != required_order:
        raise ValueError(f"invalid manipulation phase order: {phase_ids}")


def next_phase_allowed(phase: Phase, completed_checks: set[str]) -> bool:
    return all(check in completed_checks for check in phase.preconditions)


def _extract_after(text: str, marker: str) -> str | None:
    if marker not in text:
        return None
    tail = text.split(marker, 1)[1]
    return tail.split(" onto ", 1)[0].split(" to ", 1)[0].strip(" .") or None

