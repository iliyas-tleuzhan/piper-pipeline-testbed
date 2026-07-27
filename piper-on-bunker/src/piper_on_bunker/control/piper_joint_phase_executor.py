from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from time import monotonic

import numpy as np

from piper_on_bunker.control.phase_chunk_buffer import JointSafetyLimits
from piper_on_bunker.control.phase_chunk_buffer import interpolate_chunk
from piper_on_bunker.control.phase_chunk_buffer import validate_action_chunk
from piper_on_bunker.policies.openpi_piper_policy import OpenPIPiperResponse


@dataclass(frozen=True)
class ExecutionConfig:
    hardware_frequency_hz: float = 50.0
    physical_motion_permission: bool = False
    command_authority_lock: str = "/tmp/piper_openpi_command_authority.lock"
    max_state_age_s: float = 0.5
    max_camera_age_s: float = 0.5
    max_policy_response_age_s: float = 1.0
    gripper_physical_enabled: bool = False


@dataclass(frozen=True)
class PhaseExecutionResult:
    success: bool
    message: str
    published_commands: int
    shadow_mode: bool
    outputs: dict


class CommandAuthorityLock:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.acquired = False

    def __enter__(self):
        if self.path.exists():
            raise RuntimeError(f"PiPER command authority is already held: {self.path}")
        self.path.write_text("openpi_piper\n", encoding="utf-8")
        self.acquired = True
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.acquired and self.path.exists() and self.path.read_text(encoding="utf-8") == "openpi_piper\n":
            self.path.unlink()
        self.acquired = False


class PiperJointPhaseExecutor:
    def __init__(self, config: ExecutionConfig | None = None, limits: JointSafetyLimits | None = None) -> None:
        self.config = config or ExecutionConfig()
        self.limits = limits or JointSafetyLimits()

    def execute_response(
        self,
        response: OpenPIPiperResponse,
        *,
        current_state,
        state_age_s: float,
        camera_age_s: float,
        execute: bool = False,
        publisher=None,
    ) -> PhaseExecutionResult:
        if execute and not self.config.physical_motion_permission:
            raise ValueError("physical OpenPI execution requires explicit configured physical-motion permission")
        if execute and not response.metadata.piper_compatible:
            raise ValueError("physical OpenPI execution requires a PiPER-compatible checkpoint")
        if state_age_s > self.config.max_state_age_s:
            raise ValueError(f"stale robot state: {state_age_s:.3f}s > {self.config.max_state_age_s:.3f}s")
        if camera_age_s > self.config.max_camera_age_s:
            raise ValueError(f"stale camera state: {camera_age_s:.3f}s > {self.config.max_camera_age_s:.3f}s")
        response_age = monotonic() - response.received_monotonic_s
        if response_age > self.config.max_policy_response_age_s:
            raise ValueError(
                f"stale policy response: {response_age:.3f}s > {self.config.max_policy_response_age_s:.3f}s"
            )
        if execute and np.any(np.abs(response.actions[:, 6] - float(current_state[6])) > 1e-9):
            if not self.config.gripper_physical_enabled:
                raise ValueError("physical gripper channel is not verified; grasp/release phases are refused")

        validated = validate_action_chunk(
            response.actions,
            current_state,
            frequency_hz=response.metadata.control_frequency_hz,
            limits=self.limits,
        )
        stream = interpolate_chunk(
            validated,
            source_frequency_hz=response.metadata.control_frequency_hz,
            target_frequency_hz=self.config.hardware_frequency_hz,
        )
        if not execute:
            return PhaseExecutionResult(
                success=True,
                message="shadow mode validated OpenPI PiPER action chunk; no ROS publishing performed",
                published_commands=0,
                shadow_mode=True,
                outputs={"stream_samples": int(stream.shape[0]), "hardware_frequency_hz": self.config.hardware_frequency_hz},
            )
        if publisher is None:
            raise ValueError("physical execution requires an explicit ROS publisher callback")
        with CommandAuthorityLock(self.config.command_authority_lock):
            for sample in stream:
                publisher(sample)
        return PhaseExecutionResult(
            success=True,
            message="OpenPI PiPER action chunk streamed to direct joint publisher",
            published_commands=int(stream.shape[0]),
            shadow_mode=False,
            outputs={"stream_samples": int(stream.shape[0]), "hardware_frequency_hz": self.config.hardware_frequency_hz},
        )

