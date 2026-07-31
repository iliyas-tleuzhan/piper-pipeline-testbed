from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable


PIPER_X_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
PIPER_X_FEEDBACK_SOURCE_ID = "piper_x_pyagxarm_readonly_v1"
PIPER_X_JOINT_MAPPING_VERSION = "piper_x_pyagxarm_joint_order_rad_v1"
PIPER_X_TELEOP_REPO = "/home/dase-hw101/Iliyas/piper-vr-teleop"
PIPER_X_TELEOP_COMMIT = "9eec6e26d927a495efaaa0e7e5af2895310caefe"
PIPER_X_AGX_URDF_COMMIT = "f6642ce0d7872c686f29c99e9e10cd23d1d49313"
PIPER_X_CONFIGURED_ARM_MODEL = "agilex_piper_x"
PIPER_X_CONFIGURED_FIRMWARE_PROFILE = "unresolved_read_only_feedback_only"


COMMAND_METHOD_NAMES = {
    "move_j",
    "move_js",
    "move_gripper",
    "enable",
    "EnableArm",
    "EnablePiper",
    "reset",
    "Reset",
    "zero",
    "GoZero",
    "release",
    "MotionCtrl_2",
    "JointCtrl",
    "ArmJointCtrl",
    "EndPoseCtrl",
    "GripperCtrl",
}


@dataclass(frozen=True)
class PiperXFeedbackConfig:
    expected_joint_names: list[str] = field(default_factory=lambda: list(PIPER_X_JOINT_NAMES))
    source_id: str = PIPER_X_FEEDBACK_SOURCE_ID
    joint_mapping_version: str = PIPER_X_JOINT_MAPPING_VERSION
    teleop_repo_commit: str = PIPER_X_TELEOP_COMMIT
    arm_model: str = PIPER_X_CONFIGURED_ARM_MODEL
    firmware_profile: str = PIPER_X_CONFIGURED_FIRMWARE_PROFILE
    units: str = "rad"
    max_age_s: float = 0.5
    reject_unproven_all_zero: bool = True


@dataclass(frozen=True)
class PiperXFeedbackEvidence:
    connected: bool
    feedback_valid: bool
    source_update_counter: int | None = None
    source_timestamp_s: float | None = None
    sdk_error: str | None = None
    real_feedback_packet: bool = False
    communication_ready: bool = False
    no_motion_commands_sent: bool = True


@dataclass(frozen=True)
class PiperXFeedbackSample:
    joint_names: list[str]
    positions_rad: list[float]
    stamp_s: float
    source_id: str
    joint_mapping_version: str
    teleop_repo_commit: str
    arm_model: str
    firmware_profile: str
    units: str
    evidence: PiperXFeedbackEvidence

    def to_status_dict(self, now_s: float | None = None) -> dict[str, Any]:
        now = time.time() if now_s is None else float(now_s)
        return {
            "connected": self.evidence.connected,
            "feedback_valid": self.evidence.feedback_valid,
            "feedback_age_s": max(0.0, now - float(self.stamp_s)),
            "source_update_counter": self.evidence.source_update_counter,
            "joint_names": list(self.joint_names),
            "positions_rad": [float(v) for v in self.positions_rad],
            "source_id": self.source_id,
            "joint_mapping_version": self.joint_mapping_version,
            "teleop_repo_commit": self.teleop_repo_commit,
            "arm_model": self.arm_model,
            "firmware_profile": self.firmware_profile,
            "units": self.units,
            "sdk_error": self.evidence.sdk_error,
            "real_feedback_packet": self.evidence.real_feedback_packet,
            "communication_ready": self.evidence.communication_ready,
            "no_motion_commands_sent": self.evidence.no_motion_commands_sent,
        }


def ensure_no_command_methods_called(calls: list[str]) -> None:
    unsafe = [name for name in calls if name in COMMAND_METHOD_NAMES]
    if unsafe:
        raise RuntimeError(f"read-only PiPER-X feedback bridge attempted command-capable methods: {unsafe}")


def extract_six_joint_values(raw: Any) -> list[float]:
    if raw is None:
        raise ValueError("feedback read returned None")
    if isinstance(raw, dict):
        for key in ("joints_rad", "joint_angles_rad", "positions_rad", "joints", "positions"):
            value = raw.get(key)
            if value is not None:
                return extract_six_joint_values(value)
    if isinstance(raw, (list, tuple)):
        if len(raw) != 6:
            raise ValueError(f"expected six joints, got {len(raw)}")
        return [float(v) for v in raw]
    for attr in ("joints_rad", "joint_angles_rad", "positions_rad", "joints", "positions"):
        value = getattr(raw, attr, None)
        if value is not None:
            return extract_six_joint_values(value)
    names = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
    if all(hasattr(raw, name) for name in names):
        return [float(getattr(raw, name)) for name in names]
    names = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
    if all(hasattr(raw, name) for name in names):
        return [float(getattr(raw, name)) for name in names]
    raise ValueError(f"could not extract six PiPER-X joints from {type(raw).__name__}")


def validate_piper_x_feedback(
    raw: Any,
    evidence: PiperXFeedbackEvidence,
    *,
    config: PiperXFeedbackConfig | None = None,
    now_s: float | None = None,
) -> PiperXFeedbackSample:
    cfg = config or PiperXFeedbackConfig()
    if evidence.sdk_error:
        raise ValueError(f"SDK feedback error: {evidence.sdk_error}")
    if not evidence.connected or not evidence.communication_ready:
        raise ValueError("PiPER-X feedback source is not communication-ready")
    if not evidence.feedback_valid:
        raise ValueError("PiPER-X feedback source did not report valid feedback")
    if not evidence.real_feedback_packet:
        raise ValueError("PiPER-X feedback is missing real packet/update evidence")
    if not evidence.no_motion_commands_sent:
        raise ValueError("read-only feedback source reported motion command use")

    values = extract_six_joint_values(raw)
    if len(values) != 6:
        raise ValueError(f"expected six PiPER-X joints, got {len(values)}")
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError("PiPER-X feedback contains non-finite joint values")
    if cfg.reject_unproven_all_zero and max(abs(float(v)) for v in values) == 0.0:
        if evidence.source_update_counter is None and evidence.source_timestamp_s is None:
            raise ValueError("fresh all-zero feedback is refused without source update evidence")

    stamp = float(evidence.source_timestamp_s) if evidence.source_timestamp_s is not None else (time.time() if now_s is None else float(now_s))
    return PiperXFeedbackSample(
        joint_names=list(cfg.expected_joint_names),
        positions_rad=[float(v) for v in values],
        stamp_s=stamp,
        source_id=cfg.source_id,
        joint_mapping_version=cfg.joint_mapping_version,
        teleop_repo_commit=cfg.teleop_repo_commit,
        arm_model=cfg.arm_model,
        firmware_profile=cfg.firmware_profile,
        units=cfg.units,
        evidence=evidence,
    )


class ReadOnlyMethodProxy:
    def __init__(self, target: Any, calls: list[str]) -> None:
        self._target = target
        self._calls = calls

    def __getattr__(self, name: str) -> Any:
        value = getattr(self._target, name)
        if callable(value):
            def wrapped(*args: Any, **kwargs: Any) -> Any:
                self._calls.append(name)
                if name in COMMAND_METHOD_NAMES:
                    raise RuntimeError(f"unsafe command-capable pyAgxArm method refused: {name}")
                return value(*args, **kwargs)

            return wrapped
        return value


class PiperXReadOnlyFeedbackAdapter:
    """Adapter for the exact PiPER-X read-only feedback object.

    The actual pyAgxArm package is not vendored here. The bridge imports it at
    runtime and fails closed unless a configured read-only feedback method is
    present. Command-capable methods are wrapped and refused.
    """

    def __init__(
        self,
        arm_factory: Callable[[], Any],
        *,
        feedback_method: str,
        config: PiperXFeedbackConfig | None = None,
    ) -> None:
        self._calls: list[str] = []
        self.config = config or PiperXFeedbackConfig()
        self.feedback_method = feedback_method
        self._arm = ReadOnlyMethodProxy(arm_factory(), self._calls)
        self._sequence = 0

    @property
    def calls(self) -> list[str]:
        return list(self._calls)

    def read_sample(self) -> PiperXFeedbackSample:
        method = getattr(self._arm, self.feedback_method, None)
        if method is None:
            raise RuntimeError(f"configured read-only feedback method is unavailable: {self.feedback_method}")
        raw = method()
        ensure_no_command_methods_called(self._calls)
        self._sequence += 1
        evidence = PiperXFeedbackEvidence(
            connected=True,
            feedback_valid=True,
            source_update_counter=self._sequence,
            source_timestamp_s=time.time(),
            real_feedback_packet=True,
            communication_ready=True,
            no_motion_commands_sent=True,
        )
        return validate_piper_x_feedback(raw, evidence, config=self.config)


def status_json(sample: PiperXFeedbackSample | None, error: str | None = None) -> str:
    payload: dict[str, Any]
    if sample is None:
        payload = {
            "connected": False,
            "feedback_valid": False,
            "feedback_age_s": None,
            "source_id": PIPER_X_FEEDBACK_SOURCE_ID,
            "joint_mapping_version": PIPER_X_JOINT_MAPPING_VERSION,
            "teleop_repo_commit": PIPER_X_TELEOP_COMMIT,
            "arm_model": PIPER_X_CONFIGURED_ARM_MODEL,
            "firmware_profile": PIPER_X_CONFIGURED_FIRMWARE_PROFILE,
            "units": "rad",
            "no_motion_commands_sent": True,
            "error": error,
        }
    else:
        payload = sample.to_status_dict()
        payload["error"] = error
    return json.dumps(payload, sort_keys=True)


def validate_single_joint_state_authority(
    publishers: list[str],
    *,
    expected_substring: str = "piper_x_pyagxarm_joint_state_bridge",
) -> None:
    if len(publishers) != 1:
        raise ValueError(f"expected exactly one /joint_states publisher, got {publishers}")
    if expected_substring not in publishers[0]:
        raise ValueError(f"/joint_states publisher is not the PiPER-X feedback bridge: {publishers[0]}")
