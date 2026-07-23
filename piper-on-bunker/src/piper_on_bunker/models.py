from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from time import monotonic
from typing import Any


class ArmMode(str, Enum):
    IDLE = "IDLE"
    STOWED = "STOWED"
    NAVIGATION_VIEW = "NAVIGATION_VIEW"
    ACTIVE_SCAN = "ACTIVE_SCAN"
    TASK_INSPECTION = "TASK_INSPECTION"
    PRE_MANIPULATION = "PRE_MANIPULATION"
    MANIPULATION = "MANIPULATION"
    VERIFYING = "VERIFYING"
    RETRACTING = "RETRACTING"
    FAULT = "FAULT"
    ESTOP = "ESTOP"


class StatusCode(str, Enum):
    OK = "OK"
    CAMERA_UNAVAILABLE = "CAMERA_UNAVAILABLE"
    TARGET_NOT_FOUND = "TARGET_NOT_FOUND"
    TARGET_LOST = "TARGET_LOST"
    INVALID_TRANSFORM = "INVALID_TRANSFORM"
    IK_FAILURE = "IK_FAILURE"
    PLANNING_FAILURE = "PLANNING_FAILURE"
    RESOURCE_BUSY = "RESOURCE_BUSY"
    POSE_TIMEOUT = "POSE_TIMEOUT"
    CONTROLLER_FAILURE = "CONTROLLER_FAILURE"
    VERIFICATION_FAILURE = "VERIFICATION_FAILURE"
    ESTOP = "ESTOP"
    HARDWARE_DISABLED = "HARDWARE_DISABLED"
    INVALID_COMMAND = "INVALID_COMMAND"
    ERROR = "ERROR"


@dataclass
class Pose:
    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    frame_id: str = "base_link"


@dataclass
class Observation:
    camera_name: str
    frame_id: str
    timestamp: str
    rgb_path: str | None = None
    depth_path: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class Target:
    label: str
    confidence: float
    pixel: tuple[int, int] | None = None
    camera_pose: Pose | None = None
    base_pose: Pose | None = None


@dataclass
class FailureDetails:
    code: StatusCode
    detail: str
    recoverable: bool = True


@dataclass
class RecoveryInfo:
    attempted: bool = False
    action: str | None = None
    success: bool | None = None


@dataclass
class SkillResult:
    success: bool
    status_code: StatusCode
    message: str
    started_at: str
    ended_at: str
    duration_s: float
    outputs: dict[str, Any] = field(default_factory=dict)
    failure: FailureDetails | None = None
    recovery: RecoveryInfo = field(default_factory=RecoveryInfo)

    @classmethod
    def build(
        cls,
        success: bool,
        code: StatusCode,
        message: str,
        start_monotonic: float,
        outputs: dict[str, Any] | None = None,
        failure_detail: str | None = None,
    ) -> "SkillResult":
        now = datetime.now(timezone.utc).isoformat()
        failure = None if success else FailureDetails(code, failure_detail or message)
        return cls(
            success=success,
            status_code=code,
            message=message,
            started_at=now,
            ended_at=now,
            duration_s=monotonic() - start_monotonic,
            outputs=outputs or {},
            failure=failure,
        )


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()
