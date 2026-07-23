from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from time import monotonic
from typing import Any, Optional, Tuple


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
    NOT_IMPLEMENTED = "NOT_IMPLEMENTED"
    API_CONTRACT_ERROR = "API_CONTRACT_ERROR"
    STALE_STATE = "STALE_STATE"
    STALE_IMAGE = "STALE_IMAGE"
    SAFETY_VIOLATION = "SAFETY_VIOLATION"
    VISUAL_SERVO_DISABLED_OPEN_LOOP = "VISUAL_SERVO_DISABLED_OPEN_LOOP"
    BASE_NOT_LOCKED = "BASE_NOT_LOCKED"
    INVALID_DEPTH = "INVALID_DEPTH"
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
    rgb_path: Optional[str] = None
    depth_path: Optional[str] = None
    metadata: dict = field(default_factory=dict)


@dataclass
class Target:
    label: str
    confidence: float
    pixel: Optional[Tuple[int, int]] = None
    camera_pose: Optional[Pose] = None
    base_pose: Optional[Pose] = None


@dataclass
class FailureDetails:
    code: StatusCode
    detail: str
    recoverable: bool = True


@dataclass
class RecoveryInfo:
    attempted: bool = False
    action: Optional[str] = None
    success: Optional[bool] = None


@dataclass
class SkillResult:
    success: bool
    status_code: StatusCode
    message: str
    started_at: str
    ended_at: str
    duration_s: float
    outputs: dict = field(default_factory=dict)
    failure: Optional[FailureDetails] = None
    recovery: RecoveryInfo = field(default_factory=RecoveryInfo)

    @classmethod
    def build(
        cls,
        success: bool,
        code: StatusCode,
        message: str,
        start_monotonic: float,
        outputs: Optional[dict] = None,
        failure_detail: Optional[str] = None,
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
