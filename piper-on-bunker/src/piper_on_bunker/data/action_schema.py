from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional


SCHEMA_VERSION = "piper_xvla_demo_v1"
CAMERA_FEATURE_KEY = "observation.images.external_camera"
STATE_FEATURE_KEY = "observation.state"
ACTION_FEATURE_KEY = "action"
PIPER_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
PIPER_ACTION_NAMES = [
    "target_joint1",
    "target_joint2",
    "target_joint3",
    "target_joint4",
    "target_joint5",
    "target_joint6",
    "target_gripper",
]
STATE_DIM = 7
ACTION_DIM = 7
GRIPPER_UNITS = "meters_opening_width"
GRIPPER_UNIT_SOURCE = (
    "ABot-Claw PiperRobotEnv and state.py document 0.0=closed, 0.06=open; "
    "live verification remains required during hardware collection"
)


def _float_list(values: List[float], expected_len: int, label: str) -> List[float]:
    result = [float(value) for value in values]
    if len(result) != expected_len:
        raise ValueError(f"{label} must contain {expected_len} values")
    return result


@dataclass
class EndPoseMetadata:
    position_m: List[float]
    quaternion_xyzw: List[float]
    timestamp_s: float
    frame_id: str

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["position_m"] = _float_list(self.position_m, 3, "position_m")
        data["quaternion_xyzw"] = _float_list(self.quaternion_xyzw, 4, "quaternion_xyzw")
        data["timestamp_s"] = float(self.timestamp_s)
        return data


@dataclass
class PiperStateSample:
    joint_names: List[str]
    joint_positions_rad: List[float]
    gripper_value: float
    timestamp_s: float
    age_s: float
    topic: str = "/joint_states_single"
    units: str = "radians"

    def vector(self) -> List[float]:
        return _float_list(self.joint_positions_rad, 6, "joint_positions_rad") + [float(self.gripper_value)]

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["joint_positions_rad"] = _float_list(self.joint_positions_rad, 6, "joint_positions_rad")
        data["gripper_value"] = float(self.gripper_value)
        data["timestamp_s"] = float(self.timestamp_s)
        data["age_s"] = float(self.age_s)
        return data


@dataclass
class PiperCommandSample:
    command_joint_names: List[str]
    target_joint_positions_rad: List[float]
    target_gripper_value: float
    timestamp_s: float
    source: str
    max_velocity: float
    max_acceleration: float
    moveit_service: str
    moveit_request_preview: Dict[str, Any] = field(default_factory=dict)
    bridge_command_echo: Optional[Dict[str, Any]] = None

    def vector(self) -> List[float]:
        return _float_list(self.target_joint_positions_rad, 6, "target_joint_positions_rad") + [
            float(self.target_gripper_value)
        ]

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["target_joint_positions_rad"] = _float_list(
            self.target_joint_positions_rad, 6, "target_joint_positions_rad"
        )
        data["target_gripper_value"] = float(self.target_gripper_value)
        data["timestamp_s"] = float(self.timestamp_s)
        data["max_velocity"] = float(self.max_velocity)
        data["max_acceleration"] = float(self.max_acceleration)
        return data


@dataclass
class PiperFrameRecord:
    frame_index: int
    task_instruction: str
    camera_path: str
    camera_topic: str
    camera_frame_id: str
    image_timestamp_s: float
    image_age_s: float
    state: PiperStateSample
    command: PiperCommandSample
    end_pose: Optional[EndPoseMetadata]
    receive_timestamp_s: float
    notes: Optional[str] = None

    def lerobot_state_vector(self) -> List[float]:
        return self.state.vector()

    def lerobot_action_vector(self) -> List[float]:
        return self.command.vector()

    def to_dict(self) -> Dict[str, Any]:
        return {
            "frame_index": int(self.frame_index),
            "task_instruction": self.task_instruction,
            "camera_path": self.camera_path,
            "camera_topic": self.camera_topic,
            "camera_frame_id": self.camera_frame_id,
            "image_timestamp_s": float(self.image_timestamp_s),
            "image_age_s": float(self.image_age_s),
            "state": self.state.to_dict(),
            "command": self.command.to_dict(),
            "end_pose": self.end_pose.to_dict() if self.end_pose else None,
            "receive_timestamp_s": float(self.receive_timestamp_s),
            "notes": self.notes,
        }


@dataclass
class PiperEpisodeRecord:
    schema_version: str
    episode_index: int
    task_instruction: str
    success: bool
    aborted: bool
    frame_count: int
    camera_name: str
    joint_names: List[str]
    action_names: List[str]
    joint_units: str
    gripper_units: str
    gripper_unit_source: str
    action_semantics: str
    recording_method: str
    source_topics: Dict[str, str]
    notes: Optional[str]
    created_at: str
    software: Dict[str, Any]
    frames: List[PiperFrameRecord]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "episode_index": int(self.episode_index),
            "task_instruction": self.task_instruction,
            "success": bool(self.success),
            "aborted": bool(self.aborted),
            "frame_count": int(self.frame_count),
            "camera_name": self.camera_name,
            "joint_names": list(self.joint_names),
            "action_names": list(self.action_names),
            "joint_units": self.joint_units,
            "gripper_units": self.gripper_units,
            "gripper_unit_source": self.gripper_unit_source,
            "action_semantics": self.action_semantics,
            "recording_method": self.recording_method,
            "source_topics": dict(self.source_topics),
            "notes": self.notes,
            "created_at": self.created_at,
            "software": dict(self.software),
            "frames": [frame.to_dict() for frame in self.frames],
        }
