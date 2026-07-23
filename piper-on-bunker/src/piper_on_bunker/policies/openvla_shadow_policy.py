from __future__ import annotations

import base64
import hashlib
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import requests


SHADOW_BANNER = "SHADOW MODE - NO ROBOT ACTION WILL BE EXECUTED"
MODEL_NAME = "openvla-7b"
DEFAULT_ENDPOINT = "http://192.168.1.104:8018"
DEFAULT_IMAGE_TOPIC = "/table_camera/color/image_raw"
DEFAULT_TIMEOUT_S = 60.0
DEFAULT_MAX_IMAGE_AGE_S = 5.0
EXPECTED_ACTION_DIMENSION = 7
EXPECTED_STATE_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]


def _detect_image_mime(data: bytes) -> Optional[str]:
    if data.startswith(b"\x89PNG\r\n\x1a\n"):
        return "image/png"
    if data.startswith(b"\xff\xd8\xff"):
        return "image/jpeg"
    return None


def encode_image_bytes(data: bytes, max_bytes: int = 8_000_000) -> str:
    if not data:
        raise ValueError("image is empty")
    if len(data) > max_bytes:
        raise ValueError(f"image exceeds request limit: {len(data)} > {max_bytes}")
    mime = _detect_image_mime(data)
    if mime is None:
        raise ValueError("image is not a supported PNG or JPEG")
    return "data:%s;base64,%s" % (mime, base64.b64encode(data).decode("ascii"))


def encode_image_file(path: str | Path, max_bytes: int = 8_000_000) -> str:
    return encode_image_bytes(Path(path).read_bytes(), max_bytes=max_bytes)


def _validate_finite(values: Iterable[float], label: str) -> List[float]:
    result = [float(value) for value in values]
    for value in result:
        if not math.isfinite(value):
            raise ValueError(f"{label} contains non-finite values")
    return result


@dataclass
class JointStateMetadata:
    joint_names: List[str]
    joint_positions_rad: List[float]
    gripper_raw: float
    timestamp_s: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "joint_names": list(self.joint_names),
            "joint_positions_rad": _validate_finite(self.joint_positions_rad, "joint_positions_rad"),
            "gripper_raw": float(self.gripper_raw),
            "timestamp_s": float(self.timestamp_s),
        }


@dataclass
class EndPoseMetadata:
    position_m: List[float]
    quaternion_xyzw: List[float]
    timestamp_s: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "position_m": _validate_finite(self.position_m, "position_m"),
            "quaternion_xyzw": _validate_finite(self.quaternion_xyzw, "quaternion_xyzw"),
            "timestamp_s": float(self.timestamp_s),
        }


@dataclass
class OpenVLARequestRecord:
    instruction: str
    image_data_uri: str
    source_timestamp_s: Optional[float]
    camera_metadata: Dict[str, Any]
    piper_state_metadata: Optional[JointStateMetadata] = None
    end_pose_metadata: Optional[EndPoseMetadata] = None

    def to_payload(self) -> Dict[str, Any]:
        if not self.instruction.strip():
            raise ValueError("instruction must be non-empty")
        return {
            "image": self.image_data_uri,
            "instruction": self.instruction.strip(),
            "source_timestamp_s": self.source_timestamp_s,
            "camera_metadata": dict(self.camera_metadata),
            "piper_state_metadata": self.piper_state_metadata.to_dict() if self.piper_state_metadata else None,
            "end_pose_metadata": self.end_pose_metadata.to_dict() if self.end_pose_metadata else None,
            "execution_allowed": False,
        }


def _hash_data_uri(data_uri: str) -> str:
    payload = data_uri.split(",", 1)[1] if data_uri.startswith("data:") and "," in data_uri else data_uri
    decoded = base64.b64decode(payload, validate=True)
    return hashlib.sha256(decoded).hexdigest()


def _enforce_shadow_response(data: Dict[str, Any]) -> Dict[str, Any]:
    result = dict(data)
    result["execution_allowed"] = False
    return result


class OpenVLAShadowPolicyClient:
    def __init__(self, endpoint: str = DEFAULT_ENDPOINT, timeout_s: float = DEFAULT_TIMEOUT_S) -> None:
        self.endpoint = endpoint.rstrip("/")
        self.timeout_s = timeout_s

    def compatibility_check(self) -> Dict[str, Any]:
        payload = {
            "camera_count": 1,
            "camera_source": "external_realsense",
            "language_instruction": True,
            "available_state": list(EXPECTED_STATE_NAMES),
            "desired_action_dimension": EXPECTED_ACTION_DIMENSION,
            "desired_future_executor": "piper_moveit_end_pose",
            "execution_allowed": False,
        }
        try:
            response = requests.post(f"{self.endpoint}/compatibility-check", json=payload, timeout=self.timeout_s)
            response.raise_for_status()
            return _enforce_shadow_response(response.json())
        except Exception as exc:
            return {
                "request_success": False,
                "shadow_inference_allowed": False,
                "physical_execution_compatible": False,
                "error": str(exc),
                "execution_allowed": False,
            }

    def preview_action(self, record: OpenVLARequestRecord) -> Dict[str, Any]:
        compatibility = self.compatibility_check()
        if not compatibility.get("request_success") or not compatibility.get("shadow_inference_allowed"):
            return {
                "request_success": False,
                "compatibility_verified": False,
                "inference_attempted": False,
                "error": compatibility.get("error") or "compatibility check failed",
                "compatibility": compatibility,
                "execution_allowed": False,
            }
        try:
            response = requests.post(f"{self.endpoint}/action-preview", json=record.to_payload(), timeout=self.timeout_s)
            response.raise_for_status()
            data = _enforce_shadow_response(response.json())
        except Exception as exc:
            return {
                "request_success": False,
                "compatibility_verified": True,
                "inference_attempted": False,
                "error": str(exc),
                "execution_allowed": False,
            }
        action = data.get("bridge_unnormalized_action")
        if not isinstance(action, list) or len(action) != EXPECTED_ACTION_DIMENSION:
            return {
                "request_success": False,
                "compatibility_verified": True,
                "inference_attempted": True,
                "inference_success": False,
                "error": "malformed action preview response",
                "response": data,
                "execution_allowed": False,
            }
        data["image_sha256"] = _hash_data_uri(record.image_data_uri)
        data["execution_allowed"] = False
        return data


def read_live_color_frame(image_topic: str, timeout_s: float, max_age_s: float = DEFAULT_MAX_IMAGE_AGE_S) -> tuple[str, float, Dict[str, Any]]:
    try:
        import cv2
        import rospy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
    except Exception as exc:
        raise RuntimeError("live image capture requires rospy, cv_bridge, sensor_msgs, and cv2") from exc
    if not rospy.get_node_uri():
        rospy.init_node("openvla_shadow_client", anonymous=True, disable_signals=True)
    published_topics = {name for name, _ in rospy.get_published_topics()}
    if image_topic not in published_topics:
        raise RuntimeError(f"image topic not currently published: {image_topic}")
    msg = rospy.wait_for_message(image_topic, Image, timeout=timeout_s)
    image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
    ok, encoded = cv2.imencode(".jpg", image)
    if not ok:
        raise RuntimeError("failed to encode live image as JPEG")
    stamp = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
    now = rospy.Time.now().to_sec() if rospy.Time.now() else 0.0
    if stamp and now and now - stamp > max_age_s:
        raise RuntimeError(f"live image is stale: age={now - stamp:.3f}s > {max_age_s:.3f}s")
    metadata = {
        "ros_topic": image_topic,
        "encoding": msg.encoding,
        "frame_id": msg.header.frame_id,
        "width": int(msg.width),
        "height": int(msg.height),
        "image_model_input": True,
        "max_age_s": float(max_age_s),
    }
    return encode_image_bytes(encoded.tobytes()), stamp, metadata


def _map_joint_state(names: List[str], positions: List[float]) -> JointStateMetadata:
    seen = {}
    for index, name in enumerate(names):
        if name in seen:
            raise ValueError(f"duplicate joint name: {name}")
        seen[name] = index
    missing = [name for name in EXPECTED_STATE_NAMES if name not in seen]
    if missing:
        raise ValueError("missing joints: " + ", ".join(missing))
    ordered = [float(positions[seen[name]]) for name in EXPECTED_STATE_NAMES[:-1]]
    gripper = float(positions[seen["gripper"]])
    _validate_finite(ordered + [gripper], "joint_state")
    return JointStateMetadata(EXPECTED_STATE_NAMES[:-1], ordered, gripper, 0.0)


def read_live_state(timeout_s: float) -> tuple[Optional[JointStateMetadata], Optional[EndPoseMetadata]]:
    try:
        import rospy
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState
    except Exception as exc:
        raise RuntimeError("live state capture requires rospy, geometry_msgs, and sensor_msgs") from exc
    if not rospy.get_node_uri():
        rospy.init_node("openvla_shadow_client", anonymous=True, disable_signals=True)
    joint_msg = rospy.wait_for_message("/joint_states_single", JointState, timeout=timeout_s)
    pose_msg = rospy.wait_for_message("/end_pose", PoseStamped, timeout=timeout_s)
    state = _map_joint_state(list(joint_msg.name), list(joint_msg.position))
    state.timestamp_s = joint_msg.header.stamp.to_sec() if joint_msg.header.stamp else 0.0
    pose = EndPoseMetadata(
        position_m=[pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z],
        quaternion_xyzw=[
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        ],
        timestamp_s=pose_msg.header.stamp.to_sec() if pose_msg.header.stamp else 0.0,
    )
    return state, pose


def default_log_path(repo_root: Path) -> Path:
    from datetime import datetime, timezone

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    directory = repo_root / "piper-on-bunker" / "logs" / "vla_shadow" / "openvla"
    directory.mkdir(parents=True, exist_ok=True)
    return directory / f"{stamp}_openvla_shadow.json"


def save_shadow_result(path: Path, request_record: OpenVLARequestRecord, response: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "shadow_banner": SHADOW_BANNER,
        "request": request_record.to_payload(),
        "response": _enforce_shadow_response(response),
    }
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def load_saved_state_metadata(path: str | Path) -> tuple[Optional[JointStateMetadata], Optional[EndPoseMetadata]]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    joint = data.get("joint_state")
    pose = data.get("end_pose")
    joint_state = None
    end_pose = None
    if joint:
        joint_state = _map_joint_state(list(joint["names"]), list(joint["positions"]))
        joint_state.timestamp_s = float(joint.get("stamp", 0.0))
    if pose:
        end_pose = EndPoseMetadata(
            position_m=[pose["position"]["x"], pose["position"]["y"], pose["position"]["z"]],
            quaternion_xyzw=[
                pose["orientation"]["x"],
                pose["orientation"]["y"],
                pose["orientation"]["z"],
                pose["orientation"]["w"],
            ],
            timestamp_s=float(pose.get("stamp", 0.0)),
        )
    return joint_state, end_pose
