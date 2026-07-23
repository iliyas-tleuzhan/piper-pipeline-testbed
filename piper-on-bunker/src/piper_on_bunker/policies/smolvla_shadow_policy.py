from __future__ import annotations

import base64
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import requests

from piper_on_bunker.hardware.joint_state import DEFAULT_ARM_JOINT_NAMES, map_joint_state


SHADOW_BANNER = "SHADOW MODE - NO ROBOT ACTION WILL BE EXECUTED"
DEFAULT_ENDPOINT = "http://192.168.1.104:8018"
DEFAULT_TIMEOUT_S = 30.0
DEFAULT_MAX_STATE_AGE_S = 2.0


@dataclass
class JointStateRecord:
    arm_joint_names: List[str]
    arm_positions_rad: List[float]
    gripper_value: float
    stamp_s: float
    age_s: float
    gripper_units: str = "unverified"

    def validate(self) -> None:
        if len(self.arm_joint_names) != 6 or len(self.arm_positions_rad) != 6:
            raise ValueError("SmolVLA shadow client requires exactly six arm joints")
        if any(not math.isfinite(float(value)) for value in self.arm_positions_rad):
            raise ValueError("non-finite arm joint value")
        if not math.isfinite(float(self.gripper_value)):
            raise ValueError("non-finite gripper value")
        if not math.isfinite(float(self.stamp_s)):
            raise ValueError("non-finite joint-state stamp")

    def to_request_dict(self) -> dict:
        self.validate()
        return {
            "joint1": float(self.arm_positions_rad[0]),
            "joint2": float(self.arm_positions_rad[1]),
            "joint3": float(self.arm_positions_rad[2]),
            "joint4": float(self.arm_positions_rad[3]),
            "joint5": float(self.arm_positions_rad[4]),
            "joint6": float(self.arm_positions_rad[5]),
            "gripper": float(self.gripper_value),
        }

    def to_log_dict(self) -> dict:
        payload = self.to_request_dict()
        payload.update(
            {
                "arm_joint_names": list(self.arm_joint_names),
                "stamp_s": float(self.stamp_s),
                "age_s": float(self.age_s),
                "gripper_units": self.gripper_units,
            }
        )
        return payload


def encode_image_bytes(data: bytes, max_bytes: int = 8_000_000) -> str:
    if not data:
        raise ValueError("image is empty")
    if len(data) > max_bytes:
        raise ValueError(f"image exceeds request limit: {len(data)} > {max_bytes}")
    if data.startswith(b"\x89PNG\r\n\x1a\n"):
        mime = "image/png"
    elif data.startswith(b"\xff\xd8\xff"):
        mime = "image/jpeg"
    else:
        raise ValueError("image must be PNG or JPEG")
    return f"data:{mime};base64,{base64.b64encode(data).decode('ascii')}"


def encode_image_file(path: str | Path, max_bytes: int = 8_000_000) -> str:
    return encode_image_bytes(Path(path).read_bytes(), max_bytes=max_bytes)


def build_piper_schema_payload() -> dict:
    return {
        "schema": {
            "observation": {
                "images": {
                    "external_camera": {"type": "RGB image", "source": "RealSense", "required": True},
                    "wrist_camera": {"type": "RGB image", "source": "PiPER wrist camera", "required": False},
                },
                "state_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"],
                "state_dimension": 7,
                "joint_units": "radians",
                "gripper_units": "unverified",
                "state_order": "exactly as listed",
            },
            "task": {"type": "natural-language instruction"},
            "action": {
                "names": [
                    "target_joint1",
                    "target_joint2",
                    "target_joint3",
                    "target_joint4",
                    "target_joint5",
                    "target_joint6",
                    "target_gripper",
                ],
                "dimension": 7,
                "semantics": "proposed absolute joint targets",
                "joint_units": "radians",
                "gripper_units": "unverified",
                "execution_allowed": False,
            },
        }
    }


def build_action_preview_request(
    images: Iterable[str],
    task_description: str,
    joint_state: JointStateRecord,
    timestamps: Optional[dict] = None,
    source_metadata: Optional[dict] = None,
) -> dict:
    instruction = task_description.strip()
    if not instruction:
        raise ValueError("task_description must be non-empty")
    image_values = list(images)
    if not image_values:
        raise ValueError("at least one image is required")
    if len(image_values) > 2:
        raise ValueError("SmolVLA PiPER shadow request supports at most two images")
    return {
        "images": image_values,
        "joint_state": joint_state.to_request_dict(),
        "task_description": instruction,
        "timestamps": timestamps or {},
        "source_metadata": source_metadata or {},
        "execution_allowed": False,
    }


class SmolVLAShadowPolicyClient:
    def __init__(self, endpoint: str = DEFAULT_ENDPOINT, timeout_s: float = DEFAULT_TIMEOUT_S) -> None:
        self.endpoint = endpoint.rstrip("/")
        self.timeout_s = float(timeout_s)

    def compatibility_check(self, payload: Optional[dict] = None) -> dict:
        body = payload or build_piper_schema_payload()
        try:
            response = requests.post(
                f"{self.endpoint}/compatibility-check",
                json=body,
                timeout=self.timeout_s,
            )
            data = response.json()
        except Exception as exc:
            return {
                "request_success": False,
                "compatible": False,
                "error": repr(exc),
                "execution_allowed": False,
            }
        data["execution_allowed"] = False
        return data

    def preview_action(self, payload: dict, schema_payload: Optional[dict] = None) -> dict:
        compatibility = self.compatibility_check(schema_payload)
        if not compatibility.get("request_success", False):
            return {
                "request_success": False,
                "compatibility": compatibility,
                "inference_attempted": False,
                "error": compatibility.get("error", "compatibility check failed"),
                "execution_allowed": False,
            }
        if not compatibility.get("compatible", False):
            return {
                "request_success": True,
                "compatibility": compatibility,
                "compatible": False,
                "inference_attempted": False,
                "execution_allowed": False,
            }
        try:
            response = requests.post(f"{self.endpoint}/action-preview", json=payload, timeout=self.timeout_s)
            data = response.json()
        except Exception as exc:
            return {
                "request_success": False,
                "compatibility": compatibility,
                "inference_attempted": True,
                "error": repr(exc),
                "execution_allowed": False,
            }
        data["execution_allowed"] = False
        if data.get("compatibility_verified") is False:
            data["inference_attempted"] = False
        return data


def read_state_json(path: str | Path, expected_joint_names: Optional[List[str]] = None) -> JointStateRecord:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if "joint_names" in raw and "joint_positions" in raw:
        mapped = map_joint_state(
            raw["joint_names"],
            raw["joint_positions"],
            float(raw.get("stamp_s", time.time())),
            expected_joint_names or DEFAULT_ARM_JOINT_NAMES,
        )
        return JointStateRecord(
            arm_joint_names=list(mapped.arm_joint_names),
            arm_positions_rad=list(mapped.arm_positions),
            gripper_value=float(mapped.gripper_position or 0.0),
            stamp_s=float(mapped.stamp_s),
            age_s=float(mapped.age_s),
            gripper_units=str(raw.get("gripper_units", "unverified")),
        )
    names = list(expected_joint_names or DEFAULT_ARM_JOINT_NAMES)
    return JointStateRecord(
        arm_joint_names=names,
        arm_positions_rad=[float(raw[name]) for name in names],
        gripper_value=float(raw.get("gripper", raw.get("gripper_value", 0.0))),
        stamp_s=float(raw.get("stamp_s", time.time())),
        age_s=float(raw.get("age_s", 0.0)),
        gripper_units=str(raw.get("gripper_units", "unverified")),
    )


def read_live_joint_state(
    timeout_s: float,
    expected_joint_names: Optional[List[str]] = None,
    max_state_age_s: float = DEFAULT_MAX_STATE_AGE_S,
) -> JointStateRecord:
    try:
        import rospy
        from sensor_msgs.msg import JointState
    except Exception as exc:
        raise RuntimeError("live ROS joint-state capture requires rospy and sensor_msgs") from exc
    if not rospy.get_node_uri():
        rospy.init_node("smolvla_shadow_client", anonymous=True, disable_signals=True)
    msg = rospy.wait_for_message("/joint_states_single", JointState, timeout=timeout_s)
    stamp_s = float(msg.header.stamp.to_sec()) if getattr(msg, "header", None) else time.time()
    mapped = map_joint_state(msg.name, msg.position, stamp_s, expected_joint_names or DEFAULT_ARM_JOINT_NAMES)
    if mapped.age_s > max_state_age_s:
        raise RuntimeError(f"joint state is stale: age_s={mapped.age_s:.3f} max_state_age_s={max_state_age_s:.3f}")
    return JointStateRecord(
        arm_joint_names=list(mapped.arm_joint_names),
        arm_positions_rad=list(mapped.arm_positions),
        gripper_value=float(mapped.gripper_position or 0.0),
        stamp_s=float(mapped.stamp_s),
        age_s=float(mapped.age_s),
        gripper_units="unverified",
    )


def read_live_color_image(timeout_s: float) -> tuple[str, dict]:
    try:
        import cv2
        import rospy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
    except Exception as exc:
        raise RuntimeError("live ROS image capture requires rospy, cv_bridge, sensor_msgs, and cv2") from exc
    if not rospy.get_node_uri():
        rospy.init_node("smolvla_shadow_client", anonymous=True, disable_signals=True)
    msg = rospy.wait_for_message("/table_camera/color/image_raw", Image, timeout=timeout_s)
    image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
    ok, encoded = cv2.imencode(".jpg", image)
    if not ok:
        raise RuntimeError("failed to JPEG-encode live color image")
    metadata = {
        "topic": "/table_camera/color/image_raw",
        "frame_id": getattr(msg.header, "frame_id", ""),
        "stamp_s": float(msg.header.stamp.to_sec()) if getattr(msg, "header", None) else 0.0,
        "shape": list(getattr(image, "shape", ())),
    }
    return encode_image_bytes(encoded.tobytes()), metadata


def default_log_path(repo_root: Path) -> Path:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    directory = repo_root / "piper-on-bunker" / "logs" / "vla_shadow" / "smolvla"
    directory.mkdir(parents=True, exist_ok=True)
    return directory / f"{stamp}_smolvla_shadow.json"
