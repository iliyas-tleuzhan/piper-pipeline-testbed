from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import msgpack
import numpy as np
import yaml
from PIL import Image
from websockets.sync.client import connect

from piper_on_bunker.hardware.joint_state import DEFAULT_ARM_JOINT_NAMES, map_joint_state
from piper_on_bunker.hardware.piper_ros_arm import PiperRosArm
from piper_on_bunker.models import Pose
from piper_on_bunker.vendor import msgpack_numpy_compat

SHADOW_BANNER = "SHADOW MODE - NO ROBOT ACTION WILL BE EXECUTED"
MODEL_NAME = "LAP-3B"
RESAMPLE_BILINEAR = getattr(getattr(Image, "Resampling", Image), "BILINEAR")


@dataclass
class LapRuntimeConfig:
    host: str
    port: int
    color_image_topic: str
    joint_state_topic: str
    end_pose_topic: str
    axis_map: List[int]
    translation_scale: float
    max_translation_per_action_m: float
    preserve_orientation: bool
    max_speed_scaling: float
    max_acceleration_scaling: float
    workspace_bounds_m: Dict[str, List[float]]


@dataclass
class LapStateSnapshot:
    image_rgb: np.ndarray
    image_stamp_s: float
    pose_position_m: List[float]
    pose_quaternion_xyzw: List[float]
    pose_rpy_rad: List[float]
    joint_positions_rad: List[float]
    gripper_m: float
    joint_stamp_s: float
    end_pose_stamp_s: float


def load_lap_config(path: str | Path) -> LapRuntimeConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    server = raw.get("server", {})
    topics = raw.get("topics", {})
    motion = raw.get("motion", {})
    return LapRuntimeConfig(
        host=str(server.get("host", "192.168.1.104")),
        port=int(server.get("port", 8016)),
        color_image_topic=str(topics.get("color_image", "/table_camera/color/image_raw")),
        joint_state_topic=str(topics.get("joint_state", "/joint_states_single")),
        end_pose_topic=str(topics.get("end_pose", "/end_pose")),
        axis_map=[int(v) for v in motion.get("axis_map", [0, 1, 2])],
        translation_scale=float(motion.get("translation_scale", 1.0)),
        max_translation_per_action_m=float(motion.get("max_translation_per_action_m", 0.02)),
        preserve_orientation=bool(motion.get("preserve_orientation", True)),
        max_speed_scaling=float(motion.get("max_speed_scaling", 0.05)),
        max_acceleration_scaling=float(motion.get("max_acceleration_scaling", 0.05)),
        workspace_bounds_m=dict(motion.get("workspace_bounds_m", {})),
    )


def quaternion_to_rpy_rad(qx: float, qy: float, qz: float, qw: float) -> List[float]:
    values = [float(qx), float(qy), float(qz), float(qw)]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion contains non-finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 0.0:
        raise ValueError("quaternion norm is zero")
    qx, qy, qz, qw = [value / norm for value in values]
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]


def euler_to_rot6d(euler_rad: List[float]) -> List[float]:
    roll, pitch, yaw = [float(v) for v in euler_rad]
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=float)
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=float)
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=float)
    rot = rz @ ry @ rx
    return rot[:, :2].reshape(-1).tolist()


def resize_with_pad_rgb(image_rgb: np.ndarray, size: int = 224) -> np.ndarray:
    image = Image.fromarray(np.asarray(image_rgb, dtype=np.uint8))
    width, height = image.size
    scale = min(size / max(width, 1), size / max(height, 1))
    new_size = (max(1, int(round(width * scale))), max(1, int(round(height * scale))))
    resized = image.resize(new_size, RESAMPLE_BILINEAR)
    canvas = Image.new("RGB", (size, size))
    x = (size - new_size[0]) // 2
    y = (size - new_size[1]) // 2
    canvas.paste(resized, (x, y))
    return np.asarray(canvas, dtype=np.uint8)


def normalize_gripper_width(gripper_m: float) -> float:
    value = float(gripper_m)
    if not math.isfinite(value):
        raise ValueError("gripper value is non-finite")
    clamped = min(0.06, max(0.0, value))
    return clamped / 0.06 if 0.06 else 0.0


def build_lap_request(snapshot: LapStateSnapshot, instruction: str) -> dict:
    rot6d = euler_to_rot6d(snapshot.pose_rpy_rad)
    cartesian_position = snapshot.pose_position_m + rot6d
    gripper_position = np.asarray([normalize_gripper_width(snapshot.gripper_m)], dtype=np.float32)
    request = {
        "observation": {
            "base_0_rgb": resize_with_pad_rgb(snapshot.image_rgb),
            "cartesian_position": np.asarray(cartesian_position, dtype=np.float32),
            "joint_position": np.asarray(snapshot.joint_positions_rad, dtype=np.float32),
            "gripper_position": gripper_position,
            "state": np.asarray(cartesian_position + gripper_position.tolist(), dtype=np.float32),
            "euler": np.asarray(snapshot.pose_rpy_rad, dtype=np.float32),
        },
        "prompt": instruction,
    }
    return request


def capture_live_snapshot(config: LapRuntimeConfig, timeout_s: float = 5.0) -> LapStateSnapshot:
    try:
        import rospy
        from cv_bridge import CvBridge
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import Image, JointState
    except Exception as exc:
        raise RuntimeError("ROS capture requires rospy, cv_bridge, geometry_msgs, and sensor_msgs") from exc

    if not rospy.get_node_uri():
        rospy.init_node("lap_piper_client", anonymous=True, disable_signals=True)

    color = rospy.wait_for_message(config.color_image_topic, Image, timeout=timeout_s)
    joint = rospy.wait_for_message(config.joint_state_topic, JointState, timeout=timeout_s)
    pose = rospy.wait_for_message(config.end_pose_topic, PoseStamped, timeout=timeout_s)
    image_bgr = CvBridge().imgmsg_to_cv2(color, desired_encoding="bgr8")
    image_rgb = np.asarray(image_bgr[..., ::-1], dtype=np.uint8)
    mapped = map_joint_state(joint.name, joint.position, float(joint.header.stamp.to_sec()), DEFAULT_ARM_JOINT_NAMES)
    q = pose.pose.orientation
    p = pose.pose.position
    quat = [float(q.x), float(q.y), float(q.z), float(q.w)]
    return LapStateSnapshot(
        image_rgb=image_rgb,
        image_stamp_s=float(color.header.stamp.to_sec()),
        pose_position_m=[float(p.x), float(p.y), float(p.z)],
        pose_quaternion_xyzw=quat,
        pose_rpy_rad=quaternion_to_rpy_rad(*quat),
        joint_positions_rad=[float(v) for v in mapped.arm_positions],
        gripper_m=float(mapped.gripper_position or 0.0),
        joint_stamp_s=float(joint.header.stamp.to_sec()),
        end_pose_stamp_s=float(pose.header.stamp.to_sec()),
    )


class LapWebsocketClient:
    def __init__(self, host: str, port: int, timeout_s: float = 30.0) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s

    def infer(self, request: dict) -> dict:
        uri = f"ws://{self.host}:{self.port}"
        with connect(uri, open_timeout=self.timeout_s, close_timeout=self.timeout_s, max_size=None) as ws:
            metadata = msgpack_numpy_compat.unpackb(ws.recv())
            ws.send(msgpack_numpy_compat.packb(request))
            response = ws.recv()
            if isinstance(response, str):
                raise RuntimeError(f"LAP server returned text error:\n{response}")
            response = msgpack_numpy_compat.unpackb(response)
        return {"metadata": metadata, "response": response}


def clamp_translation(delta_xyz: np.ndarray, max_translation_m: float) -> np.ndarray:
    if delta_xyz.shape != (3,):
        raise ValueError("translation delta must have shape (3,)")
    norm = float(np.linalg.norm(delta_xyz))
    if norm <= max_translation_m or norm == 0.0:
        return delta_xyz
    return delta_xyz * (max_translation_m / norm)


def clamp_pose_to_workspace(target_xyz: np.ndarray, workspace_bounds_m: Dict[str, List[float]]) -> np.ndarray:
    clamped = target_xyz.copy()
    for index, axis in enumerate(("x", "y", "z")):
        bounds = workspace_bounds_m.get(axis)
        if bounds and len(bounds) == 2:
            clamped[index] = min(float(bounds[1]), max(float(bounds[0]), clamped[index]))
    return clamped


def action_to_target_pose(
    snapshot: LapStateSnapshot,
    response: dict,
    config: LapRuntimeConfig,
    max_actions: int = 1,
) -> dict:
    actions = np.asarray(response["actions"], dtype=float)
    if actions.ndim == 1:
        first = actions
        raw_actions = actions.reshape(1, -1)
    elif actions.ndim == 2:
        raw_actions = actions[: max(1, max_actions)]
        first = raw_actions[0]
    else:
        raise ValueError(f"unexpected LAP action shape: {actions.shape}")
    if first.shape[0] < 3:
        raise ValueError("LAP action must have at least xyz channels")
    axis_map = [int(v) for v in config.axis_map]
    delta_xyz = np.asarray([first[axis_map[0]], first[axis_map[1]], first[axis_map[2]]], dtype=float)
    delta_xyz = clamp_translation(delta_xyz * config.translation_scale, config.max_translation_per_action_m)
    current_xyz = np.asarray(snapshot.pose_position_m, dtype=float)
    unclamped_target = current_xyz + delta_xyz
    clamped_target = clamp_pose_to_workspace(unclamped_target, config.workspace_bounds_m)
    quat = snapshot.pose_quaternion_xyzw
    target_pose = Pose(
        x=float(clamped_target[0]),
        y=float(clamped_target[1]),
        z=float(clamped_target[2]),
        qx=float(quat[0]),
        qy=float(quat[1]),
        qz=float(quat[2]),
        qw=float(quat[3]),
        frame_id="base_link",
    )
    return {
        "raw_actions": raw_actions.tolist(),
        "selected_action": first.tolist(),
        "translation_delta_m": delta_xyz.tolist(),
        "current_pose": {
            "position": snapshot.pose_position_m,
            "quaternion_xyzw": snapshot.pose_quaternion_xyzw,
            "rpy_rad": snapshot.pose_rpy_rad,
        },
        "unclamped_target_pose": {
            "position": unclamped_target.tolist(),
            "quaternion_xyzw": quat,
        },
        "target_pose": {
            "position": clamped_target.tolist(),
            "quaternion_xyzw": quat,
        },
        "pose": target_pose,
    }


def preview_or_execute(
    target_pose: Pose,
    *,
    execute: bool,
    speed: float,
    acceleration: float,
) -> dict:
    arm = PiperRosArm(
        physical_motion_enabled=execute,
        safety={
            "max_speed_scaling": speed,
            "max_acceleration_scaling": acceleration,
        },
    )
    result = arm.move_to_pose(target_pose)
    return {
        "success": result.success,
        "status_code": result.status_code.value,
        "message": result.message,
        "outputs": result.outputs,
    }


def default_log_path() -> Path:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    directory = Path(__file__).resolve().parents[3] / "logs" / "lap"
    directory.mkdir(parents=True, exist_ok=True)
    return directory / f"{stamp}_lap_action.json"


def write_result(path: str | Path, payload: dict) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(payload, indent=2, sort_keys=True, default=str), encoding="utf-8")
