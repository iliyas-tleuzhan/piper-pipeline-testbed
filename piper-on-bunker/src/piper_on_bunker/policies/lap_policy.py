from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import yaml
from PIL import Image
from websockets.sync.client import connect

from piper_on_bunker.hardware.joint_state import DEFAULT_ARM_JOINT_NAMES, map_joint_state
from piper_on_bunker.models import Pose
from piper_on_bunker.vendor import msgpack_numpy_compat

SHADOW_BANNER = "SHADOW MODE - NO ROBOT ACTION WILL BE EXECUTED"
MODEL_NAME = "LAP-3B"
RESAMPLE_BILINEAR = getattr(getattr(Image, "Resampling", Image), "BILINEAR")


@dataclass(frozen=True)
class MotionProfile:
    name: str
    velocity_scaling: float
    acceleration_scaling: float


@dataclass
class LapRuntimeConfig:
    host: str
    port: int
    color_image_topic: str
    joint_state_topic: str
    end_pose_topic: str
    axis_map: List[int]
    translation_scale: float
    max_total_translation_m: float
    max_adjacent_waypoint_translation_m: float
    preserve_orientation: bool
    motion_profiles: Dict[str, MotionProfile]
    default_motion_profile: str
    workspace_bounds_m: Dict[str, List[float]]
    cartesian_eef_step_m: float
    cartesian_jump_threshold: float
    min_cartesian_path_fraction: float
    max_total_tcp_displacement_m: float
    max_total_joint_delta_rad: float
    max_adjacent_joint_delta_rad: float
    position_tolerance_m: float
    planning_time_s: float
    max_state_age_s: float


@dataclass
class LapStateSnapshot:
    image_rgb: np.ndarray
    image_stamp_s: float
    telemetry_end_pose_position_m: List[float]
    telemetry_end_pose_quaternion_xyzw: List[float]
    telemetry_end_pose_rpy_rad: List[float]
    joint_positions_rad: List[float]
    gripper_m: float
    joint_stamp_s: float
    end_pose_stamp_s: float


@dataclass
class MoveItCurrentTcpPose:
    planning_frame: str
    end_effector_link: str
    position_m: List[float]
    quaternion_xyzw: List[float]
    rpy_rad: List[float]


@dataclass
class LapTrajectoryPlan:
    action_semantics: str
    lap_horizon_length: int
    selected_horizon_length: int
    raw_actions: List[List[float]]
    absolute_tcp_targets: List[Pose]
    telemetry_end_pose: Dict[str, Any]
    moveit_current_tcp_pose: Dict[str, Any]
    moveit_planning_frame: str
    moveit_end_effector_link: str
    total_requested_tcp_displacement_m: float
    maximum_adjacent_waypoint_translation_m: float
    configured_total_translation_limit_m: float
    configured_adjacent_waypoint_limit_m: float
    rejected_waypoint_index: Optional[int]
    horizon_truncated: bool
    rejected_waypoint_reason: Optional[str]


def load_lap_config(path: str | Path) -> LapRuntimeConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    server = raw.get("server", {})
    topics = raw.get("topics", {})
    motion = raw.get("motion", {})
    profile_defaults = {
        "safe": MotionProfile("safe", 0.20, 0.15),
        "normal": MotionProfile("normal", 0.50, 0.35),
        "fast": MotionProfile("fast", 0.80, 0.60),
    }
    configured_profiles = {}
    for name, default_profile in profile_defaults.items():
        raw_profile = (motion.get("profiles") or {}).get(name, {})
        configured_profiles[name] = MotionProfile(
            name=name,
            velocity_scaling=max(0.0, min(1.0, float(raw_profile.get("velocity_scaling", default_profile.velocity_scaling)))),
            acceleration_scaling=max(0.0, min(1.0, float(raw_profile.get("acceleration_scaling", default_profile.acceleration_scaling)))),
        )
    return LapRuntimeConfig(
        host=str(server.get("host", "192.168.1.104")),
        port=int(server.get("port", 8016)),
        color_image_topic=str(topics.get("color_image", "/table_camera/color/image_raw")),
        joint_state_topic=str(topics.get("joint_state", "/joint_states_single")),
        end_pose_topic=str(topics.get("end_pose", "/end_pose")),
        axis_map=[int(v) for v in motion.get("axis_map", [0, 1, 2])],
        translation_scale=float(motion.get("translation_scale", 1.0)),
        max_total_translation_m=float(
            motion.get(
                "max_total_translation_m",
                motion.get("max_translation_per_action_m", 0.20),
            )
        ),
        max_adjacent_waypoint_translation_m=float(motion.get("max_adjacent_waypoint_translation_m", 0.03)),
        preserve_orientation=bool(motion.get("preserve_orientation", True)),
        motion_profiles=configured_profiles,
        default_motion_profile=str(motion.get("default_motion_profile", "fast")),
        workspace_bounds_m=dict(motion.get("workspace_bounds_m", {})),
        cartesian_eef_step_m=float(motion.get("cartesian_eef_step_m", 0.005)),
        cartesian_jump_threshold=float(motion.get("cartesian_jump_threshold", 0.0)),
        min_cartesian_path_fraction=float(motion.get("min_cartesian_path_fraction", 0.95)),
        max_total_tcp_displacement_m=float(
            motion.get(
                "max_total_tcp_displacement_m",
                motion.get("max_total_translation_m", 0.20),
            )
        ),
        max_total_joint_delta_rad=float(motion.get("max_total_joint_delta_rad", 0.75)),
        max_adjacent_joint_delta_rad=float(motion.get("max_adjacent_joint_delta_rad", 0.30)),
        position_tolerance_m=float(motion.get("position_tolerance_m", 0.01)),
        planning_time_s=float(motion.get("planning_time_s", 10.0)),
        max_state_age_s=float(motion.get("max_state_age_s", 1.0)),
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
    rot6d = euler_to_rot6d(snapshot.telemetry_end_pose_rpy_rad)
    cartesian_position = snapshot.telemetry_end_pose_position_m + rot6d
    gripper_position = np.asarray([normalize_gripper_width(snapshot.gripper_m)], dtype=np.float32)
    request = {
        "observation": {
            "base_0_rgb": resize_with_pad_rgb(snapshot.image_rgb),
            "cartesian_position": np.asarray(cartesian_position, dtype=np.float32),
            "joint_position": np.asarray(snapshot.joint_positions_rad, dtype=np.float32),
            "gripper_position": gripper_position,
            "state": np.asarray(cartesian_position + gripper_position.tolist(), dtype=np.float32),
            "euler": np.asarray(snapshot.telemetry_end_pose_rpy_rad, dtype=np.float32),
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
        telemetry_end_pose_position_m=[float(p.x), float(p.y), float(p.z)],
        telemetry_end_pose_quaternion_xyzw=quat,
        telemetry_end_pose_rpy_rad=quaternion_to_rpy_rad(*quat),
        joint_positions_rad=[float(v) for v in mapped.arm_positions],
        gripper_m=float(mapped.gripper_position or 0.0),
        joint_stamp_s=float(joint.header.stamp.to_sec()),
        end_pose_stamp_s=float(pose.header.stamp.to_sec()),
    )


def get_moveit_current_tcp_pose(
    group_name: str = "arm",
    end_effector_link: str = "gripper_tcp",
    timeout_s: float = 10.0,
) -> MoveItCurrentTcpPose:
    try:
        import moveit_commander
        import rospy
    except Exception as exc:
        raise RuntimeError("MoveIt TCP pose capture requires moveit_commander and rospy") from exc

    if not rospy.get_node_uri():
        rospy.init_node("lap_piper_moveit_pose", anonymous=True, disable_signals=True)

    moveit_commander.roscpp_initialize([])
    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_planning_time(float(timeout_s))
    current = group.get_current_pose(end_effector_link)
    quaternion_xyzw = [
        float(current.pose.orientation.x),
        float(current.pose.orientation.y),
        float(current.pose.orientation.z),
        float(current.pose.orientation.w),
    ]
    return MoveItCurrentTcpPose(
        planning_frame=str(group.get_planning_frame()),
        end_effector_link=str(group.get_end_effector_link()),
        position_m=[
            float(current.pose.position.x),
            float(current.pose.position.y),
            float(current.pose.position.z),
        ],
        quaternion_xyzw=quaternion_xyzw,
        rpy_rad=quaternion_to_rpy_rad(*quaternion_xyzw),
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


def get_motion_profile(config: LapRuntimeConfig, profile_name: Optional[str]) -> MotionProfile:
    selected = profile_name or config.default_motion_profile
    if selected not in config.motion_profiles:
        raise ValueError(f"unknown motion profile: {selected}")
    return config.motion_profiles[selected]


def lap_action_semantics() -> str:
    return (
        "future_tcp_delta_from_current_state_per_horizon_row"
    )


def _lap_actions_array(response: dict, max_actions: int) -> np.ndarray:
    actions = np.asarray(response["actions"], dtype=float)
    if actions.ndim == 1:
        actions = actions.reshape(1, -1)
    if actions.ndim != 2:
        raise ValueError(f"unexpected LAP action shape: {actions.shape}")
    return actions[: max(1, max_actions)]


def _translation_from_action_row(row: np.ndarray, config: LapRuntimeConfig) -> np.ndarray:
    axis_map = [int(v) for v in config.axis_map]
    delta_xyz = np.asarray([row[axis_map[0]], row[axis_map[1]], row[axis_map[2]]], dtype=float)
    if not np.isfinite(delta_xyz).all():
        raise ValueError("LAP translation channels must be finite")
    return delta_xyz * float(config.translation_scale)


def _validate_workspace_point(target_xyz: np.ndarray, workspace_bounds_m: Dict[str, List[float]]) -> None:
    for index, axis in enumerate(("x", "y", "z")):
        bounds = workspace_bounds_m.get(axis)
        if bounds and len(bounds) == 2:
            if float(target_xyz[index]) < float(bounds[0]) or float(target_xyz[index]) > float(bounds[1]):
                raise ValueError(
                    f"target {axis}={float(target_xyz[index]):.6f} m is outside workspace bounds [{float(bounds[0]):.6f}, {float(bounds[1]):.6f}]"
                )


def horizon_to_trajectory_plan(
    snapshot: LapStateSnapshot,
    moveit_tcp_pose: MoveItCurrentTcpPose,
    response: dict,
    config: LapRuntimeConfig,
    *,
    max_actions: int,
) -> LapTrajectoryPlan:
    raw_actions = _lap_actions_array(response, max_actions)
    current_xyz = np.asarray(moveit_tcp_pose.position_m, dtype=float)
    absolute_targets: List[Pose] = []
    total_requested_tcp_displacement_m = 0.0
    accepted_total_tcp_displacement_m = 0.0
    maximum_adjacent_waypoint_translation_m = 0.0
    rejected_waypoint_index: Optional[int] = None
    rejected_waypoint_reason: Optional[str] = None
    horizon_truncated = False
    previous_delta_xyz = np.zeros(3, dtype=float)
    total_limit = float(config.max_total_translation_m)
    adjacent_limit = float(config.max_adjacent_waypoint_translation_m)
    min_useful_waypoints = 1 if raw_actions.shape[0] <= 1 else 2
    tolerance = 1e-9

    # Official LAP real-robot postprocessing adds the current state to every row
    # of the returned chunk independently. That means each row is a future delta
    # from the current state, not an increment to the previous row.
    for index, row in enumerate(raw_actions):
        delta_xyz = _translation_from_action_row(row, config)
        displacement = float(np.linalg.norm(delta_xyz))
        adjacent_displacement = float(np.linalg.norm(delta_xyz - previous_delta_xyz))
        total_requested_tcp_displacement_m = max(total_requested_tcp_displacement_m, displacement)
        maximum_adjacent_waypoint_translation_m = max(
            maximum_adjacent_waypoint_translation_m,
            adjacent_displacement,
        )
        if displacement - total_limit > tolerance:
            rejected_waypoint_index = index
            rejected_waypoint_reason = "max_total_translation_m"
            break
        if adjacent_displacement - adjacent_limit > tolerance:
            raise ValueError(
                "LAP waypoint "
                f"{index} adjacent displacement {adjacent_displacement:.6f} m exceeds "
                f"max_adjacent_waypoint_translation_m={adjacent_limit:.6f}"
            )
        target_xyz = current_xyz + delta_xyz
        _validate_workspace_point(target_xyz, config.workspace_bounds_m)
        accepted_total_tcp_displacement_m = max(accepted_total_tcp_displacement_m, displacement)
        absolute_targets.append(
            Pose(
                x=float(target_xyz[0]),
                y=float(target_xyz[1]),
                z=float(target_xyz[2]),
                qx=float(moveit_tcp_pose.quaternion_xyzw[0]),
                qy=float(moveit_tcp_pose.quaternion_xyzw[1]),
                qz=float(moveit_tcp_pose.quaternion_xyzw[2]),
                qw=float(moveit_tcp_pose.quaternion_xyzw[3]),
                frame_id=moveit_tcp_pose.planning_frame,
            )
        )
        previous_delta_xyz = delta_xyz

    if rejected_waypoint_index is not None:
        if len(absolute_targets) < min_useful_waypoints:
            raise ValueError(
                "LAP horizon exceeded max_total_translation_m before a useful safe prefix was available: "
                f"rejected_waypoint_index={rejected_waypoint_index} "
                f"maximum_total_tcp_displacement_m={total_requested_tcp_displacement_m:.6f} "
                f"configured_total_limit_m={total_limit:.6f}"
            )
        horizon_truncated = True

    if accepted_total_tcp_displacement_m > float(config.max_total_tcp_displacement_m):
        raise ValueError(
            "accepted LAP horizon requests "
            f"{accepted_total_tcp_displacement_m:.6f} m, exceeding max_total_tcp_displacement_m={config.max_total_tcp_displacement_m:.6f}"
        )

    return LapTrajectoryPlan(
        action_semantics=lap_action_semantics(),
        lap_horizon_length=int(np.asarray(response["actions"]).shape[0] if np.asarray(response["actions"]).ndim > 1 else 1),
        selected_horizon_length=len(absolute_targets),
        raw_actions=raw_actions.tolist(),
        absolute_tcp_targets=absolute_targets,
        telemetry_end_pose={
            "position": snapshot.telemetry_end_pose_position_m,
            "quaternion_xyzw": snapshot.telemetry_end_pose_quaternion_xyzw,
            "rpy_rad": snapshot.telemetry_end_pose_rpy_rad,
        },
        moveit_current_tcp_pose={
            "position": moveit_tcp_pose.position_m,
            "quaternion_xyzw": moveit_tcp_pose.quaternion_xyzw,
            "rpy_rad": moveit_tcp_pose.rpy_rad,
        },
        moveit_planning_frame=moveit_tcp_pose.planning_frame,
        moveit_end_effector_link=moveit_tcp_pose.end_effector_link,
        total_requested_tcp_displacement_m=total_requested_tcp_displacement_m,
        maximum_adjacent_waypoint_translation_m=maximum_adjacent_waypoint_translation_m,
        configured_total_translation_limit_m=total_limit,
        configured_adjacent_waypoint_limit_m=adjacent_limit,
        rejected_waypoint_index=rejected_waypoint_index,
        horizon_truncated=horizon_truncated,
        rejected_waypoint_reason=rejected_waypoint_reason,
    )


def action_to_target_pose(
    snapshot: LapStateSnapshot,
    moveit_tcp_pose: MoveItCurrentTcpPose,
    response: dict,
    config: LapRuntimeConfig,
    max_actions: int = 1,
) -> dict:
    plan = horizon_to_trajectory_plan(snapshot, moveit_tcp_pose, response, config, max_actions=max_actions)
    first = np.asarray(plan.raw_actions[0], dtype=float)
    delta_xyz = _translation_from_action_row(first, config)
    first_target = plan.absolute_tcp_targets[0]
    return {
        "raw_actions": plan.raw_actions,
        "selected_action": plan.raw_actions[0],
        "lap_translation_delta": delta_xyz.tolist(),
        "action_semantics": plan.action_semantics,
        "telemetry_end_pose": plan.telemetry_end_pose,
        "moveit_current_tcp_pose": plan.moveit_current_tcp_pose,
        "moveit_planning_frame": plan.moveit_planning_frame,
        "moveit_end_effector_link": plan.moveit_end_effector_link,
        "proposed_tcp_target": {
            "position": [first_target.x, first_target.y, first_target.z],
            "quaternion_xyzw": [first_target.qx, first_target.qy, first_target.qz, first_target.qw],
        },
        "horizon_diagnostics": {
            "maximum_total_tcp_displacement_m": plan.total_requested_tcp_displacement_m,
            "maximum_adjacent_waypoint_translation_m": plan.maximum_adjacent_waypoint_translation_m,
            "configured_total_translation_limit_m": plan.configured_total_translation_limit_m,
            "configured_adjacent_waypoint_limit_m": plan.configured_adjacent_waypoint_limit_m,
            "rejected_waypoint_index": plan.rejected_waypoint_index,
            "rejected_waypoint_reason": plan.rejected_waypoint_reason,
            "horizon_truncated": plan.horizon_truncated,
        },
        "unclamped_tcp_target": {
            "position": [first_target.x, first_target.y, first_target.z],
            "quaternion_xyzw": [first_target.qx, first_target.qy, first_target.qz, first_target.qw],
        },
        "pose": first_target,
    }


def _trajectory_joint_metrics(trajectory) -> dict:
    points = list(getattr(trajectory.joint_trajectory, "points", []) or [])
    if not points:
        return {
            "trajectory_points": 0,
            "planned_duration_s": 0.0,
            "maximum_joint_delta_rad": 0.0,
            "maximum_adjacent_point_joint_delta_rad": 0.0,
        }
    start = np.asarray(points[0].positions, dtype=float)
    maximum_joint_delta = 0.0
    maximum_adjacent_delta = 0.0
    previous = start
    for point in points:
        current = np.asarray(point.positions, dtype=float)
        maximum_joint_delta = max(maximum_joint_delta, float(np.max(np.abs(current - start))))
        maximum_adjacent_delta = max(maximum_adjacent_delta, float(np.max(np.abs(current - previous))))
        previous = current
    return {
        "trajectory_points": len(points),
        "planned_duration_s": float(points[-1].time_from_start.to_sec()),
        "maximum_joint_delta_rad": maximum_joint_delta,
        "maximum_adjacent_point_joint_delta_rad": maximum_adjacent_delta,
    }


def _verify_trajectory_metrics(metrics: dict, config: LapRuntimeConfig) -> None:
    if metrics["maximum_joint_delta_rad"] > float(config.max_total_joint_delta_rad):
        raise ValueError(
            f"planned trajectory max joint delta {metrics['maximum_joint_delta_rad']:.6f} rad exceeds "
            f"max_total_joint_delta_rad={config.max_total_joint_delta_rad:.6f}"
        )
    if metrics["maximum_adjacent_point_joint_delta_rad"] > float(config.max_adjacent_joint_delta_rad):
        raise ValueError(
            f"planned trajectory adjacent-point delta {metrics['maximum_adjacent_point_joint_delta_rad']:.6f} rad exceeds "
            f"max_adjacent_joint_delta_rad={config.max_adjacent_joint_delta_rad:.6f}"
        )


def _normalize_plan_result(result):
    if isinstance(result, tuple):
        success, trajectory, planning_time, error_code = result
        return {
            "success": bool(success),
            "trajectory": trajectory,
            "planning_time_s": float(planning_time),
            "moveit_error_code": int(getattr(error_code, "val", error_code)),
        }
    trajectory = result
    points = list(getattr(trajectory.joint_trajectory, "points", []) or [])
    return {
        "success": bool(points),
        "trajectory": trajectory,
        "planning_time_s": None,
        "moveit_error_code": 1 if points else -1,
    }


def _make_geometry_pose(target: Pose):
    from geometry_msgs.msg import Pose as GeometryPose

    pose_msg = GeometryPose()
    pose_msg.position.x = float(target.x)
    pose_msg.position.y = float(target.y)
    pose_msg.position.z = float(target.z)
    pose_msg.orientation.x = float(target.qx)
    pose_msg.orientation.y = float(target.qy)
    pose_msg.orientation.z = float(target.qz)
    pose_msg.orientation.w = float(target.qw)
    return pose_msg


def _verify_current_tcp_pose(group, target: Pose, tolerance_m: float) -> dict:
    current = group.get_current_pose("gripper_tcp")
    current_position = [
        float(current.pose.position.x),
        float(current.pose.position.y),
        float(current.pose.position.z),
    ]
    error = math.sqrt(
        sum((float(a) - float(b)) ** 2 for a, b in zip(current_position, [target.x, target.y, target.z]))
    )
    return {
        "measured_position": current_position,
        "position_error_m": error,
        "position_tolerance_m": float(tolerance_m),
        "target_reached": bool(error <= tolerance_m),
    }


def preview_or_execute(
    trajectory_plan: LapTrajectoryPlan,
    *,
    execute: bool,
    motion_profile: MotionProfile,
    config: LapRuntimeConfig,
) -> dict:
    try:
        import moveit_commander
        import rospy
    except Exception as exc:
        raise RuntimeError("MoveIt trajectory preview requires moveit_commander and rospy") from exc

    if not rospy.get_node_uri():
        rospy.init_node("lap_piper_motion_preview", anonymous=True, disable_signals=True)

    moveit_commander.roscpp_initialize([])
    group = moveit_commander.MoveGroupCommander("arm")
    group.set_start_state_to_current_state()
    group.set_planning_time(float(config.planning_time_s))
    group.set_num_planning_attempts(20)
    group.set_pose_reference_frame(trajectory_plan.moveit_planning_frame)
    group.set_end_effector_link(trajectory_plan.moveit_end_effector_link)
    group.set_max_velocity_scaling_factor(float(motion_profile.velocity_scaling))
    group.set_max_acceleration_scaling_factor(float(motion_profile.acceleration_scaling))

    waypoint_msgs = [_make_geometry_pose(target) for target in trajectory_plan.absolute_tcp_targets]
    cartesian_trajectory = None
    cartesian_fraction = 0.0
    planning_mode = "cartesian_path"
    if waypoint_msgs:
        cartesian_trajectory, cartesian_fraction = group.compute_cartesian_path(
            waypoint_msgs,
            float(config.cartesian_eef_step_m),
            float(config.cartesian_jump_threshold),
        )
    if cartesian_trajectory and getattr(cartesian_trajectory.joint_trajectory, "points", []):
        cartesian_trajectory = group.retime_trajectory(
            group.get_current_state(),
            cartesian_trajectory,
            velocity_scaling_factor=float(motion_profile.velocity_scaling),
            acceleration_scaling_factor=float(motion_profile.acceleration_scaling),
        )

    if not cartesian_trajectory or not getattr(cartesian_trajectory.joint_trajectory, "points", []) or float(cartesian_fraction) < float(config.min_cartesian_path_fraction):
        planning_mode = "final_pose_fallback"
        final_target = trajectory_plan.absolute_tcp_targets[-1]
        group.set_pose_target(_make_geometry_pose(final_target), trajectory_plan.moveit_end_effector_link)
        normalized = _normalize_plan_result(group.plan())
        trajectory = normalized["trajectory"]
        planning_success = bool(normalized["success"])
        planning_time_s = normalized["planning_time_s"]
        moveit_error_code = normalized["moveit_error_code"]
    else:
        trajectory = cartesian_trajectory
        planning_success = True
        planning_time_s = None
        moveit_error_code = 1

    metrics = _trajectory_joint_metrics(trajectory) if planning_success else _trajectory_joint_metrics(type("EmptyTrajectory", (), {"joint_trajectory": type("JT", (), {"points": []})()})())
    if planning_success:
        _verify_trajectory_metrics(metrics, config)

    outputs = {
        "execution_allowed": bool(execute),
        "planning_success": bool(planning_success),
        "planning_mode": planning_mode,
        "lap_horizon_length": int(trajectory_plan.lap_horizon_length),
        "selected_horizon_length": int(trajectory_plan.selected_horizon_length),
        "confirmed_action_semantics": trajectory_plan.action_semantics,
        "maximum_total_tcp_displacement_m": float(trajectory_plan.total_requested_tcp_displacement_m),
        "maximum_adjacent_waypoint_translation_m": float(trajectory_plan.maximum_adjacent_waypoint_translation_m),
        "configured_total_translation_limit_m": float(trajectory_plan.configured_total_translation_limit_m),
        "configured_adjacent_waypoint_limit_m": float(trajectory_plan.configured_adjacent_waypoint_limit_m),
        "rejected_waypoint_index": trajectory_plan.rejected_waypoint_index,
        "rejected_waypoint_reason": trajectory_plan.rejected_waypoint_reason,
        "horizon_truncated": bool(trajectory_plan.horizon_truncated),
        "cartesian_path_fraction": float(cartesian_fraction),
        "velocity_scaling": float(motion_profile.velocity_scaling),
        "acceleration_scaling": float(motion_profile.acceleration_scaling),
        "controller_command_rate_hz": 50.0,
        "moveit_planning_frame": trajectory_plan.moveit_planning_frame,
        "moveit_end_effector_link": trajectory_plan.moveit_end_effector_link,
        "moveit_current_tcp_pose": trajectory_plan.moveit_current_tcp_pose,
        "telemetry_end_pose": trajectory_plan.telemetry_end_pose,
        "tcp_waypoints": [
            {
                "position": [target.x, target.y, target.z],
                "quaternion_xyzw": [target.qx, target.qy, target.qz, target.qw],
            }
            for target in trajectory_plan.absolute_tcp_targets
        ],
        "trajectory_metrics": metrics,
        "moveit_request_preview": {
            "path_type": planning_mode,
            "will_call_service": False,
            "will_execute": bool(execute),
            "target_frame": trajectory_plan.moveit_planning_frame,
            "end_effector_link": trajectory_plan.moveit_end_effector_link,
        },
        "planned_trajectory_duration_s": metrics["planned_duration_s"],
        "maximum_joint_delta_rad": metrics["maximum_joint_delta_rad"],
        "maximum_adjacent_point_joint_delta_rad": metrics["maximum_adjacent_point_joint_delta_rad"],
        "moveit_error_code": moveit_error_code,
        "planning_time_s": planning_time_s,
    }

    if not planning_success:
        return {
            "success": False,
            "status_code": "PLANNING_FAILURE",
            "message": "failed to plan a continuous LAP trajectory",
            "outputs": outputs,
        }
    if float(cartesian_fraction) < float(config.min_cartesian_path_fraction) and planning_mode == "cartesian_path":
        return {
            "success": False,
            "status_code": "PLANNING_FAILURE",
            "message": "cartesian path fraction below configured minimum",
            "outputs": outputs,
        }
    if not execute:
        return {
            "success": True,
            "status_code": "OK",
            "message": "continuous LAP trajectory planned in shadow mode",
            "outputs": outputs,
        }

    group.execute(trajectory, wait=True)
    group.stop()
    group.clear_pose_targets()
    verification = _verify_current_tcp_pose(
        group,
        trajectory_plan.absolute_tcp_targets[-1],
        tolerance_m=float(config.position_tolerance_m),
    )
    outputs["tcp_completion_verification"] = verification
    return {
        "success": bool(verification["target_reached"]),
        "status_code": "OK" if verification["target_reached"] else "POSE_NOT_REACHED",
        "message": "continuous LAP trajectory executed" if verification["target_reached"] else "continuous LAP trajectory executed but TCP target was not reached",
        "outputs": outputs,
    }


def default_log_path() -> Path:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    directory = Path(__file__).resolve().parents[3] / "logs" / "lap"
    directory.mkdir(parents=True, exist_ok=True)
    return directory / f"{stamp}_lap_action.json"


def write_result(path: str | Path, payload: dict) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(payload, indent=2, sort_keys=True, default=str), encoding="utf-8")
