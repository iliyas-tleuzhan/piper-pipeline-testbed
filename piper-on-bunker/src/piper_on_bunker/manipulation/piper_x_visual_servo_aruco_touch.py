from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from piper_on_bunker.perception.piper_x_aruco_pose import PiperXArucoPoseConfig
from piper_on_bunker.perception.piper_x_aruco_pose import camera_geometry_from_info
from piper_on_bunker.perception.piper_x_aruco_pose import detect_piper_x_aruco_pose


EXPECTED_MARKER_DICTIONARY = "DICT_ARUCO_ORIGINAL"
EXPECTED_MARKER_ID = 6
EXPECTED_MARKER_SIZE_M = 0.100


@dataclass(frozen=True)
class VisualServoTouchConfig:
    profile_id: str
    task_id: str
    marker_dictionary: str
    marker_id: int
    marker_size_m: float
    color_image_topic: str
    depth_image_topic: str
    camera_info_topic: str
    image_geometry_mode: str
    camera_frame: str
    gripper_frame: str
    planning_group: str
    end_effector_link: str
    handeye_translation_xyz_m: list[float]
    handeye_quaternion_xyzw: list[float]
    handeye_verified: bool
    handeye_source: str
    gripper_tip_offset_xyz_m: list[float]
    gripper_tip_offset_source: str
    max_image_age_s: float
    max_depth_age_s: float
    max_joint_state_age_s: float
    depth_roi_px: int
    min_depth_m: float
    max_depth_m: float
    image_center_tolerance_px: float
    max_lateral_step_m: float
    max_forward_step_m: float
    simple_up_step_m: float
    simple_forward_step_m: float
    simple_forward_axis_world: list[float]
    continuous_forward_stop_depth_m: float
    continuous_max_forward_m: float
    continuous_max_forward_steps: int
    max_alignment_iterations: int
    cartesian_eef_step_m: float
    cartesian_fraction_threshold: float
    contact_clearance_m: float
    require_verified_handeye_for_execution: bool
    physical_execution_enabled_by_default: bool

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "VisualServoTouchConfig":
        marker = data["marker"]
        camera = data["camera"]
        moveit = data["moveit"]
        frames = data["frames"]
        handeye = data["handeye"]
        gripper = data["gripper"]
        safety = data["safety"]
        depth = data["depth"]
        align = data["alignment"]
        return cls(
            profile_id=str(data["profile_id"]),
            task_id=str(data["task_id"]),
            marker_dictionary=str(marker["dictionary"]),
            marker_id=int(marker["id"]),
            marker_size_m=float(marker["size_m"]),
            color_image_topic=str(camera["color_image_topic"]),
            depth_image_topic=str(camera["depth_image_topic"]),
            camera_info_topic=str(camera["camera_info_topic"]),
            image_geometry_mode=str(camera.get("image_geometry_mode", "rectified")),
            camera_frame=str(frames["camera_frame"]),
            gripper_frame=str(frames["gripper_frame"]),
            planning_group=str(moveit["planning_group"]),
            end_effector_link=str(moveit["end_effector_link"]),
            handeye_translation_xyz_m=[float(v) for v in handeye["translation_xyz_m"]],
            handeye_quaternion_xyzw=[float(v) for v in handeye["quaternion_xyzw"]],
            handeye_verified=bool(handeye.get("verified", False)),
            handeye_source=str(handeye.get("source", "unverified_local_config")),
            gripper_tip_offset_xyz_m=[float(v) for v in gripper["tip_offset_from_gripper_base_xyz_m"]],
            gripper_tip_offset_source=str(gripper.get("tip_offset_source", "unverified_local_config")),
            max_image_age_s=float(safety["max_image_age_s"]),
            max_depth_age_s=float(safety["max_depth_age_s"]),
            max_joint_state_age_s=float(safety["max_joint_state_age_s"]),
            depth_roi_px=int(depth.get("roi_px", 7)),
            min_depth_m=float(depth["min_depth_m"]),
            max_depth_m=float(depth["max_depth_m"]),
            image_center_tolerance_px=float(align["image_center_tolerance_px"]),
            max_lateral_step_m=float(align["max_lateral_step_m"]),
            max_forward_step_m=float(align["max_forward_step_m"]),
            simple_up_step_m=float(align.get("simple_up_step_m", 0.020)),
            simple_forward_step_m=float(align.get("simple_forward_step_m", 0.020)),
            simple_forward_axis_world=[float(v) for v in align.get("simple_forward_axis_world", [1.0, 0.0, 0.0])],
            continuous_forward_stop_depth_m=float(align.get("continuous_forward_stop_depth_m", 0.080)),
            continuous_max_forward_m=float(align.get("continuous_max_forward_m", 0.120)),
            continuous_max_forward_steps=int(align.get("continuous_max_forward_steps", 8)),
            max_alignment_iterations=int(align.get("max_alignment_iterations", 6)),
            cartesian_eef_step_m=float(align.get("cartesian_eef_step_m", 0.005)),
            cartesian_fraction_threshold=float(align.get("cartesian_fraction_threshold", 1.0)),
            contact_clearance_m=float(align["contact_clearance_m"]),
            require_verified_handeye_for_execution=bool(safety.get("require_verified_handeye_for_execution", True)),
            physical_execution_enabled_by_default=bool(safety["physical_execution_enabled_by_default"]),
        )


@dataclass(frozen=True)
class DepthTouchEstimate:
    marker_visible: bool
    reason: str | None
    marker_center_uv: tuple[float, float] | None
    image_center_uv: tuple[float, float]
    pixel_error_uv: tuple[float, float] | None
    depth_m: float | None
    marker_point_camera_m: list[float] | None
    marker_point_gripper_m: list[float] | None
    gripper_tip_offset_m: list[float]
    required_gripper_delta_m: list[float] | None
    limited_step_gripper_m: list[float] | None
    image_aligned: bool
    execution_allowed: bool
    execution_blockers: list[str]

    def as_dict(self) -> dict[str, Any]:
        return {
            "marker_visible": self.marker_visible,
            "reason": self.reason,
            "marker_center_uv": list(self.marker_center_uv) if self.marker_center_uv else None,
            "image_center_uv": list(self.image_center_uv),
            "pixel_error_uv": list(self.pixel_error_uv) if self.pixel_error_uv else None,
            "depth_m": self.depth_m,
            "marker_point_camera_m": self.marker_point_camera_m,
            "marker_point_gripper_m": self.marker_point_gripper_m,
            "gripper_tip_offset_m": self.gripper_tip_offset_m,
            "required_gripper_delta_m": self.required_gripper_delta_m,
            "limited_step_gripper_m": self.limited_step_gripper_m,
            "image_aligned": self.image_aligned,
            "execution_allowed": self.execution_allowed,
            "execution_blockers": self.execution_blockers,
        }


def load_visual_servo_touch_config(path: str | Path) -> VisualServoTouchConfig:
    with Path(path).open("r", encoding="utf-8") as fh:
        return VisualServoTouchConfig.from_mapping(yaml.safe_load(fh))


def depth_roi_m(depth_image: np.ndarray, *, u: float, v: float, encoding: str, roi_px: int) -> float | None:
    depth = np.asarray(depth_image)
    if depth.ndim != 2:
        raise ValueError(f"expected single-channel depth image, got {depth.shape}")
    if roi_px <= 0 or roi_px % 2 == 0:
        raise ValueError("depth ROI must be a positive odd integer")
    center_u = int(round(float(u)))
    center_v = int(round(float(v)))
    half = roi_px // 2
    u0 = max(0, center_u - half)
    u1 = min(depth.shape[1], center_u + half + 1)
    v0 = max(0, center_v - half)
    v1 = min(depth.shape[0], center_v + half + 1)
    if u0 >= u1 or v0 >= v1:
        return None
    roi = depth[v0:v1, u0:u1]
    enc = encoding.upper()
    if enc == "16UC1":
        values = roi.astype(np.float64) * 0.001
    elif enc == "32FC1":
        values = roi.astype(np.float64)
    else:
        raise ValueError(f"unsupported depth encoding {encoding!r}")
    values = values[np.isfinite(values) & (values > 0.0)]
    if values.size == 0:
        return None
    return float(np.median(values))


def deproject_pixel(camera_matrix: Any, *, u: float, v: float, depth_m: float) -> list[float]:
    matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    fx = float(matrix[0, 0])
    fy = float(matrix[1, 1])
    cx = float(matrix[0, 2])
    cy = float(matrix[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera matrix contains invalid focal length")
    z = float(depth_m)
    return [float((u - cx) * z / fx), float((v - cy) * z / fy), z]


def quaternion_xyzw_to_matrix(quaternion_xyzw: list[float]) -> np.ndarray:
    x, y, z, w = [float(v) for v in quaternion_xyzw]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("zero-norm quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_camera_point_to_gripper(
    point_camera_m: list[float],
    *,
    translation_gripper_camera_m: list[float],
    quaternion_gripper_camera_xyzw: list[float],
) -> list[float]:
    rotation = quaternion_xyzw_to_matrix(quaternion_gripper_camera_xyzw)
    point = np.asarray(point_camera_m, dtype=np.float64).reshape(3)
    translation = np.asarray(translation_gripper_camera_m, dtype=np.float64).reshape(3)
    return (rotation @ point + translation).astype(float).tolist()


def clamp_step(delta_m: list[float], *, max_lateral_step_m: float, max_forward_step_m: float) -> list[float]:
    # Treat gripper-frame x/y as lateral alignment and z as forward approach by
    # convention. The config must be updated if the measured tip frame differs.
    dx, dy, dz = [float(v) for v in delta_m]
    return [
        float(np.clip(dx, -max_lateral_step_m, max_lateral_step_m)),
        float(np.clip(dy, -max_lateral_step_m, max_lateral_step_m)),
        float(np.clip(dz, -max_forward_step_m, max_forward_step_m)),
    ]


def split_alignment_and_forward_steps(estimate: DepthTouchEstimate) -> tuple[list[float] | None, list[float] | None]:
    if estimate.limited_step_gripper_m is None:
        return None, None
    x, y, z = [float(v) for v in estimate.limited_step_gripper_m]
    return [x, y, 0.0], [0.0, 0.0, z]


def estimate_depth_touch_step(
    *,
    image_rgb: np.ndarray,
    depth_image: np.ndarray,
    depth_encoding: str,
    camera_matrix: Any,
    dist_coeffs: Any,
    config: VisualServoTouchConfig,
) -> DepthTouchEstimate:
    blockers: list[str] = []
    image_center = (float(image_rgb.shape[1]) / 2.0, float(image_rgb.shape[0]) / 2.0)
    aruco_config = PiperXArucoPoseConfig(
        dictionary=config.marker_dictionary,
        marker_id=config.marker_id,
        marker_size_m=config.marker_size_m,
        camera_frame=config.camera_frame,
    )
    result = detect_piper_x_aruco_pose(image_rgb, camera_matrix, dist_coeffs, aruco_config)
    if not result.visible or result.center_uv is None:
        return DepthTouchEstimate(
            False,
            result.reason or "marker not visible",
            result.center_uv,
            image_center,
            None,
            None,
            None,
            None,
            list(config.gripper_tip_offset_xyz_m),
            None,
            None,
            False,
            False,
            ["marker not visible"],
        )
    if config.marker_dictionary != EXPECTED_MARKER_DICTIONARY:
        blockers.append(f"marker dictionary must be {EXPECTED_MARKER_DICTIONARY}")
    if config.marker_id != EXPECTED_MARKER_ID:
        blockers.append("marker ID must be 6")
    if abs(config.marker_size_m - EXPECTED_MARKER_SIZE_M) > 1e-9:
        blockers.append("marker size must be 0.100 m")
    depth_m = depth_roi_m(depth_image, u=result.center_uv[0], v=result.center_uv[1], encoding=depth_encoding, roi_px=config.depth_roi_px)
    if depth_m is None:
        blockers.append("no valid aligned depth at marker center")
    elif depth_m < config.min_depth_m or depth_m > config.max_depth_m:
        blockers.append(f"marker depth {depth_m:.3f} m outside [{config.min_depth_m:.3f}, {config.max_depth_m:.3f}]")
    pixel_error = (float(result.center_uv[0] - image_center[0]), float(result.center_uv[1] - image_center[1]))
    image_aligned = abs(pixel_error[0]) <= config.image_center_tolerance_px and abs(pixel_error[1]) <= config.image_center_tolerance_px
    if not image_aligned:
        blockers.append("marker is not centered in wrist image")
    if len(config.gripper_tip_offset_xyz_m) != 3 or not all(math.isfinite(v) for v in config.gripper_tip_offset_xyz_m):
        blockers.append("invalid gripper tip offset")
    if config.gripper_tip_offset_source.startswith("UNMEASURED"):
        blockers.append("gripper tip offset is not measured")
    if config.require_verified_handeye_for_execution and not config.handeye_verified:
        blockers.append("eye-in-hand calibration is not verified")

    point_camera = None
    point_gripper = None
    required_delta = None
    limited_step = None
    if depth_m is not None:
        point_camera = deproject_pixel(camera_matrix, u=result.center_uv[0], v=result.center_uv[1], depth_m=depth_m)
        point_gripper = transform_camera_point_to_gripper(
            point_camera,
            translation_gripper_camera_m=config.handeye_translation_xyz_m,
            quaternion_gripper_camera_xyzw=config.handeye_quaternion_xyzw,
        )
        raw_delta = np.asarray(point_gripper, dtype=np.float64) - np.asarray(config.gripper_tip_offset_xyz_m, dtype=np.float64)
        norm = float(np.linalg.norm(raw_delta))
        if norm > 1e-9 and config.contact_clearance_m > 0.0:
            raw_delta = raw_delta - (raw_delta / norm) * float(config.contact_clearance_m)
        required_delta = raw_delta.astype(float).tolist()
        limited_step = clamp_step(required_delta, max_lateral_step_m=config.max_lateral_step_m, max_forward_step_m=config.max_forward_step_m)

    if not config.physical_execution_enabled_by_default:
        blockers.append("physical execution disabled in config")
    return DepthTouchEstimate(
        True,
        None,
        result.center_uv,
        image_center,
        pixel_error,
        depth_m,
        point_camera,
        point_gripper,
        list(config.gripper_tip_offset_xyz_m),
        required_delta,
        limited_step,
        image_aligned,
        not blockers,
        blockers,
    )


def camera_geometry_from_ros_info(msg: Any, *, config: VisualServoTouchConfig) -> tuple[np.ndarray, np.ndarray]:
    return camera_geometry_from_info(
        k=msg.K,
        d=msg.D,
        p=msg.P,
        mode=config.image_geometry_mode,
        image_topic=config.color_image_topic,
    )
