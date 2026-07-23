from __future__ import annotations

import math

from piper_on_bunker.models import Pose, StatusCode

SAFE_NAMED_POSES = {
    "tabletop_home",
    "simulated_front_nav_view",
    "simulated_rear_nav_view",
    "scan_left",
    "scan_center",
    "scan_right",
    "inspect_workspace",
    "pre_contact",
    "retracted",
    "stowed",
    "safe_recovery",
}


def validate_named_pose(name: str) -> None:
    if name not in SAFE_NAMED_POSES:
        raise ValueError(f"Unknown named pose: {name}")


def validate_calibrated_named_pose(name: str, named_poses: dict) -> None:
    validate_named_pose(name)
    value = named_poses.get(name)
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(f"Named pose is not calibrated with six joints: {name}")
    if not all(isinstance(joint, (int, float)) and math.isfinite(float(joint)) for joint in value):
        raise ValueError(f"Named pose contains invalid joint values: {name}")


def validate_quaternion(pose: Pose) -> Pose:
    values = (pose.qx, pose.qy, pose.qz, pose.qw)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("quaternion contains non-finite values")
    norm = math.sqrt(sum(float(value) * float(value) for value in values))
    if norm < 1e-6:
        raise ValueError("quaternion norm is zero")
    return Pose(
        x=pose.x,
        y=pose.y,
        z=pose.z,
        qx=pose.qx / norm,
        qy=pose.qy / norm,
        qz=pose.qz / norm,
        qw=pose.qw / norm,
        frame_id=pose.frame_id,
    )


def validate_pose_finite(pose: Pose) -> None:
    values = (pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("pose contains non-finite values")


def validate_workspace(pose: Pose, safety: dict) -> None:
    validate_pose_finite(pose)
    bounds = safety.get("workspace_bounds_m", {})
    for axis, value in (("x", pose.x), ("y", pose.y), ("z", pose.z)):
        interval = bounds.get(axis)
        if not interval or len(interval) != 2:
            raise ValueError(f"workspace bound missing for axis {axis}")
        low, high = float(interval[0]), float(interval[1])
        if float(value) < low or float(value) > high:
            raise ValueError(f"target {axis}={value} outside workspace [{low}, {high}]")
    validate_quaternion(pose)


def validate_press_distance(depth_m: float, safety: dict) -> None:
    max_depth = float(safety.get("max_press_distance_m", 0.015))
    if not math.isfinite(float(depth_m)) or depth_m <= 0 or depth_m > max_depth:
        raise ValueError(f"press distance {depth_m} exceeds maximum {max_depth}")
