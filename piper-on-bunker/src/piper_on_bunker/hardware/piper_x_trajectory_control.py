from __future__ import annotations

import math
from dataclasses import dataclass


PIPER_X_TRAJECTORY_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
RAW_UNITS_PER_DEGREE = 1000.0


@dataclass(frozen=True)
class PiperXTrajectoryPoint:
    positions_rad: list[float]
    time_from_start_s: float


def rad_to_raw_mdeg(value_rad: float) -> int:
    if not math.isfinite(float(value_rad)):
        raise ValueError("non-finite joint target")
    return int(round(math.degrees(float(value_rad)) * RAW_UNITS_PER_DEGREE))


def raw_mdeg_to_rad(value_raw: int | float) -> float:
    return math.radians(float(value_raw) / RAW_UNITS_PER_DEGREE)


def joints_rad_to_raw_mdeg(values_rad: list[float] | tuple[float, ...]) -> list[int]:
    if len(values_rad) != 6:
        raise ValueError(f"expected six PiPER-X joints, got {len(values_rad)}")
    return [rad_to_raw_mdeg(value) for value in values_rad]


def validate_trajectory_joint_names(joint_names: list[str] | tuple[str, ...]) -> None:
    if tuple(joint_names) != PIPER_X_TRAJECTORY_JOINTS:
        raise ValueError(f"expected joints {PIPER_X_TRAJECTORY_JOINTS}, got {tuple(joint_names)}")


def validate_trajectory_points(points: list[PiperXTrajectoryPoint]) -> None:
    if not points:
        raise ValueError("trajectory contains no points")
    last_time = -1.0
    for index, point in enumerate(points):
        if len(point.positions_rad) != 6:
            raise ValueError(f"trajectory point {index} must contain six positions")
        if not all(math.isfinite(float(value)) for value in point.positions_rad):
            raise ValueError(f"trajectory point {index} contains non-finite positions")
        if float(point.time_from_start_s) < last_time:
            raise ValueError("trajectory timestamps must be monotonic")
        last_time = float(point.time_from_start_s)
