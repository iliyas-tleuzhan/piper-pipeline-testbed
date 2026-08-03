from __future__ import annotations

import math
from dataclasses import dataclass


PIPER_X_TRAJECTORY_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
RAW_UNITS_PER_DEGREE = 1000.0


@dataclass(frozen=True)
class PiperXTrajectoryPoint:
    positions_rad: list[float]
    time_from_start_s: float


@dataclass(frozen=True)
class PiperXStreamCommand:
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


def maximum_endpoint_error(actual_rad: list[float] | tuple[float, ...], target_rad: list[float] | tuple[float, ...]) -> float:
    if len(actual_rad) != 6 or len(target_rad) != 6:
        raise ValueError("endpoint comparison requires six actual and target positions")
    actual = [float(v) for v in actual_rad]
    target = [float(v) for v in target_rad]
    if not all(math.isfinite(v) for v in actual + target):
        raise ValueError("endpoint comparison contains non-finite values")
    return max(abs(a - b) for a, b in zip(actual, target))


def endpoint_within_tolerance(actual_rad: list[float] | tuple[float, ...], target_rad: list[float] | tuple[float, ...], tolerance_rad: float) -> bool:
    if float(tolerance_rad) < 0.0 or not math.isfinite(float(tolerance_rad)):
        raise ValueError("endpoint tolerance must be finite and non-negative")
    return maximum_endpoint_error(actual_rad, target_rad) <= float(tolerance_rad)


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
        if index > 0 and float(point.time_from_start_s) <= last_time:
            raise ValueError("trajectory timestamps must be strictly increasing")
        last_time = float(point.time_from_start_s)


def _interp(a: list[float], b: list[float], ratio: float) -> list[float]:
    ratio = min(1.0, max(0.0, float(ratio)))
    return [float(x) + (float(y) - float(x)) * ratio for x, y in zip(a, b)]


def interpolate_trajectory(points: list[PiperXTrajectoryPoint], t_s: float) -> list[float]:
    validate_trajectory_points(points)
    t = float(t_s)
    if t <= points[0].time_from_start_s:
        return list(points[0].positions_rad)
    if t >= points[-1].time_from_start_s:
        return list(points[-1].positions_rad)
    for before, after in zip(points, points[1:]):
        if before.time_from_start_s <= t <= after.time_from_start_s:
            dt = float(after.time_from_start_s) - float(before.time_from_start_s)
            if dt <= 0.0:
                return list(after.positions_rad)
            return _interp(before.positions_rad, after.positions_rad, (t - before.time_from_start_s) / dt)
    return list(points[-1].positions_rad)


def resample_trajectory(
    points: list[PiperXTrajectoryPoint],
    *,
    command_rate_hz: float,
    current_positions_rad: list[float] | tuple[float, ...] | None = None,
    first_point_blend_s: float = 0.25,
    start_tolerance_rad: float = 1e-4,
) -> list[PiperXStreamCommand]:
    validate_trajectory_points(points)
    if command_rate_hz <= 0.0 or not math.isfinite(float(command_rate_hz)):
        raise ValueError("command rate must be finite and positive")
    working = list(points)
    if current_positions_rad is not None:
        current = [float(v) for v in current_positions_rad]
        if len(current) != 6 or not all(math.isfinite(v) for v in current):
            raise ValueError("current feedback must contain six finite positions")
        first = working[0]
        max_delta = max(abs(a - b) for a, b in zip(current, first.positions_rad))
        if max_delta > start_tolerance_rad:
            total = max(0.0, float(working[-1].time_from_start_s))
            blend = min(max(0.0, float(first_point_blend_s)), max(total, 0.0))
            if len(working) > 1:
                next_time = float(working[1].time_from_start_s)
                if next_time > 0.0:
                    blend = min(blend, next_time * 0.5)
            if blend <= 0.0:
                raise ValueError("cannot blend from current feedback to a one-point zero-duration trajectory")
            shifted: list[PiperXTrajectoryPoint] = [PiperXTrajectoryPoint(current, 0.0)]
            shifted.append(PiperXTrajectoryPoint(list(first.positions_rad), blend))
            for point in working[1:]:
                shifted.append(point)
            working = shifted
    total_duration = max(0.0, float(working[-1].time_from_start_s))
    step = 1.0 / float(command_rate_hz)
    if total_duration == 0.0:
        return [PiperXStreamCommand(list(working[-1].positions_rad), 0.0)]
    count = int(math.floor(total_duration / step))
    commands = [
        PiperXStreamCommand(interpolate_trajectory(working, index * step), index * step)
        for index in range(count + 1)
    ]
    if not math.isclose(commands[-1].time_from_start_s, total_duration, abs_tol=1e-9):
        commands.append(PiperXStreamCommand(list(working[-1].positions_rad), total_duration))
    else:
        commands[-1] = PiperXStreamCommand(list(working[-1].positions_rad), total_duration)
    return commands
