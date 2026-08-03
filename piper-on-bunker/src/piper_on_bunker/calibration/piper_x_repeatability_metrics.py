from __future__ import annotations

import math
from itertools import combinations
from statistics import mean, pstdev
from typing import Any


def _finite(values: list[float] | tuple[float, ...]) -> list[float]:
    out = [float(v) for v in values]
    if not all(math.isfinite(v) for v in out):
        raise ValueError("metric input contains non-finite value")
    return out


def scalar_summary(values: list[float] | tuple[float, ...]) -> dict[str, float | int | None]:
    data = _finite(values)
    if not data:
        return {"count": 0, "min": None, "max": None, "mean": None, "std": None, "range": None}
    lo = min(data)
    hi = max(data)
    return {
        "count": len(data),
        "min": lo,
        "max": hi,
        "mean": mean(data),
        "std": pstdev(data) if len(data) > 1 else 0.0,
        "range": hi - lo,
    }


def joint_sample_summary(samples: list[list[float]], joint_names: list[str]) -> dict[str, Any]:
    if not samples:
        return {"sample_count": 0, "per_joint": {}}
    for index, sample in enumerate(samples):
        if len(sample) != len(joint_names):
            raise ValueError(f"joint sample {index} has wrong length")
        _finite(sample)
    per_joint = {}
    for joint_index, name in enumerate(joint_names):
        values = [sample[joint_index] for sample in samples]
        summary = scalar_summary(values)
        center = float(summary["mean"] or 0.0)
        summary["max_abs_deviation_from_mean"] = max((abs(v - center) for v in values), default=0.0)
        per_joint[name] = summary
    return {"sample_count": len(samples), "per_joint": per_joint}


def rms(values: list[float] | tuple[float, ...]) -> float:
    data = _finite(values)
    if not data:
        return 0.0
    return math.sqrt(sum(v * v for v in data) / len(data))


def joint_repeatability_metrics(
    *,
    joint_names: list[str],
    target_positions: list[float],
    final_positions_by_cycle: list[list[float]],
) -> dict[str, Any]:
    target = _finite(target_positions)
    if len(target) != len(joint_names):
        raise ValueError("target joint vector length mismatch")
    summary = joint_sample_summary(final_positions_by_cycle, joint_names)
    per_joint_errors = {name: [] for name in joint_names}
    max_errors = []
    rms_errors = []
    for sample in final_positions_by_cycle:
        errors = [float(actual) - float(goal) for actual, goal in zip(sample, target)]
        abs_errors = [abs(v) for v in errors]
        max_errors.append(max(abs_errors, default=0.0))
        rms_errors.append(rms(abs_errors))
        for name, error in zip(joint_names, errors):
            per_joint_errors[name].append(error)
    return {
        "cycle_count": len(final_positions_by_cycle),
        "target_positions": target,
        "final_position_summary": summary,
        "target_error_by_joint": {
            name: {
                "mean_signed_error_rad": mean(errors) if errors else None,
                "max_abs_error_rad": max((abs(v) for v in errors), default=0.0),
                "rms_error_rad": rms(errors),
            }
            for name, errors in per_joint_errors.items()
        },
        "target_error_max_rad": max(max_errors, default=0.0),
        "target_error_rms_rad": rms(rms_errors),
    }


def endpoint_settling_metrics(cycles: list[dict[str, Any]]) -> dict[str, Any]:
    settle_times = [float(c["settling_time_s"]) for c in cycles if c.get("settling_time_s") is not None]
    reached = [bool(c.get("endpoint_reached", False)) for c in cycles]
    return {
        "cycle_count": len(cycles),
        "reached_tolerance_count": sum(1 for value in reached if value),
        "fraction_reaching_tolerance": (sum(1 for value in reached if value) / len(reached)) if reached else 0.0,
        "mean_settle_time_s": mean(settle_times) if settle_times else None,
        "maximum_settle_time_s": max(settle_times, default=None),
        "timeout_count": sum(1 for c in cycles if c.get("timeout")),
        "controller_abort_count": sum(1 for c in cycles if c.get("controller_result") == "aborted"),
        "stale_feedback_count": sum(int(c.get("stale_feedback_events", 0)) for c in cycles),
    }


def physical_repeatability_metrics(measurements: list[dict[str, Any]]) -> dict[str, Any]:
    points = []
    uncertainties = []
    for item in measurements:
        if not item:
            continue
        if item.get("x_mm") is None or item.get("y_mm") is None or item.get("z_mm") is None:
            continue
        point = [float(item["x_mm"]), float(item["y_mm"]), float(item["z_mm"])]
        _finite(point)
        points.append(point)
        if item.get("estimated_measurement_uncertainty_mm") is not None:
            uncertainties.append(float(item["estimated_measurement_uncertainty_mm"]))
    if not points:
        return {"available": False, "sample_count": 0, "classification_basis": "physical endpoint repeatability unknown"}
    centroid = [mean([p[axis] for p in points]) for axis in range(3)]
    distances = [
        math.sqrt(sum((p[axis] - centroid[axis]) ** 2 for axis in range(3)))
        for p in points
    ]
    pairwise = [
        math.sqrt(sum((a[axis] - b[axis]) ** 2 for axis in range(3)))
        for a, b in combinations(points, 2)
    ]
    return {
        "available": True,
        "sample_count": len(points),
        "x_mm": scalar_summary([p[0] for p in points]),
        "y_mm": scalar_summary([p[1] for p in points]),
        "z_mm": scalar_summary([p[2] for p in points]),
        "centroid_mm": centroid,
        "distance_from_centroid_mm": distances,
        "rms_distance_from_centroid_mm": rms(distances),
        "maximum_distance_from_centroid_mm": max(distances, default=0.0),
        "maximum_pairwise_distance_mm": max(pairwise, default=0.0),
        "measurement_uncertainty_mm": scalar_summary(uncertainties),
    }
