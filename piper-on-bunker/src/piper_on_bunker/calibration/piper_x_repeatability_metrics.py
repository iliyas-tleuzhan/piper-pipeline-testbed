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
        "incomplete_feedback_count": sum(int(c.get("incomplete_feedback_events", 0)) for c in cycles),
        "joint_state_read_count": sum(int(c.get("joint_state_read_count", 0)) for c in cycles),
        "maximum_observed_feedback_age_s": max((float(c.get("maximum_observed_feedback_age_s") or 0.0) for c in cycles), default=0.0),
        "feedback_timestamps_monotonic": all(bool(c.get("feedback_timestamps_monotonic", True)) for c in cycles),
    }


def physical_repeatability_metrics(measurements: list[dict[str, Any]], *, maximum_measurement_uncertainty_mm: float = 5.0) -> dict[str, Any]:
    points = []
    uncertainties = []
    methods = set()
    for item in measurements:
        if not item:
            continue
        if item.get("x_mm") is None or item.get("y_mm") is None or item.get("z_mm") is None:
            continue
        point = [float(item["x_mm"]), float(item["y_mm"]), float(item["z_mm"])]
        _finite(point)
        uncertainty = item.get("estimated_measurement_uncertainty_mm")
        if uncertainty is None:
            raise ValueError("measurement uncertainty is required when XYZ is provided")
        uncertainty = float(uncertainty)
        if not math.isfinite(uncertainty) or uncertainty <= 0.0:
            raise ValueError("measurement uncertainty must be positive and finite")
        points.append(point)
        uncertainties.append(uncertainty)
        if item.get("method"):
            methods.add(str(item["method"]))
    if not points:
        return {"available": False, "sample_count": 0, "status": "UNKNOWN", "classification_basis": "physical endpoint repeatability unknown"}
    centroid = [mean([p[axis] for p in points]) for axis in range(3)]
    distances = [math.sqrt(sum((p[axis] - centroid[axis]) ** 2 for axis in range(3))) for p in points]
    pairwise = [math.sqrt(sum((a[axis] - b[axis]) ** 2 for axis in range(3))) for a, b in combinations(points, 2)]
    uncertainty_summary = scalar_summary(uncertainties)
    max_uncertainty = max(uncertainties, default=0.0)
    max_pairwise = max(pairwise, default=0.0)
    rms_distance = rms(distances)
    uncertainty_too_large = max_uncertainty > float(maximum_measurement_uncertainty_mm)
    spread_comparable_to_uncertainty = max_pairwise <= max_uncertainty * 2.0
    return {
        "available": True,
        "sample_count": len(points),
        "methods": sorted(methods),
        "x_mm": scalar_summary([p[0] for p in points]),
        "y_mm": scalar_summary([p[1] for p in points]),
        "z_mm": scalar_summary([p[2] for p in points]),
        "centroid_mm": centroid,
        "distance_from_centroid_mm": distances,
        "rms_distance_from_centroid_mm": rms_distance,
        "maximum_distance_from_centroid_mm": max(distances, default=0.0),
        "maximum_pairwise_distance_mm": max_pairwise,
        "measurement_uncertainty_mm": uncertainty_summary,
        "maximum_measurement_uncertainty_mm": max_uncertainty,
        "uncertainty_too_large_for_strong_acceptance": uncertainty_too_large,
        "spread_comparable_to_or_smaller_than_uncertainty": spread_comparable_to_uncertainty,
    }
