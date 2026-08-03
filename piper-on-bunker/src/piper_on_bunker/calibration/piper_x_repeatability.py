from __future__ import annotations

import json
import math
import os
import socket
import subprocess
import time
import uuid
from contextlib import contextmanager
from dataclasses import asdict
from pathlib import Path
from typing import Any, Protocol

import yaml

from piper_on_bunker.calibration.piper_x_repeatability_artifacts import Phase0AArtifactWriter
from piper_on_bunker.calibration.piper_x_repeatability_contract import (
    CLASS_CONTROLLER_OR_SETTLING_INCONSISTENT,
    CLASS_FEEDBACK_UNSTABLE,
    CLASS_INSUFFICIENT_DATA,
    CLASS_JOINT_AND_PHYSICAL_REPEATABILITY_ACCEPTABLE,
    CLASS_JOINT_REPEATABLE_PHYSICAL_REPEATABILITY_UNKNOWN,
    CLASS_MECHANICAL_REPEATABILITY_SUSPECT,
    CLASS_TEST_ABORTED,
    PHASE_0A_CONFIRMATION_TOKEN,
    SCHEMA_VERSION,
    Phase0AConfig,
)
from piper_on_bunker.calibration.piper_x_repeatability_metrics import (
    endpoint_settling_metrics,
    joint_repeatability_metrics,
    joint_sample_summary,
    physical_repeatability_metrics,
)
from piper_on_bunker.manipulation.moveit_aruco_touch import (
    EXPECTED_JOINT_NAMES,
    JointStateSnapshot,
    MockMoveItTouchBackend,
    PlanSummary,
    RosMoveItJointSequenceBackend,
    TaughtPose,
    load_taught_poses,
    load_touch_config,
    validate_joint_schema,
    validate_joint_values,
)
from piper_on_bunker.models import utc_now


class JointStateReader(Protocol):
    def read_joint_state(self) -> JointStateSnapshot:
        ...


def load_phase0a_config(path: str | Path) -> Phase0AConfig:
    with Path(path).open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    return Phase0AConfig.from_mapping(data)


def repository_info(repo_root: str | Path = ".") -> dict[str, str]:
    root = Path(repo_root)

    def run_git(args: list[str]) -> str:
        try:
            return subprocess.check_output(["git", *args], cwd=root, text=True, stderr=subprocess.DEVNULL).strip()
        except Exception:
            return "unknown"

    return {
        "repository_commit": run_git(["rev-parse", "HEAD"]),
        "branch": run_git(["branch", "--show-current"]),
    }


def diagnostic_id(prefix: str) -> str:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    return f"{prefix}_{stamp}_{uuid.uuid4().hex[:8]}"


def base_manifest(config: Phase0AConfig, diag_id: str, *, mode: str, repo_root: str | Path = ".") -> dict[str, Any]:
    repo = repository_info(repo_root)
    return {
        "schema_version": SCHEMA_VERSION,
        "diagnostic_id": diag_id,
        "created_at": utc_now(),
        "completed_at": None,
        "operator": os.environ.get("USER", "unknown"),
        "host": socket.gethostname(),
        "repository_commit": repo["repository_commit"],
        "branch": repo["branch"],
        "arm": {
            "arm_id": config.arm_id,
            "arm_serial": config.arm_serial,
            "firmware": config.firmware,
            "can_interface": config.can_interface,
            "physical_mounting_id": config.physical_mounting_id,
            "side": config.side,
        },
        "camera": {
            "connected": "unknown",
            "serial": config.camera_serial,
            "camera_mount_id": config.camera_mount_id,
        },
        "tool": {
            "tool_id": config.tool_id,
            "tool_description": config.tool_description,
            "tcp_verified": False,
        },
        "robot_model": {
            "urdf_path": config.urdf_path,
            "urdf_sha256": config.urdf_sha256,
            "planning_group": config.planning_group,
            "planning_frame": config.planning_frame,
            "end_effector_link": config.end_effector_link,
        },
        "state_source": {
            "topic": config.joint_state_topic,
            "source_type": config.expected_feedback_source_id,
            "source_can_ids": ["0x2A5", "0x2A6", "0x2A7"],
            "mapping_version": config.expected_joint_mapping_version,
        },
        "execution": {
            "backend": "pyagxarm_piper_x_follow_joint_trajectory",
            "command_primitive": "AgxArm.move_js",
            "command_rate_hz": config.command_rate_hz,
            "speed_percent": config.speed_percent,
            "endpoint_tolerance_rad": config.endpoint_tolerance_rad,
            "settle_timeout_s": config.settle_timeout_s,
        },
        "test": {
            "mode": mode,
            "requested_cycles": 0,
            "completed_cycles": 0,
            "pose_names": [],
            "execution_enabled": False,
        },
        "results": {
            "joint_repeatability": None,
            "endpoint_settling": None,
            "physical_measurements": None,
            "operator_observations": {"mechanical_checklist_completed": False},
            "blockers": [],
            "classification": CLASS_INSUFFICIENT_DATA,
            "phase_0a_passed": False,
        },
        "not_calibration_approval": {
            "joint_zero_verified": False,
            "fk_validated": False,
            "tcp_measured": False,
            "handeye_verified": False,
            "calibrated_touch_validated": False,
        },
    }


def validate_taught_pose_for_phase0a(config: Phase0AConfig, pose_name: str, poses: dict[str, TaughtPose], joint_limits: dict[str, list[float]]) -> TaughtPose:
    if pose_name not in config.approved_pose_names:
        raise ValueError(f"pose {pose_name} is not allowlisted for Phase 0A")
    if pose_name not in poses:
        raise ValueError(f"approved pose {pose_name} is missing from taught pose manifest")
    pose = poses[pose_name]
    validate_joint_schema(pose.joint_names)
    validate_joint_values(pose.joint_names, pose.positions, joint_limits, tolerance_rad=0.02)
    metadata = pose.metadata or {}
    expected = {
        "feedback_source_id": config.expected_feedback_source_id,
        "joint_mapping_version": config.expected_joint_mapping_version,
        "dependency_commit": config.expected_dependency_commit,
    }
    missing = [field for field in expected if field not in metadata]
    if missing:
        raise ValueError(f"pose {pose_name} requires recapture from passive PiPER-X feedback; missing {missing}")
    mismatches = {field: {"expected": value, "actual": metadata.get(field)} for field, value in expected.items() if str(metadata.get(field)) != str(value)}
    if mismatches:
        raise ValueError(f"pose {pose_name} feedback-source mismatch: {mismatches}")
    return pose


@contextmanager
def phase0a_lock(lock_path: str | Path):
    path = Path(lock_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        fd = os.open(str(path), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    except FileExistsError as exc:
        raise RuntimeError(f"Phase 0A lock already exists: {path}") from exc
    try:
        os.write(fd, str(os.getpid()).encode("ascii"))
        os.close(fd)
        yield
    finally:
        try:
            path.unlink()
        except FileNotFoundError:
            pass


def observation_summary(samples: list[JointStateSnapshot], *, stale_count: int = 0, incomplete_count: int = 0) -> dict[str, Any]:
    positions = [sample.positions for sample in samples]
    stamps = [sample.stamp_s for sample in samples if sample.stamp_s is not None]
    frequency = None
    monotonic = True
    if len(stamps) >= 2:
        duration = max(stamps) - min(stamps)
        frequency = (len(stamps) - 1) / duration if duration > 0.0 else None
        monotonic = all(after >= before for before, after in zip(stamps, stamps[1:]))
    return {
        "sample_count": len(samples),
        "sample_frequency_hz": frequency,
        "monotonic_timestamps": monotonic,
        "stale_samples": stale_count,
        "incomplete_samples": incomplete_count,
        "joint_summary": joint_sample_summary(positions, list(EXPECTED_JOINT_NAMES)),
    }


def classify_phase0a(
    *,
    completed_cycles: int,
    joint_metrics: dict[str, Any] | None,
    settling: dict[str, Any] | None,
    physical_metrics: dict[str, Any] | None,
    thresholds,
    aborted: bool = False,
    feedback_unstable: bool = False,
) -> tuple[str, bool, list[str]]:
    blockers: list[str] = []
    if aborted:
        return CLASS_TEST_ABORTED, False, ["test aborted before completion"]
    if feedback_unstable:
        return CLASS_FEEDBACK_UNSTABLE, False, ["feedback stale, incomplete, or unstable"]
    if completed_cycles < int(thresholds.minimum_completed_cycles):
        blockers.append(f"completed_cycles {completed_cycles} below minimum {thresholds.minimum_completed_cycles}")
        return CLASS_INSUFFICIENT_DATA, False, blockers
    if settling:
        if int(settling.get("timeout_count") or 0) > 0 or int(settling.get("controller_abort_count") or 0) > 0:
            return CLASS_CONTROLLER_OR_SETTLING_INCONSISTENT, False, ["controller abort or endpoint timeout observed"]
        if int(settling.get("stale_feedback_count") or 0) > int(thresholds.maximum_stale_feedback_events):
            return CLASS_FEEDBACK_UNSTABLE, False, ["stale feedback events exceeded threshold"]
    if joint_metrics:
        if float(joint_metrics.get("target_error_max_rad") or 0.0) > float(thresholds.maximum_joint_endpoint_error_rad):
            return CLASS_CONTROLLER_OR_SETTLING_INCONSISTENT, False, ["joint endpoint error exceeded threshold"]
        per_joint = ((joint_metrics.get("final_position_summary") or {}).get("per_joint") or {})
        for name, summary in per_joint.items():
            if float(summary.get("std") or 0.0) > float(thresholds.maximum_joint_repeatability_std_rad):
                blockers.append(f"{name} repeatability std exceeds threshold")
            if float(summary.get("range") or 0.0) > float(thresholds.maximum_joint_repeatability_range_rad):
                blockers.append(f"{name} repeatability range exceeds threshold")
        if blockers:
            return CLASS_CONTROLLER_OR_SETTLING_INCONSISTENT, False, blockers
    if not physical_metrics or not physical_metrics.get("available"):
        return CLASS_JOINT_REPEATABLE_PHYSICAL_REPEATABILITY_UNKNOWN, True, ["physical endpoint measurements not provided"]
    if float(physical_metrics.get("maximum_pairwise_distance_mm") or 0.0) > float(thresholds.maximum_pairwise_physical_spread_mm):
        return CLASS_MECHANICAL_REPEATABILITY_SUSPECT, False, ["physical endpoint pairwise spread exceeded threshold"]
    if float(physical_metrics.get("rms_distance_from_centroid_mm") or 0.0) > float(thresholds.physical_repeatability_target_mm):
        return CLASS_MECHANICAL_REPEATABILITY_SUSPECT, False, ["physical endpoint RMS spread exceeded target"]
    return CLASS_JOINT_AND_PHYSICAL_REPEATABILITY_ACCEPTABLE, True, []


def read_physical_measurements_csv(path: str | Path) -> list[dict[str, Any]]:
    import csv

    csv_path = Path(path)
    if not csv_path.exists():
        return []
    out = []
    with csv_path.open("r", encoding="utf-8", newline="") as fh:
        for row in csv.DictReader(fh):
            parsed = dict(row)
            for field in ("x_mm", "y_mm", "z_mm", "estimated_measurement_uncertainty_mm"):
                parsed[field] = None if parsed.get(field) in (None, "") else float(parsed[field])
            out.append(parsed)
    return out


def plan_repeatability_segments(config: Phase0AConfig, *, pose_name: str, backend, touch_config, poses: dict[str, TaughtPose], cycles: int) -> list[PlanSummary]:
    target = validate_taught_pose_for_phase0a(config, pose_name, poses, touch_config.joint_limits)
    staging = validate_taught_pose_for_phase0a(config, config.staging_pose_name, poses, touch_config.joint_limits)
    start_state = backend.read_joint_state()
    plans: list[PlanSummary] = []
    profile = touch_config.motion_profiles.get("transit", {"velocity_scaling": 0.10, "acceleration_scaling": 0.10})
    current = start_state
    for cycle in range(int(cycles)):
        for name, pose in ((f"cycle_{cycle + 1}_to_{pose_name}", target), (f"cycle_{cycle + 1}_return_staging", staging)):
            plan = backend.plan_joint_pose(name, pose, touch_config, start_state=current, motion_profile=profile)
            if not plan.success:
                raise ValueError(plan.reason or f"planning failed for {name}")
            plans.append(plan)
            metrics = plan.metrics or {}
            current = JointStateSnapshot(
                joint_names=list(metrics.get("joint_names", pose.joint_names)),
                positions=[float(v) for v in metrics.get("final_point_positions", pose.positions)],
                age_s=0.0,
                velocities=[0.0] * 6,
                source_topic=f"planned_endpoint:{name}",
            )
    return plans


def run_observation_mode(config: Phase0AConfig, reader: JointStateReader, *, duration_s: float | None = None, artifact_dir: str | Path | None = None) -> dict[str, Any]:
    diag_id = diagnostic_id(config.diagnostic_id_prefix)
    out_dir = Path(artifact_dir or Path(config.output_root) / diag_id)
    writer = Phase0AArtifactWriter(out_dir)
    writer.write_checklist()
    duration = config.observation_duration_s if duration_s is None else float(duration_s)
    deadline = time.monotonic() + duration
    samples: list[JointStateSnapshot] = []
    stale = 0
    incomplete = 0
    period = 1.0 / max(1.0, config.observation_sample_rate_hz)
    while time.monotonic() < deadline:
        try:
            sample = reader.read_joint_state()
            if sample.joint_names != list(EXPECTED_JOINT_NAMES):
                incomplete += 1
            elif sample.age_s > config.max_joint_state_age_s:
                stale += 1
            else:
                samples.append(sample)
        except Exception:
            incomplete += 1
        time.sleep(period)
    summary = observation_summary(samples, stale_count=stale, incomplete_count=incomplete)
    classification, passed, blockers = classify_phase0a(
        completed_cycles=0,
        joint_metrics=None,
        settling=None,
        physical_metrics=None,
        thresholds=config.thresholds,
        feedback_unstable=stale > 0 or incomplete > 0 or not samples,
    )
    manifest = base_manifest(config, diag_id, mode="observe")
    manifest["completed_at"] = utc_now()
    manifest["test"]["completed_cycles"] = 0
    manifest["results"].update(
        {
            "joint_repeatability": summary,
            "blockers": blockers,
            "classification": classification,
            "phase_0a_passed": passed,
        }
    )
    writer.write_manifest(manifest)
    writer.write_joint_summary(summary)
    writer.write_physical_measurement_template(0)
    writer.write_report(
        {
            "diagnostic_id": diag_id,
            "classification": classification,
            "phase_0a_passed": passed,
            "completed_cycles": 0,
            "blockers": blockers,
        }
    )
    return {"artifact_dir": str(out_dir), "manifest": manifest, "summary": summary}


def build_readiness_report(config: Phase0AConfig, *, repo_root: str | Path = ".") -> dict[str, Any]:
    repo = repository_info(repo_root)
    pose_manifest = Path(config.taught_pose_manifest)
    stop_script = Path("tools/stop_piper_x_moveit_motion.sh")
    rejected = str(config.rejected_handeye_path)
    poses: dict[str, Any] = {}
    try:
        loaded = load_taught_poses(pose_manifest)
        for name in config.approved_pose_names:
            poses[name] = "exists" if name in loaded else "missing"
    except Exception as exc:
        poses["error"] = repr(exc)
    output_root = Path(config.output_root)
    writable_candidate = output_root
    while not writable_candidate.exists() and writable_candidate.parent != writable_candidate:
        writable_candidate = writable_candidate.parent
    checks = {
        "branch": repo["branch"],
        "repository_commit": repo["repository_commit"],
        "can_interface": config.can_interface,
        "joint_state_topic": config.joint_state_topic,
        "state_source_contract": config.expected_feedback_source_id,
        "joint_mapping_version": config.expected_joint_mapping_version,
        "pyagxarm_dependency_commit": "cc498c00af0bcb9e297943e94f4792c0e3ee5b2c",
        "moveit_planning_group": config.planning_group,
        "approved_poses": poses,
        "taught_pose_manifest": str(pose_manifest),
        "stop_script": str(stop_script),
        "stop_script_executable": stop_script.exists() and os.access(stop_script, os.X_OK),
        "output_root": str(output_root),
        "output_root_writable_parent": str(writable_candidate),
        "output_root_writable": writable_candidate.exists() and os.access(writable_candidate, os.W_OK),
        "lock_available": not Path(config.lock_path).exists(),
        "rejected_handeye_selected": "rejected_piper_x_d435i_handeye_20260730" in rejected and False,
        "physical_execution_enabled_by_default": config.physical_execution_enabled_by_default,
        "identity": {
            "arm_id": config.arm_id,
            "camera_serial": config.camera_serial,
            "camera_mount_id": config.camera_mount_id,
            "tool_id": config.tool_id,
            "physical_mounting_id": config.physical_mounting_id,
        },
    }
    blockers = []
    if not stop_script.exists() or not os.access(stop_script, os.X_OK):
        blockers.append("stop script missing or not executable")
    if not checks["lock_available"]:
        blockers.append("another Phase 0A diagnostic lock is active")
    if any(value == "missing" for value in poses.values()):
        blockers.append("one or more approved poses are missing")
    return {
        **checks,
        "blockers": blockers,
        "observe_ready": not blockers,
        "planning_ready": not blockers,
        "physical_execution_blocked_by_default": not config.physical_execution_enabled_by_default,
    }


def result_json(payload: dict[str, Any]) -> str:
    return json.dumps(payload, indent=2, sort_keys=True, default=str)
