from __future__ import annotations

import csv
import json
import math
import os
import signal
import socket
import subprocess
import time
import uuid
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

import yaml

from piper_on_bunker.calibration.piper_x_repeatability_artifacts import Phase0AArtifactWriter, validate_checklist
from piper_on_bunker.calibration.piper_x_repeatability_contract import (
    CLASS_CONTROLLER_OR_SETTLING_INCONSISTENT,
    CLASS_FEEDBACK_UNSTABLE,
    CLASS_INSUFFICIENT_DATA,
    CLASS_JOINT_AND_PHYSICAL_REPEATABILITY_ACCEPTABLE,
    CLASS_JOINT_REPEATABLE_PHYSICAL_REPEATABILITY_UNKNOWN,
    CLASS_MECHANICAL_REPEATABILITY_SUSPECT,
    CLASS_TEST_ABORTED,
    PHASE_0A_BASE_STOPPED_TOKEN,
    PHASE_0A_CONFIRMATION_TOKEN,
    PHYSICAL_STATUS_ACCEPTABLE,
    PHYSICAL_STATUS_SUSPECT,
    PHYSICAL_STATUS_UNKNOWN,
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
    PlanSummary,
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


@dataclass
class MotionExecutionState:
    physical_backend_connected: bool = False
    execution_attempted: bool = False
    motion_commanded: bool = False
    completed_motion_segments: int = 0
    current_cycle: int = 0
    stop_attempted: bool = False
    stop_succeeded: bool | None = None
    backend_stop_result: str | None = None
    external_stop_result: dict[str, Any] | None = None
    return_to_staging_status: str = "not_attempted"
    cancellation_requested: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "physical_backend_connected": self.physical_backend_connected,
            "execution_attempted": self.execution_attempted,
            "motion_commanded": self.motion_commanded,
            "completed_motion_segments": self.completed_motion_segments,
            "current_cycle": self.current_cycle,
            "stop_attempted": self.stop_attempted,
            "stop_succeeded": self.stop_succeeded,
            "backend_stop_result": self.backend_stop_result,
            "external_stop_result": self.external_stop_result,
            "return_to_staging_status": self.return_to_staging_status,
            "cancellation_requested": self.cancellation_requested,
        }


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

    return {"repository_commit": run_git(["rev-parse", "HEAD"]), "branch": run_git(["branch", "--show-current"])}


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
        "arm": {"arm_id": config.arm_id, "arm_serial": config.arm_serial, "firmware": config.firmware, "can_interface": config.can_interface, "physical_mounting_id": config.physical_mounting_id, "side": config.side},
        "camera": {"connected": "unknown", "serial": config.camera_serial, "camera_mount_id": config.camera_mount_id},
        "tool": {"tool_id": config.tool_id, "tool_description": config.tool_description, "tcp_verified": False},
        "robot_model": {"urdf_path": config.urdf_path, "urdf_sha256": config.urdf_sha256, "planning_group": config.planning_group, "planning_frame": config.planning_frame, "end_effector_link": config.end_effector_link},
        "state_source": {"topic": config.joint_state_topic, "source_type": config.expected_feedback_source_id, "source_can_ids": ["0x2A5", "0x2A6", "0x2A7"], "mapping_version": config.expected_joint_mapping_version},
        "execution": {"backend": "pyagxarm_piper_x_follow_joint_trajectory", "command_primitive": "AgxArm.move_js", "command_rate_hz": config.command_rate_hz, "speed_percent": config.speed_percent, "endpoint_tolerance_rad": config.endpoint_tolerance_rad, "settle_timeout_s": config.settle_timeout_s},
        "test": {"mode": mode, "requested_cycles": 0, "completed_cycles": 0, "measurement_pose_name": config.measurement_pose_name, "departure_pose_name": config.default_departure_pose_name, "pose_names": [], "execution_enabled": False, "base_stopped_acknowledgement": None},
        "results": {"joint_repeatability": None, "endpoint_settling": None, "physical_measurements": None, "operator_observations": {"mechanical_checklist_completed": False}, "blockers": [], "classification": CLASS_INSUFFICIENT_DATA, "phase_0a_passed": False, "phase_0a": phase0a_status(False, False, PHYSICAL_STATUS_UNKNOWN, False, False)},
        "motion_reporting": MotionExecutionState().as_dict(),
        "not_calibration_approval": {"joint_zero_verified": False, "fk_validated": False, "tcp_measured": False, "handeye_verified": False, "calibrated_touch_validated": False},
    }


def phase0a_status(observation: bool, joint: bool, physical_status: str, checklist: bool, ready: bool) -> dict[str, Any]:
    return {
        "observation_passed": bool(observation),
        "joint_repeatability_passed": bool(joint),
        "controller_settling_passed": bool(joint),
        "physical_repeatability": {"status": physical_status},
        "mechanical_checklist_passed": bool(checklist),
        "ready_for_joint_zero_fk_investigation": bool(ready),
    }


def validate_measurement_departure(config: Phase0AConfig, measurement_pose: str, departure_pose: str) -> None:
    if measurement_pose == departure_pose:
        raise ValueError("measurement pose and departure pose must be different; staging-as-both is a no-op")
    if departure_pose not in config.allowed_departure_pose_names:
        raise ValueError(f"departure pose {departure_pose} is not allowlisted for Phase 0A")


def validate_taught_pose_for_phase0a(config: Phase0AConfig, pose_name: str, poses: dict[str, TaughtPose], joint_limits: dict[str, list[float]], *, role: str) -> TaughtPose:
    if role == "measurement" and pose_name != config.measurement_pose_name:
        raise ValueError(f"measurement pose must be {config.measurement_pose_name}, got {pose_name}")
    if role == "departure" and pose_name not in config.allowed_departure_pose_names:
        raise ValueError(f"departure pose {pose_name} is not allowlisted for Phase 0A")
    if pose_name not in poses:
        raise ValueError(f"approved {role} pose {pose_name} is missing from taught pose manifest")
    pose = poses[pose_name]
    validate_joint_schema(pose.joint_names)
    validate_joint_values(pose.joint_names, pose.positions, joint_limits, tolerance_rad=0.02)
    metadata = pose.metadata or {}
    expected = {"feedback_source_id": config.expected_feedback_source_id, "joint_mapping_version": config.expected_joint_mapping_version, "dependency_commit": config.expected_dependency_commit}
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
    return {"sample_count": len(samples), "sample_frequency_hz": frequency, "monotonic_timestamps": monotonic, "stale_samples": stale_count, "incomplete_samples": incomplete_count, "joint_summary": joint_sample_summary(positions, list(EXPECTED_JOINT_NAMES))}


def classify_phase0a(*, completed_cycles: int, joint_metrics: dict[str, Any] | None, settling: dict[str, Any] | None, physical_metrics: dict[str, Any] | None, thresholds, aborted: bool = False, feedback_unstable: bool = False, checklist_passed: bool = False) -> tuple[str, bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    physical_status = PHYSICAL_STATUS_UNKNOWN
    joint_passed = False
    if aborted:
        return CLASS_TEST_ABORTED, False, ["test aborted before completion"], phase0a_status(False, False, physical_status, checklist_passed, False)
    if feedback_unstable:
        return CLASS_FEEDBACK_UNSTABLE, False, ["feedback stale, incomplete, or unstable"], phase0a_status(False, False, physical_status, checklist_passed, False)
    if completed_cycles < int(thresholds.minimum_completed_cycles):
        blockers.append(f"completed_cycles {completed_cycles} below minimum {thresholds.minimum_completed_cycles}")
        return CLASS_INSUFFICIENT_DATA, False, blockers, phase0a_status(False, False, physical_status, checklist_passed, False)
    if settling:
        if int(settling.get("timeout_count") or 0) > 0 or int(settling.get("controller_abort_count") or 0) > 0:
            return CLASS_CONTROLLER_OR_SETTLING_INCONSISTENT, False, ["controller abort or endpoint timeout observed"], phase0a_status(False, False, physical_status, checklist_passed, False)
        if int(settling.get("stale_feedback_count") or 0) > int(thresholds.maximum_stale_feedback_events):
            return CLASS_FEEDBACK_UNSTABLE, False, ["stale feedback events exceeded threshold"], phase0a_status(False, False, physical_status, checklist_passed, False)
    if joint_metrics:
        if float(joint_metrics.get("target_error_max_rad") or 0.0) > float(thresholds.maximum_joint_endpoint_error_rad):
            return CLASS_CONTROLLER_OR_SETTLING_INCONSISTENT, False, ["joint endpoint error exceeded threshold"], phase0a_status(False, False, physical_status, checklist_passed, False)
        per_joint = ((joint_metrics.get("final_position_summary") or {}).get("per_joint") or {})
        for name, summary in per_joint.items():
            if float(summary.get("std") or 0.0) > float(thresholds.maximum_joint_repeatability_std_rad):
                blockers.append(f"{name} repeatability std exceeds threshold")
            if float(summary.get("range") or 0.0) > float(thresholds.maximum_joint_repeatability_range_rad):
                blockers.append(f"{name} repeatability range exceeds threshold")
        if blockers:
            return CLASS_CONTROLLER_OR_SETTLING_INCONSISTENT, False, blockers, phase0a_status(False, False, physical_status, checklist_passed, False)
        joint_passed = True
    if not physical_metrics or not physical_metrics.get("available"):
        blockers = ["physical endpoint measurements not provided"]
        return CLASS_JOINT_REPEATABLE_PHYSICAL_REPEATABILITY_UNKNOWN, False, blockers, phase0a_status(True, joint_passed, PHYSICAL_STATUS_UNKNOWN, checklist_passed, False)
    if physical_metrics.get("uncertainty_too_large_for_strong_acceptance"):
        return CLASS_MECHANICAL_REPEATABILITY_SUSPECT, False, ["measurement uncertainty too large for strong acceptance"], phase0a_status(True, joint_passed, PHYSICAL_STATUS_SUSPECT, checklist_passed, False)
    if float(physical_metrics.get("maximum_pairwise_distance_mm") or 0.0) > float(thresholds.maximum_pairwise_physical_spread_mm):
        return CLASS_MECHANICAL_REPEATABILITY_SUSPECT, False, ["physical endpoint pairwise spread exceeded threshold"], phase0a_status(True, joint_passed, PHYSICAL_STATUS_SUSPECT, checklist_passed, False)
    if float(physical_metrics.get("rms_distance_from_centroid_mm") or 0.0) > float(thresholds.physical_repeatability_target_mm):
        return CLASS_MECHANICAL_REPEATABILITY_SUSPECT, False, ["physical endpoint RMS spread exceeded target"], phase0a_status(True, joint_passed, PHYSICAL_STATUS_SUSPECT, checklist_passed, False)
    return CLASS_JOINT_AND_PHYSICAL_REPEATABILITY_ACCEPTABLE, True, [], phase0a_status(True, joint_passed, PHYSICAL_STATUS_ACCEPTABLE, checklist_passed, True)


def read_physical_measurements_csv(path: str | Path, *, expected_cycles: set[int] | None = None) -> list[dict[str, Any]]:
    csv_path = Path(path)
    if not csv_path.exists():
        return []
    out = []
    seen: set[int] = set()
    with csv_path.open("r", encoding="utf-8", newline="") as fh:
        for row in csv.DictReader(fh):
            if not any((value or "").strip() for value in row.values()):
                continue
            parsed = dict(row)
            cycle_raw = parsed.get("cycle_index")
            if cycle_raw in (None, ""):
                raise ValueError("physical measurement row missing cycle_index")
            cycle_index = int(cycle_raw)
            if cycle_index in seen:
                raise ValueError(f"duplicate physical measurement for cycle {cycle_index}")
            if expected_cycles is not None and cycle_index not in expected_cycles:
                raise ValueError(f"physical measurement cycle {cycle_index} does not exist in cycles.jsonl")
            seen.add(cycle_index)
            parsed["cycle_index"] = cycle_index
            coords_present = any(parsed.get(field) not in (None, "") for field in ("x_mm", "y_mm", "z_mm"))
            for field in ("x_mm", "y_mm", "z_mm", "estimated_measurement_uncertainty_mm"):
                parsed[field] = None if parsed.get(field) in (None, "") else float(parsed[field])
            if coords_present:
                if parsed["x_mm"] is None or parsed["y_mm"] is None or parsed["z_mm"] is None:
                    raise ValueError(f"cycle {cycle_index} has partial XYZ measurement")
                if parsed["estimated_measurement_uncertainty_mm"] is None:
                    raise ValueError(f"cycle {cycle_index} missing measurement uncertainty")
                if not all(math.isfinite(float(parsed[field])) for field in ("x_mm", "y_mm", "z_mm", "estimated_measurement_uncertainty_mm")):
                    raise ValueError(f"cycle {cycle_index} measurement contains non-finite value")
                if float(parsed["estimated_measurement_uncertainty_mm"]) <= 0.0:
                    raise ValueError(f"cycle {cycle_index} measurement uncertainty must be positive")
            out.append(parsed)
    return out


def invoke_stop(config: Phase0AConfig, backend: Any | None, motion_state: MotionExecutionState) -> None:
    motion_state.stop_attempted = True
    try:
        if backend is not None:
            backend.stop()
            motion_state.backend_stop_result = "succeeded"
    except Exception as exc:
        motion_state.backend_stop_result = f"failed: {exc!r}"
    try:
        result = subprocess.run(list(config.stop_command), cwd=Path.cwd(), shell=False, text=True, capture_output=True, timeout=10.0)
        motion_state.external_stop_result = {"command": list(config.stop_command), "returncode": result.returncode, "stdout": result.stdout[-1000:], "stderr": result.stderr[-1000:]}
        motion_state.stop_succeeded = result.returncode == 0
    except Exception as exc:
        motion_state.external_stop_result = {"command": list(config.stop_command), "error": repr(exc)}
        motion_state.stop_succeeded = False


def plan_repeatability_segments(config: Phase0AConfig, *, measurement_pose_name: str, departure_pose_name: str, backend, touch_config, poses: dict[str, TaughtPose], cycles: int) -> list[PlanSummary]:
    validate_measurement_departure(config, measurement_pose_name, departure_pose_name)
    measurement = validate_taught_pose_for_phase0a(config, measurement_pose_name, poses, touch_config.joint_limits, role="measurement")
    departure = validate_taught_pose_for_phase0a(config, departure_pose_name, poses, touch_config.joint_limits, role="departure")
    start_state = backend.read_joint_state()
    plans: list[PlanSummary] = []
    profile = touch_config.motion_profiles.get("transit", {"velocity_scaling": 0.10, "acceleration_scaling": 0.10})
    current = start_state
    sequence: list[tuple[str, TaughtPose]] = [("initial_to_measurement", measurement)]
    for cycle in range(int(cycles)):
        sequence.append((f"cycle_{cycle + 1}_to_departure", departure))
        sequence.append((f"cycle_{cycle + 1}_return_measurement", measurement))
    for name, pose in sequence:
        plan = backend.plan_joint_pose(name, pose, touch_config, start_state=current, motion_profile=profile)
        if not plan.success:
            raise ValueError(plan.reason or f"planning failed for {name}")
        plans.append(plan)
        metrics = plan.metrics or {}
        current = JointStateSnapshot(joint_names=list(metrics.get("joint_names", pose.joint_names)), positions=[float(v) for v in metrics.get("final_point_positions", pose.positions)], age_s=0.0, velocities=[0.0] * 6, source_topic=f"planned_endpoint:{name}")
    return plans


def _probe_live_joint_state(config: Phase0AConfig) -> tuple[dict[str, Any], list[str]]:
    status: dict[str, Any] = {
        "topic": config.joint_state_topic,
        "available": False,
        "fresh": False,
        "six_joint_contract": False,
        "source_contract": config.expected_feedback_source_id,
    }
    blockers: list[str] = []
    try:
        import rospy
        from sensor_msgs.msg import JointState
        from piper_on_bunker.manipulation.moveit_aruco_touch import extract_named_joint_state

        if not rospy.get_node_uri():
            rospy.init_node("piper_x_phase_0a_readiness", anonymous=True, disable_signals=True)
        msg = rospy.wait_for_message(config.joint_state_topic, JointState, timeout=config.max_joint_state_age_s)
        snapshot = extract_named_joint_state(
            names=list(msg.name),
            positions=list(msg.position),
            velocities=list(msg.velocity),
            stamp_s=float(msg.header.stamp.to_sec()),
            now_s=float(rospy.Time.now().to_sec()),
            max_age_s=config.max_joint_state_age_s,
            max_abs_velocity_rad_s=config.max_abs_velocity_rad_s,
            source_topic=config.joint_state_topic,
        )
        status.update(
            {
                "available": True,
                "fresh": True,
                "six_joint_contract": snapshot.joint_names == list(EXPECTED_JOINT_NAMES),
                "joint_names": list(snapshot.joint_names),
                "positions": list(snapshot.positions),
                "age_s": snapshot.age_s,
            }
        )
        if not status["six_joint_contract"]:
            blockers.append("/piper_x/joint_states does not provide joint1 through joint6 in the expected contract")
    except Exception as exc:
        status["error"] = repr(exc)
        blockers.append(f"fresh six-joint {config.joint_state_topic} feedback unavailable: {exc!r}")
    return status, blockers


def run_observation_mode(config: Phase0AConfig, reader: JointStateReader, *, duration_s: float | None = None, artifact_dir: str | Path | None = None) -> dict[str, Any]:
    diag_id = diagnostic_id(config.diagnostic_id_prefix)
    out_dir = Path(artifact_dir or Path(config.output_root) / diag_id)
    writer = Phase0AArtifactWriter(out_dir)
    writer.write_checklist_templates(config.identity_dict())
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
    feedback_unstable = stale > 0 or incomplete > 0 or not samples
    classification = CLASS_FEEDBACK_UNSTABLE if feedback_unstable else CLASS_INSUFFICIENT_DATA
    blockers = ["observation-only mode does not include repeated-return cycles"] if not feedback_unstable else ["feedback stale, incomplete, or unavailable"]
    phase = phase0a_status(not feedback_unstable, False, PHYSICAL_STATUS_UNKNOWN, False, False)
    manifest = base_manifest(config, diag_id, mode="observe")
    manifest["completed_at"] = utc_now()
    manifest["results"].update({"joint_repeatability": summary, "blockers": blockers, "classification": classification, "phase_0a_passed": False, "phase_0a": phase})
    writer.write_manifest(manifest)
    writer.write_joint_summary(summary)
    writer.write_physical_measurement_template(0)
    writer.write_report({"diagnostic_id": diag_id, "classification": classification, "phase_0a": phase, "completed_cycles": 0, "blockers": blockers, "motion_reporting": manifest["motion_reporting"]})
    return {"artifact_dir": str(out_dir), "manifest": manifest, "summary": summary}


def build_readiness_report(config: Phase0AConfig, *, repo_root: str | Path = ".", physical: bool = False, confirm: str = "", confirm_base_stopped: str = "", checklist_path: str = "") -> dict[str, Any]:
    repo = repository_info(repo_root)
    pose_manifest = Path(config.taught_pose_manifest)
    stop_script = Path(config.stop_command[0]) if config.stop_command else Path("")
    poses: dict[str, Any] = {}
    required_names = [config.measurement_pose_name, *config.allowed_departure_pose_names]
    try:
        loaded = load_taught_poses(pose_manifest)
        for name in required_names:
            poses[name] = "exists" if name in loaded else "missing"
    except Exception as exc:
        poses["error"] = repr(exc)
    output_root = Path(config.output_root)
    writable_candidate = output_root
    while not writable_candidate.exists() and writable_candidate.parent != writable_candidate:
        writable_candidate = writable_candidate.parent
    joint_state_status, joint_state_blockers = _probe_live_joint_state(config)
    observe_blockers: list[str] = list(joint_state_blockers)
    planning_blockers: list[str] = []
    physical_blockers: list[str] = []
    if any(value == "missing" for value in poses.values()):
        planning_blockers.append("one or more measurement/departure poses are missing")
    if config.default_departure_pose_name == config.measurement_pose_name:
        planning_blockers.append("measurement and departure poses must differ")
    if config.default_departure_pose_name not in config.allowed_departure_pose_names:
        planning_blockers.append("default departure pose is not allowlisted")
    if not stop_script.exists() or not os.access(stop_script, os.X_OK):
        physical_blockers.append("stop script missing or not executable")
    if Path(config.lock_path).exists():
        physical_blockers.append("another Phase 0A diagnostic lock is active")
    if "rejected_piper_x_d435i_handeye_20260730" in str(config.rejected_handeye_path) and False:
        physical_blockers.append("rejected hand-eye selected")
    identity = config.identity_dict()
    for key, value in identity.items():
        if value == "unknown":
            physical_blockers.append(f"local identity {key} is unknown")
    checklist_ok = False
    checklist_blockers: list[str] = []
    selected_checklist = checklist_path or config.checklist_path
    if physical:
        if confirm != PHASE_0A_CONFIRMATION_TOKEN:
            physical_blockers.append(f"physical execution requires --confirm {PHASE_0A_CONFIRMATION_TOKEN}")
        if config.require_base_stopped_ack and confirm_base_stopped != PHASE_0A_BASE_STOPPED_TOKEN:
            physical_blockers.append(f"base stopped operator acknowledgement requires --confirm-base-stopped {PHASE_0A_BASE_STOPPED_TOKEN}")
        if not config.physical_execution_enabled_by_default:
            physical_blockers.append("local physical execution is not enabled")
        checklist_ok, checklist_blockers, _ = validate_checklist(selected_checklist, identity) if selected_checklist else (False, ["checklist path missing"], {})
        physical_blockers.extend(checklist_blockers)
    physical_blockers.extend(planning_blockers)
    return {
        "branch": repo["branch"],
        "repository_commit": repo["repository_commit"],
        "can_interface": config.can_interface,
        "joint_state_topic": config.joint_state_topic,
        "joint_state_status": joint_state_status,
        "state_source_contract": config.expected_feedback_source_id,
        "joint_mapping_version": config.expected_joint_mapping_version,
        "pyagxarm_dependency_commit": "cc498c00af0bcb9e297943e94f4792c0e3ee5b2c",
        "moveit_planning_group": config.planning_group,
        "measurement_pose_name": config.measurement_pose_name,
        "default_departure_pose_name": config.default_departure_pose_name,
        "allowed_departure_pose_names": list(config.allowed_departure_pose_names),
        "approved_poses": poses,
        "taught_pose_manifest": str(pose_manifest),
        "stop_script": str(stop_script),
        "stop_script_executable": stop_script.exists() and os.access(stop_script, os.X_OK),
        "output_root": str(output_root),
        "output_root_writable_parent": str(writable_candidate),
        "output_root_writable": writable_candidate.exists() and os.access(writable_candidate, os.W_OK),
        "lock_available": not Path(config.lock_path).exists(),
        "physical_execution_enabled_by_default": config.physical_execution_enabled_by_default,
        "identity": identity,
        "checklist_path": selected_checklist,
        "checklist_passed": checklist_ok,
        "base_stopped_acknowledgement": "operator_provided" if confirm_base_stopped == PHASE_0A_BASE_STOPPED_TOKEN else "missing",
        "observe_blockers": observe_blockers,
        "planning_blockers": planning_blockers,
        "physical_blockers": physical_blockers,
        "observe_ready": not observe_blockers,
        "planning_ready": not observe_blockers and not planning_blockers,
        "physical_ready": physical and not observe_blockers and not physical_blockers,
        "physical_execution_blocked_by_default": not config.physical_execution_enabled_by_default,
    }


def finalize_artifact(config: Phase0AConfig, artifact_dir: str | Path, *, no_physical_measurement: bool = False) -> dict[str, Any]:
    writer = Phase0AArtifactWriter(artifact_dir)
    manifest = writer.read_manifest()
    cycles = writer.read_cycles()
    measurement_cycles = [c for c in cycles if c.get("measurement_recorded")]
    final_positions = [c["final_measured_joint_values"] for c in measurement_cycles if c.get("final_measured_joint_values")]
    target = manifest.get("test", {}).get("measurement_pose_target_positions") or (final_positions[0] if final_positions else [0.0] * 6)
    joint_names = manifest.get("test", {}).get("joint_names") or list(EXPECTED_JOINT_NAMES)
    expected_cycle_indices = {int(c["cycle_index"]) for c in measurement_cycles if c.get("cycle_index") is not None}
    measurements = [] if no_physical_measurement else read_physical_measurements_csv(Path(artifact_dir) / "physical_measurements.csv", expected_cycles=expected_cycle_indices)
    joint_metrics = joint_repeatability_metrics(joint_names=list(joint_names), target_positions=[float(v) for v in target], final_positions_by_cycle=final_positions) if final_positions else None
    settling = endpoint_settling_metrics(measurement_cycles)
    physical = physical_repeatability_metrics(measurements, maximum_measurement_uncertainty_mm=config.thresholds.maximum_measurement_uncertainty_mm)
    classification, ready, blockers, phase = classify_phase0a(completed_cycles=len(final_positions), joint_metrics=joint_metrics, settling=settling, physical_metrics=physical, thresholds=config.thresholds, checklist_passed=bool((manifest.get("results") or {}).get("operator_observations", {}).get("mechanical_checklist_completed", False)))
    manifest["completed_at"] = utc_now()
    manifest["results"].update({"joint_repeatability": joint_metrics, "endpoint_settling": settling, "physical_measurements": physical, "blockers": blockers, "classification": classification, "phase_0a_passed": ready, "phase_0a": phase})
    writer.write_manifest(manifest)
    writer.write_joint_summary(joint_metrics or {})
    writer.write_report({"diagnostic_id": manifest.get("diagnostic_id"), "classification": classification, "phase_0a": phase, "completed_cycles": len(final_positions), "blockers": blockers, "motion_reporting": manifest.get("motion_reporting", {})})
    return {"artifact_dir": str(artifact_dir), "manifest": manifest, "physical_measurements_loaded": len(measurements), "motion_commanded": False}


def result_json(payload: dict[str, Any]) -> str:
    return json.dumps(payload, indent=2, sort_keys=True, default=str)
