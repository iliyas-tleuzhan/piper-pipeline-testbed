from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from piper_on_bunker.calibration.piper_x_repeatability import (
    MotionExecutionState,
    build_readiness_report,
    classify_phase0a,
    finalize_artifact,
    invoke_stop,
    phase0a_lock,
    plan_repeatability_segments,
    read_physical_measurements_csv,
    run_observation_mode,
    validate_measurement_departure,
    validate_taught_pose_for_phase0a,
)
from piper_on_bunker.calibration.piper_x_repeatability_artifacts import Phase0AArtifactWriter, checklist_template, validate_checklist
from piper_on_bunker.calibration.piper_x_repeatability_contract import (
    CLASS_JOINT_AND_PHYSICAL_REPEATABILITY_ACCEPTABLE,
    CLASS_JOINT_REPEATABLE_PHYSICAL_REPEATABILITY_UNKNOWN,
    CLASS_MECHANICAL_REPEATABILITY_SUSPECT,
    PHASE_0A_BASE_STOPPED_TOKEN,
    PHASE_0A_CONFIRMATION_TOKEN,
    PHYSICAL_STATUS_ACCEPTABLE,
    PHYSICAL_STATUS_UNKNOWN,
    Phase0AConfig,
    Phase0AThresholds,
)
from piper_on_bunker.calibration.piper_x_repeatability_metrics import joint_repeatability_metrics, physical_repeatability_metrics, rms, scalar_summary
from piper_on_bunker.manipulation.moveit_aruco_touch import JointStateSnapshot, MockMoveItTouchBackend, TaughtPose


JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
METADATA = {
    "feedback_source_id": "piper_x_passive_socketcan_feedback_v1",
    "joint_mapping_version": "piper_x_lora_feedback_2a5_2a6_2a7_raw001deg_to_rad_v1",
    "dependency_commit": "521c9c5fdfd9ee63bd96c0f9342fca6b2398092e",
}
LIMITS = {name: [-3.0, 3.0] for name in JOINTS}


class StaticReader:
    def __init__(self):
        self.calls = 0
        self.commanded = False

    def read_joint_state(self):
        self.calls += 1
        return JointStateSnapshot(JOINTS, [0.01 * i for i in range(6)], 0.0, [0.0] * 6, float(self.calls), "/piper_x/joint_states")


class PlanningBackend(MockMoveItTouchBackend):
    def __init__(self):
        super().__init__(joint_state=JointStateSnapshot(JOINTS, [0.0] * 6, 0.0))


def poses():
    return {
        "staging": TaughtPose("staging", JOINTS, [0.0] * 6, "test", METADATA),
        "repeatability_departure": TaughtPose("repeatability_departure", JOINTS, [0.1, 0.0, -0.1, 0.0, 0.0, 0.0], "test", METADATA),
    }


def test_scalar_summary_and_rms_formulas():
    summary = scalar_summary([1.0, 2.0, 3.0])
    assert summary["mean"] == pytest.approx(2.0)
    assert summary["range"] == pytest.approx(2.0)
    assert rms([3.0, 4.0]) == pytest.approx((12.5) ** 0.5)


def test_measurement_and_departure_must_differ():
    cfg = Phase0AConfig(measurement_pose_name="staging", allowed_departure_pose_names=("repeatability_departure",))
    validate_measurement_departure(cfg, "staging", "repeatability_departure")
    with pytest.raises(ValueError, match="must be different"):
        validate_measurement_departure(cfg, "staging", "staging")


def test_pose_allowlist_and_feedback_source_validation():
    cfg = Phase0AConfig(measurement_pose_name="staging", allowed_departure_pose_names=("repeatability_departure",))
    assert validate_taught_pose_for_phase0a(cfg, "staging", poses(), LIMITS, role="measurement").name == "staging"
    with pytest.raises(ValueError, match="not allowlisted"):
        validate_taught_pose_for_phase0a(cfg, "other", poses(), LIMITS, role="departure")
    bad = poses()
    bad["staging"] = TaughtPose("staging", JOINTS, [0.0] * 6, "test", {})
    with pytest.raises(ValueError, match="requires recapture"):
        validate_taught_pose_for_phase0a(cfg, "staging", bad, LIMITS, role="measurement")


def test_one_complete_cycle_plans_departure_and_return_to_measurement():
    cfg = Phase0AConfig(measurement_pose_name="staging", allowed_departure_pose_names=("repeatability_departure",), default_departure_pose_name="repeatability_departure")
    backend = PlanningBackend()
    touch_config = type("Cfg", (), {"joint_limits": LIMITS, "motion_profiles": {"transit": {"velocity_scaling": 0.1, "acceleration_scaling": 0.1}}})()
    plans = plan_repeatability_segments(cfg, measurement_pose_name="staging", departure_pose_name="repeatability_departure", backend=backend, touch_config=touch_config, poses=poses(), cycles=1)
    assert [p.name for p in plans] == ["initial_to_measurement", "cycle_1_to_departure", "cycle_1_return_measurement"]
    assert plans[-1].metrics["target_positions"] == [0.0] * 6


def test_observation_only_never_commands_motion(tmp_path):
    reader = StaticReader()
    cfg = Phase0AConfig(output_root=str(tmp_path), observation_sample_rate_hz=200.0)
    result = run_observation_mode(cfg, reader, duration_s=0.02)
    assert reader.calls > 0
    assert not reader.commanded
    assert result["manifest"]["test"]["mode"] == "observe"


def test_checklist_blocks_physical_when_missing_or_incomplete(tmp_path):
    cfg = Phase0AConfig(arm_id="left", physical_mounting_id="mount", camera_serial="cam", camera_mount_id="cam_mount", tool_id="tool")
    ok, blockers, _ = validate_checklist(tmp_path / "missing.yaml", cfg.identity_dict())
    assert not ok
    assert blockers
    checklist = checklist_template(cfg.identity_dict())
    path = tmp_path / "operator_checklist.yaml"
    path.write_text(yaml.safe_dump(checklist), encoding="utf-8")
    ok, blockers, _ = validate_checklist(path, cfg.identity_dict())
    assert not ok
    assert any("operator missing" in item for item in blockers)


def test_complete_checklist_passes_and_wrong_identity_fails(tmp_path):
    cfg = Phase0AConfig(arm_id="left", physical_mounting_id="mount", camera_serial="cam", camera_mount_id="cam_mount", tool_id="tool")
    checklist = checklist_template(cfg.identity_dict())
    checklist["operator"] = "operator"
    checklist["timestamp"] = "2026-08-03T00:00:00Z"
    for item in checklist["items"].values():
        item["checked"] = True
    path = tmp_path / "operator_checklist.yaml"
    path.write_text(yaml.safe_dump(checklist), encoding="utf-8")
    assert validate_checklist(path, cfg.identity_dict())[0]
    assert not validate_checklist(path, {**cfg.identity_dict(), "tool_id": "other"})[0]


def test_physical_readiness_requires_confirm_base_and_local_identity(tmp_path):
    checklist = checklist_template({"arm_id": "left", "physical_mounting_id": "mount", "camera_serial": "cam", "camera_mount_id": "cam_mount", "tool_id": "tool"})
    checklist["operator"] = "operator"
    checklist["timestamp"] = "2026-08-03T00:00:00Z"
    for item in checklist["items"].values():
        item["checked"] = True
    path = tmp_path / "operator_checklist.yaml"
    path.write_text(yaml.safe_dump(checklist), encoding="utf-8")
    cfg = Phase0AConfig(arm_id="left", physical_mounting_id="mount", camera_serial="cam", camera_mount_id="cam_mount", tool_id="tool", physical_execution_enabled_by_default=True, checklist_path=str(path))
    report = build_readiness_report(cfg, physical=True, confirm=PHASE_0A_CONFIRMATION_TOKEN, confirm_base_stopped=PHASE_0A_BASE_STOPPED_TOKEN, checklist_path=str(path))
    assert "local identity" not in " ".join(report["physical_blockers"])
    report = build_readiness_report(cfg, physical=True, confirm=PHASE_0A_CONFIRMATION_TOKEN, confirm_base_stopped="", checklist_path=str(path))
    assert not report["physical_ready"]
    assert any("BUNKER_STOPPED" in blocker for blocker in report["physical_blockers"])


def test_finalize_reads_measurements_and_accepts_tight_repeatability(tmp_path):
    cfg = Phase0AConfig(output_root=str(tmp_path), thresholds=Phase0AThresholds(minimum_completed_cycles=2))
    writer = Phase0AArtifactWriter(tmp_path)
    manifest = {
        "diagnostic_id": "diag",
        "test": {"measurement_pose_target_positions": [0.0] * 6, "joint_names": JOINTS, "completed_cycles": 2},
        "results": {"operator_observations": {"mechanical_checklist_completed": True}},
        "motion_reporting": {"motion_commanded": True},
    }
    writer.write_manifest(manifest)
    writer.append_cycle({"cycle_index": 1, "measurement_recorded": True, "final_measured_joint_values": [0.001] * 6, "settling_time_s": 1.0, "endpoint_reached": True})
    writer.append_cycle({"cycle_index": 2, "measurement_recorded": True, "final_measured_joint_values": [-0.001] * 6, "settling_time_s": 1.1, "endpoint_reached": True})
    writer.write_physical_measurement_template(2)
    (tmp_path / "physical_measurements.csv").write_text("cycle_index,method,reference_frame,reference_description,x_mm,y_mm,z_mm,estimated_measurement_uncertainty_mm,notes\n1,manual_xyz_mm,ref,,0,0,0,1,\n2,manual_xyz_mm,ref,,1,0,0,1,\n", encoding="utf-8")
    result = finalize_artifact(cfg, tmp_path)
    assert result["manifest"]["results"]["physical_measurements"]["available"]
    assert result["manifest"]["results"]["phase_0a"]["physical_repeatability"]["status"] == PHYSICAL_STATUS_ACCEPTABLE


def test_finalize_preserves_unknown_without_rows(tmp_path):
    cfg = Phase0AConfig(output_root=str(tmp_path))
    writer = Phase0AArtifactWriter(tmp_path)
    writer.write_manifest({"diagnostic_id": "diag", "test": {"measurement_pose_target_positions": [0.0] * 6, "joint_names": JOINTS}, "results": {}, "motion_reporting": {}})
    writer.append_cycle({"cycle_index": 1, "measurement_recorded": True, "final_measured_joint_values": [0.0] * 6, "settling_time_s": 1.0, "endpoint_reached": True})
    writer.write_physical_measurement_template(1)
    result = finalize_artifact(cfg, tmp_path)
    assert result["manifest"]["results"]["phase_0a"]["physical_repeatability"]["status"] == PHYSICAL_STATUS_UNKNOWN
    assert not result["manifest"]["results"]["phase_0a_passed"]


def test_finalize_rejects_duplicate_and_malformed_measurement_rows(tmp_path):
    path = tmp_path / "physical_measurements.csv"
    path.write_text("cycle_index,method,reference_frame,reference_description,x_mm,y_mm,z_mm,estimated_measurement_uncertainty_mm,notes\n1,manual,ref,,0,0,0,1,\n1,manual,ref,,0,0,0,1,\n", encoding="utf-8")
    with pytest.raises(ValueError, match="duplicate"):
        read_physical_measurements_csv(path, expected_cycles={1})
    path.write_text("cycle_index,method,reference_frame,reference_description,x_mm,y_mm,z_mm,estimated_measurement_uncertainty_mm,notes\n1,manual,ref,,0,0,0,,\n", encoding="utf-8")
    with pytest.raises(ValueError, match="uncertainty"):
        read_physical_measurements_csv(path, expected_cycles={1})


def test_large_measurement_uncertainty_prevents_strong_acceptance():
    metrics = physical_repeatability_metrics([
        {"x_mm": 0.0, "y_mm": 0.0, "z_mm": 0.0, "estimated_measurement_uncertainty_mm": 99.0, "method": "manual"},
        {"x_mm": 1.0, "y_mm": 0.0, "z_mm": 0.0, "estimated_measurement_uncertainty_mm": 99.0, "method": "manual"},
    ], maximum_measurement_uncertainty_mm=5.0)
    assert metrics["uncertainty_too_large_for_strong_acceptance"]
    classification, passed, blockers, phase = classify_phase0a(completed_cycles=3, joint_metrics={"target_error_max_rad": 0.001, "final_position_summary": {"per_joint": {name: {"std": 0.001, "range": 0.002} for name in JOINTS}}}, settling={"timeout_count": 0, "controller_abort_count": 0, "stale_feedback_count": 0}, physical_metrics=metrics, thresholds=Phase0AConfig().thresholds, checklist_passed=True)
    assert classification == CLASS_MECHANICAL_REPEATABILITY_SUSPECT
    assert not passed


def test_software_unknown_is_not_complete_phase0a_pass():
    classification, passed, blockers, phase = classify_phase0a(completed_cycles=3, joint_metrics={"target_error_max_rad": 0.001, "final_position_summary": {"per_joint": {name: {"std": 0.001, "range": 0.002} for name in JOINTS}}}, settling={"timeout_count": 0, "controller_abort_count": 0, "stale_feedback_count": 0}, physical_metrics={"available": False}, thresholds=Phase0AConfig().thresholds)
    assert classification == CLASS_JOINT_REPEATABLE_PHYSICAL_REPEATABILITY_UNKNOWN
    assert not passed
    assert phase["physical_repeatability"]["status"] == PHYSICAL_STATUS_UNKNOWN


def test_stop_attempts_backend_and_external_command(tmp_path):
    stop_script = tmp_path / "stop.sh"
    stop_script.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    stop_script.chmod(0o755)
    state = MotionExecutionState(motion_commanded=True)
    backend = type("Backend", (), {"stop": lambda self: None})()
    invoke_stop(Phase0AConfig(stop_command=(str(stop_script),)), backend, state)
    assert state.stop_attempted
    assert state.stop_succeeded is True


def test_external_stop_failure_is_reported(tmp_path):
    stop_script = tmp_path / "stop.sh"
    stop_script.write_text("#!/usr/bin/env bash\nexit 3\n", encoding="utf-8")
    stop_script.chmod(0o755)
    state = MotionExecutionState(motion_commanded=True)
    invoke_stop(Phase0AConfig(stop_command=(str(stop_script),)), None, state)
    assert state.stop_succeeded is False
    assert state.external_stop_result["returncode"] == 3


def test_committed_config_contains_no_actual_hardware_identity():
    cfg_path = Path("piper-on-bunker/config/piper_x_phase_0a_repeatability.yaml")
    if not cfg_path.exists():
        cfg_path = Path("/home/dase-hw101/piper-pipeline-testbed") / cfg_path
    payload = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
    identity = payload["identity"]
    assert identity["camera_serial"] == "unknown"
    assert identity["tool_id"] == "unknown"
    assert payload["execution"]["physical_execution_enabled_by_default"] is False


def test_local_config_is_ignored_by_gitignore_contract():
    gitignore = Path(".gitignore")
    if not gitignore.exists():
        gitignore = Path("/home/dase-hw101/piper-pipeline-testbed/.gitignore")
    assert "piper-on-bunker/config/*.local.yaml" in gitignore.read_text(encoding="utf-8")
