from __future__ import annotations

import math
from pathlib import Path

import pytest

from piper_on_bunker.calibration.piper_x_repeatability import (
    classify_phase0a,
    observation_summary,
    phase0a_lock,
    run_observation_mode,
    validate_taught_pose_for_phase0a,
)
from piper_on_bunker.calibration.piper_x_repeatability_artifacts import Phase0AArtifactWriter
from piper_on_bunker.calibration.piper_x_repeatability_contract import (
    CLASS_CONTROLLER_OR_SETTLING_INCONSISTENT,
    CLASS_FEEDBACK_UNSTABLE,
    CLASS_INSUFFICIENT_DATA,
    CLASS_JOINT_AND_PHYSICAL_REPEATABILITY_ACCEPTABLE,
    CLASS_JOINT_REPEATABLE_PHYSICAL_REPEATABILITY_UNKNOWN,
    CLASS_MECHANICAL_REPEATABILITY_SUSPECT,
    Phase0AConfig,
)
from piper_on_bunker.calibration.piper_x_repeatability_metrics import (
    joint_repeatability_metrics,
    physical_repeatability_metrics,
    rms,
    scalar_summary,
)
from piper_on_bunker.manipulation.moveit_aruco_touch import JointStateSnapshot, TaughtPose


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


def test_scalar_summary_and_rms_formulas():
    summary = scalar_summary([1.0, 2.0, 3.0])
    assert summary["mean"] == pytest.approx(2.0)
    assert summary["range"] == pytest.approx(2.0)
    assert summary["std"] == pytest.approx(math.sqrt(2.0 / 3.0))
    assert rms([3.0, 4.0]) == pytest.approx(math.sqrt(12.5))


def test_joint_repeatability_metrics_formulas():
    metrics = joint_repeatability_metrics(
        joint_names=JOINTS,
        target_positions=[0.0] * 6,
        final_positions_by_cycle=[[0.0] * 6, [0.001] * 6, [-0.001] * 6],
    )
    assert metrics["cycle_count"] == 3
    assert metrics["target_error_max_rad"] == pytest.approx(0.001)
    assert metrics["final_position_summary"]["per_joint"]["joint1"]["range"] == pytest.approx(0.002)


def test_physical_repeatability_metrics_centroid_and_pairwise():
    metrics = physical_repeatability_metrics(
        [
            {"x_mm": 0.0, "y_mm": 0.0, "z_mm": 0.0, "estimated_measurement_uncertainty_mm": 1.0},
            {"x_mm": 3.0, "y_mm": 4.0, "z_mm": 0.0, "estimated_measurement_uncertainty_mm": 1.0},
        ]
    )
    assert metrics["available"]
    assert metrics["centroid_mm"] == pytest.approx([1.5, 2.0, 0.0])
    assert metrics["maximum_pairwise_distance_mm"] == pytest.approx(5.0)


def test_no_physical_measurement_is_unknown_not_physical_pass():
    classification, passed, blockers = classify_phase0a(
        completed_cycles=3,
        joint_metrics={
            "target_error_max_rad": 0.001,
            "final_position_summary": {"per_joint": {name: {"std": 0.001, "range": 0.002} for name in JOINTS}},
        },
        settling={"timeout_count": 0, "controller_abort_count": 0, "stale_feedback_count": 0},
        physical_metrics={"available": False},
        thresholds=Phase0AConfig().thresholds,
    )
    assert classification == CLASS_JOINT_REPEATABLE_PHYSICAL_REPEATABILITY_UNKNOWN
    assert passed
    assert blockers == ["physical endpoint measurements not provided"]


def test_classification_branches():
    thresholds = Phase0AConfig().thresholds
    assert classify_phase0a(completed_cycles=0, joint_metrics=None, settling=None, physical_metrics=None, thresholds=thresholds, feedback_unstable=True)[0] == CLASS_FEEDBACK_UNSTABLE
    assert classify_phase0a(completed_cycles=1, joint_metrics=None, settling=None, physical_metrics=None, thresholds=thresholds)[0] == CLASS_INSUFFICIENT_DATA
    assert classify_phase0a(completed_cycles=3, joint_metrics={"target_error_max_rad": 0.2}, settling={"timeout_count": 0, "controller_abort_count": 0, "stale_feedback_count": 0}, physical_metrics=None, thresholds=thresholds)[0] == CLASS_CONTROLLER_OR_SETTLING_INCONSISTENT
    assert classify_phase0a(
        completed_cycles=3,
        joint_metrics={"target_error_max_rad": 0.001, "final_position_summary": {"per_joint": {name: {"std": 0.001, "range": 0.002} for name in JOINTS}}},
        settling={"timeout_count": 0, "controller_abort_count": 0, "stale_feedback_count": 0},
        physical_metrics={"available": True, "maximum_pairwise_distance_mm": 30.0, "rms_distance_from_centroid_mm": 2.0},
        thresholds=thresholds,
    )[0] == CLASS_MECHANICAL_REPEATABILITY_SUSPECT
    assert classify_phase0a(
        completed_cycles=3,
        joint_metrics={"target_error_max_rad": 0.001, "final_position_summary": {"per_joint": {name: {"std": 0.001, "range": 0.002} for name in JOINTS}}},
        settling={"timeout_count": 0, "controller_abort_count": 0, "stale_feedback_count": 0},
        physical_metrics={"available": True, "maximum_pairwise_distance_mm": 3.0, "rms_distance_from_centroid_mm": 1.0},
        thresholds=thresholds,
    )[0] == CLASS_JOINT_AND_PHYSICAL_REPEATABILITY_ACCEPTABLE


def test_pose_allowlist_and_feedback_source_validation():
    config = Phase0AConfig(approved_pose_names=("staging",))
    poses = {
        "staging": TaughtPose("staging", JOINTS, [0.0] * 6, "test", METADATA),
        "other": TaughtPose("other", JOINTS, [0.0] * 6, "test", METADATA),
    }
    assert validate_taught_pose_for_phase0a(config, "staging", poses, LIMITS).name == "staging"
    with pytest.raises(ValueError, match="not allowlisted"):
        validate_taught_pose_for_phase0a(config, "other", poses, LIMITS)
    poses["staging"] = TaughtPose("staging", JOINTS, [0.0] * 6, "test", {})
    with pytest.raises(ValueError, match="requires recapture"):
        validate_taught_pose_for_phase0a(config, "staging", poses, LIMITS)


def test_observation_only_never_commands_motion(tmp_path):
    reader = StaticReader()
    config = Phase0AConfig(output_root=str(tmp_path), observation_sample_rate_hz=200.0)
    result = run_observation_mode(config, reader, duration_s=0.02)
    assert reader.calls > 0
    assert not reader.commanded
    assert result["manifest"]["test"]["mode"] == "observe"
    assert Path(result["artifact_dir"], "manifest.yaml").exists()


def test_observation_summary_rejects_wrong_joint_count():
    with pytest.raises(ValueError):
        observation_summary([JointStateSnapshot(["joint1"], [0.0], 0.0)])


def test_lock_refuses_concurrent_owner(tmp_path):
    lock = tmp_path / "phase0a.lock"
    with phase0a_lock(lock):
        with pytest.raises(RuntimeError, match="lock already exists"):
            with phase0a_lock(lock):
                pass


def test_artifact_writer_outputs_csv_and_checklist(tmp_path):
    writer = Phase0AArtifactWriter(tmp_path)
    csv_path = writer.write_physical_measurement_template(2)
    checklist = writer.write_checklist()
    assert csv_path.read_text(encoding="utf-8").count("\n") == 3
    assert "Bunker stationary" in checklist.read_text(encoding="utf-8")
