from pathlib import Path

import pytest

from piper_on_bunker.manipulation.moveit_aruco_touch import EXPECTED_JOINT_NAMES
from piper_on_bunker.manipulation.moveit_aruco_touch import MarkerStatus
from piper_on_bunker.manipulation.moveit_aruco_touch import MockMoveItTouchBackend
from piper_on_bunker.manipulation.moveit_aruco_touch import MoveItArucoTouchController
from piper_on_bunker.manipulation.moveit_aruco_touch import RestrictedArucoTouchAPI
from piper_on_bunker.manipulation.moveit_aruco_touch import TaughtPose
from piper_on_bunker.manipulation.moveit_aruco_touch import TouchFailure
from piper_on_bunker.manipulation.moveit_aruco_touch import build_inverse_press
from piper_on_bunker.manipulation.moveit_aruco_touch import load_touch_config
from piper_on_bunker.manipulation.moveit_aruco_touch import save_taught_pose_manifest
from piper_on_bunker.mission_logging import MissionLogger


CONFIG = "piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml"


def _config():
    return load_touch_config(CONFIG)


def _poses(joint_names=None):
    names = list(joint_names or EXPECTED_JOINT_NAMES)
    return {
        "home": TaughtPose("home", names, [0.0, -0.05, -0.08, 0.0, 0.05, 0.0], "test"),
        "pre_touch": TaughtPose("pre_touch", names, [0.02, -0.04, -0.07, 0.0, 0.04, 0.0], "test"),
    }


def _run(marker=None, backend=None, poses=None, logger=None):
    cfg = _config()
    backend = backend or MockMoveItTouchBackend(marker=marker)
    return MoveItArucoTouchController(cfg, backend, taught_poses=poses or _poses(), logger=logger).run(planning_only=True)


def test_marker_id_6_accepted():
    result = _run()
    assert result.success
    assert result.outputs["marker_status"]["marker_id"] == 6
    assert result.outputs["rejected_handeye_used_for_targeting"] is False


def test_wrong_marker_rejected():
    marker = MarkerStatus(True, 7, "DICT_ARUCO_ORIGINAL", 0.100, 0.0, 0.0, 1.0, [7])
    result = _run(marker)
    assert not result.success
    assert result.failure_reason == TouchFailure.WRONG_MARKER


def test_marker_absent_rejected():
    marker = MarkerStatus(False, None, "DICT_ARUCO_ORIGINAL", 0.100, 0.0, 0.0, 0.0, [])
    result = _run(marker)
    assert not result.success
    assert result.failure_reason == TouchFailure.MARKER_MISSING


def test_stale_marker_rejected():
    marker = MarkerStatus(True, 6, "DICT_ARUCO_ORIGINAL", 0.100, 9.0, 9.0, 1.0, [6])
    result = _run(marker)
    assert not result.success
    assert result.failure_reason == TouchFailure.STALE_CAMERA


def test_stale_joint_state_rejected():
    backend = MockMoveItTouchBackend()
    backend.joint_state = backend.joint_state.__class__(list(EXPECTED_JOINT_NAMES), [0.0] * 6, 9.0)
    result = _run(backend=backend)
    assert not result.success
    assert result.failure_reason == TouchFailure.STALE_JOINT_STATE


def test_missing_taught_pose_rejected():
    result = _run(poses={"home": _poses()["home"]})
    assert not result.success
    assert TouchFailure.MISSING_TAUGHT_POSE in result.failure_reason


def test_mismatched_joint_schema_rejected():
    result = _run(poses=_poses(["joint1", "joint2", "joint3", "joint4", "joint5", "wrong_joint"]))
    assert not result.success
    assert "expected joint schema" in result.failure_reason


def test_planning_only_never_executes():
    backend = MockMoveItTouchBackend()
    result = _run(backend=backend)
    assert result.success
    assert result.planning_only is True
    assert result.physical_motion_performed is False
    assert backend.executed == []


def test_cartesian_fraction_below_one_rejected():
    backend = MockMoveItTouchBackend(cartesian_fraction=0.8)
    result = _run(backend=backend)
    assert not result.success
    assert "Cartesian path fraction" in result.failure_reason


def test_joint_jump_rejected():
    backend = MockMoveItTouchBackend(max_joint_delta_rad=0.5)
    result = _run(backend=backend)
    assert not result.success
    assert "joint jump" in result.failure_reason


def test_retract_generated_as_inverse_press_displacement():
    assert build_inverse_press([0.0, 0.0, 0.02]) == [-0.0, -0.0, -0.02]


def test_physical_execution_requires_explicit_confirmation():
    controller = MoveItArucoTouchController(_config(), MockMoveItTouchBackend(), taught_poses=_poses())
    result = controller.run(planning_only=False, execute=True, confirm="")
    assert not result.success
    assert "confirm" in result.outputs["message"]


def test_restricted_api_rejects_arbitrary_targets():
    api = RestrictedArucoTouchAPI(MoveItArucoTouchController(_config(), MockMoveItTouchBackend(), taught_poses=_poses()))
    result = api.call("move_to_raw_joint_target", joints=[0] * 6)
    assert result["accepted"] is False


def test_audit_log_records_complete_transitions(tmp_path):
    log_path = tmp_path / "audit.jsonl"
    result = _run(logger=MissionLogger(path=str(log_path), enabled=True))
    assert result.success
    text = log_path.read_text(encoding="utf-8")
    assert '"state": "CHECK_MARKER"' in text
    assert '"state": "COMPLETE"' in text


def test_save_taught_pose_manifest(tmp_path):
    path = tmp_path / "poses.yaml"
    pose = _poses()["home"]
    save_taught_pose_manifest(path, pose, {"planning_group": "arm"})
    assert "home" in path.read_text(encoding="utf-8")
