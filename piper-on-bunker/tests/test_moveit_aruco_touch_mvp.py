from dataclasses import replace
import subprocess
import sys

import pytest

from piper_on_bunker.manipulation.moveit_aruco_touch import EXPECTED_JOINT_NAMES
from piper_on_bunker.manipulation.moveit_aruco_touch import MarkerStatus
from piper_on_bunker.manipulation.moveit_aruco_touch import MockMoveItTouchBackend
from piper_on_bunker.manipulation.moveit_aruco_touch import MoveItArucoTouchController
from piper_on_bunker.manipulation.moveit_aruco_touch import RestrictedArucoTouchAPI
from piper_on_bunker.manipulation.moveit_aruco_touch import TaughtPose
from piper_on_bunker.manipulation.moveit_aruco_touch import TouchFailure
from piper_on_bunker.manipulation.moveit_aruco_touch import extract_named_joint_state
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
        "touch": TaughtPose("touch", names, [0.03, -0.035, -0.065, 0.0, 0.035, 0.0], "test"),
        "retract": TaughtPose("retract", names, [0.015, -0.055, -0.085, 0.0, 0.055, 0.0], "test"),
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
    poses = _poses()
    del poses["touch"]
    result = _run(poses=poses)
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


def test_joint_jump_rejected():
    backend = MockMoveItTouchBackend(max_joint_delta_rad=0.5)
    result = _run(backend=backend)
    assert not result.success
    assert "joint jump" in result.failure_reason


def test_full_touch_uses_taught_touch_pose():
    result = _run()
    assert result.success
    states = result.outputs["transitions"]
    assert "MOVE_TOUCH" in states
    assert "MOVE_RETRACT" in states


def test_pre_touch_test_never_enters_touch():
    controller = MoveItArucoTouchController(_config(), MockMoveItTouchBackend(), taught_poses=_poses())
    result = controller.run(planning_only=True, sequence="pre_touch_test")
    assert result.success
    assert "MOVE_TOUCH" not in result.outputs["transitions"]
    assert "MOVE_RETRACT" in result.outputs["transitions"]


def test_physical_execution_requires_explicit_confirmation():
    controller = MoveItArucoTouchController(_config(), MockMoveItTouchBackend(), taught_poses=_poses())
    result = controller.run(planning_only=False, execute=True, confirm="")
    assert not result.success
    assert "confirm" in result.outputs["message"]


def test_pre_touch_test_requires_own_confirmation():
    controller = MoveItArucoTouchController(_config(), MockMoveItTouchBackend(), taught_poses=_poses())
    result = controller.run(planning_only=False, execute=True, confirm="FIXED_ARUCO_TOUCH", sequence="pre_touch_test")
    assert not result.success
    assert "PRE_TOUCH_TEST" in result.outputs["message"]


def test_marker_loss_blocks_touch():
    class LosingMarkerBackend(MockMoveItTouchBackend):
        def __init__(self):
            super().__init__()
            self.calls = 0

        def read_marker_status(self):
            self.calls += 1
            if self.calls >= 2:
                return MarkerStatus(False, None, "DICT_ARUCO_ORIGINAL", 0.100, 0.0, 0.0, 0.0, [])
            return super().read_marker_status()

    result = _run(backend=LosingMarkerBackend())
    assert not result.success
    assert result.failure_reason == TouchFailure.MARKER_MISSING


def test_wrong_dictionary_blocks_touch():
    marker = MarkerStatus(True, 6, "DICT_4X4_50", 0.100, 0.0, 0.0, 1.0, [6])
    result = _run(marker)
    assert not result.success
    assert result.failure_reason == TouchFailure.WRONG_MARKER


def test_emergency_stop_called_on_execution_failure():
    cfg = replace(_config(), physical_execution_enabled_by_default=True, piper_x_model_verified=True)
    backend = MockMoveItTouchBackend(execute_ok=False)
    controller = MoveItArucoTouchController(cfg, backend, taught_poses=_poses())
    result = controller.run(planning_only=False, execute=True, confirm="FIXED_ARUCO_TOUCH")
    assert not result.success
    assert backend.stopped is True


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


def test_live_joint_state_name_based_extraction():
    snapshot = extract_named_joint_state(
        names=["gripper", "joint3", "joint1", "joint6", "joint2", "joint5", "joint4"],
        positions=[9, 0.3, 0.1, 0.6, 0.2, 0.5, 0.4],
        velocities=[0, 0, 0, 0, 0, 0, 0],
        stamp_s=10.0,
        now_s=10.1,
    )
    assert snapshot.positions == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]


def test_stale_state_extraction_rejected():
    with pytest.raises(ValueError, match="stale joint state"):
        extract_named_joint_state(names=list(EXPECTED_JOINT_NAMES), positions=[0] * 6, velocities=[0] * 6, stamp_s=1.0, now_s=9.0)


def test_moving_state_extraction_rejected():
    with pytest.raises(ValueError, match="robot is not stopped"):
        extract_named_joint_state(names=list(EXPECTED_JOINT_NAMES), positions=[0] * 6, velocities=[0, 0, 0.2, 0, 0, 0], stamp_s=1.0, now_s=1.0)


def test_missing_joint_extraction_rejected():
    with pytest.raises(ValueError, match="missing expected joints"):
        extract_named_joint_state(names=["joint1"], positions=[0], velocities=[0], stamp_s=1.0, now_s=1.0)


def test_cli_requires_exactly_one_backend():
    result = subprocess.run(
        [sys.executable, "piper-on-bunker/scripts/run_moveit_aruco_touch.py", "--planning-only"],
        cwd=".",
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 2
    assert "Select exactly one backend" in result.stderr


def test_cli_refuses_mock_execute():
    result = subprocess.run(
        [
            sys.executable,
            "piper-on-bunker/scripts/run_moveit_aruco_touch.py",
            "--mock",
            "--mock-taught-poses",
            "--execute",
            "--confirm",
            "FIXED_ARUCO_TOUCH",
        ],
        cwd=".",
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 2
    assert "--execute is refused with --mock" in result.stderr
