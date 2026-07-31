import math

import pytest

from piper_on_bunker.hardware.piper_x_feedback import PiperXFeedbackConfig
from piper_on_bunker.hardware.piper_x_feedback import PiperXFeedbackEvidence
from piper_on_bunker.hardware.piper_x_feedback import PiperXReadOnlyFeedbackAdapter
from piper_on_bunker.hardware.piper_x_feedback import ensure_no_command_methods_called
from piper_on_bunker.hardware.piper_x_feedback import extract_six_joint_values
from piper_on_bunker.hardware.piper_x_feedback import validate_piper_x_feedback
from piper_on_bunker.hardware.piper_x_feedback import validate_single_joint_state_authority
from piper_on_bunker.manipulation.moveit_aruco_touch import EXPECTED_JOINT_NAMES
from piper_on_bunker.manipulation.moveit_aruco_touch import MockMoveItTouchBackend
from piper_on_bunker.manipulation.moveit_aruco_touch import MoveItArucoTouchController
from piper_on_bunker.manipulation.moveit_aruco_touch import TaughtPose
from piper_on_bunker.manipulation.moveit_aruco_touch import load_touch_config


CONFIG = "piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml"


def _evidence(**overrides):
    payload = {
        "connected": True,
        "feedback_valid": True,
        "source_update_counter": 1,
        "source_timestamp_s": 10.0,
        "real_feedback_packet": True,
        "communication_ready": True,
        "no_motion_commands_sent": True,
    }
    payload.update(overrides)
    return PiperXFeedbackEvidence(**payload)


def test_valid_pyagxarm_feedback_conversion():
    sample = validate_piper_x_feedback([0.1, 0.2, -0.3, 0.4, 0.5, -0.6], _evidence())
    assert sample.joint_names == EXPECTED_JOINT_NAMES
    assert sample.positions_rad == [0.1, 0.2, -0.3, 0.4, 0.5, -0.6]
    assert sample.units == "rad"
    assert sample.evidence.no_motion_commands_sent is True


def test_wrong_joint_count_rejected():
    with pytest.raises(ValueError, match="expected six"):
        validate_piper_x_feedback([0.0] * 5, _evidence())


def test_non_finite_feedback_rejected():
    with pytest.raises(ValueError, match="non-finite"):
        validate_piper_x_feedback([0.0, 0.0, math.nan, 0.0, 0.0, 0.0], _evidence())


def test_communication_failure_rejected():
    with pytest.raises(ValueError, match="communication-ready"):
        validate_piper_x_feedback([0.0] * 6, _evidence(communication_ready=False))


def test_false_fresh_zero_source_rejected():
    with pytest.raises(ValueError, match="all-zero"):
        validate_piper_x_feedback([0.0] * 6, _evidence(source_update_counter=None, source_timestamp_s=None))


def test_genuine_verified_zero_pose_acceptance_with_update_evidence():
    sample = validate_piper_x_feedback([0.0] * 6, _evidence(source_update_counter=42, source_timestamp_s=11.0))
    assert sample.positions_rad == [0.0] * 6


def test_joint_ordering_mapping_from_named_object():
    class Raw:
        joint1 = 1
        joint2 = 2
        joint3 = 3
        joint4 = 4
        joint5 = 5
        joint6 = 6

    assert extract_six_joint_values(Raw()) == [1, 2, 3, 4, 5, 6]


def test_command_capable_method_call_rejected():
    with pytest.raises(RuntimeError, match="command-capable"):
        ensure_no_command_methods_called(["get_leader_joint_angles", "move_js"])


def test_adapter_refuses_command_method():
    class FakeArm:
        def move_js(self):
            return [0.0] * 6

    adapter = PiperXReadOnlyFeedbackAdapter(lambda: FakeArm(), feedback_method="move_js", config=PiperXFeedbackConfig())
    with pytest.raises(RuntimeError, match="unsafe command-capable"):
        adapter.read_sample()


def test_adapter_calls_only_configured_read_method():
    class FakeArm:
        def get_leader_joint_angles(self):
            return [0.01, 0.02, -0.03, 0.04, 0.05, -0.06]

    adapter = PiperXReadOnlyFeedbackAdapter(lambda: FakeArm(), feedback_method="get_leader_joint_angles")
    sample = adapter.read_sample()
    assert sample.positions_rad[0] == pytest.approx(0.01)
    assert adapter.calls == ["get_leader_joint_angles"]


def test_taught_pose_source_mismatch_rejected():
    cfg = load_touch_config(CONFIG)
    poses = {
        "staging": TaughtPose("staging", EXPECTED_JOINT_NAMES, [0.0, -0.04, -0.07, 0.0, 0.04, 0.0], "old"),
        "pre_touch": TaughtPose("pre_touch", EXPECTED_JOINT_NAMES, [0.01, -0.04, -0.07, 0.0, 0.04, 0.0], "old"),
        "touch": TaughtPose("touch", EXPECTED_JOINT_NAMES, [0.02, -0.04, -0.07, 0.0, 0.04, 0.0], "old"),
        "retract": TaughtPose("retract", EXPECTED_JOINT_NAMES, [0.0, -0.04, -0.07, 0.0, 0.04, 0.0], "old"),
    }
    result = MoveItArucoTouchController(cfg, MockMoveItTouchBackend(), taught_poses=poses).run(planning_only=True)
    assert not result.success
    assert "requires recapture" in result.failure_reason


def test_duplicate_ros_publisher_rejection():
    with pytest.raises(ValueError, match="exactly one"):
        validate_single_joint_state_authority(["/bridge", "/piper_ctrl_single_node"])


def test_normal_piper_joint_state_source_rejected():
    with pytest.raises(ValueError, match="not the PiPER-X feedback bridge"):
        validate_single_joint_state_authority(["/piper_ctrl_single_node"])


def test_expected_joint_state_bridge_publisher_accepts():
    validate_single_joint_state_authority(["/piper_x_pyagxarm_joint_state_bridge"])
