import math

import pytest

from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_FEEDBACK_CAN_IDS
from piper_on_bunker.hardware.piper_x_feedback import PassivePiperXSocketcanDecoder
from piper_on_bunker.hardware.piper_x_feedback import PiperXFeedbackConfig
from piper_on_bunker.hardware.piper_x_feedback import PiperXFeedbackEvidence
from piper_on_bunker.hardware.piper_x_feedback import validate_piper_x_feedback
from piper_on_bunker.hardware.piper_x_feedback import validate_single_joint_state_authority
from piper_on_bunker.manipulation.moveit_aruco_touch import EXPECTED_JOINT_NAMES
from piper_on_bunker.manipulation.moveit_aruco_touch import MockMoveItTouchBackend
from piper_on_bunker.manipulation.moveit_aruco_touch import MoveItArucoTouchController
from piper_on_bunker.manipulation.moveit_aruco_touch import TaughtPose
from piper_on_bunker.manipulation.moveit_aruco_touch import load_touch_config


CONFIG = "piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml"


def _frame(a: int, b: int) -> bytes:
    return int(a).to_bytes(4, "big", signed=True) + int(b).to_bytes(4, "big", signed=True)


def _complete_decoder() -> PassivePiperXSocketcanDecoder:
    decoder = PassivePiperXSocketcanDecoder(config=PiperXFeedbackConfig(max_age_s=1.0, frame_set_window_s=0.2))
    decoder.update_from_can(0x2A5, _frame(1000, 2000), 10.00)
    decoder.update_from_can(0x2A6, _frame(-3000, 4000), 10.01)
    decoder.update_from_can(0x2A7, _frame(5000, -6000), 10.02)
    return decoder


def test_passive_socketcan_feedback_conversion():
    sample = _complete_decoder().sample(now_s=10.03)
    assert sample.joint_names == EXPECTED_JOINT_NAMES
    assert sample.positions_rad == pytest.approx([math.radians(v / 1000.0) for v in [1000, 2000, -3000, 4000, 5000, -6000]])
    assert sample.evidence.raw_joint_values == [1000, 2000, -3000, 4000, 5000, -6000]
    assert sample.evidence.source_can_ids == list(PIPER_X_FEEDBACK_CAN_IDS)
    assert sample.evidence.source_frame_receive_counters == {0x2A5: 1, 0x2A6: 2, 0x2A7: 3}
    assert sample.evidence.source_frame_timestamps_s == {0x2A5: 10.00, 0x2A6: 10.01, 0x2A7: 10.02}
    assert sample.evidence.tx_frames_sent_by_bridge == 0


def test_repeated_sample_without_new_packet_rejected():
    decoder = _complete_decoder()
    decoder.sample(now_s=10.03)
    with pytest.raises(ValueError, match="no fresh complete"):
        decoder.sample(now_s=10.04)


def test_single_updated_frame_does_not_produce_new_sample():
    decoder = _complete_decoder()
    decoder.sample(now_s=10.03)
    decoder.update_from_can(0x2A5, _frame(1010, 2010), 10.04)
    with pytest.raises(ValueError, match="stale frame IDs: 0x2A6, 0x2A7"):
        decoder.sample(now_s=10.05)


def test_two_updated_frames_do_not_produce_new_sample():
    decoder = _complete_decoder()
    decoder.sample(now_s=10.03)
    decoder.update_from_can(0x2A5, _frame(1010, 2010), 10.04)
    decoder.update_from_can(0x2A6, _frame(-3010, 4010), 10.05)
    with pytest.raises(ValueError, match="stale frame IDs: 0x2A7"):
        decoder.sample(now_s=10.06)


def test_all_three_updated_frames_produce_new_sample():
    decoder = _complete_decoder()
    decoder.sample(now_s=10.03)
    decoder.update_from_can(0x2A5, _frame(1010, 2010), 10.04)
    decoder.update_from_can(0x2A6, _frame(-3010, 4010), 10.05)
    decoder.update_from_can(0x2A7, _frame(5010, -6010), 10.06)
    sample = decoder.sample(now_s=10.07)
    assert sample.evidence.source_frame_receive_counters == {0x2A5: 4, 0x2A6: 5, 0x2A7: 6}
    assert sample.positions_rad == pytest.approx(
        [math.radians(v / 1000.0) for v in [1010, 2010, -3010, 4010, 5010, -6010]]
    )


def test_incomplete_six_joint_frame_set_rejected():
    decoder = PassivePiperXSocketcanDecoder()
    decoder.update_from_can(0x2A5, _frame(1, 2), 10.0)
    decoder.update_from_can(0x2A6, _frame(3, 4), 10.0)
    with pytest.raises(ValueError, match="incomplete"):
        decoder.sample(now_s=10.0)


def test_stale_frames_rejected():
    decoder = _complete_decoder()
    with pytest.raises(ValueError, match="incomplete or stale"):
        decoder.sample(now_s=12.0)


def test_frame_window_rejected():
    decoder = PassivePiperXSocketcanDecoder(config=PiperXFeedbackConfig(max_age_s=10.0, frame_set_window_s=0.01))
    decoder.update_from_can(0x2A5, _frame(1, 2), 10.00)
    decoder.update_from_can(0x2A6, _frame(3, 4), 10.02)
    decoder.update_from_can(0x2A7, _frame(5, 6), 10.04)
    with pytest.raises(ValueError, match="incomplete or stale"):
        decoder.sample(now_s=10.05)


def test_non_finite_feedback_rejected():
    evidence = PiperXFeedbackEvidence(
        connected=True,
        feedback_valid=True,
        source_update_counter=1,
        source_timestamp_s=10.0,
        source_can_ids=list(PIPER_X_FEEDBACK_CAN_IDS),
        source_frame_receive_counters={can_id: index + 1 for index, can_id in enumerate(PIPER_X_FEEDBACK_CAN_IDS)},
        source_frame_timestamps_s={can_id: 10.0 for can_id in PIPER_X_FEEDBACK_CAN_IDS},
        raw_joint_values=[0] * 6,
        real_feedback_packet=True,
        complete_frame_set=True,
        communication_ready=True,
    )
    with pytest.raises(ValueError, match="non-finite"):
        validate_piper_x_feedback([0.0, 0.0, math.nan, 0.0, 0.0, 0.0], evidence, now_s=10.0)


def test_communication_failure_rejected():
    evidence = PiperXFeedbackEvidence(
        connected=True,
        feedback_valid=True,
        source_update_counter=1,
        source_timestamp_s=10.0,
        source_can_ids=list(PIPER_X_FEEDBACK_CAN_IDS),
        source_frame_receive_counters={can_id: index + 1 for index, can_id in enumerate(PIPER_X_FEEDBACK_CAN_IDS)},
        source_frame_timestamps_s={can_id: 10.0 for can_id in PIPER_X_FEEDBACK_CAN_IDS},
        raw_joint_values=[0] * 6,
        real_feedback_packet=True,
        complete_frame_set=True,
        communication_ready=False,
    )
    with pytest.raises(ValueError, match="communication-ready"):
        validate_piper_x_feedback([0.0] * 6, evidence, now_s=10.0)


def test_all_zero_data_requires_complete_current_frame_set():
    evidence = PiperXFeedbackEvidence(
        connected=True,
        feedback_valid=True,
        source_update_counter=1,
        source_timestamp_s=10.0,
        source_can_ids=list(PIPER_X_FEEDBACK_CAN_IDS),
        source_frame_receive_counters={can_id: index + 1 for index, can_id in enumerate(PIPER_X_FEEDBACK_CAN_IDS)},
        source_frame_timestamps_s={can_id: 10.0 for can_id in PIPER_X_FEEDBACK_CAN_IDS},
        raw_joint_values=[],
        real_feedback_packet=True,
        complete_frame_set=False,
        communication_ready=True,
    )
    with pytest.raises(ValueError, match="complete real packet set"):
        validate_piper_x_feedback([0.0] * 6, evidence, now_s=10.0)


def test_passive_decoder_accepts_genuine_zero_complete_frame_set():
    decoder = PassivePiperXSocketcanDecoder(config=PiperXFeedbackConfig(max_age_s=1.0))
    decoder.update_from_can(0x2A5, _frame(0, 0), 10.00)
    decoder.update_from_can(0x2A6, _frame(0, 0), 10.01)
    decoder.update_from_can(0x2A7, _frame(0, 0), 10.02)
    assert decoder.sample(now_s=10.03).positions_rad == [0.0] * 6


def test_dependency_commit_mismatch_rejected_by_pose_gate():
    cfg = load_touch_config(CONFIG)
    metadata = {
        "feedback_source_id": cfg.required_feedback_source_id,
        "joint_mapping_version": cfg.required_joint_mapping_version,
        "dependency_commit": "wrong",
    }
    poses = {
        "staging": TaughtPose("staging", EXPECTED_JOINT_NAMES, [0.0, -0.04, -0.07, 0.0, 0.04, 0.0], "old", metadata),
        "pre_touch": TaughtPose("pre_touch", EXPECTED_JOINT_NAMES, [0.01, -0.04, -0.07, 0.0, 0.04, 0.0], "old", metadata),
        "touch": TaughtPose("touch", EXPECTED_JOINT_NAMES, [0.02, -0.04, -0.07, 0.0, 0.04, 0.0], "old", metadata),
        "retract": TaughtPose("retract", EXPECTED_JOINT_NAMES, [0.0, -0.04, -0.07, 0.0, 0.04, 0.0], "old", metadata),
    }
    result = MoveItArucoTouchController(cfg, MockMoveItTouchBackend(), taught_poses=poses).run(planning_only=True)
    assert not result.success
    assert "feedback-source mismatch" in result.failure_reason


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
    validate_single_joint_state_authority(["/piper_x_passive_socketcan_joint_state_bridge"])
