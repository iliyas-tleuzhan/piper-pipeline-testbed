import math

import pytest

from piper_on_bunker.data.episode_collection import PiperCanCommandDecoder
from piper_on_bunker.data.episode_collection import convert_gripper_raw_to_m
from piper_on_bunker.data.episode_collection import piper_raw_joints_to_rad


class Msg:
    def __init__(self, arbitration_id, data):
        self.arbitration_id = arbitration_id
        self.data = data


def _i32(value):
    return int(value).to_bytes(4, byteorder="big", signed=True)


def test_raw_piper_joint_units_convert_to_radians():
    got = piper_raw_joints_to_rad([1000, 0, -1000, 90000, -90000, 180000])
    assert got[0] == pytest.approx(math.radians(1.0))
    assert got[2] == pytest.approx(math.radians(-1.0))


def test_can_decoder_emits_action_after_complete_joint_target():
    decoder = PiperCanCommandDecoder()
    assert decoder.decode(0x155, _i32(1000) + _i32(2000), stamp_s=1.0, current_gripper_m=0.01) is None
    assert decoder.decode(0x156, _i32(-3000) + _i32(4000), stamp_s=1.0, current_gripper_m=0.01) is None
    sample = decoder.decode(0x157, _i32(5000) + _i32(-6000), stamp_s=1.0, current_gripper_m=0.01)
    assert sample is not None
    assert sample.source == "socketcan_piper_command"
    assert sample.arm_action_rad[0] == pytest.approx(math.radians(1.0))
    assert sample.gripper_action_m == pytest.approx(0.01)
    assert sample.gripper_action_source == "current_feedback_hold"
    assert decoder.decode(0x155, _i32(7000) + _i32(8000), stamp_s=1.0, current_gripper_m=0.01) is None


def test_can_decoder_uses_calibrated_gripper_raw_when_available():
    decoder = PiperCanCommandDecoder(gripper_raw_closed=0, gripper_raw_open=60000)
    decoder.decode(0x159, _i32(30000) + int(1000).to_bytes(2, "big") + bytes([1, 0]), stamp_s=1, current_gripper_m=0)
    decoder.decode(0x155, _i32(0) + _i32(0), stamp_s=1, current_gripper_m=0)
    decoder.decode(0x156, _i32(0) + _i32(0), stamp_s=1, current_gripper_m=0)
    sample = decoder.decode(0x157, _i32(0) + _i32(0), stamp_s=1, current_gripper_m=0)
    assert sample.gripper_action_m == pytest.approx(0.03)
    assert sample.gripper_action_source == "can_raw_linear_calibration"


def test_gripper_raw_conversion_rejects_degenerate_calibration():
    with pytest.raises(ValueError, match="must differ"):
        convert_gripper_raw_to_m(0, raw_closed=1, raw_open=1)
