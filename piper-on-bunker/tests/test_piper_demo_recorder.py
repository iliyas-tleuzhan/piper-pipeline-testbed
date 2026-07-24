import pytest

from piper_on_bunker.data.action_schema import GRIPPER_UNIT_SOURCE, GRIPPER_UNITS
from piper_on_bunker.data.piper_demo_recorder import build_target_from_command, parse_manual_command


def test_parse_manual_command():
    assert parse_manual_command("j3+") == ("joint3", 1)
    assert parse_manual_command("j6-") == ("joint6", -1)
    assert parse_manual_command("g+") == ("gripper", 1)


def test_parse_manual_command_rejects_invalid():
    with pytest.raises(ValueError):
        parse_manual_command("x1+")


def test_build_target_from_command_joint():
    joints, gripper = build_target_from_command([0.0] * 6, 0.01, "j2+", 0.05, 0.005)
    assert joints == [0.0, 0.05, 0.0, 0.0, 0.0, 0.0]
    assert gripper == pytest.approx(0.01)


def test_build_target_from_command_gripper_clamps():
    joints, gripper = build_target_from_command([0.0] * 6, 0.059, "g+", 0.05, 0.005)
    assert joints == [0.0] * 6
    assert gripper == pytest.approx(0.06)


def test_gripper_units_documented():
    assert GRIPPER_UNITS == "meters_opening_width"
    assert "0.06" in GRIPPER_UNIT_SOURCE
