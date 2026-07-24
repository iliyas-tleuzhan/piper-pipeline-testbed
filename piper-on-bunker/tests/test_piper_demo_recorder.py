import pytest

from piper_on_bunker.configuration import PipelineConfig
from piper_on_bunker.data.action_schema import GRIPPER_UNIT_SOURCE, GRIPPER_UNITS
from piper_on_bunker.data.piper_demo_recorder import (
    LIVE_DEMO_MODE,
    PiperDemoRecorder,
    build_target_from_command,
    parse_manual_command,
    require_interactive_live_demo_session,
)


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


def test_record_command_rejects_dry_run_config():
    recorder = PiperDemoRecorder.__new__(PiperDemoRecorder)
    recorder.config = PipelineConfig(physical_motion_enabled=False)
    with pytest.raises(RuntimeError):
        recorder.record_command(None, "j1+")


def test_live_demo_collection_requires_interactive_terminal(monkeypatch):
    config = PipelineConfig(mode=LIVE_DEMO_MODE, configured_physical_motion_enabled=True, physical_motion_enabled=True)
    monkeypatch.setattr("piper_on_bunker.data.piper_demo_recorder.os.isatty", lambda _fd: False)
    with pytest.raises(RuntimeError, match="interactive terminal"):
        require_interactive_live_demo_session(config, "piper-on-bunker/config/piper_laptop_demo_collection.yaml")


def test_live_demo_collection_requires_local_activation(monkeypatch):
    config = PipelineConfig(mode=LIVE_DEMO_MODE, configured_physical_motion_enabled=True, physical_motion_enabled=False)
    monkeypatch.setattr("piper_on_bunker.data.piper_demo_recorder.os.isatty", lambda _fd: True)
    with pytest.raises(RuntimeError, match="local activation"):
        require_interactive_live_demo_session(config, "piper-on-bunker/config/piper_laptop_demo_collection.yaml")
