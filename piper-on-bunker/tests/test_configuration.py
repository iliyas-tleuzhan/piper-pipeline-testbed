from pathlib import Path

import pytest
import yaml

from piper_on_bunker.configuration import PipelineConfig, load_config


def test_development_config_loads():
    cfg = load_config(Path("piper-on-bunker/config/development_mock.yaml"))
    assert cfg.mode == "mock"
    assert not cfg.physical_motion_enabled


def test_hardware_disabled_by_default():
    PipelineConfig(mode="piper_laptop_hardware", physical_motion_enabled=False).validate()


def test_hardware_motion_requires_local_activation():
    with pytest.raises(ValueError):
        PipelineConfig(mode="piper_laptop_hardware", physical_motion_enabled=True).validate()


def test_hardware_and_dry_run_use_piper_ros_backend():
    hardware = yaml.safe_load(Path("piper-on-bunker/config/piper_laptop_hardware.yaml").read_text())
    dry_run = yaml.safe_load(Path("piper-on-bunker/config/piper_laptop_dry_run.yaml").read_text())
    readonly = yaml.safe_load(Path("piper-on-bunker/config/piper_laptop_readonly_8891.yaml").read_text())
    demo = yaml.safe_load(Path("piper-on-bunker/config/piper_laptop_demo_collection.yaml").read_text())
    assert hardware["arm_adapter"] == "piper_ros"
    assert dry_run["arm_adapter"] == "piper_ros"
    assert demo["arm_adapter"] == "piper_ros"
    assert not dry_run["physical_motion_enabled"]
    assert readonly["arm_adapter"] == "abotclaw_api"


def test_demo_collection_config_requires_local_activation_to_be_effective():
    cfg = load_config(Path("piper-on-bunker/config/piper_laptop_demo_collection.yaml"))
    assert cfg.mode == "piper_demo_collection"
    assert cfg.configured_physical_motion_enabled is True
    assert cfg.physical_motion_enabled is False
