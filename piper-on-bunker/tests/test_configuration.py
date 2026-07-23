from pathlib import Path

import pytest

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
