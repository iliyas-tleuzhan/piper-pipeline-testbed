from __future__ import annotations

from pathlib import Path

from piper_on_bunker.configuration import PipelineConfig, load_config
from piper_on_bunker.hardware.abotclaw_api_arm import ABotClawApiArm
from piper_on_bunker.hardware.mock_arm import MockArm
from piper_on_bunker.hardware.mock_base import MockBase
from piper_on_bunker.hardware.piper_ros_arm import PiperRosArm
from piper_on_bunker.hardware.replay_arm import ReplayArm
from piper_on_bunker.mission_supervisor import MissionSupervisor
from piper_on_bunker.perception.mock_camera import MockCamera
from piper_on_bunker.perception.replay_camera import ReplayCamera


def build_supervisor(config: PipelineConfig) -> MissionSupervisor:
    if config.arm_adapter == "mock":
        arm = MockArm(config.named_poses)
    elif config.arm_adapter == "replay":
        arm = ReplayArm(config.replay_fixture or "piper-on-bunker/fixtures/mock/synthetic_button_fixture.json")
    elif config.arm_adapter == "abotclaw_api":
        arm = ABotClawApiArm(config.action_server_url, dry_run=not config.physical_motion_enabled)
    elif config.arm_adapter == "piper_ros":
        arm = PiperRosArm(config.physical_motion_enabled)
    else:
        raise ValueError(f"unknown arm adapter: {config.arm_adapter}")

    if config.camera_adapter == "mock":
        camera = MockCamera()
    elif config.camera_adapter == "replay":
        camera = ReplayCamera(config.replay_fixture or "piper-on-bunker/fixtures/mock/synthetic_button_fixture.json")
    elif config.camera_adapter == "external_realsense":
        from piper_on_bunker.perception.external_realsense import ExternalFixedCamera

        camera = ExternalFixedCamera()
    else:
        raise ValueError(f"unknown camera adapter: {config.camera_adapter}")
    return MissionSupervisor(arm=arm, camera=camera, base=MockBase())


def build_supervisor_from_path(path: str | Path) -> MissionSupervisor:
    return build_supervisor(load_config(path))
