from __future__ import annotations

from pathlib import Path

from piper_on_bunker.configuration import PipelineConfig, load_config
from piper_on_bunker.hardware.abotclaw_api_arm import ABotClawApiArm
from piper_on_bunker.hardware.mock_arm import MockArm
from piper_on_bunker.hardware.mock_base import MockBase
from piper_on_bunker.hardware.piper_ros_arm import PiperRosArm
from piper_on_bunker.hardware.replay_arm import ReplayArm
from piper_on_bunker.mission_logging import MissionLogger
from piper_on_bunker.mission_supervisor import MissionSupervisor
from piper_on_bunker.perception.mock_camera import MockCamera
from piper_on_bunker.perception.replay_camera import ReplayCamera
from piper_on_bunker.transforms import TransformResolver


def build_supervisor(config: PipelineConfig) -> MissionSupervisor:
    if config.arm_adapter == "mock":
        arm = MockArm(config.named_poses)
    elif config.arm_adapter == "replay":
        arm = ReplayArm(config.replay_fixture or "piper-on-bunker/fixtures/mock/synthetic_button_fixture.json")
    elif config.arm_adapter == "abotclaw_api":
        arm = ABotClawApiArm(config.action_server_url, dry_run=not config.physical_motion_enabled)
    elif config.arm_adapter == "piper_ros":
        arm = PiperRosArm(
            config.physical_motion_enabled,
            named_poses=config.named_poses,
            safety=config.safety,
            expected_joint_names=config.expected_joint_names,
        )
    else:
        raise ValueError(f"unknown arm adapter: {config.arm_adapter}")

    if config.camera_adapter == "mock":
        camera = MockCamera()
    elif config.camera_adapter == "replay":
        camera = ReplayCamera(config.replay_fixture or "piper-on-bunker/fixtures/mock/synthetic_button_fixture.json")
    elif config.camera_adapter == "external_realsense":
        from piper_on_bunker.perception.external_realsense import ExternalFixedCamera

        camera = ExternalFixedCamera(
            timeout_s=float(config.safety.get("max_camera_age_s", 2.0)),
            max_color_depth_delta_s=float(config.safety.get("max_color_depth_delta_s", 0.08)),
            marker_id=config.safety.get("aruco_marker_id"),
            aruco_dictionary=config.safety.get("aruco_dictionary", "DICT_4X4_50"),
        )
    else:
        raise ValueError(f"unknown camera adapter: {config.camera_adapter}")
    planning_frame = "piper_base"
    for raw in config.transforms.values():
        if isinstance(raw, dict) and raw.get("target_frame"):
            planning_frame = raw["target_frame"]
            break
    logging_cfg = config.logging or {}
    logger = MissionLogger(
        directory=logging_cfg.get("directory"),
        enabled=bool(logging_cfg.get("enabled", True)),
    )
    return MissionSupervisor(
        arm=arm,
        camera=camera,
        base=MockBase(),
        safety=config.safety,
        transform_resolver=TransformResolver(config.transforms),
        planning_frame=planning_frame,
        logger=logger,
        mode=config.mode,
        physical_motion_enabled=config.physical_motion_enabled,
    )


def build_supervisor_from_path(path: str | Path) -> MissionSupervisor:
    return build_supervisor(load_config(path))
