from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class PipelineConfig:
    mode: str = "mock"
    physical_motion_enabled: bool = False
    arm_adapter: str = "mock"
    camera_adapter: str = "mock"
    base_adapter: str = "mock"
    action_server_url: str = "http://localhost:8891"
    named_poses: dict[str, Any] = field(default_factory=dict)
    safety: dict[str, Any] = field(default_factory=dict)
    replay_fixture: str | None = None

    def validate(self) -> None:
        if self.mode in {"hardware", "piper_laptop_hardware"} and not self.physical_motion_enabled:
            raise ValueError("Hardware mode requires physical_motion_enabled: true")
        if self.arm_adapter in {"piper_ros", "abotclaw_api"} and self.mode == "mock":
            raise ValueError("Hardware arm adapters are not allowed in mock mode")
        if self.mode != "mock" and self.arm_adapter == "mock" and self.physical_motion_enabled:
            raise ValueError("Physical motion cannot use the mock arm adapter")


def load_config(path: str | Path) -> PipelineConfig:
    with Path(path).open("r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh) or {}
    cfg = PipelineConfig(
        mode=raw.get("mode", "mock"),
        physical_motion_enabled=bool(raw.get("physical_motion_enabled", False)),
        arm_adapter=raw.get("arm_adapter", "mock"),
        camera_adapter=raw.get("camera_adapter", "mock"),
        base_adapter=raw.get("base_adapter", "mock"),
        action_server_url=raw.get("action_server_url", "http://localhost:8891"),
        named_poses=raw.get("named_poses", {}),
        safety=raw.get("safety", {}),
        replay_fixture=raw.get("replay_fixture"),
    )
    cfg.validate()
    return cfg
