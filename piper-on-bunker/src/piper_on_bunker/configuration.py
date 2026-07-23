from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import yaml


@dataclass
class PipelineConfig:
    mode: str = "mock"
    physical_motion_enabled: bool = False
    arm_adapter: str = "mock"
    camera_adapter: str = "mock"
    base_adapter: str = "mock"
    action_server_url: str = "http://localhost:8891"
    named_poses: dict = field(default_factory=dict)
    safety: dict = field(default_factory=dict)
    transforms: dict = field(default_factory=dict)
    logging: dict = field(default_factory=dict)
    expected_joint_names: list = field(default_factory=list)
    local_activation: dict = field(default_factory=dict)
    replay_fixture: Optional[str] = None

    def validate(self) -> None:
        if self.mode in {"dry_run", "live_dry_run"} and self.physical_motion_enabled:
            raise ValueError("dry-run configurations may never enable physical motion")
        if self.mode in {"hardware", "piper_laptop_hardware"} and self.physical_motion_enabled:
            if self.arm_adapter != "piper_ros":
                raise ValueError("Complete hardware mission requires arm_adapter: piper_ros")
            local_enabled = bool(self.local_activation.get("physical_motion_enabled", False))
            if not local_enabled:
                raise ValueError("Physical motion requires ignored local activation")
            if self.safety.get("require_calibrated_named_poses", True):
                missing = [name for name in REQUIRED_HARDWARE_POSES if not self.named_poses.get(name)]
                if missing:
                    raise ValueError("Physical motion requires calibrated named poses: " + ", ".join(missing))
            bounds = self.safety.get("workspace_bounds_m", {})
            if not all(axis in bounds for axis in ("x", "y", "z")):
                raise ValueError("Physical motion requires configured workspace_bounds_m")
            if not self.transforms:
                raise ValueError("Physical motion requires a configured camera-to-PiPER transform")
        if self.arm_adapter in {"piper_ros", "abotclaw_api"} and self.mode == "mock":
            raise ValueError("Hardware arm adapters are not allowed in mock mode")
        if self.mode != "mock" and self.arm_adapter == "mock" and self.physical_motion_enabled:
            raise ValueError("Physical motion cannot use the mock arm adapter")


REQUIRED_HARDWARE_POSES = (
    "tabletop_home",
    "simulated_front_nav_view",
    "inspect_workspace",
    "scan_left",
    "scan_center",
    "scan_right",
    "pre_contact",
    "retracted",
    "stowed",
    "safe_recovery",
)


def _deep_merge(base: dict, overlay: dict) -> dict:
    merged = dict(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_config(path: str | Path) -> PipelineConfig:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh) or {}
    local_path = config_path.with_name(config_path.stem + ".local" + config_path.suffix)
    if local_path.exists():
        with local_path.open("r", encoding="utf-8") as fh:
            raw = _deep_merge(raw, yaml.safe_load(fh) or {})
    local_activation = raw.get("local_activation", {})
    physical_motion_enabled = bool(raw.get("physical_motion_enabled", False)) and bool(
        local_activation.get("physical_motion_enabled", False)
    )
    cfg = PipelineConfig(
        mode=raw.get("mode", "mock"),
        physical_motion_enabled=physical_motion_enabled,
        arm_adapter=raw.get("arm_adapter", "mock"),
        camera_adapter=raw.get("camera_adapter", "mock"),
        base_adapter=raw.get("base_adapter", "mock"),
        action_server_url=raw.get("action_server_url", "http://localhost:8891"),
        named_poses=raw.get("named_poses", {}),
        safety=raw.get("safety", {}),
        transforms=raw.get("transforms", {}),
        logging=raw.get("logging", {}),
        expected_joint_names=raw.get("expected_joint_names", []),
        local_activation=local_activation,
        replay_fixture=raw.get("replay_fixture"),
    )
    cfg.validate()
    return cfg
