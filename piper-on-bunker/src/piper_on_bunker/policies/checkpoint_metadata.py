from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import json

from piper_on_bunker.policies.openpi_piper_policy import OPENPI_COMMIT
from piper_on_bunker.policies.openpi_piper_policy import PIPER_ACTION_SEMANTICS
from piper_on_bunker.policies.openpi_piper_policy import PIPER_JOINT_NAMES


REQUIRED_CAMERA_KEYS = ("observation/exterior_image", "observation/wrist_image")


@dataclass(frozen=True)
class CheckpointEligibility:
    eligible: bool
    failures: tuple[str, ...]
    metadata: dict[str, Any]

    def require(self) -> None:
        if not self.eligible:
            raise ValueError("checkpoint is not eligible for PiPER physical execution: " + "; ".join(self.failures))


def load_checkpoint_metadata(path: str | Path) -> dict[str, Any]:
    metadata_path = Path(path)
    if metadata_path.is_dir():
        metadata_path = metadata_path / "piper_checkpoint_metadata.json"
    return json.loads(metadata_path.read_text(encoding="utf-8"))


def validate_checkpoint_metadata(metadata: dict[str, Any], *, require_gripper: bool = True) -> CheckpointEligibility:
    failures: list[str] = []

    def require(condition: bool, message: str) -> None:
        if not condition:
            failures.append(message)

    require(metadata.get("piper_compatible") is True, "piper_compatible must be true")
    require(metadata.get("openpi_commit") == OPENPI_COMMIT, f"openpi_commit must be {OPENPI_COMMIT}")
    require(metadata.get("action_semantics") == PIPER_ACTION_SEMANTICS, "action_semantics must be absolute PiPER joints")
    require(tuple(metadata.get("joint_names", ())) == PIPER_JOINT_NAMES, "joint_names must match PiPER joint order")
    require(metadata.get("action_dim") == 7, "action_dim must be 7")
    require(int(metadata.get("action_horizon", 0)) >= 2, "action_horizon must be at least 2")
    require(float(metadata.get("control_frequency_hz", 0.0)) > 0.0, "control_frequency_hz must be positive")

    units = metadata.get("units") or {}
    require(units.get("arm") == "rad", "arm units must be radians")
    require(units.get("gripper") == "total_jaw_opening_m", "gripper units must be total_jaw_opening_m")

    normalization = metadata.get("normalization_metadata") or {}
    require(bool(normalization.get("asset_id")), "normalization_metadata.asset_id is required")
    require(bool(normalization.get("statistics_hash")), "normalization_metadata.statistics_hash is required")

    dataset = metadata.get("dataset_provenance") or {}
    require(bool(dataset.get("dataset_id") or dataset.get("repo_id") or dataset.get("path")), "dataset provenance is required")
    require(int(dataset.get("episode_count", 0)) > 0, "dataset episode_count must be positive")
    require(bool(dataset.get("dataset_hash")), "dataset_hash is required")

    camera_schema = metadata.get("camera_schema") or {}
    require(all(key in camera_schema for key in REQUIRED_CAMERA_KEYS), "camera schema must include exterior and wrist keys")

    validation = metadata.get("offline_validation") or {}
    require(validation.get("passed") is True, "offline_validation.passed must be true")
    require(bool(validation.get("report_hash")), "offline_validation.report_hash is required")

    if require_gripper:
        gripper = metadata.get("gripper_hardware_verification") or {}
        require(gripper.get("passed") is True, "gripper hardware verification must pass")
        require(bool(gripper.get("report_hash")), "gripper hardware verification report_hash is required")

    return CheckpointEligibility(eligible=not failures, failures=tuple(failures), metadata=dict(metadata))
