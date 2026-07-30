"""Robot-profile checkpoint metadata and physical eligibility checks."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence


ROBOT_CHECKPOINT_SCHEMA_V2 = "openpi_robot_checkpoint.v2"


@dataclass(frozen=True)
class EligibilityResult:
    eligible: bool
    failures: tuple[str, ...]
    metadata: Mapping[str, Any]


def load_checkpoint_metadata(path: str | Path) -> Mapping[str, Any]:
    with Path(path).open("r", encoding="utf-8") as f:
        return json.load(f)


def validate_robot_profile_checkpoint(
    metadata: Mapping[str, Any],
    *,
    expected_robot_profile_id: str,
    expected_robot_model: str,
    expected_task_id: str,
    expected_joint_order: Sequence[str],
    expected_action_semantics: str,
    expected_state_units: Mapping[str, Any],
    expected_action_units: Mapping[str, Any],
    expected_control_frequency_hz: float,
    expected_camera_schema: Mapping[str, Any],
    expected_camera_mount_id: str,
    expected_image_preprocessing_id: str,
    expected_gripper_mode: str,
    expected_normalization_asset_id: str,
    expected_openpi_commit: str,
) -> EligibilityResult:
    failures: list[str] = []

    if metadata.get("schema_version") != ROBOT_CHECKPOINT_SCHEMA_V2:
        if "piper_compatible" in metadata:
            failures.append("legacy PiPER checkpoint metadata cannot authorize robot-profile execution")
        else:
            failures.append(f"schema_version must be {ROBOT_CHECKPOINT_SCHEMA_V2}")
        return EligibilityResult(False, tuple(failures), metadata)

    exact_checks = {
        "robot_profile_id": expected_robot_profile_id,
        "robot_model": expected_robot_model,
        "task_id": expected_task_id,
        "action_semantics": expected_action_semantics,
        "camera_mount_id": expected_camera_mount_id,
        "image_preprocessing_id": expected_image_preprocessing_id,
        "gripper_mode": expected_gripper_mode,
        "normalization_asset_id": expected_normalization_asset_id,
        "openpi_commit": expected_openpi_commit,
    }
    for key, expected in exact_checks.items():
        if metadata.get(key) != expected:
            failures.append(f"{key} must match profile: expected {expected!r}, got {metadata.get(key)!r}")

    if metadata.get("action_dim") != 7:
        failures.append("action_dim must be 7")
    if tuple(metadata.get("joint_order", ())) != tuple(expected_joint_order):
        failures.append("joint_order must match profile exactly")
    if metadata.get("state_units") != dict(expected_state_units):
        failures.append("state_units must match profile exactly")
    if metadata.get("action_units") != dict(expected_action_units):
        failures.append("action_units must match profile exactly")
    if float(metadata.get("control_frequency_hz", -1.0)) != float(expected_control_frequency_hz):
        failures.append("control_frequency_hz must match profile exactly")
    if metadata.get("camera_schema") != dict(expected_camera_schema):
        failures.append("camera_schema must match profile exactly")

    normalization = metadata.get("normalization", {})
    if not isinstance(normalization, Mapping) or not normalization.get("hash"):
        failures.append("normalization hash is required")
    if not metadata.get("dataset_id") or not metadata.get("dataset_hash"):
        failures.append("dataset provenance is required")
    if not metadata.get("dataset_manifest_hash"):
        failures.append("dataset manifest hash is required")
    offline = metadata.get("offline_validation", {})
    if not isinstance(offline, Mapping) or offline.get("passed") is not True:
        failures.append("offline_validation.passed must be true")
    hardware = metadata.get("hardware_verification", {})
    if not isinstance(hardware, Mapping) or hardware.get("passed") is not True:
        failures.append("hardware_verification.passed must be true")
    if metadata.get("physical_execution_allowed") is not True:
        failures.append("physical_execution_allowed must be true")
    if metadata.get("piper_x_compatible") is not True:
        failures.append("piper_x_compatible must be true for PiPER-X")

    return EligibilityResult(not failures, tuple(failures), metadata)
