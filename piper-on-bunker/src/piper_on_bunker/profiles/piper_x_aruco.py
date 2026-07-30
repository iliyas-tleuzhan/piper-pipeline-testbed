"""PiPER-X wrist-camera ArUco-touch profile utilities.

This module intentionally contains no robot-commanding code. It defines the
read-only collection contract and fail-closed compatibility checks for the
PiPER-X ArUco touch dataset.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from piper_on_bunker.data.image_transforms import OPENPI_PADDED_RGB_224_V1, resize_with_pad_rgb


PIPER_X_PROFILE_ID = "agilex_piper_x_single_arm_wrist_aruco_v1"
PIPER_X_ROBOT_MODEL = "agilex_piper_x"
PIPER_X_TASK_ID = "touch_aruco_marker_vertical_v1"
PIPER_X_DATASET_ROBOT_TYPE = "piper_x_single_arm"
PIPER_X_NORMALIZATION_ASSET_ID = "piper_x_touch_aruco_v1"
PIPER_X_ACTION_SEMANTICS = "absolute_piper_x_joint_targets"
PIPER_X_JOINT_ORDER = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper")
PIPER_X_DEFAULT_PROMPT = "Touch the center of the ArUco marker and retract."


@dataclass(frozen=True)
class PiperXProfile:
    path: Path
    raw: Mapping[str, Any]

    @property
    def robot_profile_id(self) -> str:
        return str(self.raw["robot_profile_id"])

    @property
    def robot_model(self) -> str:
        return str(self.raw["robot_model"])

    @property
    def task_id(self) -> str:
        return str(self.raw["task_id"])

    @property
    def joint_order(self) -> tuple[str, ...]:
        return tuple(self.raw["joint_order"])

    @property
    def action_semantics(self) -> str:
        return str(self.raw["action_semantics"])

    @property
    def control_frequency_hz(self) -> float:
        return float(self.raw["control_frequency_hz"])

    @property
    def preprocessing_id(self) -> str:
        return str(self.raw["image_preprocessing"]["id"])

    @property
    def camera_mount_id(self) -> str:
        return str(self.raw["camera"]["mount_id"])

    @property
    def gripper_mode(self) -> str:
        return str(self.raw["gripper"]["mode"])

    @property
    def max_timestamp_skew_s(self) -> float:
        return float(self.raw["max_timestamp_skew_s"])


def load_piper_x_profile(path: str | Path) -> PiperXProfile:
    import yaml

    profile_path = Path(path)
    with profile_path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    validate_profile_dict(raw)
    return PiperXProfile(path=profile_path, raw=raw)


def validate_profile_dict(raw: Mapping[str, Any]) -> None:
    required = {
        "robot_profile_id": PIPER_X_PROFILE_ID,
        "robot_model": PIPER_X_ROBOT_MODEL,
        "task_id": PIPER_X_TASK_ID,
        "dataset_robot_type": PIPER_X_DATASET_ROBOT_TYPE,
        "normalization_asset_id": PIPER_X_NORMALIZATION_ASSET_ID,
        "action_semantics": PIPER_X_ACTION_SEMANTICS,
    }
    for key, expected in required.items():
        if raw.get(key) != expected:
            raise ValueError(f"{key} must be {expected!r}")
    if tuple(raw.get("joint_order", ())) != PIPER_X_JOINT_ORDER:
        raise ValueError("PiPER-X joint_order mismatch")
    if raw.get("joint_limits", {}).get("status") != "unresolved":
        raise ValueError("PiPER-X joint limits must remain unresolved until measured")
    if raw.get("gripper", {}).get("mode") != "fixed_hold":
        raise ValueError("PiPER-X ArUco profile requires gripper fixed_hold mode")
    if raw.get("image_preprocessing", {}).get("id") != OPENPI_PADDED_RGB_224_V1:
        raise ValueError("unexpected image preprocessing ID")


def make_wrist_only_openpi_observation(
    *,
    wrist_image_rgb: np.ndarray,
    state: Sequence[float],
    prompt: str,
    output_size: int = 224,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Build the PiPER-X OpenPI observation with a real wrist image only."""

    wrist = resize_with_pad_rgb(wrist_image_rgb, size=output_size).image
    exterior = np.zeros_like(wrist, dtype=np.uint8)
    state_array = np.asarray(state, dtype=np.float32)
    if state_array.shape != (7,):
        raise ValueError(f"PiPER-X state must be 7D, got {state_array.shape}")
    metadata = {
        "camera_schema": {
            "observation/wrist_image": "real_wrist_camera",
            "observation/exterior_image": "zero_image_placeholder",
        },
        "image_preprocessing_id": OPENPI_PADDED_RGB_224_V1,
        "exterior_image_placeholder": "deterministic_zero_rgb_224",
    }
    return (
        {
            "observation/exterior_image": exterior,
            "observation/wrist_image": wrist,
            "observation/state": state_array,
            "prompt": str(prompt),
        },
        metadata,
    )


def validate_action_shape(actions: np.ndarray) -> np.ndarray:
    arr = np.asarray(actions, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 7:
        raise ValueError(f"actions must have shape [horizon, 7], got {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise ValueError("actions contain non-finite values")
    return arr


def validate_fixed_gripper_channel(actions: np.ndarray, fixed_target: float, tolerance: float) -> None:
    arr = validate_action_shape(actions)
    if tolerance < 0:
        raise ValueError("fixed gripper tolerance must be non-negative")
    max_error = float(np.max(np.abs(arr[:, 6] - float(fixed_target)))) if len(arr) else 0.0
    if max_error > tolerance:
        raise ValueError(f"fixed gripper channel varies by {max_error:.6g}, tolerance {tolerance:.6g}")


def deterministic_sample_times(start_s: float, end_s: float, fps: float) -> list[float]:
    if fps <= 0:
        raise ValueError("fps must be positive")
    if end_s < start_s:
        raise ValueError("end_s must be >= start_s")
    period = 1.0 / float(fps)
    count = int(np.floor((end_s - start_s) / period)) + 1
    return [start_s + i * period for i in range(count)]


def stable_json_hash(data: Mapping[str, Any]) -> str:
    encoded = json.dumps(data, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def file_sha256(path: str | Path) -> str:
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()
