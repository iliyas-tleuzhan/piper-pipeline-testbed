"""PiPER-X ArUco episode storage, inspection, and conversion helpers."""

from __future__ import annotations

import json
import random
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from piper_on_bunker.perception.aruco_touch_diagnostics import marker_visibility_summary
from piper_on_bunker.profiles.piper_x_aruco import (
    PIPER_X_ACTION_SEMANTICS,
    PIPER_X_JOINT_ORDER,
    PIPER_X_PROFILE_ID,
    PIPER_X_TASK_ID,
    stable_json_hash,
    validate_fixed_gripper_channel,
)


FRAME_FILE = "frames.jsonl"
SUMMARY_FILE = "summary.json"
OUTCOME_FILE = "outcome.json"
INCOMPLETE_MARKER = ".incomplete"


@dataclass(frozen=True)
class EpisodeInspection:
    episode_dir: Path
    passed: bool
    blockers: tuple[str, ...]
    report: Mapping[str, Any]


def read_json(path: str | Path) -> Any:
    with Path(path).open("r", encoding="utf-8") as f:
        return json.load(f)


def write_json_atomic(path: str | Path, data: Mapping[str, Any]) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = output_path.with_suffix(output_path.suffix + ".tmp")
    with tmp_path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write("\n")
    tmp_path.replace(output_path)


def iter_frames(episode_dir: str | Path) -> list[dict[str, Any]]:
    path = Path(episode_dir) / FRAME_FILE
    frames: list[dict[str, Any]] = []
    if not path.exists():
        return frames
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                frames.append(json.loads(line))
    return frames


def append_frame(episode_dir: str | Path, frame: Mapping[str, Any]) -> None:
    with (Path(episode_dir) / FRAME_FILE).open("a", encoding="utf-8") as f:
        f.write(json.dumps(frame, sort_keys=True, separators=(",", ":")) + "\n")


def label_episode_outcome(
    episode_dir: str | Path,
    *,
    status: str,
    contact_confirmed: bool | None = None,
    marker_touched: bool | None = None,
    retraction_completed: bool | None = None,
    failure_reason: str | None = None,
    operator_notes: str | None = None,
) -> dict[str, Any]:
    if status not in {"success", "failure", "aborted"}:
        raise ValueError("status must be success, failure, or aborted")
    outcome = {
        "schema_version": "piper_x_aruco_episode_outcome.v1",
        "status": status,
        "contact_confirmed": contact_confirmed,
        "marker_touched": marker_touched,
        "retraction_completed": retraction_completed,
        "failure_reason": failure_reason,
        "operator_notes": operator_notes,
        "contact_evidence_source": "operator_confirmation",
    }
    write_json_atomic(Path(episode_dir) / OUTCOME_FILE, outcome)
    return outcome


def inspect_episode(episode_dir: str | Path, *, expected_profile: Mapping[str, Any]) -> EpisodeInspection:
    root = Path(episode_dir)
    blockers: list[str] = []
    summary = read_json(root / SUMMARY_FILE) if (root / SUMMARY_FILE).exists() else {}
    metadata = read_json(root / "episode_metadata.json") if (root / "episode_metadata.json").exists() else {}
    outcome = read_json(root / OUTCOME_FILE) if (root / OUTCOME_FILE).exists() else None
    frames = iter_frames(root)

    if (root / INCOMPLETE_MARKER).exists():
        blockers.append("episode is marked incomplete")
    if metadata.get("robot_profile_id") != PIPER_X_PROFILE_ID:
        blockers.append("wrong robot_profile_id")
    if metadata.get("task_id") != PIPER_X_TASK_ID:
        blockers.append("wrong task_id")
    if metadata.get("camera_schema", {}).get("observation/wrist_image") != "real_wrist_camera":
        blockers.append("wrist-only camera schema missing")
    if metadata.get("camera_schema", {}).get("observation/exterior_image") != "zero_image_placeholder":
        blockers.append("exterior zero-image placeholder metadata missing")
    if tuple(metadata.get("joint_order", ())) != PIPER_X_JOINT_ORDER:
        blockers.append("joint order mismatch")
    if metadata.get("action_semantics") != PIPER_X_ACTION_SEMANTICS:
        blockers.append("action semantics mismatch")
    if not frames:
        blockers.append("no frames recorded")

    timestamps = [float(frame.get("sample_time_s", -1.0)) for frame in frames]
    if any(b <= a for a, b in zip(timestamps, timestamps[1:])):
        blockers.append("sample timestamps are not strictly monotonic")

    actions: list[list[float]] = []
    max_skew = float(expected_profile.get("max_timestamp_skew_s", 0.05))
    invalid_skews = 0
    for index, frame in enumerate(frames):
        state = np.asarray(frame.get("state", []), dtype=float)
        action = np.asarray(frame.get("action", []), dtype=float)
        if state.shape != (7,):
            blockers.append(f"frame {index} state shape is {state.shape}")
        if action.shape != (7,):
            blockers.append(f"frame {index} action shape is {action.shape}")
        if state.shape == (7,) and not np.all(np.isfinite(state)):
            blockers.append(f"frame {index} state contains non-finite values")
        if action.shape == (7,) and not np.all(np.isfinite(action)):
            blockers.append(f"frame {index} action contains non-finite values")
        if action.shape == (7,):
            actions.append(action.tolist())
        if frame.get("wrist_image_path") is None:
            blockers.append(f"frame {index} missing wrist image path")
        if frame.get("exterior_image_path") is None:
            blockers.append(f"frame {index} missing zero exterior image path")
        if float(frame.get("max_source_skew_s", 999.0)) > max_skew:
            invalid_skews += 1

    if invalid_skews:
        blockers.append(f"{invalid_skews} frames exceed max timestamp skew")

    gripper = expected_profile.get("gripper", {})
    fixed_target = metadata.get("fixed_gripper_target")
    if gripper.get("mode") == "fixed_hold" and actions and fixed_target is not None:
        try:
            validate_fixed_gripper_channel(
                np.asarray(actions), float(fixed_target), float(gripper.get("fixed_target_tolerance", 1e-6))
            )
        except ValueError as exc:
            blockers.append(str(exc))
    elif gripper.get("mode") == "fixed_hold":
        blockers.append("fixed gripper target missing")

    if outcome is None:
        blockers.append("episode outcome label missing")

    visibility = marker_visibility_summary(frames, int(expected_profile.get("marker", {}).get("id", 6)))
    if not visibility["marker_visible_at_start"]:
        blockers.append("marker not visible at episode start")
    if visibility["wrong_marker_detections"]:
        blockers.append("wrong marker ID detected")

    report = {
        "episode_dir": str(root),
        "summary": summary,
        "metadata": metadata,
        "outcome": outcome,
        "frame_count": len(frames),
        "marker_visibility": visibility,
        "blockers": blockers,
        "passed": not blockers,
    }
    return EpisodeInspection(root, not blockers, tuple(blockers), report)


def write_episode_splits(
    dataset_root: str | Path,
    output_json: str | Path,
    *,
    seed: int,
    train_fraction: float = 0.8,
    val_fraction: float = 0.1,
) -> dict[str, Any]:
    root = Path(dataset_root)
    episodes = sorted(str(path) for path in root.iterdir() if path.is_dir() and (path / SUMMARY_FILE).exists())
    rng = random.Random(seed)
    shuffled = list(episodes)
    rng.shuffle(shuffled)
    n = len(shuffled)
    n_train = int(round(n * train_fraction))
    n_val = int(round(n * val_fraction))
    splits = {
        "schema_version": "piper_x_aruco_episode_splits.v1",
        "seed": seed,
        "train": shuffled[:n_train],
        "validation": shuffled[n_train : n_train + n_val],
        "test": shuffled[n_train + n_val :],
    }
    write_json_atomic(output_json, splits)
    return splits


def convert_to_lerobot_scaffold(
    dataset_root: str | Path,
    output_dir: str | Path,
    *,
    profile: Mapping[str, Any],
) -> dict[str, Any]:
    """Create a compact OpenPI/LeRobot-compatible scaffold from raw episodes.

    This intentionally writes metadata and row records only. It does not compute
    normalization statistics and it does not start training.
    """

    root = Path(dataset_root)
    out = Path(output_dir)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    rows: list[dict[str, Any]] = []
    episodes: list[dict[str, Any]] = []
    for episode_index, episode_dir in enumerate(sorted(p for p in root.iterdir() if p.is_dir())):
        if not (episode_dir / SUMMARY_FILE).exists():
            continue
        start_row = len(rows)
        frames = iter_frames(episode_dir)
        for frame_index, frame in enumerate(frames):
            rows.append(
                {
                    "episode_index": episode_index,
                    "frame_index": frame_index,
                    "timestamp": frame.get("sample_time_s"),
                    "observation/wrist_image": frame.get("wrist_image_path"),
                    "observation/exterior_image": frame.get("exterior_image_path"),
                    "observation/state": frame.get("state"),
                    "action": frame.get("action"),
                    "prompt": frame.get("prompt"),
                    "task_id": PIPER_X_TASK_ID,
                }
            )
        episodes.append(
            {
                "episode_index": episode_index,
                "episode_dir": str(episode_dir),
                "start_row": start_row,
                "length": len(frames),
            }
        )

    features = {
        "schema_version": "openpi_lerobot_scaffold.v1",
        "robot_profile_id": PIPER_X_PROFILE_ID,
        "task_id": PIPER_X_TASK_ID,
        "features": {
            "observation/wrist_image": {"dtype": "image", "shape": [224, 224, 3]},
            "observation/exterior_image": {"dtype": "image", "shape": [224, 224, 3], "placeholder": "zero"},
            "observation/state": {"dtype": "float32", "shape": [7]},
            "action": {"dtype": "float32", "shape": [7], "semantics": PIPER_X_ACTION_SEMANTICS},
            "prompt": {"dtype": "string"},
        },
        "image_preprocessing_id": profile.get("image_preprocessing", {}).get("id"),
        "normalization_asset_id": profile.get("normalization_asset_id"),
    }
    write_json_atomic(out / "features.json", features)
    write_json_atomic(out / "episodes.json", {"episodes": episodes})
    write_json_atomic(out / "data.json", {"rows": rows})
    manifest = {
        "schema_version": "piper_x_converted_dataset_manifest.v1",
        "dataset_root": str(root),
        "row_count": len(rows),
        "episode_count": len(episodes),
        "features_hash": stable_json_hash(features),
        "normalization_asset_id": profile.get("normalization_asset_id"),
    }
    write_json_atomic(out / "dataset_manifest.json", manifest)
    return manifest
