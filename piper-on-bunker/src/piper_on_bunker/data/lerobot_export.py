from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Callable, Dict, Optional

import numpy as np
from PIL import Image

from .action_schema import ACTION_FEATURE_KEY, CAMERA_FEATURE_KEY, STATE_FEATURE_KEY
from .dataset_validator import summarize_dataset
from .observation_schema import build_lerobot_feature_spec


def build_lerobot_features(image_height: int, image_width: int, use_videos: bool = False) -> Dict[str, dict]:
    return build_lerobot_feature_spec(image_height=image_height, image_width=image_width, use_videos=use_videos)


def _load_lerobot_dataset_class():
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except Exception as exc:
        raise RuntimeError(
            "LeRobot is not available. Install the official LeRobot environment or set PYTHONPATH to an official checkout."
        ) from exc
    return LeRobotDataset


def export_raw_dataset_to_lerobot(
    raw_root: str | Path,
    export_root: str | Path,
    repo_id: str = "local/piper_xvla_target_v0",
    fps: int = 5,
    use_videos: bool = False,
    dataset_factory: Optional[Callable[..., Any]] = None,
) -> Dict[str, Any]:
    raw_root = Path(raw_root)
    export_root = Path(export_root)
    summary = summarize_dataset(raw_root)
    if summary.invalid_episodes:
        raise ValueError("raw dataset contains invalid episodes: " + "; ".join(summary.issues))
    episode_dirs = sorted(path for path in raw_root.glob("episode_*") if path.is_dir())
    if not episode_dirs:
        raise ValueError("no episodes found in raw dataset")
    first_episode = json.loads((episode_dirs[0] / "episode.json").read_text(encoding="utf-8"))
    first_frame = first_episode["frames"][0]
    first_image = np.asarray(Image.open(episode_dirs[0] / first_frame["camera_path"]))
    features = build_lerobot_features(first_image.shape[0], first_image.shape[1], use_videos=use_videos)
    if export_root.exists():
        raise ValueError(f"export root already exists: {export_root}")
    dataset_cls = dataset_factory or _load_lerobot_dataset_class()
    dataset = dataset_cls.create(
        repo_id=repo_id,
        fps=fps,
        features=features,
        root=export_root,
        use_videos=use_videos,
    )
    for episode_dir in episode_dirs:
        payload = json.loads((episode_dir / "episode.json").read_text(encoding="utf-8"))
        for frame in payload["frames"]:
            image = np.asarray(Image.open(episode_dir / frame["camera_path"]))
            state = np.asarray(frame["state"]["joint_positions_rad"] + [frame["state"]["gripper_value"]], dtype=np.float32)
            action = np.asarray(
                frame["command"]["target_joint_positions_rad"] + [frame["command"]["target_gripper_value"]],
                dtype=np.float32,
            )
            dataset.add_frame(
                {
                    CAMERA_FEATURE_KEY: image,
                    STATE_FEATURE_KEY: state,
                    ACTION_FEATURE_KEY: action,
                    "task": frame["task_instruction"],
                }
            )
        dataset.save_episode()
    if hasattr(dataset, "finalize"):
        dataset.finalize()
    return {
        "request_success": True,
        "repo_id": repo_id,
        "export_root": str(export_root),
        "episodes": len(episode_dirs),
        "frames": summary.total_frames,
        "features": features,
        "use_videos": use_videos,
    }


def validate_lerobot_dataset(root: str | Path) -> Dict[str, Any]:
    dataset_cls = _load_lerobot_dataset_class()
    dataset = dataset_cls(repo_id="local/piper_xvla_target_v0", root=root, download_videos=False)
    return {
        "request_success": True,
        "root": str(root),
        "num_episodes": int(dataset.num_episodes),
        "num_frames": int(dataset.num_frames),
        "feature_keys": sorted(dataset.features.keys()),
        "stats_keys": sorted((dataset.meta.stats or {}).keys()),
    }
