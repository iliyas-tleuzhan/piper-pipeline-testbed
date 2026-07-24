from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import numpy as np
from PIL import Image

from .action_schema import ACTION_DIM, STATE_DIM
from .episode_writer import EpisodeWriter


@dataclass
class EpisodeValidationResult:
    episode_path: Path
    ok: bool
    issues: List[str]
    frame_count: int


@dataclass
class DatasetValidationSummary:
    root: Path
    total_episodes: int
    valid_episodes: int
    invalid_episodes: int
    total_frames: int
    issues: List[str]


def _is_finite_vector(values: List[float], expected_len: int) -> bool:
    return len(values) == expected_len and all(math.isfinite(float(value)) for value in values)


def validate_episode_directory(
    episode_dir: str | Path,
    max_state_age_s: float = 1.0,
    max_image_age_s: float = 1.0,
) -> EpisodeValidationResult:
    episode_path = Path(episode_dir)
    issues: List[str] = []
    episode_file = episode_path / "episode.json"
    if not episode_file.exists():
        return EpisodeValidationResult(episode_path, False, ["missing episode.json"], 0)
    data = json.loads(episode_file.read_text(encoding="utf-8"))
    frames = data.get("frames", [])
    for expected_index, frame in enumerate(frames):
        if int(frame.get("frame_index", -1)) != expected_index:
            issues.append(f"frame_index mismatch at frame {expected_index}")
        state = frame.get("state") or {}
        command = frame.get("command") or {}
        if not _is_finite_vector(state.get("joint_positions_rad", []), 6):
            issues.append(f"state joint vector invalid at frame {expected_index}")
        if not math.isfinite(float(state.get("gripper_value", float("nan")))):
            issues.append(f"state gripper invalid at frame {expected_index}")
        if not _is_finite_vector(command.get("target_joint_positions_rad", []), 6):
            issues.append(f"command joint vector invalid at frame {expected_index}")
        if not math.isfinite(float(command.get("target_gripper_value", float("nan")))):
            issues.append(f"command gripper invalid at frame {expected_index}")
        if float(state.get("age_s", float("inf"))) > max_state_age_s:
            issues.append(f"stale state at frame {expected_index}")
        if float(frame.get("image_age_s", float("inf"))) > max_image_age_s:
            issues.append(f"stale image at frame {expected_index}")
        image_path = episode_path / str(frame.get("camera_path", ""))
        if not image_path.exists():
            issues.append(f"missing image at frame {expected_index}")
        else:
            image = np.asarray(Image.open(image_path))
            if image.ndim != 3 or image.shape[2] != 3:
                issues.append(f"image shape invalid at frame {expected_index}")
        if len((state.get("joint_positions_rad") or [])) + 1 != STATE_DIM:
            issues.append(f"state dimension invalid at frame {expected_index}")
        if len((command.get("target_joint_positions_rad") or [])) + 1 != ACTION_DIM:
            issues.append(f"action dimension invalid at frame {expected_index}")
    return EpisodeValidationResult(episode_path, not issues, issues, len(frames))


def summarize_dataset(root: str | Path) -> DatasetValidationSummary:
    dataset_root = Path(root)
    issues: List[str] = []
    results = [
        validate_episode_directory(path)
        for path in sorted(dataset_root.glob("episode_*"))
        if path.is_dir()
    ]
    total_frames = sum(result.frame_count for result in results)
    invalid = [result for result in results if not result.ok]
    for result in invalid:
        issues.extend([f"{result.episode_path.name}: {issue}" for issue in result.issues])
    return DatasetValidationSummary(
        root=dataset_root,
        total_episodes=len(results),
        valid_episodes=len(results) - len(invalid),
        invalid_episodes=len(invalid),
        total_frames=total_frames,
        issues=issues,
    )


def create_synthetic_episode(root: str | Path, image_size: Tuple[int, int] = (96, 128)) -> Path:
    from .action_schema import EndPoseMetadata, PiperCommandSample, PiperFrameRecord, PiperStateSample

    writer = EpisodeWriter(root)
    session = writer.start_episode("Move the gripper toward the marked target.")
    height, width = image_size
    for frame_index in range(3):
        state = PiperStateSample(
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            joint_positions_rad=[0.1 * frame_index, 0.2, -0.3, 0.4, -0.5, 0.6],
            gripper_value=0.01,
            timestamp_s=1000.0 + frame_index,
            age_s=0.02,
        )
        command = PiperCommandSample(
            command_joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            target_joint_positions_rad=[0.1 * (frame_index + 1), 0.25, -0.25, 0.45, -0.45, 0.65],
            target_gripper_value=0.01,
            timestamp_s=1000.1 + frame_index,
            source="synthetic",
            max_velocity=0.05,
            max_acceleration=0.05,
            moveit_service="/joint_moveit_ctrl_piper",
        )
        frame = PiperFrameRecord(
            frame_index=frame_index,
            task_instruction="Move the gripper toward the marked target.",
            camera_path="",
            camera_topic="/table_camera/color/image_raw",
            camera_frame_id="table_camera_color_optical_frame",
            image_timestamp_s=1000.0 + frame_index,
            image_age_s=0.02,
            state=state,
            command=command,
            end_pose=EndPoseMetadata(
                position_m=[0.1, 0.0, 0.2],
                quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
                timestamp_s=1000.0 + frame_index,
                frame_id="piper_base",
            ),
            receive_timestamp_s=1000.0 + frame_index,
        )
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[:, :, 0] = 40 * (frame_index + 1)
        image[10:30, 10:30, 1] = 255
        session.append_frame(frame, image)
    session.save(success=True, software={"mode": "synthetic"})
    return session.episode_dir
