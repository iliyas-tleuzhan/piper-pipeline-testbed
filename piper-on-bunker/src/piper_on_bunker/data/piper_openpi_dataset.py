from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import json

import numpy as np


FEATURES = {
    "image": {"dtype": "image", "shape": (224, 224, 3), "names": ["height", "width", "channel"]},
    "wrist_image": {"dtype": "image", "shape": (224, 224, 3), "names": ["height", "width", "channel"]},
    "state": {"dtype": "float32", "shape": (7,), "names": ["state"]},
    "actions": {"dtype": "float32", "shape": (7,), "names": ["actions"]},
}


@dataclass(frozen=True)
class PiperFrame:
    exterior_image: np.ndarray
    wrist_image: np.ndarray
    state: np.ndarray
    action: np.ndarray
    instruction: str
    phase_prompt: str
    phase_id: str
    timestamp_s: float
    command_timestamp_s: float
    camera_timestamp_s: float
    state_timestamp_s: float
    valid: bool = True

    def validate(self, *, max_skew_s: float) -> None:
        if self.exterior_image.shape[-3:] != (224, 224, 3) or self.wrist_image.shape[-3:] != (224, 224, 3):
            raise ValueError("PiPER OpenPI images must be RGB 224x224x3 for this dataset config")
        if self.exterior_image.dtype != np.uint8 or self.wrist_image.dtype != np.uint8:
            raise ValueError("PiPER OpenPI images must be uint8")
        if np.asarray(self.state).shape != (7,) or np.asarray(self.action).shape != (7,):
            raise ValueError("PiPER state and action must both be 7D")
        if not np.all(np.isfinite(self.state)) or not np.all(np.isfinite(self.action)):
            raise ValueError("PiPER dataset contains non-finite state/action")
        skew = max(
            abs(self.timestamp_s - self.command_timestamp_s),
            abs(self.timestamp_s - self.camera_timestamp_s),
            abs(self.timestamp_s - self.state_timestamp_s),
        )
        if skew > max_skew_s:
            raise ValueError(f"timestamp skew {skew:.6f}s exceeds max_skew_s={max_skew_s:.6f}s")


def make_synthetic_episode(frame_count: int = 8) -> list[PiperFrame]:
    frames = []
    for i in range(frame_count):
        state = np.array([0.01 * i, 0.02, -0.03, 0.0, 0.04, -0.02, 0.04], dtype=np.float32)
        action = state + np.array([0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        image = np.zeros((224, 224, 3), dtype=np.uint8)
        image[:, :, 0] = i
        timestamp = float(i) * 0.05
        frames.append(
            PiperFrame(
                exterior_image=image,
                wrist_image=np.zeros_like(image),
                state=state,
                action=action,
                instruction="Move the red cup onto the paper.",
                phase_prompt="Approach the red cup and finish in a grasp-ready pose.",
                phase_id="approach",
                timestamp_s=timestamp,
                command_timestamp_s=timestamp,
                camera_timestamp_s=timestamp,
                state_timestamp_s=timestamp,
            )
        )
    return frames


def validate_episode(frames: list[PiperFrame], *, max_skew_s: float = 0.03) -> None:
    if not frames:
        raise ValueError("episode is empty")
    last_timestamp = -float("inf")
    for frame in frames:
        frame.validate(max_skew_s=max_skew_s)
        if frame.timestamp_s <= last_timestamp:
            raise ValueError("episode timestamps must be strictly monotonic")
        last_timestamp = frame.timestamp_s


def write_synthetic_lerobot_like_dataset(output_dir: str | Path, frames: list[PiperFrame]) -> Path:
    validate_episode(frames)
    path = Path(output_dir)
    path.mkdir(parents=True, exist_ok=True)
    rows = []
    for index, frame in enumerate(frames):
        rows.append(
            {
                "frame_index": index,
                "task": frame.phase_prompt,
                "instruction": frame.instruction,
                "phase_id": frame.phase_id,
                "state": np.asarray(frame.state, dtype=np.float32).tolist(),
                "actions": np.asarray(frame.action, dtype=np.float32).tolist(),
                "timestamp_s": frame.timestamp_s,
                "valid": frame.valid,
            }
        )
    (path / "features.json").write_text(json.dumps(FEATURES, indent=2), encoding="utf-8")
    (path / "episode_000000.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    return path

