from pathlib import Path

import numpy as np

from piper_on_bunker.data.lerobot_export import build_lerobot_features, export_raw_dataset_to_lerobot
from piper_on_bunker.data.dataset_validator import create_synthetic_episode


class FakeDataset:
    def __init__(self):
        self.frames = []
        self.episodes = 0
        self.finalized = False

    @classmethod
    def create(cls, **kwargs):
        instance = cls()
        instance.kwargs = kwargs
        return instance

    def add_frame(self, frame):
        self.frames.append(frame)

    def save_episode(self):
        self.episodes += 1

    def finalize(self):
        self.finalized = True


def test_build_lerobot_features():
    features = build_lerobot_features(96, 128, use_videos=False)
    assert features["observation.images.external_camera"]["dtype"] == "image"
    assert features["observation.state"]["shape"] == (7,)
    assert features["action"]["shape"] == (7,)


def test_export_raw_dataset_to_fake_lerobot(tmp_path: Path):
    raw_root = tmp_path / "raw"
    create_synthetic_episode(raw_root)
    result = export_raw_dataset_to_lerobot(
        raw_root=raw_root,
        export_root=tmp_path / "export",
        dataset_factory=FakeDataset,
        use_videos=False,
    )
    assert result["request_success"] is True
    assert result["episodes"] == 1
