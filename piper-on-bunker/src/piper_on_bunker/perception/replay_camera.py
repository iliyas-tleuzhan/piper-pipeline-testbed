from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

from piper_on_bunker.models import Observation, Pose, Target, utc_now


class ReplayCamera:
    def __init__(self, fixture_path: str | Path) -> None:
        self.fixture = json.loads(Path(fixture_path).read_text(encoding="utf-8"))

    def capture_observation(self) -> Observation:
        obs = self.fixture["observations"][0]
        return Observation(
            camera_name=obs.get("camera_name", "ReplayCamera"),
            frame_id=obs.get("frame_id", "table_camera_color_optical_frame"),
            timestamp=obs.get("timestamp", utc_now()),
            rgb_path=obs.get("rgb_path"),
            depth_path=obs.get("depth_path"),
            metadata=obs.get("metadata", {}),
        )

    def detect_target(self, observation: Observation, label: str) -> Optional[Target]:
        for raw in self.fixture.get("targets", []):
            if raw.get("label") == label:
                p = raw["base_pose"]
                return Target(
                    label=label,
                    confidence=raw.get("confidence", 1.0),
                    pixel=tuple(raw.get("pixel", [0, 0])),
                    base_pose=Pose(**p),
                )
        return None
