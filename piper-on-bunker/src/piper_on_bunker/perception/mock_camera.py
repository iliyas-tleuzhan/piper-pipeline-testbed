from __future__ import annotations

from typing import Optional

from piper_on_bunker.models import Observation, Pose, Target, utc_now


class MockCamera:
    def __init__(self, unavailable: bool = False, target_found: bool = True, invalid_transform: bool = False) -> None:
        self.unavailable = unavailable
        self.target_found = target_found
        self.invalid_transform = invalid_transform

    def capture_observation(self) -> Observation:
        if self.unavailable:
            raise RuntimeError("mock camera unavailable")
        return Observation(
            camera_name="MockCamera",
            frame_id="mock_camera_optical",
            timestamp=utc_now(),
            metadata={"synthetic": True},
        )

    def detect_target(self, observation: Observation, label: str) -> Optional[Target]:
        if not self.target_found:
            return None
        camera_pose = Pose(x=0.02, y=0.01, z=0.45, frame_id=observation.frame_id)
        base_pose = None if self.invalid_transform else Pose(x=0.32, y=0.04, z=0.09)
        return Target(label=label, confidence=0.98, pixel=(320, 180), camera_pose=camera_pose, base_pose=base_pose)
