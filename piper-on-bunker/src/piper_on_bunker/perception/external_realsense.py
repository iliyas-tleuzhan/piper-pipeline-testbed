from __future__ import annotations

from piper_on_bunker.models import Observation, Target


class ExternalFixedCamera:
    def __init__(self, camera_name: str = "table_camera") -> None:
        self.camera_name = camera_name
        try:
            import rospy  # noqa: F401
        except Exception as exc:
            raise RuntimeError("ExternalFixedCamera requires ROS/rospy; use MockCamera or ReplayCamera on this laptop") from exc

    def capture_observation(self) -> Observation:
        raise RuntimeError("ExternalFixedCamera is an interface placeholder until run on the PiPER laptop")

    def detect_target(self, observation: Observation, label: str) -> Target | None:
        raise RuntimeError("Use a detector adapter with real ROS image/depth data")
