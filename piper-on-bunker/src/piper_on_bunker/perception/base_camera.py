from __future__ import annotations

from typing import Optional, Protocol

from piper_on_bunker.models import Observation, Target


class CameraAdapter(Protocol):
    def capture_observation(self) -> Observation: ...
    def detect_target(self, observation: Observation, label: str) -> Optional[Target]: ...
