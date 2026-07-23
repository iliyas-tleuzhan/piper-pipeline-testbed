from typing import Optional

from piper_on_bunker.models import Observation, Target


class ColorDetector:
    def detect(self, observation: Observation, label: str) -> Optional[Target]:
        return None
