from piper_on_bunker.models import Observation, Target


class MarkerDetector:
    def detect(self, observation: Observation, label: str) -> Target | None:
        return None
