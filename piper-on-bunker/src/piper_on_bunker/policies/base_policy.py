from typing import Protocol

from piper_on_bunker.models import Pose


class ManipulationPolicy(Protocol):
    def pre_contact_pose(self, target: Pose) -> Pose: ...
