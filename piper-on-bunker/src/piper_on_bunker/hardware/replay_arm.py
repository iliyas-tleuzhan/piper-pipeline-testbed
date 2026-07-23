from __future__ import annotations

import json
from pathlib import Path

from piper_on_bunker.hardware.mock_arm import MockArm


class ReplayArm(MockArm):
    def __init__(self, fixture_path: str | Path) -> None:
        data = json.loads(Path(fixture_path).read_text(encoding="utf-8"))
        super().__init__(named_poses=data.get("named_poses", {}))
        self.fixture = data
