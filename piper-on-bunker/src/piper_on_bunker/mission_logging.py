from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

from piper_on_bunker.models import utc_now

class MissionLogger:
    def __init__(self, path: Optional[str] = None, directory: Optional[str] = None, enabled: bool = True) -> None:
        self.path = Path(path) if path else None
        self.directory = Path(directory) if directory else None
        self.enabled = enabled
        self.events = []

    def start_mission(self, mission_id: str) -> None:
        if self.path is None and self.directory and self.enabled:
            stamp = utc_now().replace(":", "").replace("+", "Z").replace(".", "_")
            self.path = self.directory / f"{stamp}_{mission_id}.jsonl"

    def append(self, event: dict) -> None:
        event.setdefault("timestamp", utc_now())
        self.events.append(event)
        if self.path and self.enabled:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as fh:
                fh.write(json.dumps(event, sort_keys=True) + "\n")
