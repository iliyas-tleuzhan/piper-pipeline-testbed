from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

from piper_on_bunker.models import utc_now

class MissionLogger:
    def __init__(self, path: Optional[str] = None) -> None:
        self.path = Path(path) if path else None
        self.events = []

    def append(self, event: dict) -> None:
        event.setdefault("timestamp", utc_now())
        self.events.append(event)
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as fh:
                fh.write(json.dumps(event, sort_keys=True) + "\n")
