from __future__ import annotations

import json
from pathlib import Path


class MissionLogger:
    def __init__(self, path: str | Path | None = None) -> None:
        self.path = Path(path) if path else None
        self.events: list[dict] = []

    def append(self, event: dict) -> None:
        self.events.append(event)
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.path.write_text(json.dumps(self.events, indent=2), encoding="utf-8")
