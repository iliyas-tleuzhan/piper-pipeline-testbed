from __future__ import annotations

from time import monotonic

import requests

from piper_on_bunker.models import Pose, SkillResult, StatusCode


class ABotClawApiArm:
    """Adapter for the current 8891 safe action server.

    It intentionally maps only bounded, high-level operations. It does not call
    the old 8888 /code/execute route and never publishes raw joint commands.
    """

    def __init__(self, base_url: str = "http://localhost:8891", dry_run: bool = True) -> None:
        self.base_url = base_url.rstrip("/")
        self.dry_run = dry_run

    def _get(self, path: str) -> dict:
        response = requests.get(f"{self.base_url}{path}", timeout=5)
        response.raise_for_status()
        return response.json()

    def _post(self, path: str, payload: dict | None = None) -> dict:
        if self.dry_run:
            return {"success": True, "dry_run": True, "path": path, "payload": payload or {}}
        response = requests.post(f"{self.base_url}{path}", json=payload or {}, timeout=20)
        response.raise_for_status()
        return response.json()

    def get_state(self) -> dict:
        return self._get("/state")

    def move_to_named_pose(self, name: str) -> SkillResult:
        return SkillResult.build(True, StatusCode.OK, "dry-run named pose mapped", monotonic(), {"pose": name, "adapter": "abotclaw_api"})

    def move_to_pose(self, pose: Pose) -> SkillResult:
        return SkillResult.build(True, StatusCode.OK, "dry-run pose mapped", monotonic(), {"pose": pose.__dict__})

    def press(self, pose: Pose, depth_m: float) -> SkillResult:
        data = self._post("/move_down", {"joint_step": 0.03, "speed": 0.03, "accel": 0.03})
        return SkillResult.build(bool(data.get("success", False)), StatusCode.OK, "press mapped to bounded move_down", monotonic(), data)

    def retract(self) -> SkillResult:
        data = self._post("/move_up", {"joint_step": 0.03, "speed": 0.03, "accel": 0.03})
        return SkillResult.build(bool(data.get("success", False)), StatusCode.OK, "retract mapped to bounded move_up", monotonic(), data)

    def stop(self) -> SkillResult:
        return SkillResult.build(True, StatusCode.ESTOP, "8891 server has no estop endpoint; local pipeline stopped", monotonic())
