from __future__ import annotations

from time import monotonic
from typing import Optional

import requests

from piper_on_bunker.models import Pose, SkillResult, StatusCode


class ABotClawApiArm:
    """Adapter for the current 8891 safe action server.

    It intentionally maps only bounded, high-level operations. It does not call
    the old 8888 /code/execute route and never publishes raw joint commands.
    """

    SERVER_MARKER = "piper_language_action_server_v1"
    SUPPORTED_ACTIONS = {"move_up", "move_down", "open_gripper", "close_gripper", "set_gripper"}

    def __init__(self, base_url: str = "http://localhost:8891", dry_run: bool = True, timeout_s: float = 5.0) -> None:
        self.base_url = base_url.rstrip("/")
        self.dry_run = dry_run
        self.timeout_s = timeout_s

    def _get(self, path: str) -> dict:
        try:
            response = requests.get(f"{self.base_url}{path}", timeout=self.timeout_s)
            response.raise_for_status()
            return response.json()
        except requests.RequestException as exc:
            return {"success": False, "error": repr(exc), "path": path}
        except ValueError as exc:
            return {"success": False, "error": "invalid JSON response: " + repr(exc), "path": path}

    def _post(self, path: str, payload: Optional[dict] = None, timeout_s: Optional[float] = None) -> dict:
        payload = payload or {}
        if self.dry_run:
            return {"success": True, "dry_run": True, "path": path, "payload": payload}
        try:
            response = requests.post(f"{self.base_url}{path}", json=payload, timeout=timeout_s or 20)
            response.raise_for_status()
            return response.json()
        except requests.RequestException as exc:
            return {"success": False, "error": repr(exc), "path": path, "payload": payload}
        except ValueError as exc:
            return {"success": False, "error": "invalid JSON response: " + repr(exc), "path": path, "payload": payload}

    def health(self) -> SkillResult:
        start = monotonic()
        data = self._get("/health")
        ok = bool(data.get("success")) and data.get("server") == self.SERVER_MARKER
        code = StatusCode.OK if ok else StatusCode.API_CONTRACT_ERROR
        return SkillResult.build(ok, code, "8891 health checked", start, data)

    def get_state(self) -> dict:
        return self._get("/state")

    def read_state(self) -> SkillResult:
        start = monotonic()
        data = self.get_state()
        ok = bool(data.get("success")) and isinstance(data.get("joint_positions"), list) and len(data.get("joint_positions", [])) >= 6
        code = StatusCode.OK if ok else StatusCode.API_CONTRACT_ERROR
        return SkillResult.build(ok, code, "8891 state checked", start, data)

    def move_to_named_pose(self, name: str) -> SkillResult:
        if self.dry_run:
            return SkillResult.build(
                True,
                StatusCode.OK,
                "dry-run named pose simulated",
                monotonic(),
                {"pose": name, "adapter": "abotclaw_api", "dry_run": True},
            )
        return SkillResult.build(
            False,
            StatusCode.NOT_IMPLEMENTED,
            "8891 contract does not support named-pose execution",
            monotonic(),
            {"pose": name, "adapter": "abotclaw_api", "supported_actions": sorted(self.SUPPORTED_ACTIONS)},
        )

    def move_to_pose(self, pose: Pose) -> SkillResult:
        if self.dry_run:
            return SkillResult.build(
                True,
                StatusCode.OK,
                "dry-run target pose simulated",
                monotonic(),
                {"pose": pose.__dict__, "adapter": "abotclaw_api", "dry_run": True},
            )
        return SkillResult.build(
            False,
            StatusCode.NOT_IMPLEMENTED,
            "8891 contract does not support arbitrary target-pose execution",
            monotonic(),
            {"pose": pose.__dict__, "adapter": "abotclaw_api", "supported_actions": sorted(self.SUPPORTED_ACTIONS)},
        )

    def press(self, pose: Pose, depth_m: float) -> SkillResult:
        start = monotonic()
        if depth_m <= 0 or depth_m > 0.02:
            return SkillResult.build(False, StatusCode.SAFETY_VIOLATION, "press depth exceeds adapter bound", start, {"depth_m": depth_m})
        data = self._post("/move_down", {"joint_step": min(0.03, depth_m * 2.0), "speed": 0.03, "accel": 0.03})
        data["target_pose_used_by_pipeline"] = pose.__dict__
        code = StatusCode.OK if data.get("success") else StatusCode.CONTROLLER_FAILURE
        return SkillResult.build(bool(data.get("success", False)), code, "press mapped to bounded 8891 move_down", start, data)

    def retract(self) -> SkillResult:
        start = monotonic()
        data = self._post("/move_up", {"joint_step": 0.03, "speed": 0.03, "accel": 0.03})
        code = StatusCode.OK if data.get("success") else StatusCode.CONTROLLER_FAILURE
        return SkillResult.build(bool(data.get("success", False)), code, "retract mapped to bounded 8891 move_up", start, data)

    def stop(self) -> SkillResult:
        return SkillResult.build(
            False,
            StatusCode.NOT_IMPLEMENTED,
            "8891 server has no physical stop/cancel/estop endpoint; local mission latch only",
            monotonic(),
        )
