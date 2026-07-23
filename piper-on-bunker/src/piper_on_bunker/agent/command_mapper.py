from __future__ import annotations

from collections.abc import Callable

from piper_on_bunker.models import SkillResult, StatusCode

ALLOWED_AGENT_TOOLS = {
    "prepare_navigation_view",
    "inspect_workspace",
    "scan_region",
    "find_target",
    "press_target",
    "retract",
    "return_to_navigation_view",
    "run_button_mission",
    "get_pipeline_status",
    "stop",
}

DENIED_TERMS = {"python", "shell", "ros publish", "joint array", "raw motor", "disable safety", "code execute"}


class CommandMapper:
    def __init__(self, supervisor) -> None:
        self.supervisor = supervisor

    def map_command(self, command: str, target: str = "marked_button") -> Callable[[], SkillResult]:
        lowered = command.lower().strip()
        if any(term in lowered for term in DENIED_TERMS):
            return lambda: SkillResult.build(False, StatusCode.INVALID_COMMAND, "restricted agent command denied", __import__("time").monotonic())
        if lowered in {"prepare_navigation_view", "navigation", "nav"}:
            return self.supervisor.prepare_navigation_view
        if lowered in {"inspect_workspace", "inspect"}:
            return self.supervisor.inspect_workspace
        if lowered in {"scan_region", "scan"}:
            return self.supervisor.scan_region
        if lowered in {"find_target", "detect", "detect_target"}:
            return lambda: self.supervisor.detect_target(target)
        if lowered in {"press_target", "press", "touch"}:
            return lambda: self.supervisor.press_target(target)
        if lowered in {"retract", "retract_arm"}:
            return self.supervisor.retract_arm
        if lowered in {"return_to_navigation_view", "return"}:
            return self.supervisor.return_to_navigation_view
        if lowered in {"run_button_mission", "button mission", "mission"}:
            return lambda: self.supervisor.run_button_mission(target)
        if lowered in {"get_pipeline_status", "status"}:
            return self.supervisor.get_robot_state
        if lowered in {"stop", "estop"}:
            return self.supervisor.stop_motion
        return lambda: SkillResult.build(False, StatusCode.INVALID_COMMAND, f"unknown restricted command: {command}", __import__("time").monotonic())
