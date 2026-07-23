from __future__ import annotations

from time import monotonic
from typing import Optional, Set

from piper_on_bunker.models import Pose, SkillResult, StatusCode


class MockArm:
    def __init__(self, named_poses: Optional[dict] = None, failures: Optional[Set[str]] = None) -> None:
        self.named_poses = named_poses or {}
        self.failures = failures or set()
        self.current_pose_name = "tabletop_home"
        self.stopped = False
        self.press_count = 0
        self.actions = []

    def _result(self, action: str, ok_code: StatusCode = StatusCode.OK, outputs: Optional[dict] = None) -> SkillResult:
        start = monotonic()
        mapped_failures = {
            "move_to_pose": {"ik_failure", "planning_failure"},
            "press": {"controller_failure"},
            "retract": {"pose_timeout"},
        }
        active_failure = action if action in self.failures else next(iter(mapped_failures.get(action, set()) & self.failures), None)
        if active_failure:
            code = {
                "ik_failure": StatusCode.IK_FAILURE,
                "planning_failure": StatusCode.PLANNING_FAILURE,
                "pose_timeout": StatusCode.POSE_TIMEOUT,
                "controller_failure": StatusCode.CONTROLLER_FAILURE,
                "target_lost": StatusCode.TARGET_LOST,
            }.get(active_failure, StatusCode.ERROR)
            return SkillResult.build(False, code, f"Injected failure: {active_failure}", start)
        return SkillResult.build(True, ok_code, f"{action} complete", start, outputs)

    def get_state(self) -> dict:
        return {"adapter": "mock", "pose": self.current_pose_name, "stopped": self.stopped, "press_count": self.press_count}

    def move_to_named_pose(self, name: str) -> SkillResult:
        self.actions.append(("move_to_named_pose", name))
        result = self._result(name, outputs={"pose": name})
        if result.success:
            self.current_pose_name = name
        return result

    def move_to_pose(self, pose: Pose) -> SkillResult:
        self.actions.append(("move_to_pose", pose))
        return self._result("move_to_pose", outputs={"pose": pose.__dict__})

    def press(self, pose: Pose, depth_m: float) -> SkillResult:
        self.actions.append(("press", pose))
        result = self._result("press", outputs={"pose": pose.__dict__, "depth_m": depth_m})
        if result.success:
            self.press_count += 1
        return result

    def retract(self) -> SkillResult:
        self.actions.append(("retract", None))
        result = self._result("retract", outputs={"pose": "retracted"})
        if result.success:
            self.current_pose_name = "retracted"
        return result

    def stop(self) -> SkillResult:
        self.actions.append(("stop", None))
        self.stopped = True
        return SkillResult.build(True, StatusCode.ESTOP, "motion stopped", monotonic())
