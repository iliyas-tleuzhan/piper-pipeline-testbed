from __future__ import annotations

from time import monotonic

from piper_on_bunker.arm_state_machine import ArmStateMachine
from piper_on_bunker.hardware.mock_base import MockBase
from piper_on_bunker.models import ArmMode, Observation, SkillResult, StatusCode, Target
from piper_on_bunker.policies.moveit_policy import MoveItPolicy
from piper_on_bunker.resource_manager import ResourceManager
from piper_on_bunker.safety import validate_named_pose
from piper_on_bunker.transforms import require_base_pose


class MissionSupervisor:
    def __init__(self, arm, camera, base: MockBase | None = None, owner: str = "mission") -> None:
        self.arm = arm
        self.camera = camera
        self.base = base or MockBase()
        self.owner = owner
        self.resources = ResourceManager()
        self.state = ArmStateMachine()
        self.policy = MoveItPolicy()
        self.last_observation: Observation | None = None
        self.last_target: Target | None = None
        self.verification_should_pass = True

    def _ok(self, message: str, outputs: dict | None = None) -> SkillResult:
        return SkillResult.build(True, StatusCode.OK, message, monotonic(), outputs)

    def _fail(self, code: StatusCode, message: str) -> SkillResult:
        return SkillResult.build(False, code, message, monotonic())

    def get_robot_state(self) -> SkillResult:
        return self._ok("state read", {"arm": self.arm.get_state(), "mode": self.state.mode.value, "base_locked": self.base.locked})

    def set_arm_mode(self, mode: ArmMode) -> SkillResult:
        self.state.transition(mode)
        return self._ok("mode changed", {"mode": mode.value})

    def move_to_named_pose(self, name: str) -> SkillResult:
        validate_named_pose(name)
        return self.arm.move_to_named_pose(name)

    def prepare_navigation_view(self, pose: str = "simulated_front_nav_view") -> SkillResult:
        self.state.transition(ArmMode.NAVIGATION_VIEW)
        return self.move_to_named_pose(pose)

    def inspect_workspace(self) -> SkillResult:
        self.base.request_navigation_pause()
        self.state.transition(ArmMode.TASK_INSPECTION)
        return self.move_to_named_pose("inspect_workspace")

    def scan_region(self) -> SkillResult:
        self.state.transition(ArmMode.ACTIVE_SCAN)
        for pose in ("scan_left", "scan_center", "scan_right"):
            result = self.move_to_named_pose(pose)
            if not result.success:
                return result
        return self._ok("scan complete")

    def capture_observation(self) -> SkillResult:
        try:
            self.last_observation = self.camera.capture_observation()
        except Exception as exc:
            return self._fail(StatusCode.CAMERA_UNAVAILABLE, str(exc))
        return self._ok("observation captured", {"observation": self.last_observation.__dict__})

    def detect_target(self, label: str = "marked_button") -> SkillResult:
        if self.last_observation is None:
            obs_result = self.capture_observation()
            if not obs_result.success:
                return obs_result
        self.last_target = self.camera.detect_target(self.last_observation, label)
        if self.last_target is None:
            return self._fail(StatusCode.TARGET_NOT_FOUND, f"target not found: {label}")
        return self._ok("target detected", {"target": self.last_target.__dict__})

    def estimate_target_pose(self) -> SkillResult:
        if not self.last_target:
            return self._fail(StatusCode.TARGET_NOT_FOUND, "no target to estimate")
        try:
            pose = require_base_pose(self.last_target.base_pose)
        except ValueError as exc:
            return self._fail(StatusCode.INVALID_TRANSFORM, str(exc))
        return self._ok("target pose estimated", {"base_pose": pose.__dict__})

    def validate_target(self) -> SkillResult:
        if not self.last_target or not self.last_target.base_pose:
            return self._fail(StatusCode.INVALID_TRANSFORM, "target lacks a valid base pose")
        return self._ok("target validated")

    def move_to_pre_contact(self) -> SkillResult:
        if not self.last_target:
            return self._fail(StatusCode.TARGET_NOT_FOUND, "no target")
        self.state.transition(ArmMode.PRE_MANIPULATION)
        pose = self.policy.pre_contact_pose(require_base_pose(self.last_target.base_pose))
        return self.arm.move_to_pose(pose)

    def visual_servo_to_target(self) -> SkillResult:
        if "target_lost" in getattr(self.arm, "failures", set()):
            return self._fail(StatusCode.TARGET_LOST, "target lost during visual servo")
        return self._ok("visual servo complete")

    def press_target(self) -> SkillResult:
        if not self.last_target:
            return self._fail(StatusCode.TARGET_NOT_FOUND, "no target")
        self.state.transition(ArmMode.MANIPULATION)
        return self.arm.press(require_base_pose(self.last_target.base_pose), depth_m=0.015)

    def touch_target(self) -> SkillResult:
        return self.press_target()

    def retract_arm(self) -> SkillResult:
        self.state.transition(ArmMode.RETRACTING)
        return self.arm.retract()

    def verify_task(self) -> SkillResult:
        self.state.transition(ArmMode.VERIFYING)
        if not self.verification_should_pass:
            return self._fail(StatusCode.VERIFICATION_FAILURE, "verification failed")
        return self._ok("verification passed")

    def return_to_navigation_view(self) -> SkillResult:
        self.state.transition(ArmMode.NAVIGATION_VIEW)
        return self.move_to_named_pose("simulated_front_nav_view")

    def stop_motion(self) -> SkillResult:
        self.state.transition(ArmMode.ESTOP)
        return self.arm.stop()

    def recover_to_safe_pose(self) -> SkillResult:
        if self.state.mode != ArmMode.ESTOP:
            self.state.transition(ArmMode.FAULT)
            self.state.transition(ArmMode.RETRACTING)
            return self.move_to_named_pose("safe_recovery")
        return self._fail(StatusCode.ESTOP, "cannot recover from ESTOP without operator reset")

    def run_button_mission(self) -> SkillResult:
        if not self.resources.acquire("arm", self.owner):
            return self._fail(StatusCode.RESOURCE_BUSY, "arm resource is busy")
        steps = [
            self.prepare_navigation_view,
            self.inspect_workspace,
            self.capture_observation,
            self.detect_target,
            self.estimate_target_pose,
            self.validate_target,
            self.move_to_pre_contact,
            self.visual_servo_to_target,
            self.press_target,
            self.retract_arm,
            self.verify_task,
            self.return_to_navigation_view,
        ]
        history = []
        try:
            for step in steps:
                result = step()
                history.append({"step": step.__name__, "success": result.success, "status_code": result.status_code.value})
                if not result.success:
                    self.arm.stop()
                    if self.state.mode != ArmMode.ESTOP:
                        try:
                            self.retract_arm()
                            self.recover_to_safe_pose()
                        except Exception:
                            pass
                    return SkillResult.build(False, result.status_code, f"mission failed at {step.__name__}", monotonic(), {"history": history})
            self.base.unlock_base()
            return SkillResult.build(True, StatusCode.OK, "button mission complete", monotonic(), {"history": history})
        finally:
            self.resources.release("arm", self.owner)
