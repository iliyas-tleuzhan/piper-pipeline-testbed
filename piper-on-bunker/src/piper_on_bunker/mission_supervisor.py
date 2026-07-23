from __future__ import annotations

from time import monotonic
from typing import Optional
from uuid import uuid4

from piper_on_bunker.arm_state_machine import ArmStateMachine
from piper_on_bunker.hardware.mock_base import MockBase
from piper_on_bunker.mission_logging import MissionLogger
from piper_on_bunker.models import ArmMode, Observation, SkillResult, StatusCode, Target
from piper_on_bunker.policies.moveit_policy import MoveItPolicy
from piper_on_bunker.resource_manager import ResourceManager
from piper_on_bunker.safety import validate_named_pose, validate_workspace
from piper_on_bunker.transforms import TransformResolver, require_base_pose


class MissionSupervisor:
    def __init__(self, arm, camera, base=None, owner: str = "mission", logger=None, safety=None, transform_resolver=None, planning_frame: str = "piper_base", mode: str = "mock", physical_motion_enabled: bool = False) -> None:
        self.arm = arm
        self.camera = camera
        self.base = base or MockBase()
        self.owner = owner
        self.resources = ResourceManager()
        self.logger = logger or MissionLogger()
        self.safety = safety or {}
        self.transform_resolver = transform_resolver or TransformResolver({})
        self.planning_frame = planning_frame
        self.mode = mode
        self.physical_motion_enabled = physical_motion_enabled
        self.state = ArmStateMachine()
        self.policy = MoveItPolicy()
        self.last_observation = None
        self.last_target = None
        self.verification_should_pass = True
        self.manual_verification_ack = False
        self.manual_verification_rejected = False
        self.current_mission_id = None

    def _ok(self, message: str, outputs: Optional[dict] = None) -> SkillResult:
        return SkillResult.build(True, StatusCode.OK, message, monotonic(), outputs)

    def _fail(self, code: StatusCode, message: str) -> SkillResult:
        return SkillResult.build(False, code, message, monotonic())

    def _public_observation(self, observation: Observation) -> dict:
        metadata = {}
        for key, value in observation.metadata.items():
            if key.startswith("_"):
                continue
            metadata[key] = value
        return {
            "camera_name": observation.camera_name,
            "frame_id": observation.frame_id,
            "timestamp": observation.timestamp,
            "rgb_path": observation.rgb_path,
            "depth_path": observation.depth_path,
            "metadata": metadata,
        }

    def get_robot_state(self) -> SkillResult:
        arm_state = self.arm.get_state()
        outputs = {"arm": arm_state, "mode": self.state.mode.value, "base_locked": self.base.locked}
        if isinstance(arm_state, dict) and arm_state.get("success") is False:
            return SkillResult.build(False, StatusCode.STALE_STATE, "arm state unavailable", monotonic(), outputs)
        return self._ok("state read", outputs)

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
        if not self.base.request_navigation_pause():
            return self._fail(StatusCode.BASE_NOT_LOCKED, "base could not be locked for inspection")
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
        return self._ok("observation captured", {"observation": self._public_observation(self.last_observation)})

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
            if self.last_target.base_pose is None:
                if self.last_target.camera_pose is None:
                    return self._fail(StatusCode.INVALID_TRANSFORM, "target lacks a camera-frame pose")
                self.last_target.base_pose = self.transform_resolver.transform_pose(self.last_target.camera_pose, self.planning_frame)
            pose = require_base_pose(self.last_target.base_pose)
        except ValueError as exc:
            return self._fail(StatusCode.INVALID_TRANSFORM, str(exc))
        return self._ok("target pose estimated", {"base_pose": pose.__dict__})

    def validate_target(self) -> SkillResult:
        if not self.last_target or not self.last_target.base_pose:
            return self._fail(StatusCode.INVALID_TRANSFORM, "target lacks a valid base pose")
        if self.safety.get("workspace_bounds_m"):
            try:
                validate_workspace(self.last_target.base_pose, self.safety)
            except ValueError as exc:
                return self._fail(StatusCode.SAFETY_VIOLATION, str(exc))
        return self._ok("target validated")

    def move_to_pre_contact(self) -> SkillResult:
        if not self.last_target:
            return self._fail(StatusCode.TARGET_NOT_FOUND, "no target")
        self.state.transition(ArmMode.PRE_MANIPULATION)
        pose = self.policy.pre_contact_pose(require_base_pose(self.last_target.base_pose))
        try:
            self._validate_motion_prerequisites(target_dependent=True)
            self._validate_distance(require_base_pose(self.last_target.base_pose), pose, "max_pre_contact_distance_m")
        except ValueError as exc:
            return self._fail(StatusCode.SAFETY_VIOLATION, str(exc))
        return self.arm.move_to_pose(pose)

    def visual_servo_to_target(self) -> SkillResult:
        if "target_lost" in getattr(self.arm, "failures", set()):
            return self._fail(StatusCode.TARGET_LOST, "target lost during visual servo")
        return SkillResult.build(
            True,
            StatusCode.VISUAL_SERVO_DISABLED_OPEN_LOOP,
            "VISUAL_SERVO_DISABLED_OPEN_LOOP",
            monotonic(),
        )

    def press_target(self, label: str = "marked_button") -> SkillResult:
        if not self.last_target:
            detect = self.detect_target(label)
            if not detect.success:
                return detect
        if not self.base.is_base_stopped():
            return self._fail(StatusCode.BASE_NOT_LOCKED, "base must be locked before pressing target")
        try:
            self._validate_motion_prerequisites(target_dependent=True)
        except ValueError as exc:
            return self._fail(StatusCode.SAFETY_VIOLATION, str(exc))
        self.state.transition(ArmMode.MANIPULATION)
        return self.arm.press(require_base_pose(self.last_target.base_pose), depth_m=0.015)

    def touch_target(self) -> SkillResult:
        return self.press_target()

    def retract_arm(self) -> SkillResult:
        self.state.transition(ArmMode.RETRACTING)
        return self.arm.retract()

    def verify_task(self) -> SkillResult:
        if self.manual_verification_rejected:
            self.state.transition(ArmMode.VERIFYING)
            return self._fail(StatusCode.VERIFICATION_FAILURE, "manual verification rejected")
        if not self.manual_verification_ack:
            self.state.transition(ArmMode.WAITING_FOR_VERIFICATION)
            return SkillResult.build(
                False,
                StatusCode.WAITING_FOR_VERIFICATION,
                "WAITING_FOR_VERIFICATION",
                monotonic(),
                {"mission_id": self.current_mission_id, "prompt": "Did the button press succeed? [y/N]"},
            )
        self.state.transition(ArmMode.VERIFYING)
        if not self.verification_should_pass:
            return self._fail(StatusCode.VERIFICATION_FAILURE, "verification failed")
        return self._ok("manual verification acknowledged")

    def acknowledge_manual_verification(self) -> SkillResult:
        self.manual_verification_ack = True
        self.manual_verification_rejected = False
        return self._ok("manual verification acknowledgement recorded")

    def reject_manual_verification(self) -> SkillResult:
        self.manual_verification_ack = False
        self.manual_verification_rejected = True
        self.verification_should_pass = False
        return self._ok("manual verification rejection recorded")

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

    def run_button_mission(self, label: str = "marked_button") -> SkillResult:
        mission_id = str(uuid4())
        self.current_mission_id = mission_id
        self.logger.start_mission(mission_id)
        if not self.resources.acquire("arm", self.owner):
            return self._fail(StatusCode.RESOURCE_BUSY, "arm resource is busy")
        steps = [
            self.get_robot_state,
            self.prepare_navigation_view,
            self.inspect_workspace,
            self.capture_observation,
            lambda: self.detect_target(label),
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
        self.logger.append({"mission_id": mission_id, "event": "mission_started", "label": label, "mode": self.mode, "physical_motion_enabled": self.physical_motion_enabled})
        try:
            for step in steps:
                step_name = getattr(step, "__name__", "detect_target")
                result = step()
                event = {"mission_id": mission_id, "step": step_name, "success": result.success, "status_code": result.status_code.value, "message": result.message}
                if result.outputs:
                    event["outputs"] = self._json_safe(result.outputs)
                history.append(event)
                self.logger.append(event)
                if not result.success:
                    if result.status_code == StatusCode.WAITING_FOR_VERIFICATION:
                        final = SkillResult.build(False, result.status_code, "mission waiting for manual verification", monotonic(), {"mission_id": mission_id, "history": history, "prompt": "Did the button press succeed? [y/N]"})
                        self.logger.append({"mission_id": mission_id, "event": "waiting_for_verification"})
                        return final
                    recovery = self._maybe_recover_after_failure(result)
                    final = SkillResult.build(False, result.status_code, f"mission failed at {step_name}", monotonic(), {"mission_id": mission_id, "history": history})
                    final.recovery = recovery
                    self.logger.append({"mission_id": mission_id, "event": "mission_failed", "status_code": result.status_code.value})
                    return final
            self.base.unlock_base()
            final = SkillResult.build(True, StatusCode.OK, "button mission complete", monotonic(), {"mission_id": mission_id, "history": history})
            self.logger.append({"mission_id": mission_id, "event": "mission_complete"})
            return final
        finally:
            self.resources.release("arm", self.owner)

    def _maybe_recover_after_failure(self, result: SkillResult):
        from piper_on_bunker.models import RecoveryInfo

        unsafe = {
            StatusCode.CONTROLLER_FAILURE,
            StatusCode.STALE_STATE,
            StatusCode.POSE_NOT_REACHED,
            StatusCode.ESTOP,
            StatusCode.NOT_IMPLEMENTED,
        }
        if result.status_code in unsafe:
            self.state.transition(ArmMode.FAULT)
            self.logger.append({"mission_id": self.current_mission_id, "event": "recovery_skipped", "reason": result.status_code.value})
            return RecoveryInfo(attempted=False, action="operator_intervention_required", success=False)
        recoverable = {StatusCode.TARGET_NOT_FOUND, StatusCode.INVALID_TRANSFORM, StatusCode.SAFETY_VIOLATION, StatusCode.VERIFICATION_FAILURE}
        if result.status_code not in recoverable:
            return RecoveryInfo(attempted=False, action="not_classified_recoverable", success=False)
        self.logger.append({"mission_id": self.current_mission_id, "event": "recovery_skipped", "reason": "no_auto_motion_after_failure"})
        self.state.transition(ArmMode.FAULT)
        return RecoveryInfo(attempted=False, action="operator_review_required", success=False)

    def _validate_motion_prerequisites(self, target_dependent: bool) -> None:
        if self.physical_motion_enabled and not self.base.is_base_stopped():
            raise ValueError("base must be locked before physical motion")
        if hasattr(self.arm, "validate_live_state"):
            state_result = self.arm.validate_live_state()
            if not state_result.success:
                raise ValueError(state_result.message)
        if target_dependent and self.last_observation:
            age = float(self.last_observation.metadata.get("color_age_s", 0.0))
            max_age = float(self.safety.get("max_camera_age_s", 1.0))
            if age > max_age:
                raise ValueError(f"camera observation is stale: age_s={age:.3f} max_age_s={max_age:.3f}")

    def _validate_distance(self, a, b, limit_name: str) -> None:
        limit = self.safety.get(limit_name)
        if limit is None:
            return
        distance = ((float(a.x) - float(b.x)) ** 2 + (float(a.y) - float(b.y)) ** 2 + (float(a.z) - float(b.z)) ** 2) ** 0.5
        if distance > float(limit):
            raise ValueError(f"{limit_name} exceeded: distance_m={distance:.4f} limit_m={float(limit):.4f}")

    def _json_safe(self, value):
        if isinstance(value, dict):
            return {str(key): self._json_safe(item) for key, item in value.items() if not str(key).startswith("_")}
        if isinstance(value, (list, tuple)):
            return [self._json_safe(item) for item in value]
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        return repr(value)
