from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from time import monotonic
from typing import Optional

from piper_on_bunker.hardware.joint_state import DEFAULT_ARM_JOINT_NAMES, map_joint_state, within_joint_tolerance
from piper_on_bunker.models import Pose, SkillResult, StatusCode


class PiperRosArm:
    JOINT_TOPIC = "/joint_states_single"
    END_POSE_TOPIC = "/end_pose"
    MOVEIT_SERVICES = (
        "/joint_moveit_ctrl_arm",
        "/joint_moveit_ctrl_endpose",
        "/joint_moveit_ctrl_gripper",
        "/joint_moveit_ctrl_piper",
    )
    JOINT_LIMIT_SOURCE = (
        "Loaded from live /robot_description joint limits published by the PiPER ROS/MoveIt stack; "
        "reference URDF: ~/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/src/"
        "piper_ros/src/piper_description/urdf/piper_description.urdf"
    )
    GRIPPER_LIMIT_SOURCE = (
        "ABot-Claw robot_sdk/config.yaml gripper_min=0.0, gripper_max=0.06 meters opening width"
    )

    def __init__(self, physical_motion_enabled: bool = False, named_poses: Optional[dict] = None, safety: Optional[dict] = None, expected_joint_names=None) -> None:
        self.physical_motion_enabled = physical_motion_enabled
        self.named_poses = named_poses or {}
        self.safety = safety or {}
        self.expected_joint_names = list(expected_joint_names or self.safety.get("expected_joint_names") or DEFAULT_ARM_JOINT_NAMES)
        self.last_preview = None
        try:
            import rospy
        except Exception as exc:
            raise RuntimeError("ROS/rospy is required for PiperRosArm; use mock or replay mode on this laptop") from exc
        self.rospy = rospy
        if not rospy.get_node_uri():
            rospy.init_node("piper_pipeline_testbed", anonymous=True, disable_signals=True)

    def get_state(self) -> dict:
        state = {"adapter": "piper_ros", "physical_motion_enabled": self.physical_motion_enabled}
        state["joint_state"] = self.read_joint_state(timeout_s=0.5).outputs
        state["services"] = self.inspect_services(timeout_s=0.1).outputs
        return state

    def read_joint_state(self, timeout_s: float = 2.0) -> SkillResult:
        start = monotonic()
        try:
            from sensor_msgs.msg import JointState

            msg = self.rospy.wait_for_message(self.JOINT_TOPIC, JointState, timeout=timeout_s)
            mapped = map_joint_state(msg.name, msg.position, float(msg.header.stamp.to_sec()), self.expected_joint_names)
            positions = mapped.arm_positions
            max_age = float(self.safety.get("max_state_age_s", 1.0))
            ok = len(positions) == 6 and mapped.age_s <= max_age
            code = StatusCode.OK if ok else StatusCode.STALE_STATE
            return SkillResult.build(
                ok,
                code,
                "joint state read" if ok else "joint state is stale or invalid",
                start,
                {
                    "topic": self.JOINT_TOPIC,
                    "names": mapped.names,
                    "arm_joint_names": mapped.arm_joint_names,
                    "positions": positions,
                    "gripper_position": mapped.gripper_position,
                    "stamp": mapped.stamp_s,
                    "age_s": mapped.age_s,
                    "max_state_age_s": max_age,
                },
            )
        except Exception as exc:
            return SkillResult.build(False, StatusCode.STALE_STATE, "joint state unavailable", start, {"topic": self.JOINT_TOPIC, "error": repr(exc)})

    def inspect_services(self, timeout_s: float = 1.0) -> SkillResult:
        start = monotonic()
        found = {}
        for name in self.MOVEIT_SERVICES:
            try:
                import rosservice

                service_type = rosservice.get_service_type(name)
                found[name] = {"available": service_type is not None, "type": service_type}
            except Exception as exc:
                found[name] = {"available": False, "error": repr(exc)}
        ok = any(value.get("available") for value in found.values())
        return SkillResult.build(ok, StatusCode.OK if ok else StatusCode.CONTROLLER_FAILURE, "MoveIt services inspected", start, found)

    def _motion_disabled(self, action: str, outputs: Optional[dict] = None) -> SkillResult:
        data = outputs or {}
        data["action"] = action
        return SkillResult.build(False, StatusCode.HARDWARE_DISABLED, "physical motion is disabled", monotonic(), data)

    def validate_live_state(self) -> SkillResult:
        return self.read_joint_state(timeout_s=float(self.safety.get("state_read_timeout_s", 2.0)))

    def move_to_named_pose(self, name: str) -> SkillResult:
        start = monotonic()
        if name not in self.named_poses or not self.named_poses[name]:
            return SkillResult.build(False, StatusCode.SAFETY_VIOLATION, "named pose is not calibrated", start, {"pose": name})
        joints = [float(value) for value in self.named_poses[name]]
        if len(joints) != 6:
            return SkillResult.build(False, StatusCode.SAFETY_VIOLATION, "named pose must contain six joints", start, {"pose": name})
        return self._call_joint_moveit(
            "/joint_moveit_ctrl_arm",
            joint_states=joints,
            joint_endpose=[0.0] * 7,
            gripper=0.0,
            start=start,
            action="move_to_named_pose",
            verification_mode="joint_target",
            extra={"pose": name},
        )

    def move_to_joint_target(self, joint_states, gripper: float, label: str = "move_to_joint_target") -> SkillResult:
        start = monotonic()
        joints = [float(value) for value in joint_states]
        if len(joints) != 6:
            return SkillResult.build(
                False,
                StatusCode.SAFETY_VIOLATION,
                "joint target must contain six joints",
                start,
                {"joint_states": joints, "gripper": float(gripper)},
            )
        return self._call_joint_moveit(
            "/joint_moveit_ctrl_piper",
            joint_states=joints,
            joint_endpose=[0.0] * 7,
            gripper=float(gripper),
            start=start,
            action=label,
            verification_mode="joint_target",
            extra={"joint_target": joints, "gripper_target": float(gripper)},
        )

    def move_to_pose(self, pose: Pose) -> SkillResult:
        start = monotonic()
        return self._call_joint_moveit(
            "/joint_moveit_ctrl_endpose",
            joint_states=[0.0] * 6,
            joint_endpose=[pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw],
            gripper=0.0,
            start=start,
            action="move_to_pose",
            verification_mode="end_pose",
            extra={"pose": pose.__dict__},
        )

    def press(self, pose: Pose, depth_m: float) -> SkillResult:
        start = monotonic()
        max_depth = float(self.safety.get("max_press_distance_m", 0.015))
        if depth_m <= 0 or depth_m > max_depth:
            return SkillResult.build(False, StatusCode.SAFETY_VIOLATION, "press distance exceeds configured bound", start, {"depth_m": depth_m, "max_press_distance_m": max_depth})
        target = Pose(pose.x, pose.y, pose.z - depth_m, pose.qx, pose.qy, pose.qz, pose.qw, pose.frame_id)
        return self.move_to_pose(target)

    def retract(self) -> SkillResult:
        start = monotonic()
        if "retracted" in self.named_poses and self.named_poses["retracted"]:
            return self.move_to_named_pose("retracted")
        return SkillResult.build(False, StatusCode.SAFETY_VIOLATION, "retracted named pose is not calibrated", start)

    def stop(self) -> SkillResult:
        start = monotonic()
        try:
            import actionlib
            from control_msgs.msg import FollowJointTrajectoryAction

            client = actionlib.SimpleActionClient("/arm_controllers/follow_joint_trajectory", FollowJointTrajectoryAction)
            if client.wait_for_server(self.rospy.Duration(0.5)):
                client.cancel_all_goals()
                return SkillResult.build(True, StatusCode.ESTOP, "active MoveIt trajectory goals cancelled; no controller estop verified", start)
        except Exception as exc:
            return SkillResult.build(False, StatusCode.NOT_IMPLEMENTED, "no verified physical stop API available", start, {"error": repr(exc)})
        return SkillResult.build(False, StatusCode.NOT_IMPLEMENTED, "MoveIt trajectory action server unavailable; no physical stop API verified", start)

    def _call_joint_moveit(
        self,
        service_name: str,
        joint_states,
        joint_endpose,
        gripper: float,
        start: float,
        action: str,
        verification_mode: str = "none",
        extra: Optional[dict] = None,
    ) -> SkillResult:
        speed = float(self.safety.get("max_speed_scaling", 0.1))
        speed = max(1e-6, min(0.2, speed))
        acceleration = float(self.safety.get("max_acceleration_scaling", speed))
        acceleration = max(1e-6, min(0.2, acceleration))
        preview = self.build_moveit_request_preview(service_name, joint_states, joint_endpose, gripper, speed, acceleration)
        self.last_preview = preview
        if not self.physical_motion_enabled:
            outputs = {"dry_run": True, "moveit_request_preview": preview}
            outputs.update(extra or {})
            return SkillResult.build(True, StatusCode.OK, action + " dry-run MoveIt request preview", start, outputs)
        pre_state = self.validate_live_state()
        if not pre_state.success:
            return SkillResult.build(
                False,
                pre_state.status_code,
                "fresh joint state required before physical motion",
                start,
                {"pre_command_state": pre_state.outputs, **(extra or {})},
            )
        joint_limits = self._load_joint_limits()
        joint_validation_error = self._validate_joint_target(joint_states, joint_limits)
        if joint_validation_error:
            return SkillResult.build(
                False,
                StatusCode.SAFETY_VIOLATION,
                joint_validation_error,
                start,
                {"joint_limits": joint_limits, "joint_limit_source": self.JOINT_LIMIT_SOURCE, **(extra or {})},
            )
        gripper_validation_error = self._validate_gripper_target(gripper)
        if gripper_validation_error:
            return SkillResult.build(
                False,
                StatusCode.SAFETY_VIOLATION,
                gripper_validation_error,
                start,
                {
                    "gripper_target": float(gripper),
                    "gripper_limits_m": [self._gripper_min_m(), self._gripper_max_m()],
                    "gripper_limit_source": self.GRIPPER_LIMIT_SOURCE,
                    **(extra or {}),
                },
            )
        try:
            from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlRequest

            self.rospy.wait_for_service(service_name, timeout=float(self.safety.get("command_timeout_s", 20.0)))
            proxy = self.rospy.ServiceProxy(service_name, JointMoveitCtrl)
            request = JointMoveitCtrlRequest()
            request.joint_states = list(joint_states)
            request.gripper = float(gripper)
            request.joint_endpose = list(joint_endpose)
            request.max_velocity = speed
            request.max_acceleration = acceleration
            response = proxy(request)
            ok = bool(getattr(response, "status", False))
            outputs = {
                "service": service_name,
                "service_type": "moveit_ctrl/JointMoveitCtrl",
                "error_code": int(getattr(response, "error_code", -1)),
                "max_velocity": speed,
                "max_acceleration": acceleration,
                "service_response_success": ok,
                "joint_limits": joint_limits,
                "joint_limit_source": self.JOINT_LIMIT_SOURCE,
                "gripper_limits_m": [self._gripper_min_m(), self._gripper_max_m()],
                "gripper_limit_source": self.GRIPPER_LIMIT_SOURCE,
                "pre_command_state": pre_state.outputs,
            }
            outputs.update(extra or {})
            if ok:
                reached = self._verify_reached(
                    verification_mode,
                    joint_states,
                    joint_endpose,
                    gripper,
                    pre_state.outputs,
                )
                outputs["pose_reached_verification"] = reached.outputs
                if not reached.success:
                    return SkillResult.build(False, reached.status_code, reached.message, start, outputs)
            return SkillResult.build(ok, StatusCode.OK if ok else StatusCode.CONTROLLER_FAILURE, action + " executed via MoveIt service", start, outputs)
        except Exception as exc:
            return SkillResult.build(False, StatusCode.CONTROLLER_FAILURE, action + " failed via MoveIt service", start, {"service": service_name, "error": repr(exc), **(extra or {})})

    def build_moveit_request_preview(self, service_name: str, joint_states, joint_endpose, gripper: float, max_velocity: float, max_acceleration: float) -> dict:
        return {
            "service": service_name,
            "service_type": "moveit_ctrl/JointMoveitCtrl",
            "request_fields_in_order": ["joint_states", "gripper", "joint_endpose", "max_velocity", "max_acceleration"],
            "joint_states": [float(value) for value in joint_states],
            "gripper": float(gripper),
            "joint_endpose": [float(value) for value in joint_endpose],
            "max_velocity": float(max_velocity),
            "max_acceleration": float(max_acceleration),
            "will_call_service": bool(self.physical_motion_enabled),
        }

    def _verify_reached(self, verification_mode: str, joint_states, joint_endpose, gripper: float, pre_state_outputs: dict) -> SkillResult:
        start = monotonic()
        timeout_s = float(self.safety.get("pose_reached_timeout_s", 5.0))
        tolerance = float(self.safety.get("joint_tolerance_rad", 0.03))
        deadline = self.rospy.Time.now() + self.rospy.Duration(timeout_s)
        if verification_mode == "joint_target":
            initial_positions = list(pre_state_outputs.get("positions", []))
            initial_error = self._max_abs_joint_error(joint_states, initial_positions)
            best_error = float("inf")
            last_positions = initial_positions
            last_gripper = pre_state_outputs.get("gripper_position")
            gripper_tol = float(self.safety.get("gripper_tolerance_m", 0.005))
            while not self.rospy.is_shutdown() and self.rospy.Time.now() < deadline:
                state = self.read_joint_state(timeout_s=0.5)
                if state.success:
                    measured_positions = list(state.outputs.get("positions", []))
                    last_positions = measured_positions
                    last_gripper = state.outputs.get("gripper_position")
                    current_error = self._max_abs_joint_error(joint_states, measured_positions)
                    best_error = min(best_error, current_error)
                    gripper_verified = None
                    if last_gripper is not None and math.isfinite(float(last_gripper)):
                        gripper_verified = abs(float(last_gripper) - float(gripper)) <= gripper_tol
                    if within_joint_tolerance(joint_states, measured_positions, tolerance):
                        if gripper_verified is False:
                            return SkillResult.build(
                                False,
                                StatusCode.POSE_NOT_REACHED,
                                "gripper target was not reached",
                                start,
                                {
                                    **state.outputs,
                                    "commanded_gripper": float(gripper),
                                    "gripper_tolerance_m": gripper_tol,
                                    "gripper_result_verified": False,
                                },
                            )
                        return SkillResult.build(
                            True,
                            StatusCode.OK,
                            "joint target reached",
                            start,
                            {
                                **state.outputs,
                                "commanded": list(joint_states),
                                "joint_tolerance_rad": tolerance,
                                "initial_max_abs_error_rad": initial_error,
                                "best_max_abs_error_rad": best_error,
                                "gripper_result_verified": gripper_verified,
                            },
                        )
                self.rospy.sleep(0.05)
            final_error = self._max_abs_joint_error(joint_states, last_positions)
            return SkillResult.build(
                False,
                StatusCode.POSE_NOT_REACHED,
                "joint target was not reached",
                start,
                {
                    "commanded": list(joint_states),
                    "joint_tolerance_rad": tolerance,
                    "initial_max_abs_error_rad": initial_error,
                    "best_max_abs_error_rad": best_error,
                    "final_max_abs_error_rad": final_error,
                    "lack_of_progress": best_error >= max(0.0, initial_error - 0.002),
                    "last_measured_positions": last_positions,
                    "commanded_gripper": float(gripper),
                    "last_measured_gripper": last_gripper,
                    "gripper_result_verified": (
                        None
                        if last_gripper is None
                        else abs(float(last_gripper) - float(gripper)) <= gripper_tol
                    ),
                },
            )
        if verification_mode == "end_pose":
            return self._verify_end_pose(joint_endpose, start, timeout_s)
        return SkillResult.build(True, StatusCode.OK, "no pose verification required", start)

    def _verify_end_pose(self, joint_endpose, start: float, timeout_s: float) -> SkillResult:
        try:
            from geometry_msgs.msg import PoseStamped
        except Exception as exc:
            return SkillResult.build(False, StatusCode.POSE_NOT_REACHED, "PoseStamped unavailable for end-pose verification", start, {"error": repr(exc)})
        pos_tol = float(self.safety.get("position_tolerance_m", 0.02))
        deadline = self.rospy.Time.now() + self.rospy.Duration(timeout_s)
        target = [float(value) for value in joint_endpose]
        last = None
        while not self.rospy.is_shutdown() and self.rospy.Time.now() < deadline:
            try:
                msg = self.rospy.wait_for_message(self.END_POSE_TOPIC, PoseStamped, timeout=0.5)
                current = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                last = current
                distance = sum((float(a) - float(b)) ** 2 for a, b in zip(target[:3], current)) ** 0.5
                if distance <= pos_tol:
                    return SkillResult.build(True, StatusCode.OK, "end pose target reached", start, {"measured_position": current, "position_error_m": distance})
            except Exception:
                pass
        return SkillResult.build(False, StatusCode.POSE_NOT_REACHED, "end pose target was not reached", start, {"target": target, "last_measured_position": last, "position_tolerance_m": pos_tol})

    def _load_joint_limits(self) -> dict:
        root = ET.fromstring(self.rospy.get_param("/robot_description"))
        limits = {}
        wanted = set(self.expected_joint_names)
        for joint in root.findall("joint"):
            name = joint.attrib.get("name")
            if name not in wanted:
                continue
            limit = joint.find("limit")
            if limit is None:
                continue
            limits[name] = (
                float(limit.attrib.get("lower", "-3.141592653589793")),
                float(limit.attrib.get("upper", "3.141592653589793")),
            )
        missing = [name for name in self.expected_joint_names if name not in limits]
        if missing:
            raise RuntimeError("missing live joint limits for " + ", ".join(missing))
        return limits

    def _validate_joint_target(self, joint_states, joint_limits: dict) -> Optional[str]:
        if len(joint_states) != len(self.expected_joint_names):
            return "joint target length does not match expected arm joints"
        for name, value in zip(self.expected_joint_names, joint_states):
            scalar = float(value)
            if not math.isfinite(scalar):
                return f"{name} target must be finite"
            lower, upper = joint_limits[name]
            if scalar < lower or scalar > upper:
                return f"{name} target {scalar:.6f} rad is outside verified joint limits [{lower:.6f}, {upper:.6f}]"
        return None

    def _validate_gripper_target(self, gripper: float) -> Optional[str]:
        scalar = float(gripper)
        if not math.isfinite(scalar):
            return "gripper target must be finite"
        lower = self._gripper_min_m()
        upper = self._gripper_max_m()
        if scalar < lower or scalar > upper:
            return f"gripper target {scalar:.6f} m is outside verified live range [{lower:.6f}, {upper:.6f}]"
        return None

    def _gripper_min_m(self) -> float:
        return float(self.safety.get("gripper_min_m", 0.0))

    def _gripper_max_m(self) -> float:
        return float(self.safety.get("gripper_max_m", 0.06))

    @staticmethod
    def _max_abs_joint_error(commanded, measured) -> float:
        if len(commanded) != len(measured):
            return float("inf")
        return max(abs(float(a) - float(b)) for a, b in zip(commanded, measured)) if commanded else 0.0
