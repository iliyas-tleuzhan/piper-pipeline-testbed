from __future__ import annotations

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
        return self._call_joint_moveit("/joint_moveit_ctrl_arm", joint_states=joints, joint_endpose=[0.0] * 7, gripper=0.0, start=start, action="move_to_named_pose", extra={"pose": name})

    def move_to_pose(self, pose: Pose) -> SkillResult:
        start = monotonic()
        return self._call_joint_moveit(
            "/joint_moveit_ctrl_endpose",
            joint_states=[0.0] * 6,
            joint_endpose=[pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw],
            gripper=0.0,
            start=start,
            action="move_to_pose",
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

    def _call_joint_moveit(self, service_name: str, joint_states, joint_endpose, gripper: float, start: float, action: str, extra: Optional[dict] = None) -> SkillResult:
        speed = float(self.safety.get("max_speed_scaling", 0.1))
        speed = max(1e-6, min(0.2, speed))
        preview = self.build_moveit_request_preview(service_name, joint_states, joint_endpose, gripper, speed, speed)
        self.last_preview = preview
        if not self.physical_motion_enabled:
            outputs = {"dry_run": True, "moveit_request_preview": preview}
            outputs.update(extra or {})
            return SkillResult.build(True, StatusCode.OK, action + " dry-run MoveIt request preview", start, outputs)
        try:
            from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlRequest

            self.rospy.wait_for_service(service_name, timeout=float(self.safety.get("command_timeout_s", 20.0)))
            proxy = self.rospy.ServiceProxy(service_name, JointMoveitCtrl)
            request = JointMoveitCtrlRequest()
            request.joint_states = list(joint_states)
            request.gripper = float(gripper)
            request.joint_endpose = list(joint_endpose)
            request.max_velocity = speed
            request.max_acceleration = speed
            response = proxy(request)
            ok = bool(getattr(response, "status", False))
            outputs = {
                "service": service_name,
                "service_type": "moveit_ctrl/JointMoveitCtrl",
                "error_code": int(getattr(response, "error_code", -1)),
                "max_velocity": speed,
                "max_acceleration": speed,
            }
            outputs.update(extra or {})
            if ok:
                reached = self._verify_reached(action, joint_states, joint_endpose)
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

    def _verify_reached(self, action: str, joint_states, joint_endpose) -> SkillResult:
        start = monotonic()
        timeout_s = float(self.safety.get("pose_reached_timeout_s", 5.0))
        tolerance = float(self.safety.get("joint_tolerance_rad", 0.03))
        deadline = self.rospy.Time.now() + self.rospy.Duration(timeout_s)
        if action == "move_to_named_pose":
            while not self.rospy.is_shutdown() and self.rospy.Time.now() < deadline:
                state = self.read_joint_state(timeout_s=0.5)
                if state.success and within_joint_tolerance(joint_states, state.outputs.get("positions", []), tolerance):
                    return SkillResult.build(True, StatusCode.OK, "joint target reached", start, state.outputs)
                self.rospy.sleep(0.05)
            return SkillResult.build(False, StatusCode.POSE_NOT_REACHED, "joint target was not reached", start, {"commanded": list(joint_states), "joint_tolerance_rad": tolerance})
        if action == "move_to_pose":
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
