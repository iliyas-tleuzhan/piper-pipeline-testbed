from __future__ import annotations

from time import monotonic
from typing import Optional

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

    def __init__(self, physical_motion_enabled: bool = False, named_poses: Optional[dict] = None, safety: Optional[dict] = None) -> None:
        self.physical_motion_enabled = physical_motion_enabled
        self.named_poses = named_poses or {}
        self.safety = safety or {}
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
            positions = [float(value) for value in msg.position]
            ok = len(positions) >= 6
            code = StatusCode.OK if ok else StatusCode.STALE_STATE
            return SkillResult.build(
                ok,
                code,
                "joint state read" if ok else "joint state has fewer than six arm joints",
                start,
                {"topic": self.JOINT_TOPIC, "names": list(msg.name), "positions": positions, "stamp": float(msg.header.stamp.to_sec())},
            )
        except Exception as exc:
            return SkillResult.build(False, StatusCode.STALE_STATE, "joint state unavailable", start, {"topic": self.JOINT_TOPIC, "error": repr(exc)})

    def inspect_services(self, timeout_s: float = 1.0) -> SkillResult:
        start = monotonic()
        found = {}
        for name in self.MOVEIT_SERVICES:
            try:
                service_class = self.rospy.get_service_class_by_name(name)
                found[name] = {"available": service_class is not None, "type": getattr(service_class, "_type", None)}
            except Exception as exc:
                found[name] = {"available": False, "error": repr(exc)}
        ok = any(value.get("available") for value in found.values())
        return SkillResult.build(ok, StatusCode.OK if ok else StatusCode.CONTROLLER_FAILURE, "MoveIt services inspected", start, found)

    def _motion_disabled(self, action: str, outputs: Optional[dict] = None) -> SkillResult:
        data = outputs or {}
        data["action"] = action
        return SkillResult.build(False, StatusCode.HARDWARE_DISABLED, "physical motion is disabled", monotonic(), data)

    def move_to_named_pose(self, name: str) -> SkillResult:
        start = monotonic()
        if not self.physical_motion_enabled:
            return self._motion_disabled("move_to_named_pose", {"pose": name})
        if name not in self.named_poses or not self.named_poses[name]:
            return SkillResult.build(False, StatusCode.SAFETY_VIOLATION, "named pose is not calibrated", start, {"pose": name})
        joints = [float(value) for value in self.named_poses[name]]
        if len(joints) != 6:
            return SkillResult.build(False, StatusCode.SAFETY_VIOLATION, "named pose must contain six joints", start, {"pose": name})
        return self._call_joint_moveit("/joint_moveit_ctrl_arm", joint_states=joints, joint_endpose=[0.0] * 7, gripper=0.0, start=start, action="move_to_named_pose", extra={"pose": name})

    def move_to_pose(self, pose: Pose) -> SkillResult:
        start = monotonic()
        if not self.physical_motion_enabled:
            return self._motion_disabled("move_to_pose", {"pose": pose.__dict__})
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
        if not self.physical_motion_enabled:
            return self._motion_disabled("press", {"pose": pose.__dict__, "depth_m": depth_m})
        max_depth = float(self.safety.get("max_press_distance_m", 0.015))
        if depth_m <= 0 or depth_m > max_depth:
            return SkillResult.build(False, StatusCode.SAFETY_VIOLATION, "press distance exceeds configured bound", start, {"depth_m": depth_m, "max_press_distance_m": max_depth})
        target = Pose(pose.x, pose.y, pose.z - depth_m, pose.qx, pose.qy, pose.qz, pose.qw, pose.frame_id)
        return self.move_to_pose(target)

    def retract(self) -> SkillResult:
        start = monotonic()
        if not self.physical_motion_enabled:
            return self._motion_disabled("retract")
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
        try:
            from moveit_ctrl.srv import JointMoveitCtrl

            self.rospy.wait_for_service(service_name, timeout=float(self.safety.get("command_timeout_s", 20.0)))
            proxy = self.rospy.ServiceProxy(service_name, JointMoveitCtrl)
            response = proxy(joint_states, float(gripper), joint_endpose, speed, speed)
            ok = bool(getattr(response, "status", False))
            outputs = {
                "service": service_name,
                "service_type": "moveit_ctrl/JointMoveitCtrl",
                "error_code": int(getattr(response, "error_code", -1)),
                "max_velocity": speed,
                "max_acceleration": speed,
            }
            outputs.update(extra or {})
            return SkillResult.build(ok, StatusCode.OK if ok else StatusCode.CONTROLLER_FAILURE, action + " executed via MoveIt service", start, outputs)
        except Exception as exc:
            return SkillResult.build(False, StatusCode.CONTROLLER_FAILURE, action + " failed via MoveIt service", start, {"service": service_name, "error": repr(exc), **(extra or {})})
