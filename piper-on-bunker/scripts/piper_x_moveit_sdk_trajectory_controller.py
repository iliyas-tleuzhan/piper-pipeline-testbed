#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

from _bootstrap import add_repo_src_to_syspath

add_repo_src_to_syspath()

from piper_on_bunker.hardware.piper_x_trajectory_control import PIPER_X_TRAJECTORY_JOINTS
from piper_on_bunker.hardware.piper_x_trajectory_control import PiperXTrajectoryPoint
from piper_on_bunker.hardware.piper_x_trajectory_control import maximum_endpoint_error
from piper_on_bunker.hardware.piper_x_trajectory_control import joints_rad_to_raw_mdeg
from piper_on_bunker.hardware.piper_x_trajectory_control import resample_trajectory
from piper_on_bunker.hardware.piper_x_trajectory_control import validate_trajectory_joint_names
from piper_on_bunker.hardware.piper_x_trajectory_control import validate_trajectory_points


PIPER_SDK_COMMAND_PRIMITIVE = "C_PiperInterface_V2.JointCtrl"


class PiperSdkJointCtrlAdapter:
    def __init__(self, can_name: str) -> None:
        import piper_sdk

        interface_cls = getattr(piper_sdk, "C_PiperInterface_V2", None)
        if interface_cls is None:
            interface_cls = getattr(piper_sdk, "C_PiperInterface")
        self._piper = interface_cls(can_name)

    def connect(self) -> None:
        self._piper.ConnectPort()

    def enable_all(self) -> None:
        if hasattr(self._piper, "EnableArm"):
            self._piper.EnableArm(7)
        elif hasattr(self._piper, "EnableArmStandbyMode"):
            self._piper.EnableArmStandbyMode(7)
        else:
            raise AttributeError("piper_sdk does not expose an arm enable method")

    def configure_motion(self, *, speed_percent: int, high_follow: bool) -> None:
        follow_mode = 0xAD if high_follow else 0x00
        if hasattr(self._piper, "MotionCtrl_2"):
            self._piper.MotionCtrl_2(0x01, 0x01, int(speed_percent), follow_mode)
        else:
            self._piper.ModeCtrl(0x01, 0x01, int(speed_percent), follow_mode)

    def write_joints_raw(self, joints_raw: list[int]) -> None:
        if len(joints_raw) != 6:
            raise ValueError("JointCtrl requires exactly six joints")
        self._piper.JointCtrl(*[int(value) for value in joints_raw])


class PiperXMoveItSdkTrajectoryController:
    def __init__(self, args: argparse.Namespace) -> None:
        import actionlib
        import rospy
        from control_msgs.msg import FollowJointTrajectoryAction
        from sensor_msgs.msg import JointState

        self.rospy = rospy
        self.args = args
        self.arm = None
        self.latest_positions: dict[str, float] = {}
        self.latest_feedback_stamp_s: float | None = None
        self.feedback_sub = rospy.Subscriber(args.feedback_topic, JointState, self._on_feedback, queue_size=1)
        self.server = actionlib.SimpleActionServer(
            args.action_name,
            FollowJointTrajectoryAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self.server.start()
        rospy.set_param("~controller_type", "piper_x_sdk_jointctrl_follow_joint_trajectory")
        rospy.set_param("~can_interface", args.can)
        rospy.set_param("~feedback_topic", args.feedback_topic)
        rospy.set_param("~sdk_command_primitive", PIPER_SDK_COMMAND_PRIMITIVE)
        rospy.set_param("~connects_on_first_goal", True)
        rospy.set_param("~motion_commanded_at_startup", False)
        rospy.set_param("~speed_percent", int(args.speed_percent))
        rospy.set_param("~command_rate_hz", float(args.command_rate_hz))
        rospy.set_param("~endpoint_tolerance_rad", float(args.endpoint_tolerance_rad))
        rospy.set_param("~settle_timeout_s", float(args.settle_timeout_s))
        rospy.loginfo(
            "PiPER-X MoveIt SDK trajectory controller ready on %s; hardware connects only on first accepted goal",
            args.action_name,
        )

    def _on_feedback(self, msg) -> None:
        self.latest_positions = {str(name): float(value) for name, value in zip(msg.name, msg.position)}
        try:
            self.latest_feedback_stamp_s = float(msg.header.stamp.to_sec())
        except Exception:
            self.latest_feedback_stamp_s = None

    def _current_positions(self) -> list[float]:
        if not all(name in self.latest_positions for name in PIPER_X_TRAJECTORY_JOINTS):
            raise RuntimeError(f"missing live feedback joints on {self.args.feedback_topic}")
        return [self.latest_positions[name] for name in PIPER_X_TRAJECTORY_JOINTS]

    def _current_raw(self) -> list[int]:
        return joints_rad_to_raw_mdeg(self._current_positions())

    def _connect_for_goal(self):
        if self.arm is not None:
            return self.arm
        arm = PiperSdkJointCtrlAdapter(self.args.can)
        arm.connect()
        initial_joints = self._current_raw()
        arm.enable_all()
        arm.configure_motion(speed_percent=int(self.args.speed_percent), high_follow=bool(self.args.high_follow))
        arm.write_joints_raw(initial_joints)
        self.arm = arm
        return arm

    def _goal_points(self, goal) -> list[PiperXTrajectoryPoint]:
        validate_trajectory_joint_names(list(goal.trajectory.joint_names))
        points = [
            PiperXTrajectoryPoint(
                positions_rad=[float(value) for value in point.positions],
                time_from_start_s=float(point.time_from_start.to_sec()),
            )
            for point in goal.trajectory.points
        ]
        validate_trajectory_points(points)
        return points

    def _publish_feedback(self, desired_positions: list[float]) -> None:
        from control_msgs.msg import FollowJointTrajectoryFeedback

        feedback = FollowJointTrajectoryFeedback()
        feedback.joint_names = list(PIPER_X_TRAJECTORY_JOINTS)
        feedback.desired.positions = list(desired_positions)
        feedback.actual.positions = [self.latest_positions.get(name, 0.0) for name in PIPER_X_TRAJECTORY_JOINTS]
        try:
            feedback.error.positions = [
                float(actual) - float(desired)
                for actual, desired in zip(feedback.actual.positions, desired_positions)
            ]
        except Exception:
            pass
        self.server.publish_feedback(feedback)

    def _endpoint_error(self, final_positions: list[float]) -> float:
        return maximum_endpoint_error(self._current_positions(), final_positions)

    def _execute(self, goal) -> None:
        from control_msgs.msg import FollowJointTrajectoryResult

        result = FollowJointTrajectoryResult()
        try:
            points = self._goal_points(goal)
            current_positions = self._current_positions()
            commands = resample_trajectory(
                points,
                command_rate_hz=float(self.args.command_rate_hz),
                current_positions_rad=current_positions,
                first_point_blend_s=float(self.args.first_point_blend_s),
            )
            arm = self._connect_for_goal()
        except Exception as exc:
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            result.error_string = str(exc)
            self.server.set_aborted(result, result.error_string)
            return

        original_intervals = [
            float(after.time_from_start_s) - float(before.time_from_start_s)
            for before, after in zip(points, points[1:])
        ]
        original_max_interval = max(original_intervals, default=0.0)
        planned_duration = float(commands[-1].time_from_start_s)
        rospy = self.rospy
        rospy.loginfo(
            "Executing PiPER-X trajectory: original_points=%d original_max_waypoint_interval_s=%.3f "
            "command_rate_hz=%.1f streamed_commands=%d planned_duration_s=%.3f sdk_speed_percent=%d",
            len(points),
            original_max_interval,
            float(self.args.command_rate_hz),
            len(commands),
            planned_duration,
            int(self.args.speed_percent),
        )
        started = time.monotonic()
        next_progress_log = started
        for index, command in enumerate(commands):
            target_time = started + float(command.time_from_start_s)
            while not rospy.is_shutdown():
                if self.server.is_preempt_requested():
                    self._hold_current_feedback(arm)
                    self.server.set_preempted(text="PiPER-X trajectory preempted; held current feedback")
                    return
                remaining = target_time - time.monotonic()
                if remaining <= 0.0:
                    break
                time.sleep(min(0.002, remaining))
            raw = joints_rad_to_raw_mdeg(command.positions_rad)
            arm.write_joints_raw(raw)
            self._publish_feedback(command.positions_rad)
            now = time.monotonic()
            if now >= next_progress_log:
                rospy.loginfo(
                    "PiPER-X trajectory progress: command=%d/%d elapsed_s=%.2f planned_s=%.2f",
                    index + 1,
                    len(commands),
                    now - started,
                    planned_duration,
                )
                next_progress_log = now + 1.0

        final_positions = list(commands[-1].positions_rad)
        settle_deadline = time.monotonic() + float(self.args.settle_timeout_s)
        period = 1.0 / float(self.args.command_rate_hz)
        while not rospy.is_shutdown() and time.monotonic() <= settle_deadline:
            if self.server.is_preempt_requested():
                self._hold_current_feedback(arm)
                self.server.set_preempted(text="PiPER-X trajectory preempted during endpoint settle; held current feedback")
                return
            arm.write_joints_raw(joints_rad_to_raw_mdeg(final_positions))
            self._publish_feedback(final_positions)
            error = self._endpoint_error(final_positions)
            if error <= float(self.args.endpoint_tolerance_rad):
                result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
                self.server.set_succeeded(result, f"PiPER-X trajectory endpoint reached; max_error_rad={error:.4f}")
                return
            time.sleep(period)

        error = self._endpoint_error(final_positions)
        result.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
        result.error_string = f"endpoint settle timeout; max_error_rad={error:.4f}"
        self.server.set_aborted(result, result.error_string)

    def _hold_current_feedback(self, arm) -> None:
        try:
            arm.write_joints_raw(self._current_raw())
        except Exception as exc:
            self.rospy.logwarn("Failed to hold current PiPER-X feedback on stop: %s", exc)


def main() -> int:
    parser = argparse.ArgumentParser(description="PiPER-X MoveIt FollowJointTrajectory action server using piper_sdk JointCtrl.")
    parser.add_argument("--action-name", default="arm_controllers/follow_joint_trajectory")
    parser.add_argument("--feedback-topic", default="/joint_states")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--speed-percent", type=int, default=30)
    parser.add_argument("--command-rate-hz", type=float, default=50.0)
    parser.add_argument("--endpoint-tolerance-rad", type=float, default=0.03)
    parser.add_argument("--settle-timeout-s", type=float, default=3.0)
    parser.add_argument("--first-point-blend-s", type=float, default=0.25)
    parser.add_argument("--high-follow", action="store_true", default=True)
    args = parser.parse_args()

    import rospy

    rospy.init_node("piper_x_moveit_sdk_trajectory_controller", anonymous=False)
    PiperXMoveItSdkTrajectoryController(args)
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
