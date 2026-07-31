#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import time

from _bootstrap import add_repo_src_to_syspath

add_repo_src_to_syspath()

from piper_on_bunker.hardware.piper_x_trajectory_control import PIPER_X_TRAJECTORY_JOINTS
from piper_on_bunker.hardware.piper_x_trajectory_control import PiperXTrajectoryPoint
from piper_on_bunker.hardware.piper_x_trajectory_control import maximum_endpoint_error
from piper_on_bunker.hardware.piper_x_trajectory_control import resample_trajectory
from piper_on_bunker.hardware.piper_x_trajectory_control import validate_trajectory_joint_names
from piper_on_bunker.hardware.piper_x_trajectory_control import validate_trajectory_points


PYAGXARM_REPO = "https://github.com/agilexrobotics/pyAgxArm"
PYAGXARM_COMMIT = "cc498c00af0bcb9e297943e94f4792c0e3ee5b2c"
PYAGXARM_ARM_MODEL = "PIPER_X"
PYAGXARM_FIRMWARE_PROFILE = "V189"
PYAGXARM_MOTION_MODE = "js"
PYAGXARM_COMMAND_PRIMITIVE = "AgxArm.move_js"


class PyAgxArmPiperXJointSpaceAdapter:
    def __init__(self, can_name: str, *, bitrate: int = 1_000_000) -> None:
        try:
            from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config
            import pyAgxArm
        except Exception as exc:
            raise ImportError(
                "pyAgxArm import failed; run tools/install_piper_x_feedback_dependency.sh "
                f"for commit {PYAGXARM_COMMIT}: {exc!r}"
            ) from exc

        self.module_path = str(getattr(pyAgxArm, "__file__", "unknown"))
        self.config = create_agx_arm_config(
            robot=ArmModel.PIPER_X,
            comm="can",
            firmeware_version=PiperFW.V189,
            interface="socketcan",
            channel=can_name,
            bitrate=int(bitrate),
        )
        self._arm = AgxArmFactory.create_arm(self.config)
        self.connected = False
        self.configured = False

    def connect(self) -> None:
        self._arm.connect()
        self.connected = True

    def configure_joint_space_stream(self, *, speed_percent: int) -> None:
        percent = int(speed_percent)
        if percent < 0 or percent > 100:
            raise ValueError("speed percent must be in [0, 100]")
        self._arm.set_follower_mode()
        self._arm.set_speed_percent(percent)
        self._arm.set_motion_mode(PYAGXARM_MOTION_MODE)
        enabled = self._arm.enable(255)
        if enabled is False:
            raise RuntimeError("pyAgxArm enable(255) returned False")
        self.configured = True

    def write_joints_rad(self, joints_rad: list[float]) -> None:
        values = [float(value) for value in joints_rad]
        if len(values) != 6:
            raise ValueError("move_js requires exactly six joints")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("move_js target contains non-finite joints")
        self._arm.move_js(values)


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
        rospy.set_param("~controller_type", "piper_x_pyagxarm_follow_joint_trajectory")
        rospy.set_param("~backend", "pyagxarm_piper_x")
        rospy.set_param("~can_interface", args.can)
        rospy.set_param("~feedback_topic", args.feedback_topic)
        rospy.set_param("~arm_model", PYAGXARM_ARM_MODEL)
        rospy.set_param("~firmware_profile", PYAGXARM_FIRMWARE_PROFILE)
        rospy.set_param("~motion_mode", PYAGXARM_MOTION_MODE)
        rospy.set_param("~command_primitive", PYAGXARM_COMMAND_PRIMITIVE)
        rospy.set_param("~dependency_repo", PYAGXARM_REPO)
        rospy.set_param("~dependency_commit", PYAGXARM_COMMIT)
        rospy.set_param("~connects_on_first_goal", True)
        rospy.set_param("~motion_commanded_at_startup", False)
        rospy.set_param("~speed_percent", int(args.speed_percent))
        rospy.set_param("~command_rate_hz", float(args.command_rate_hz))
        rospy.set_param("~endpoint_tolerance_rad", float(args.endpoint_tolerance_rad))
        rospy.set_param("~settle_timeout_s", float(args.settle_timeout_s))
        rospy.loginfo(
            "PiPER-X pyAgxArm MoveIt trajectory controller ready on %s; hardware connects only on first accepted goal",
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

    def _connect_for_goal(self):
        if self.arm is not None:
            return self.arm
        arm = PyAgxArmPiperXJointSpaceAdapter(self.args.can)
        try:
            arm.connect()
        except Exception as exc:
            raise RuntimeError(f"pyAgxArm connection failure: {exc!r}") from exc
        initial_joints = self._current_positions()
        try:
            arm.configure_joint_space_stream(speed_percent=int(self.args.speed_percent))
        except Exception as exc:
            raise RuntimeError(f"pyAgxArm follower/js/enable initialization failure: {exc!r}") from exc
        try:
            arm.write_joints_rad(initial_joints)
        except Exception as exc:
            raise RuntimeError(f"pyAgxArm initial current-pose command failure: {exc!r}") from exc
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
            try:
                arm.write_joints_rad(command.positions_rad)
            except Exception as exc:
                result.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
                result.error_string = f"pyAgxArm move_js failed at command {index + 1}/{len(commands)}: {exc!r}"
                self.server.set_aborted(result, result.error_string)
                return
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
            try:
                arm.write_joints_rad(final_positions)
            except Exception as exc:
                result.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
                result.error_string = f"pyAgxArm final hold move_js failed: {exc!r}"
                self.server.set_aborted(result, result.error_string)
                return
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
            arm.write_joints_rad(self._current_positions())
        except Exception as exc:
            self.rospy.logwarn("Failed to hold current PiPER-X feedback on stop: %s", exc)


def main() -> int:
    parser = argparse.ArgumentParser(description="PiPER-X MoveIt FollowJointTrajectory action server using pyAgxArm move_js.")
    parser.add_argument("--action-name", default="arm_controllers/follow_joint_trajectory")
    parser.add_argument("--feedback-topic", default="/joint_states")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--speed-percent", type=int, default=30)
    parser.add_argument("--command-rate-hz", type=float, default=50.0)
    parser.add_argument("--endpoint-tolerance-rad", type=float, default=0.03)
    parser.add_argument("--settle-timeout-s", type=float, default=3.0)
    parser.add_argument("--first-point-blend-s", type=float, default=0.25)
    args = parser.parse_args()

    import rospy

    rospy.init_node("piper_x_moveit_sdk_trajectory_controller", anonymous=False)
    PiperXMoveItSdkTrajectoryController(args)
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
