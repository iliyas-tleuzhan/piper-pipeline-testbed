#!/usr/bin/env python3
"""Publish arm-only PiPER-X joint states for MoveIt.

The PiPER ROS driver exposes gripper feedback using normal-PiPER names on
``/joint_states`` in some stacks. The PiPER-X MoveIt model uses ``gripper`` for
the passive gripper coordinate, so this relay republishes the authoritative six
arm joints plus optional ``gripper`` feedback from ``/joint_states_single`` to
``/joint_states``. It never publishes commands.
"""

from __future__ import annotations

import argparse
import math


EXPECTED_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
OPTIONAL_JOINTS = ["gripper"]


def main() -> int:
    parser = argparse.ArgumentParser(description="Relay PiPER-X arm-only joint states for MoveIt.")
    parser.add_argument("--input-topic", default="/joint_states_single")
    parser.add_argument("--output-topic", default="/joint_states")
    args = parser.parse_args()

    import rospy
    from sensor_msgs.msg import JointState

    rospy.init_node("piper_x_arm_joint_state_relay", anonymous=False)
    pub = rospy.Publisher(args.output_topic, JointState, queue_size=10)

    def on_joint_state(msg: JointState) -> None:
        by_name = {name: index for index, name in enumerate(msg.name)}
        if any(name not in by_name for name in EXPECTED_ARM_JOINTS):
            rospy.logwarn_throttle(5.0, "Skipping joint state without all PiPER-X arm joints: %s", list(msg.name))
            return
        output_names = list(EXPECTED_ARM_JOINTS) + [name for name in OPTIONAL_JOINTS if name in by_name]
        out = JointState()
        out.header = msg.header
        out.name = output_names
        out.position = [float(msg.position[by_name[name]]) for name in output_names]
        if msg.velocity and len(msg.velocity) >= len(msg.name):
            out.velocity = [float(msg.velocity[by_name[name]]) for name in output_names]
        if msg.effort and len(msg.effort) >= len(msg.name):
            out.effort = [float(msg.effort[by_name[name]]) for name in output_names]
        if not all(math.isfinite(value) for value in out.position):
            rospy.logwarn_throttle(5.0, "Skipping non-finite PiPER-X arm joint state")
            return
        pub.publish(out)

    rospy.Subscriber(args.input_topic, JointState, on_joint_state, queue_size=10)
    rospy.loginfo("Relaying PiPER-X joints %s plus %s from %s to %s", EXPECTED_ARM_JOINTS, OPTIONAL_JOINTS, args.input_topic, args.output_topic)
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
