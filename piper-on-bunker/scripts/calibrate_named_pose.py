#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path

import yaml

from piper_on_bunker.hardware.joint_state import DEFAULT_ARM_JOINT_NAMES, map_joint_state
from piper_on_bunker.safety import SAFE_NAMED_POSES


def read_joint_state(timeout_s: float, expected_joint_names):
    try:
        import rospy
        from sensor_msgs.msg import JointState
    except Exception as exc:
        raise RuntimeError("Calibration must run in the ROS Noetic environment with rospy and sensor_msgs") from exc
    if not rospy.get_node_uri():
        rospy.init_node("piper_named_pose_calibration", anonymous=True, disable_signals=True)
    msg = rospy.wait_for_message("/joint_states_single", JointState, timeout=timeout_s)
    mapped = map_joint_state(msg.name, msg.position, float(msg.header.stamp.to_sec()), expected_joint_names)
    return mapped.names, mapped.arm_joint_names, mapped.arm_positions, mapped.gripper_position, mapped.stamp_s


def read_end_pose(timeout_s: float):
    try:
        import rospy
        from geometry_msgs.msg import PoseStamped
    except Exception:
        return None
    try:
        msg = rospy.wait_for_message("/end_pose", PoseStamped, timeout=timeout_s)
        return {
            "frame_id": msg.header.frame_id,
            "stamp": float(msg.header.stamp.to_sec()),
            "position": [float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z)],
            "orientation_xyzw": [
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
                float(msg.pose.orientation.w),
            ],
        }
    except Exception as exc:
        return {"error": repr(exc)}


def main() -> None:
    parser = argparse.ArgumentParser(description="Record the current live PiPER joint state as a named pose.")
    parser.add_argument("name", choices=sorted(SAFE_NAMED_POSES))
    parser.add_argument("--config", default="piper-on-bunker/config/piper_laptop_hardware.local.yaml")
    parser.add_argument("--joint-names", nargs=6, default=DEFAULT_ARM_JOINT_NAMES)
    parser.add_argument("--update", action="store_true", help="Allow overwriting an existing calibrated pose")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    path = Path(args.config)
    data = {}
    if path.exists():
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    named_poses = data.setdefault("named_poses", {})
    if named_poses.get(args.name) and not args.update:
        raise SystemExit(f"{args.name} already exists in {path}; rerun with --update to overwrite it")
    names, arm_joint_names, joints, gripper, stamp = read_joint_state(args.timeout, args.joint_names)
    named_poses[args.name] = joints
    records = data.setdefault("calibration_records", {})
    records[args.name] = {
        "recorded_at": datetime.now(timezone.utc).isoformat(),
        "source_topic": "/joint_states_single",
        "source_stamp": float(stamp),
        "ros_joint_names": names,
        "arm_joint_names": arm_joint_names,
        "gripper_position": gripper,
        "joint_units": "radians_from_joint_states_single",
        "end_pose": read_end_pose(1.0),
    }
    data.setdefault("local_activation", {}).setdefault("physical_motion_enabled", False)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    print(f"Recorded {args.name} to ignored local config: {path}")
    print("Physical motion remains disabled unless local_activation.physical_motion_enabled is explicitly true.")


if __name__ == "__main__":
    main()
