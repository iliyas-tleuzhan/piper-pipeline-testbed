#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time

from _bootstrap import add_repo_src_to_syspath

add_repo_src_to_syspath()

from piper_on_bunker.manipulation.moveit_aruco_touch import EXPECTED_JOINT_NAMES
from piper_on_bunker.manipulation.moveit_aruco_touch import JointStateSnapshot
from piper_on_bunker.manipulation.moveit_aruco_touch import extract_named_joint_state
from piper_on_bunker.manipulation.moveit_aruco_touch import load_taught_poses
from piper_on_bunker.manipulation.moveit_aruco_touch import load_touch_config
from piper_on_bunker.manipulation.moveit_aruco_touch import validate_joint_values


def _snapshot_from_args(args) -> JointStateSnapshot | None:
    if args.positions:
        values = [float(v) for v in args.positions.split(",")]
        return JointStateSnapshot(list(EXPECTED_JOINT_NAMES), values, 0.0, [0.0] * 6, time.time(), "offline --positions")
    return None


def _read_live_joint_state(topic: str, max_age_s: float, max_velocity: float) -> JointStateSnapshot:
    import rospy
    from sensor_msgs.msg import JointState

    if not rospy.get_node_uri():
        rospy.init_node("inspect_piper_x_moveit_pose", anonymous=True, disable_signals=True)
    msg = rospy.wait_for_message(topic, JointState, timeout=max_age_s)
    return extract_named_joint_state(
        names=list(msg.name),
        positions=list(msg.position),
        velocities=list(msg.velocity),
        stamp_s=float(msg.header.stamp.to_sec()),
        now_s=float(rospy.Time.now().to_sec()),
        max_age_s=max_age_s,
        max_abs_velocity_rad_s=max_velocity,
        source_topic=topic,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect the current or saved PiPER-X MoveIt taught pose without moving.")
    parser.add_argument("--config", default="piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml")
    parser.add_argument("--pose-name", default="")
    parser.add_argument("--positions", default="", help="Offline comma-separated six-joint sample for validation.")
    parser.add_argument("--joint-topic", default="/joint_states_single")
    parser.add_argument("--max-velocity-rad-s", type=float, default=0.01)
    args = parser.parse_args()

    config = load_touch_config(args.config)
    poses = load_taught_poses(config.taught_pose_manifest)
    snapshot = _snapshot_from_args(args)
    if snapshot is None:
        snapshot = _read_live_joint_state(args.joint_topic, config.max_joint_state_age_s, args.max_velocity_rad_s)
    output = {
        "config": args.config,
        "planning_group": config.planning_group,
        "end_effector_link": config.end_effector_link,
        "manifest": config.taught_pose_manifest,
        "saved_pose_names": sorted(poses),
        "robot_model": config.robot_model,
        "piper_x_model_verified": config.piper_x_model_verified,
        "timestamp_unix_s": time.time(),
        "motion_commanded": False,
    }
    if args.pose_name:
        output["selected_pose"] = None if args.pose_name not in poses else poses[args.pose_name].__dict__
    output["joint_limit_validation"] = {"passed": True, "error": None}
    try:
        validate_joint_values(snapshot.joint_names, snapshot.positions, config.joint_limits)
    except ValueError as exc:
        output["joint_limit_validation"] = {"passed": False, "error": str(exc)}
    output["current_joint_state"] = snapshot.__dict__
    print(json.dumps(output, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
