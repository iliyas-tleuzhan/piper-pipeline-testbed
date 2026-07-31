#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time

from _bootstrap import add_repo_src_to_syspath

add_repo_src_to_syspath()

from piper_on_bunker.manipulation.moveit_aruco_touch import EXPECTED_JOINT_NAMES
from piper_on_bunker.manipulation.moveit_aruco_touch import TaughtPose
from piper_on_bunker.manipulation.moveit_aruco_touch import extract_named_joint_state
from piper_on_bunker.manipulation.moveit_aruco_touch import load_touch_config
from piper_on_bunker.manipulation.moveit_aruco_touch import save_taught_pose_manifest
from piper_on_bunker.manipulation.moveit_aruco_touch import validate_joint_values


def _read_live_joint_state(topic: str, max_age_s: float, max_velocity: float):
    import rospy
    from sensor_msgs.msg import JointState

    if not rospy.get_node_uri():
        rospy.init_node("save_piper_x_moveit_taught_pose", anonymous=True, disable_signals=True)
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
    parser = argparse.ArgumentParser(description="Save a stopped PiPER-X joint state as a MoveIt MVP taught pose; never moves the robot.")
    parser.add_argument("--config", default="piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml")
    parser.add_argument("--pose-name", required=True, choices=["staging", "home", "pre_touch", "touch", "retract", "safe_recovery"])
    parser.add_argument("--positions", default="", help="Offline comma-separated six current joint positions in radians. Normal live use reads --joint-topic.")
    parser.add_argument("--ack", required=True, help="Must be SAVE_STOPPED_POSE.")
    parser.add_argument("--source", default="operator_read_current_joint_state")
    parser.add_argument("--joint-topic", default="/joint_states_single")
    parser.add_argument("--max-velocity-rad-s", type=float, default=0.01)
    args = parser.parse_args()
    if args.ack != "SAVE_STOPPED_POSE":
        raise SystemExit("--ack SAVE_STOPPED_POSE is required before replacing a saved taught pose")
    config = load_touch_config(args.config)
    if args.positions:
        snapshot = extract_named_joint_state(
            names=list(EXPECTED_JOINT_NAMES),
            positions=[float(v) for v in args.positions.split(",")],
            velocities=[0.0] * 6,
            stamp_s=time.time(),
            now_s=time.time(),
            max_age_s=config.max_joint_state_age_s,
            max_abs_velocity_rad_s=args.max_velocity_rad_s,
            source_topic="offline --positions",
        )
    else:
        snapshot = _read_live_joint_state(args.joint_topic, config.max_joint_state_age_s, args.max_velocity_rad_s)
    strict_validation = {"passed": True, "error": None}
    try:
        validate_joint_values(snapshot.joint_names, snapshot.positions, config.joint_limits)
    except ValueError as exc:
        strict_validation = {"passed": False, "error": str(exc)}
    validate_joint_values(snapshot.joint_names, snapshot.positions, config.joint_limits, tolerance_rad=config.taught_pose_limit_tolerance_rad)
    teaching_validation = {
        "passed": True,
        "tolerance_rad": config.taught_pose_limit_tolerance_rad,
        "strict_validation": strict_validation,
    }
    print(json.dumps({"about_to_save": args.pose_name, "joint_state": snapshot.__dict__, "joint_limit_validation": teaching_validation, "motion_commanded": False}, indent=2, sort_keys=True))
    pose = TaughtPose(args.pose_name, list(snapshot.joint_names), list(snapshot.positions), args.source)
    metadata = {
        "profile_id": config.profile_id,
        "task_id": config.task_id,
        "planning_group": config.planning_group,
        "end_effector_link": config.end_effector_link,
        "robot_model": config.robot_model,
        "robot_urdf_candidate_path": config.robot_urdf_candidate_path,
        "robot_urdf_candidate_sha256": config.robot_urdf_candidate_sha256,
        "piper_x_model_verified": config.piper_x_model_verified,
        "taught_pose_limit_tolerance_rad": config.taught_pose_limit_tolerance_rad,
        "joint_limit_validation": teaching_validation,
        "source_topic": snapshot.source_topic,
        "source_stamp_s": snapshot.stamp_s,
        "motion_commanded": False,
    }
    save_taught_pose_manifest(config.taught_pose_manifest, pose, metadata)
    print(json.dumps({"saved": True, "manifest": config.taught_pose_manifest, "pose": pose.__dict__, "timestamp_unix_s": time.time()}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
