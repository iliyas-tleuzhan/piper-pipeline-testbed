#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time

from _bootstrap import add_repo_src_to_syspath

add_repo_src_to_syspath()

from piper_on_bunker.manipulation.moveit_aruco_touch import EXPECTED_JOINT_NAMES
from piper_on_bunker.manipulation.moveit_aruco_touch import TaughtPose
from piper_on_bunker.manipulation.moveit_aruco_touch import load_touch_config
from piper_on_bunker.manipulation.moveit_aruco_touch import save_taught_pose_manifest
from piper_on_bunker.manipulation.moveit_aruco_touch import validate_joint_values


def main() -> int:
    parser = argparse.ArgumentParser(description="Save a stopped PiPER-X joint state as a MoveIt MVP taught pose; never moves the robot.")
    parser.add_argument("--config", default="piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml")
    parser.add_argument("--pose-name", required=True, choices=["home", "pre_touch", "safe_recovery"])
    parser.add_argument("--positions", required=True, help="Comma-separated six current joint positions in radians.")
    parser.add_argument("--ack", required=True, help="Must be SAVE_STOPPED_POSE.")
    parser.add_argument("--source", default="operator_read_current_joint_state")
    args = parser.parse_args()
    if args.ack != "SAVE_STOPPED_POSE":
        raise SystemExit("--ack SAVE_STOPPED_POSE is required before replacing a saved taught pose")
    config = load_touch_config(args.config)
    positions = [float(v) for v in args.positions.split(",")]
    validate_joint_values(list(EXPECTED_JOINT_NAMES), positions, config.joint_limits)
    pose = TaughtPose(args.pose_name, list(EXPECTED_JOINT_NAMES), positions, args.source)
    metadata = {
        "profile_id": config.profile_id,
        "task_id": config.task_id,
        "planning_group": config.planning_group,
        "end_effector_link": config.end_effector_link,
        "robot_model": config.robot_model,
        "robot_urdf_candidate_path": config.robot_urdf_candidate_path,
        "robot_urdf_candidate_sha256": config.robot_urdf_candidate_sha256,
        "piper_x_model_verified": config.piper_x_model_verified,
        "motion_commanded": False,
    }
    save_taught_pose_manifest(config.taught_pose_manifest, pose, metadata)
    print(json.dumps({"saved": True, "manifest": config.taught_pose_manifest, "pose": pose.__dict__, "timestamp_unix_s": time.time()}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

