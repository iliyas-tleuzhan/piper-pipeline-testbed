#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time

from _bootstrap import add_repo_src_to_syspath

add_repo_src_to_syspath()

from piper_on_bunker.manipulation.moveit_aruco_touch import EXPECTED_JOINT_NAMES
from piper_on_bunker.manipulation.moveit_aruco_touch import JointStateSnapshot
from piper_on_bunker.manipulation.moveit_aruco_touch import load_taught_poses
from piper_on_bunker.manipulation.moveit_aruco_touch import load_touch_config
from piper_on_bunker.manipulation.moveit_aruco_touch import validate_joint_values


def _snapshot_from_args(args) -> JointStateSnapshot | None:
    if args.positions:
        values = [float(v) for v in args.positions.split(",")]
        return JointStateSnapshot(list(EXPECTED_JOINT_NAMES), values, 0.0)
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect the current or saved PiPER-X MoveIt taught pose without moving.")
    parser.add_argument("--config", default="piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml")
    parser.add_argument("--pose-name", default="")
    parser.add_argument("--positions", default="", help="Offline comma-separated six-joint sample for validation.")
    args = parser.parse_args()

    config = load_touch_config(args.config)
    poses = load_taught_poses(config.taught_pose_manifest)
    snapshot = _snapshot_from_args(args)
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
    if snapshot is not None:
        validate_joint_values(snapshot.joint_names, snapshot.positions, config.joint_limits)
        output["current_joint_state"] = snapshot.__dict__
    print(json.dumps(output, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

