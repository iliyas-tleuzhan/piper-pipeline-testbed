#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from _bootstrap import add_repo_src_to_syspath

add_repo_src_to_syspath()

from piper_on_bunker.manipulation.moveit_aruco_touch import EXPECTED_JOINT_NAMES
from piper_on_bunker.manipulation.moveit_aruco_touch import MarkerStatus
from piper_on_bunker.manipulation.moveit_aruco_touch import MockMoveItTouchBackend
from piper_on_bunker.manipulation.moveit_aruco_touch import MoveItArucoTouchController
from piper_on_bunker.manipulation.moveit_aruco_touch import TaughtPose
from piper_on_bunker.manipulation.moveit_aruco_touch import load_taught_poses
from piper_on_bunker.manipulation.moveit_aruco_touch import load_touch_config
from piper_on_bunker.manipulation.moveit_aruco_touch import result_to_json
from piper_on_bunker.mission_logging import MissionLogger


def _mock_taught_poses() -> dict[str, TaughtPose]:
    return {
        "home": TaughtPose("home", list(EXPECTED_JOINT_NAMES), [0.0, -0.05, -0.08, 0.0, 0.05, 0.0], "mock"),
        "pre_touch": TaughtPose("pre_touch", list(EXPECTED_JOINT_NAMES), [0.03, -0.04, -0.07, 0.02, 0.04, 0.02], "mock"),
        "safe_recovery": TaughtPose("safe_recovery", list(EXPECTED_JOINT_NAMES), [0.0, -0.06, -0.09, 0.0, 0.06, 0.0], "mock"),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Plan a fixed-position PiPER-X MoveIt ArUco touch MVP mission.")
    parser.add_argument("--config", default="piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml")
    parser.add_argument("--planning-only", action="store_true", default=True)
    parser.add_argument("--execute", action="store_true", help="Requires --confirm FIXED_ARUCO_TOUCH and enabled config; not for automated use.")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--mock", action="store_true", help="Use a mock MoveIt backend.")
    parser.add_argument("--mock-taught-poses", action="store_true", help="Use deterministic mock taught poses.")
    parser.add_argument("--mock-marker-visible", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--mock-marker-id", type=int, default=6)
    parser.add_argument("--mock-cartesian-fraction", type=float, default=1.0)
    parser.add_argument("--audit-log", default="piper-on-bunker/logs/moveit_aruco_touch/mock_mission.jsonl")
    args = parser.parse_args()

    config = load_touch_config(args.config)
    marker = MarkerStatus(
        visible=bool(args.mock_marker_visible),
        marker_id=args.mock_marker_id if args.mock_marker_visible else None,
        dictionary=config.marker_dictionary,
        marker_size_m=config.marker_size_m,
        pose_age_s=0.0,
        image_age_s=0.0,
        stable_duration_s=config.marker_stability_required_s,
        detected_ids=[args.mock_marker_id] if args.mock_marker_visible else [],
    )
    backend = MockMoveItTouchBackend(marker=marker, cartesian_fraction=args.mock_cartesian_fraction)
    if not args.mock:
        # The real backend is intentionally not constructed here yet; this CLI is
        # safe-by-default and proves the deterministic state machine without ROS.
        print("Real MoveIt backend is not enabled by this planning-only CLI yet; pass --mock.", file=sys.stderr)
        return 2
    taught_poses = _mock_taught_poses() if args.mock_taught_poses else load_taught_poses(config.taught_pose_manifest)
    logger = MissionLogger(path=args.audit_log, enabled=True)
    result = MoveItArucoTouchController(config, backend, taught_poses=taught_poses, logger=logger).run(
        planning_only=not args.execute,
        execute=args.execute,
        confirm=args.confirm or None,
    )
    print(result_to_json(result))
    return 0 if result.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
