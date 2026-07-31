#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

from _bootstrap import add_repo_src_to_syspath

add_repo_src_to_syspath()

from piper_on_bunker.manipulation.moveit_aruco_touch import EXPECTED_JOINT_NAMES
from piper_on_bunker.manipulation.moveit_aruco_touch import MarkerStatus
from piper_on_bunker.manipulation.moveit_aruco_touch import MockMoveItTouchBackend
from piper_on_bunker.manipulation.moveit_aruco_touch import MoveItArucoTouchController
from piper_on_bunker.manipulation.moveit_aruco_touch import RosMoveItJointSequenceBackend
from piper_on_bunker.manipulation.moveit_aruco_touch import TaughtPose
from piper_on_bunker.manipulation.moveit_aruco_touch import load_taught_poses
from piper_on_bunker.manipulation.moveit_aruco_touch import load_touch_config
from piper_on_bunker.manipulation.moveit_aruco_touch import result_to_json
from piper_on_bunker.mission_logging import MissionLogger


def _rospy_available() -> bool:
    try:
        import rospy  # noqa: F401
    except Exception:
        return False
    return True


def _rerun_live_in_noetic_container(argv: list[str]) -> int | None:
    if _rospy_available() or os.environ.get("PIPER_MOVEIT_ARUCO_NO_DOCKER_REEXEC") == "1":
        return None
    if "--live" not in argv:
        return None
    container = os.environ.get("ROS_CONTAINER", "abot-piper-noetic")
    cmd = [
        "docker",
        "exec",
        "-i",
        container,
        "bash",
        "-lc",
        "source /opt/ros/noetic/setup.bash && "
        "source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash 2>/dev/null || true; "
        "export ROS_PACKAGE_PATH=/root/piper-pipeline-testbed/piper-on-bunker/ros:/tmp/piper_x_moveit_ros:${ROS_PACKAGE_PATH:-}; "
        "cd /root/piper-pipeline-testbed && "
        "python3 piper-on-bunker/scripts/run_moveit_aruco_touch.py "
        + " ".join(subprocess.list2cmdline([arg]) for arg in argv[1:]),
    ]
    print(
        f"Host Python cannot import rospy; re-running live MoveIt command inside {container}.",
        file=sys.stderr,
    )
    try:
        return subprocess.call(cmd)
    except FileNotFoundError:
        print(
            "Docker is unavailable. Run live mode inside ROS Noetic, for example:\n"
            "./tools/run_in_noetic_container.sh python3 "
            "piper-on-bunker/scripts/run_moveit_aruco_touch.py --config "
            "piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml --live --check-only",
            file=sys.stderr,
        )
        return 2


def _mock_taught_poses() -> dict[str, TaughtPose]:
    metadata = {
        "feedback_source_id": "piper_x_pyagxarm_readonly_v1",
        "joint_mapping_version": "piper_x_pyagxarm_joint_order_rad_v1",
        "pyagxarm_commit": "9eec6e26d927a495efaaa0e7e5af2895310caefe",
    }
    return {
        "staging": TaughtPose("staging", list(EXPECTED_JOINT_NAMES), [0.025, -0.045, -0.075, 0.015, 0.045, 0.015], "mock", metadata),
        "home": TaughtPose("home", list(EXPECTED_JOINT_NAMES), [0.0, -0.05, -0.08, 0.0, 0.05, 0.0], "mock", metadata),
        "pre_touch": TaughtPose("pre_touch", list(EXPECTED_JOINT_NAMES), [0.03, -0.04, -0.07, 0.02, 0.04, 0.02], "mock", metadata),
        "touch": TaughtPose("touch", list(EXPECTED_JOINT_NAMES), [0.04, -0.035, -0.065, 0.02, 0.035, 0.02], "mock", metadata),
        "retract": TaughtPose("retract", list(EXPECTED_JOINT_NAMES), [0.025, -0.055, -0.085, 0.02, 0.055, 0.02], "mock", metadata),
        "safe_recovery": TaughtPose("safe_recovery", list(EXPECTED_JOINT_NAMES), [0.0, -0.06, -0.09, 0.0, 0.06, 0.0], "mock", metadata),
    }


def main() -> int:
    container_status = _rerun_live_in_noetic_container(sys.argv)
    if container_status is not None:
        return container_status

    parser = argparse.ArgumentParser(description="Plan a fixed-position PiPER-X MoveIt ArUco touch MVP mission.")
    parser.add_argument("--config", default="piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml")
    parser.add_argument("--planning-only", action="store_true", default=True)
    parser.add_argument("--check-only", action="store_true", help="Run live/mock readiness and marker checks without planning motion segments.")
    parser.add_argument("--execute", action="store_true", help="Requires sequence-specific --confirm and enabled config; not for automated use.")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--mock", action="store_true", help="Use a mock MoveIt backend.")
    parser.add_argument("--live", action="store_true", help="Use the live ROS MoveIt backend. Never silently falls back to mock.")
    parser.add_argument("--sequence", choices=["staging_test", "fixed_touch", "home_transit_diagnostic"], default="fixed_touch")
    parser.add_argument("--publish-plans-to-rviz", action="store_true", help="Publish planning-only DisplayTrajectory messages for RViz review.")
    parser.add_argument("--mock-taught-poses", action="store_true", help="Use deterministic mock taught poses.")
    marker_visible = parser.add_mutually_exclusive_group()
    marker_visible.add_argument("--mock-marker-visible", dest="mock_marker_visible", action="store_true", default=True)
    marker_visible.add_argument("--no-mock-marker-visible", dest="mock_marker_visible", action="store_false")
    parser.add_argument("--mock-marker-id", type=int, default=6)
    parser.add_argument("--audit-log", default="piper-on-bunker/logs/moveit_aruco_touch/mock_mission.jsonl")
    args = parser.parse_args()
    if args.mock == args.live:
        print("Select exactly one backend: --mock or --live.", file=sys.stderr)
        return 2
    if args.execute and args.mock:
        print("--execute is refused with --mock; use planning-only mock tests only.", file=sys.stderr)
        return 2

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
    backend = MockMoveItTouchBackend(marker=marker)
    if args.live:
        try:
            backend = RosMoveItJointSequenceBackend(config, joint_topic=config.authoritative_joint_state_topic)
        except Exception as exc:
            print(f"Live MoveIt backend unavailable: {exc!r}", file=sys.stderr)
            return 2
    taught_poses = _mock_taught_poses() if args.mock_taught_poses else load_taught_poses(config.taught_pose_manifest)
    logger = MissionLogger(path=args.audit_log, enabled=True)
    controller = MoveItArucoTouchController(config, backend, taught_poses=taught_poses, logger=logger)
    if args.check_only:
        result = controller.check_only()
    else:
        result = controller.run(
            planning_only=not args.execute,
            execute=args.execute,
            confirm=args.confirm or None,
            sequence=args.sequence,
            publish_plans_to_rviz=args.publish_plans_to_rviz,
        )
    print(result_to_json(result))
    return 0 if result.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
