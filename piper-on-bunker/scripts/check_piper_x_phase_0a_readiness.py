#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys

from _bootstrap import add_repo_src_to_syspath

add_repo_src_to_syspath()

from piper_on_bunker.calibration.piper_x_repeatability import build_readiness_report
from piper_on_bunker.calibration.piper_x_repeatability import load_phase0a_config


def _rospy_available() -> bool:
    try:
        import rospy  # noqa: F401
    except Exception:
        return False
    return True


def _rerun_live_in_noetic_container(argv: list[str]) -> int | None:
    if _rospy_available() or os.environ.get("PIPER_PHASE0A_NO_DOCKER_REEXEC") == "1":
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
        "python3 piper-on-bunker/scripts/check_piper_x_phase_0a_readiness.py "
        + " ".join(subprocess.list2cmdline([arg]) for arg in argv[1:]),
    ]
    print(f"Host Python cannot import rospy; re-running Phase 0A readiness inside {container}.", file=sys.stderr)
    try:
        return subprocess.call(cmd)
    except FileNotFoundError:
        return None


def main() -> int:
    container_status = _rerun_live_in_noetic_container(sys.argv)
    if container_status is not None:
        return container_status
    parser = argparse.ArgumentParser(description="Check PiPER-X Phase 0A repeatability readiness.")
    parser.add_argument("--config", default="piper-on-bunker/config/piper_x_phase_0a_repeatability.yaml")
    args = parser.parse_args()
    config = load_phase0a_config(args.config)
    report = build_readiness_report(config)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report.get("observe_ready") else 1


if __name__ == "__main__":
    raise SystemExit(main())
