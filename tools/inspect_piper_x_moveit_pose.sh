#!/usr/bin/env bash
set -euo pipefail

CONTAINER=${CONTAINER:-abot-piper-noetic}
STAGED_ROS_PKGS=${STAGED_ROS_PKGS:-/tmp/piper_x_moveit_ros}

cd "$(dirname "$0")/.."

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "Container $CONTAINER is not running; start the PiPER-X MoveIt runtime first." >&2
  exit 2
fi

ARGS=""
if (($#)); then
  printf -v ARGS ' %q' "$@"
fi
docker exec -i "$CONTAINER" bash -lc "
  source /opt/ros/noetic/setup.bash
  source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash 2>/dev/null || true
  export ROS_PACKAGE_PATH=/root/piper-pipeline-testbed/piper-on-bunker/ros:$STAGED_ROS_PKGS:\${ROS_PACKAGE_PATH:-}
  export ROS_MASTER_URI=http://localhost:11311
  export ROS_HOSTNAME=localhost
  cd /root/piper-pipeline-testbed
  python3 piper-on-bunker/scripts/inspect_piper_x_moveit_pose.py$ARGS
"
