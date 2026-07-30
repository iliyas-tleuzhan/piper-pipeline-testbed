#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${CONTAINER:-abot-piper-noetic}"
xhost +SI:localuser:root >/dev/null
docker exec -it \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  "$CONTAINER" bash -lc '
source /opt/ros/noetic/setup.bash
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash
source /root/easy_handeye_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
rosrun image_view image_view image:=/aruco_simple/debug_image
'
