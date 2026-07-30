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
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export ROS_NAMESPACE=/piper_x_d435i_wrist_eye_on_hand
rosrun rqt_easy_handeye rqt_easy_handeye
'
