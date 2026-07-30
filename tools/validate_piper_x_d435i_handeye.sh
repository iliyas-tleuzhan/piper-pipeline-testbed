#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${CONTAINER:-abot-piper-noetic}"

docker exec -i "$CONTAINER" bash -lc '
source /opt/ros/noetic/setup.bash
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash
source /root/easy_handeye_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

echo "Checking saved hand-eye TF..."
timeout 5 rosrun tf tf_echo gripper_base wrist_camera_color_optical_frame
echo "Checking fixed marker in base frame..."
timeout 5 rosrun tf tf_echo base_link aruco_marker_frame
echo "Logging five observed base_link -> aruco_marker_frame samples. Move the arm manually between separate runs if needed."
for i in 1 2 3 4 5; do
  timeout 5 rosrun tf tf_echo base_link aruco_marker_frame | sed -n "1,8p" || exit 1
  sleep 1
done
echo "Validation is read-only. No robot motion was commanded."
'
