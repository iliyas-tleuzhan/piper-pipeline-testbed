#!/usr/bin/env bash
set -euo pipefail

SESSION=${SESSION:-piper_x_moveit_aruco_touch}
CONTAINER=${CONTAINER:-abot-piper-noetic}
MARKER_DICTIONARY=${MARKER_DICTIONARY:-DICT_ARUCO_ORIGINAL}
MARKER_ID=${MARKER_ID:-6}
MARKER_SIZE_M=${MARKER_SIZE_M:-0.100}
CAMERA_SERIAL=${CAMERA_SERIAL:-243322074578}

cd "$(dirname "$0")/.."

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "Container $CONTAINER is not running; start it before launching runtime." >&2
  exit 2
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n roscore "docker exec -i $CONTAINER bash -lc 'source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://localhost:11311 ROS_HOSTNAME=localhost; roscore'"
sleep 2

ROS_PREFIX="source /opt/ros/noetic/setup.bash; source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash 2>/dev/null || true; source /root/easy_handeye_ws/devel/setup.bash 2>/dev/null || true; export ROS_MASTER_URI=http://localhost:11311 ROS_HOSTNAME=localhost;"

tmux new-window -t "$SESSION" -n piper_driver_readonly "docker exec -i $CONTAINER bash -lc '$ROS_PREFIX roslaunch piper start_single_piper.launch auto_enable:=false'"
tmux new-window -t "$SESSION" -n robot_state_pub "docker exec -i $CONTAINER bash -lc '$ROS_PREFIX roslaunch piper_description display_xacro.launch'"
tmux new-window -t "$SESSION" -n moveit "docker exec -i $CONTAINER bash -lc '$ROS_PREFIX roslaunch piper_moveit_config demo.launch'"
tmux new-window -t "$SESSION" -n d435i_wrist "docker exec -i $CONTAINER bash -lc '$ROS_PREFIX roslaunch realsense2_camera rs_camera.launch camera:=wrist_camera serial_no:=$CAMERA_SERIAL align_depth:=true enable_depth:=true enable_color:=true'"
tmux new-window -t "$SESSION" -n image_rectify "docker exec -i $CONTAINER bash -lc '$ROS_PREFIX rosrun image_proc image_proc __name:=wrist_image_proc __ns:=/wrist_camera/color'"
tmux new-window -t "$SESSION" -n aruco "docker exec -i $CONTAINER bash -lc '$ROS_PREFIX python3 /root/piper-pipeline-testbed/piper-on-bunker/scripts/piper_x_aruco_pose_node.py _image_topic:=/wrist_camera/color/image_rect_color _camera_info_topic:=/wrist_camera/color/camera_info _image_geometry_mode:=rectified _dictionary:=$MARKER_DICTIONARY _marker_id:=$MARKER_ID _marker_size_m:=$MARKER_SIZE_M'"

echo "Started tmux session: $SESSION"
echo "No OpenPI, VLA, Bunker navigation, mission execution, or rejected hand-eye publisher was started."
echo "PiPER driver auto_enable:=false"
echo "Marker contract: $MARKER_DICTIONARY ID $MARKER_ID size $MARKER_SIZE_M m"
echo
echo "Readiness:"
"$(dirname "$0")/check_piper_x_moveit_aruco_touch_readiness.sh" || true

