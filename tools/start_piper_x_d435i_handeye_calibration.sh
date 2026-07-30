#!/usr/bin/env bash
set -euo pipefail

SESSION="${SESSION:-piper_x_wrist_calib}"
CONTAINER="${CONTAINER:-abot-piper-noetic}"
CAMERA_SERIAL="${CAMERA_SERIAL:-243322074578}"
MARKER_ID="${MARKER_ID:-6}"
MARKER_SIZE_M="${MARKER_SIZE_M:-0.100}"
MARKER_DICTIONARY="${MARKER_DICTIONARY:-DICT_ARUCO_ORIGINAL}"
CAN_INTERFACE="${CAN_INTERFACE:-can0}"

if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
  echo "ERROR: container $CONTAINER does not exist" >&2
  exit 1
fi
if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" != "true" ]; then
  docker start "$CONTAINER" >/dev/null
fi

docker exec -i "$CONTAINER" bash -lc '
source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash >/dev/null 2>&1 || true
source /root/easy_handeye_ws/devel/setup.bash >/dev/null 2>&1 || true
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
for node in $(rosnode list 2>/dev/null || true); do
  case "$node" in
    /aruco_simple|\
    /piper_x_aruco_pose_node|\
    /dummy_handeye|\
    /piper_x_d435i_wrist_eye_on_hand/easy_handeye_calibration_server|\
    /robot_state_publisher|\
    /wrist_camera/color/image_proc|\
    /piper_joint_state_relay|\
    /piper_ctrl_single_node|\
    /tf_echo_*)
      timeout 3 rosnode kill "$node" >/dev/null 2>&1 || true
      ;;
    /realsense_d555_py_publisher_*)
      timeout 3 rosnode kill "$node" >/dev/null 2>&1 || true
      ;;
  esac
done
python3 - <<PY
import os
import signal

targets = [
    "piper_x_aruco_pose_node.py",
    "realsense_d555_py_publisher.py",
    "image_proc",
    "robot_state_publisher",
    "piper_joint_state_relay.py",
    "start_single_piper.launch",
    "easy_handeye",
]
required_context = {
    "image_proc": "/wrist_camera/color",
    "easy_handeye": "piper_x_d435i_wrist",
}
protected = {os.getpid()}
parent = os.getppid()
while parent and parent not in protected:
    protected.add(parent)
    try:
        with open(f"/proc/{parent}/stat", "r", encoding="utf-8") as fh:
            parent = int(fh.read().split()[3])
    except Exception:
        break

for name in os.listdir("/proc"):
    if not name.isdigit():
        continue
    pid = int(name)
    if pid in protected:
        continue
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as fh:
            cmd = fh.read().replace(b"\\x00", b" ").decode("utf-8", "ignore")
    except Exception:
        continue
    for target in targets:
        if target not in cmd:
            continue
        context = required_context.get(target)
        if context and context not in cmd:
            continue
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        break
PY
rosparam delete /aruco_simple >/dev/null 2>&1 || true
rosparam delete /piper_x_aruco_pose_node >/dev/null 2>&1 || true
timeout 3 rosnode cleanup >/dev/null 2>&1 || true
'

STAGE="/tmp/piper_x_handeye"
docker exec -i "$CONTAINER" bash -lc "rm -rf '$STAGE'; mkdir -p '$STAGE/src/piper_on_bunker/perception' '$STAGE/scripts'; touch '$STAGE/src/piper_on_bunker/__init__.py' '$STAGE/src/piper_on_bunker/perception/__init__.py'"
docker cp "piper-on-bunker/src/piper_on_bunker/perception/piper_x_aruco_pose.py" "$CONTAINER:$STAGE/src/piper_on_bunker/perception/piper_x_aruco_pose.py"
docker cp "piper-on-bunker/scripts/piper_x_aruco_pose_node.py" "$CONTAINER:$STAGE/scripts/piper_x_aruco_pose_node.py"

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n roscore

send_window() {
  local name="$1"
  local command="$2"
  tmux new-window -t "$SESSION" -n "$name"
  tmux send-keys -t "$SESSION:$name" "$command" C-m
}

docker_ros_prefix='source /opt/ros/noetic/setup.bash; source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash; source /root/easy_handeye_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_HOSTNAME=localhost; rospack profile >/dev/null 2>&1 || true;'

tmux send-keys -t "$SESSION:roscore" "docker exec -i $CONTAINER bash -lc 'source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://localhost:11311; export ROS_HOSTNAME=localhost; roscore'" C-m
sleep 2

send_window piper_driver_readonly "docker exec -i $CONTAINER bash -lc '$docker_ros_prefix ip link set $CAN_INTERFACE down || true; ip link set $CAN_INTERFACE type can bitrate 1000000 || true; ip link set $CAN_INTERFACE txqueuelen 1000 || true; ip link set $CAN_INTERFACE up || true; roslaunch piper start_single_piper.launch can_port:=$CAN_INTERFACE auto_enable:=false'"
sleep 2
send_window joint_relay "docker exec -i $CONTAINER bash -lc '$docker_ros_prefix cd /root/ABot-Claw/robot_layer/arm_piper/agent_server; python3 piper_joint_state_relay.py'"
send_window robot_state_pub "docker exec -i $CONTAINER bash -lc '$docker_ros_prefix while ! timeout 2 rostopic echo -n 1 /joint_states >/dev/null 2>&1; do echo waiting for /joint_states; sleep 1; done; rosparam set --textfile=\$(rospack find piper_description)/urdf/piper_description.urdf /robot_description; rosrun robot_state_publisher robot_state_publisher __name:=robot_state_publisher'"
send_window d435i_wrist "docker exec -i $CONTAINER bash -lc '$docker_ros_prefix cd /root/ABot-Claw/robot_layer/arm_piper/agent_server; python3 realsense_d555_py_publisher.py --camera wrist_camera --serial $CAMERA_SERIAL --width 640 --height 480 --fps 15'"
send_window image_rectify "docker exec -i $CONTAINER bash -lc '$docker_ros_prefix while ! timeout 2 rostopic echo -n 1 /wrist_camera/color/image_raw >/dev/null 2>&1; do echo waiting for wrist raw image; sleep 1; done; rosrun image_proc image_proc __name:=image_proc __ns:=/wrist_camera/color'"
send_window aruco "docker exec -i $CONTAINER bash -lc '$docker_ros_prefix export PYTHONPATH=$STAGE/src:\${PYTHONPATH:-}; while ! timeout 2 rostopic echo -n 1 /wrist_camera/color/image_rect_color >/dev/null 2>&1; do echo waiting for rectified wrist image; sleep 1; done; python3 $STAGE/scripts/piper_x_aruco_pose_node.py _image_topic:=/wrist_camera/color/image_rect_color _camera_info_topic:=/wrist_camera/color/camera_info _pose_topic:=/aruco_simple/pose _debug_image_topic:=/aruco_simple/debug_image _dictionary:=$MARKER_DICTIONARY _marker_id:=$MARKER_ID _marker_size_m:=$MARKER_SIZE_M _camera_frame:=wrist_camera_color_optical_frame _marker_frame:=aruco_marker_frame'"
send_window handeye_backend "docker exec -i $CONTAINER bash -lc '$docker_ros_prefix export PYTHONPATH=/usr/lib/python3/dist-packages:\${PYTHONPATH:-}; roslaunch easy_handeye calibrate.launch eye_on_hand:=true namespace_prefix:=piper_x_d435i_wrist freehand_robot_movement:=true robot_base_frame:=base_link robot_effector_frame:=gripper_base tracking_base_frame:=wrist_camera_color_optical_frame tracking_marker_frame:=aruco_marker_frame start_rviz:=false start_sampling_gui:=false'"

tmux select-window -t "$SESSION:handeye_backend"
for window in roscore piper_driver_readonly joint_relay robot_state_pub d435i_wrist image_rectify aruco handeye_backend; do
  tmux clear-history -t "$SESSION:$window" 2>/dev/null || true
done
echo "Started detached tmux session: $SESSION"
echo "Marker dictionary: $MARKER_DICTIONARY"
echo "Marker ID: $MARKER_ID"
echo "Marker size: $MARKER_SIZE_M m"
echo "No robot motion was commanded. PiPER driver requested auto_enable:=false."
