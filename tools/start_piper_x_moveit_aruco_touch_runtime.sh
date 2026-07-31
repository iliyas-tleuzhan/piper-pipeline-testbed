#!/usr/bin/env bash
set -euo pipefail

SESSION=${SESSION:-piper_x_moveit_aruco_touch}
CONTAINER=${CONTAINER:-abot-piper-noetic}
MARKER_DICTIONARY=${MARKER_DICTIONARY:-DICT_ARUCO_ORIGINAL}
MARKER_ID=${MARKER_ID:-6}
MARKER_SIZE_M=${MARKER_SIZE_M:-0.100}
CAMERA_SERIAL=${CAMERA_SERIAL:-243322074578}
AGX_ARM_URDF_HOST=${AGX_ARM_URDF_HOST:-/home/dase-hw101/Iliyas/piper-vr-teleop/third_party/agx_arm_urdf}
STAGED_ROS_PKGS=${STAGED_ROS_PKGS:-/tmp/piper_x_moveit_ros}
PIPER_X_JOINT3_UPPER_OVERRIDE_RAD=${PIPER_X_JOINT3_UPPER_OVERRIDE_RAD:-0.02}
PIPER_X_MOVEIT_SPEED_PERCENT=${PIPER_X_MOVEIT_SPEED_PERCENT:-30}
PIPER_X_MOVEIT_COMMAND_RATE_HZ=${PIPER_X_MOVEIT_COMMAND_RATE_HZ:-50}

cd "$(dirname "$0")/.."

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "Container $CONTAINER is not running; start it before launching runtime." >&2
  exit 2
fi

if [[ ! -d "$AGX_ARM_URDF_HOST/piper_x" ]]; then
  echo "PiPER-X URDF asset tree not found: $AGX_ARM_URDF_HOST/piper_x" >&2
  exit 2
fi

docker exec -i "$CONTAINER" bash -lc "rm -rf '$STAGED_ROS_PKGS/agx_arm_description'; mkdir -p '$STAGED_ROS_PKGS/agx_arm_description/agx_arm_urdf'"
tar -C "$AGX_ARM_URDF_HOST" -cf - piper_x | docker exec -i "$CONTAINER" bash -lc "tar -C '$STAGED_ROS_PKGS/agx_arm_description/agx_arm_urdf' -xf -"
docker exec -i "$CONTAINER" bash -lc "cat > '$STAGED_ROS_PKGS/agx_arm_description/package.xml' <<'XML'
<?xml version=\"1.0\"?>
<package format=\"2\">
  <name>agx_arm_description</name>
  <version>0.0.0</version>
  <description>Runtime-staged PiPER-X URDF assets for piper-pipeline-testbed.</description>
  <maintainer email=\"noreply@example.com\">piper-pipeline-testbed</maintainer>
  <license>Proprietary</license>
</package>
XML
python3 - <<PY
from pathlib import Path
import xml.etree.ElementTree as ET

path = Path('$STAGED_ROS_PKGS/agx_arm_description/agx_arm_urdf/piper_x/urdf/piper_x_description.urdf')
tree = ET.parse(path)
root = tree.getroot()
for joint in root.findall('joint'):
    if joint.attrib.get('name') == 'joint3':
        limit = joint.find('limit')
        if limit is None:
            raise SystemExit('joint3 has no limit tag')
        old_upper = limit.attrib.get('upper')
        limit.set('upper', '$PIPER_X_JOINT3_UPPER_OVERRIDE_RAD')
        print(f'Applied unverified PiPER-X MoveIt bounds override: joint3 upper {old_upper} -> {limit.attrib[\"upper\"]} rad')
        break
else:
    raise SystemExit('joint3 not found in staged PiPER-X URDF')
tree.write(path, encoding='unicode')
PY
sha256sum '$STAGED_ROS_PKGS/agx_arm_description/agx_arm_urdf/piper_x/urdf/piper_x_with_gripper_description.xacro'"

docker exec -i "$CONTAINER" bash -lc 'ip link set can0 down >/dev/null 2>&1 || true; ip link set can0 type can bitrate 1000000 >/dev/null 2>&1 || true; ip link set can0 txqueuelen 1000 >/dev/null 2>&1 || true; ip link set can0 up >/dev/null 2>&1 || true'

docker exec -i "$CONTAINER" bash -lc '
source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash >/dev/null 2>&1 || true
export ROS_PACKAGE_PATH=/root/piper-pipeline-testbed/piper-on-bunker/ros:'"$STAGED_ROS_PKGS"':${ROS_PACKAGE_PATH:-}
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
for node in $(rosnode list 2>/dev/null || true); do
  case "$node" in
    /piper_ct|/robot_state_publisher|/move_group|/aruco_simple|/piper_x_aruco_pose_node|/wrist_camera/color/image_proc|/wrist_camera/realsense2_camera_manager|/wrist_camera/realsense2_camera)
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
    "start_single_piper.launch",
    "piper_ctrl_single_node.py",
    "piper_joint_state_relay.py",
    "relay_piper_x_arm_joint_states.py",
    "piper_x_passive_socketcan_joint_state_bridge.py",
    "piper_x_moveit_sdk_trajectory_controller.py",
    "piper_with_gripper_moveit",
    "piper_x_moveit_config",
    "move_group",
    "realsense2_camera",
    "realsense_d555_py_publisher.py",
    "image_proc",
    "piper_x_aruco_pose_node.py",
]
protected = {os.getpid(), os.getppid()}
for name in os.listdir("/proc"):
    if not name.isdigit():
        continue
    pid = int(name)
    if pid in protected:
        continue
    try:
        cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\x00", b" ").decode("utf-8", "ignore")
    except Exception:
        continue
    if any(target in cmd for target in targets):
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
PY
timeout 3 rosnode cleanup >/dev/null 2>&1 || true
'

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n roscore "docker exec -i $CONTAINER bash -lc 'source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://localhost:11311 ROS_HOSTNAME=localhost; if timeout 3 rostopic list >/dev/null 2>&1; then echo Reusing existing ROS master; sleep infinity; else roscore; fi'"
sleep 2

ROS_PREFIX="source /opt/ros/noetic/setup.bash; source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash 2>/dev/null || true; source /root/easy_handeye_ws/devel/setup.bash 2>/dev/null || true; export ROS_PACKAGE_PATH=/root/piper-pipeline-testbed/piper-on-bunker/ros:$STAGED_ROS_PKGS:\${ROS_PACKAGE_PATH:-}; export ROS_MASTER_URI=http://localhost:11311 ROS_HOSTNAME=localhost;"

tmux new-window -t "$SESSION" -n piper_x_feedback "docker exec -i $CONTAINER bash -lc '$ROS_PREFIX export PYTHONPATH=/root/piper-pipeline-testbed/piper-on-bunker/src:\${PYTHONPATH:-}; python3 /root/piper-pipeline-testbed/piper-on-bunker/scripts/piper_x_passive_socketcan_joint_state_bridge.py --can can0 --joint-topic /piper_x/joint_states'"
tmux new-window -t "$SESSION" -n joint_relay "docker exec -i $CONTAINER bash -lc '$ROS_PREFIX while ! timeout 2 rostopic echo -n 1 /piper_x/joint_states >/dev/null 2>&1; do echo waiting for /piper_x/joint_states; sleep 1; done; python3 /root/piper-pipeline-testbed/piper-on-bunker/scripts/relay_piper_x_arm_joint_states.py --input-topic /piper_x/joint_states --output-topic /joint_states'"
tmux new-window -t "$SESSION" -n trajectory_controller "docker exec -i $CONTAINER bash -lc '$ROS_PREFIX export PYTHONPATH=/root/piper-pipeline-testbed/piper-on-bunker/src:\${PYTHONPATH:-}; while ! timeout 2 rostopic echo -n 1 /joint_states >/dev/null 2>&1; do echo waiting for /joint_states; sleep 1; done; python3 /root/piper-pipeline-testbed/piper-on-bunker/scripts/piper_x_moveit_sdk_trajectory_controller.py --feedback-topic /joint_states --can can0 --speed-percent $PIPER_X_MOVEIT_SPEED_PERCENT --command-rate-hz $PIPER_X_MOVEIT_COMMAND_RATE_HZ'"
tmux new-window -t "$SESSION" -n move_group "docker exec -i $CONTAINER bash -lc '$ROS_PREFIX while ! timeout 2 rostopic echo -n 1 /joint_states >/dev/null 2>&1; do echo waiting for /joint_states; sleep 1; done; roslaunch piper_x_moveit_config planning_only.launch use_rviz:=false info:=true'"
tmux new-window -t "$SESSION" -n d435i_wrist "docker exec -i $CONTAINER bash -lc '$ROS_PREFIX cd /root/ABot-Claw/robot_layer/arm_piper/agent_server; python3 realsense_d555_py_publisher.py --camera wrist_camera --serial $CAMERA_SERIAL --width 640 --height 480 --fps 15'"
tmux new-window -t "$SESSION" -n image_rectify "docker exec -i $CONTAINER bash -lc '$ROS_PREFIX while ! timeout 2 rostopic echo -n 1 /wrist_camera/color/image_raw >/dev/null 2>&1; do echo waiting for wrist raw image; sleep 1; done; rosrun image_proc image_proc __name:=image_proc __ns:=/wrist_camera/color'"
tmux new-window -t "$SESSION" -n aruco "docker exec -i $CONTAINER bash -lc '$ROS_PREFIX export PYTHONPATH=/root/piper-pipeline-testbed/piper-on-bunker/src:\${PYTHONPATH:-}; while ! timeout 2 rostopic echo -n 1 /wrist_camera/color/image_rect_color >/dev/null 2>&1; do echo waiting for rectified wrist image; sleep 1; done; python3 /root/piper-pipeline-testbed/piper-on-bunker/scripts/piper_x_aruco_pose_node.py _image_topic:=/wrist_camera/color/image_rect_color _camera_info_topic:=/wrist_camera/color/camera_info _image_geometry_mode:=rectified _pose_topic:=/aruco_simple/pose _debug_image_topic:=/aruco_simple/debug_image _dictionary:=$MARKER_DICTIONARY _marker_id:=$MARKER_ID _marker_size_m:=$MARKER_SIZE_M _camera_frame:=wrist_camera_color_optical_frame _marker_frame:=aruco_marker_frame'"

docker exec -i "$CONTAINER" bash -lc '
source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
deadline=$((SECONDS + 20))
while (( SECONDS < deadline )); do
  timeout 2 rostopic echo -n 1 /wrist_camera/color/image_rect_color >/dev/null 2>&1 \
    && timeout 2 rostopic echo -n 1 /aruco_simple/debug_image >/dev/null 2>&1 \
    && break
  sleep 1
done
'

echo "Started tmux session: $SESSION"
echo "No OpenPI, VLA, Bunker navigation, mission execution, or rejected hand-eye publisher was started."
echo "Normal PiPER piper_ctrl_single_node is not started for PiPER-X state."
echo "PiPER-X feedback bridge: passive SocketCAN RX-only decoder"
echo "Feedback decoder source: /home/dase-hw101/Iliyas/piper-lora-teleop-bridge @ 521c9c5fdfd9ee63bd96c0f9342fca6b2398092e"
echo "PiPER driver auto_enable:=false (normal driver disabled in this runtime)"
echo "MoveIt model: PiPER-X staged from $AGX_ARM_URDF_HOST/piper_x"
echo "MoveIt bounds override: joint3 upper -> $PIPER_X_JOINT3_UPPER_OVERRIDE_RAD rad in staged URDF only"
echo "PiPER-X FK/model verification: false; taught-joint execution does not use FK/hand-eye targeting"
echo "MoveIt joint-state authority: /piper_x/joint_states -> /joint_states"
echo "MoveIt trajectory controller: /arm_controllers/follow_joint_trajectory -> piper_sdk JointCtrl on first execution goal"
echo "MoveIt SDK speed percent: $PIPER_X_MOVEIT_SPEED_PERCENT"
echo "MoveIt command streaming rate: $PIPER_X_MOVEIT_COMMAND_RATE_HZ Hz"
echo "Marker contract: $MARKER_DICTIONARY ID $MARKER_ID size $MARKER_SIZE_M m"
echo
echo "Readiness:"
"$(dirname "$0")/check_piper_x_moveit_aruco_touch_readiness.sh" || true
