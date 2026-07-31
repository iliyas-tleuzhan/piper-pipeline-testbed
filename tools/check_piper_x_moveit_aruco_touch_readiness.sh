#!/usr/bin/env bash
set -euo pipefail

CONTAINER=${CONTAINER:-abot-piper-noetic}
cd "$(dirname "$0")/.."

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "container: not_ready ($CONTAINER not running)"
  echo "mock_ready: true"
  echo "live_read_only_ready: false"
  echo "live_planning_ready: false"
  echo "physical_execution_blocked: true"
  echo "physical_execution_ready: false"
  exit 0
fi

docker exec -i "$CONTAINER" bash -lc '
source /opt/ros/noetic/setup.bash 2>/dev/null || true
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash 2>/dev/null || true
source /root/easy_handeye_ws/devel/setup.bash 2>/dev/null || true
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

ok_topic() {
  local topic=$1
  timeout 3 rostopic echo -n 1 "$topic" >/tmp/topic_sample 2>/tmp/topic_err && echo "$topic: ok" || echo "$topic: not_ready ($(cat /tmp/topic_err 2>/dev/null))"
}

echo "camera image:"
ok_topic /wrist_camera/color/image_rect_color
echo "aruco debug image:"
ok_topic /aruco_simple/debug_image
echo "marker pose:"
ok_topic /aruco_simple/pose
echo "joint state:"
ok_topic /joint_states_single

echo "aruco dictionary: $(rosparam get /aruco_simple/dictionary 2>/dev/null || echo unknown)"
echo "aruco marker_id: $(rosparam get /aruco_simple/marker_id 2>/dev/null || echo unknown)"
echo "aruco marker_size: $(rosparam get /aruco_simple/marker_size 2>/dev/null || echo unknown)"

echo "move_group nodes:"
rosnode list 2>/dev/null | grep move_group || true
echo "moveit planning interfaces:"
if rosnode list 2>/dev/null | grep -qx /move_group; then
  echo "/move_group: ok"
else
  echo "/move_group: not_ready"
fi
for t in /move_group/goal /move_group/result /move_group/status /execute_trajectory/goal /execute_trajectory/result /execute_trajectory/status; do
  rostopic info "$t" >/dev/null 2>&1 && echo "$t: ok" || echo "$t: not_ready"
done
echo "legacy moveit_ctrl services:"
for s in /joint_moveit_ctrl_arm /joint_moveit_ctrl_endpose /joint_moveit_ctrl_gripper /joint_moveit_ctrl_piper; do
  echo "$s: $(rosservice type "$s" 2>/dev/null || echo not_expected_for_move_group_backend)"
done
echo "robot_description_sha256: $(rosparam get /robot_description 2>/dev/null | sha256sum | awk '\''{print $1}'\'' || echo unavailable)"
echo "robot_description_name: $(python3 - <<PY
import subprocess
import xml.etree.ElementTree as ET
import yaml
try:
    raw = subprocess.check_output(["rosparam", "get", "/robot_description"], text=True, stderr=subprocess.DEVNULL)
    text = yaml.safe_load(raw)
    print(ET.fromstring(text).attrib.get("name", "unknown"))
except Exception:
    print("unavailable")
PY
)"
echo "piper_x_moveit_model_source: $(rosparam get /piper_x_moveit/model_source 2>/dev/null || echo unavailable)"
echo "piper_x_moveit_model_verified: $(rosparam get /piper_x_moveit/model_verified 2>/dev/null || echo unavailable)"
echo "piper_x_physical_execution_blocked_reason: $(rosparam get /piper_x_moveit/physical_execution_blocked_reason 2>/dev/null || echo unavailable)"
'

python3 piper-on-bunker/scripts/inspect_piper_x_moveit_pose.py \
  --config piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml \
  --positions "0,0,0,0,0,0" >/tmp/piper_x_moveit_pose_offline_check.json

for pose in home pre_touch touch retract; do
  if grep -q "\"$pose\"" piper-on-bunker/data/local/moveit_aruco_touch/taught_poses.yaml 2>/dev/null; then
    echo "taught_$pose: exists"
  else
    echo "taught_$pose: missing"
  fi
done

echo "physical_execution_enabled: false (committed config default)"
echo "rejected_handeye_transform_used_for_targeting: false"
echo "mock_ready: true"
echo "live_read_only_ready: requires camera image, aruco debug image, joint state, and move_group; marker pose additionally requires marker ID 6 visible"
echo "live_planning_ready: requires moveit_commander/move_group and all taught poses"
echo "physical_execution_blocked: true"
echo "physical_execution_ready: false"
