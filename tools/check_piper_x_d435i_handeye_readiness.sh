#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${CONTAINER:-abot-piper-noetic}"
MARKER_ID="${MARKER_ID:-6}"
MARKER_SIZE_M="${MARKER_SIZE_M:-0.100}"
MARKER_DICTIONARY="${MARKER_DICTIONARY:-DICT_ARUCO_ORIGINAL}"

docker exec -i "$CONTAINER" bash -lc '
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash
source /root/easy_handeye_ws/devel/setup.bash
set -u
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

fail=0
check_topic() {
  local topic="$1"
  if timeout 4 rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
    echo "$topic: ok"
  else
    echo "$topic: not_ready"
    fail=1
  fi
}

check_tf() {
  local from="$1"
  local to="$2"
  timeout 4 rosrun tf tf_echo "$from" "$to" >/tmp/tf_check.log 2>&1 || true
  if grep -Fq "Translation:" /tmp/tf_check.log; then
    echo "$from -> $to TF: ok"
  else
    echo "$from -> $to TF: not_ready"
    fail=1
  fi
}

auto_enable="$(rosparam get /piper_ctrl_single_node/auto_enable 2>/dev/null || echo missing)"
echo "auto_enable: $auto_enable"
if [ "$auto_enable" != "false" ]; then fail=1; fi

check_topic /wrist_camera/color/image_raw
check_topic /wrist_camera/color/camera_info
python3 - <<PY
import rospy
from sensor_msgs.msg import CameraInfo
rospy.init_node("check_camera_info_nonzero", anonymous=True, disable_signals=True)
msg = rospy.wait_for_message("/wrist_camera/color/camera_info", CameraInfo, timeout=4)
ok = msg.width > 0 and msg.height > 0 and msg.K[0] > 0 and msg.K[4] > 0
print("CameraInfo nonzero: " + ("ok" if ok else "not_ready"))
raise SystemExit(0 if ok else 1)
PY
check_topic /wrist_camera/color/image_rect_color
check_topic /joint_states_single
check_topic /joint_states
fk_verified="$(rosparam get /piper_x_handeye_model/fk_verified 2>/dev/null || echo false)"
model_id="$(rosparam get /piper_x_handeye_model/physical_model_id 2>/dev/null || echo unresolved)"
firmware="$(rosparam get /piper_x_handeye_model/firmware_version 2>/dev/null || echo unresolved)"
selected_urdf="$(rosparam get /piper_x_handeye_model/selected_urdf_path 2>/dev/null || echo unresolved)"
selected_urdf_sha="$(rosparam get /piper_x_handeye_model/selected_urdf_sha256 2>/dev/null || echo unresolved)"
echo "physical_model_id: $model_id"
echo "firmware_version: $firmware"
echo "selected_urdf_path: $selected_urdf"
echo "selected_urdf_sha256: $selected_urdf_sha"
echo "fk_verified: $fk_verified"
if [ "$fk_verified" != "true" ]; then fail=1; fi
check_tf base_link gripper_base
check_topic /aruco_simple/pose
check_tf wrist_camera_color_optical_frame aruco_marker_frame

dict_param="$(rosparam get /piper_x_aruco_pose_node/dictionary 2>/dev/null || true)"
marker_id_param="$(rosparam get /piper_x_aruco_pose_node/marker_id 2>/dev/null || true)"
marker_size_param="$(rosparam get /piper_x_aruco_pose_node/marker_size_m 2>/dev/null || true)"
echo "aruco_dictionary: $dict_param"
echo "aruco_marker_id: $marker_id_param"
echo "aruco_marker_size_m: $marker_size_param"
if [ "$dict_param" != "'"$MARKER_DICTIONARY"'" ]; then fail=1; fi
if [ "$marker_id_param" != "'"$MARKER_ID"'" ]; then fail=1; fi
python3 - <<PY
value = float("${marker_size_param:-nan}")
expected = float("'"$MARKER_SIZE_M"'")
ok = abs(value - expected) < 1e-9
print("marker size source of truth: " + ("ok" if ok else "wrong"))
raise SystemExit(0 if ok else 1)
PY

if rosnode list 2>/dev/null | grep -Fxq /piper_x_d435i_wrist_eye_on_hand/easy_handeye_calibration_server; then
  echo "easy_handeye namespace: ok"
else
  echo "easy_handeye namespace: not_ready"
  fail=1
fi

if rosnode list 2>/dev/null | grep -E "move_group|trajectory_bridge|openpi|demo_replay" >/dev/null; then
  echo "motion publishers from calibration stack: unexpected"
  fail=1
else
  echo "motion publishers from calibration stack: none_detected"
fi

if [ "$fail" -eq 0 ]; then
  echo "handeye_collection_allowed: true"
else
  echo "handeye_collection_allowed: false"
fi

exit "$fail"
'
