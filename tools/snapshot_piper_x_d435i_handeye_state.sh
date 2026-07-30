#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${CONTAINER:-abot-piper-noetic}"
SESSION="${SESSION:-piper_x_wrist_calib}"
CHECKPOINT_ROOT="${CHECKPOINT_ROOT:-piper-on-bunker/data/local/piper_x_d435i_handeye_checkpoints}"
TIMEOUT_S="${TIMEOUT_S:-4}"
MARKER_DICTIONARY="${MARKER_DICTIONARY:-DICT_ARUCO_ORIGINAL}"
MARKER_ID="${MARKER_ID:-6}"
MARKER_SIZE_M="${MARKER_SIZE_M:-0.100}"

cd "$(dirname "$0")/.."

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
checkpoint_dir="$CHECKPOINT_ROOT/$timestamp"
mkdir -p "$checkpoint_dir"

run_host() {
  local output="$1"
  shift
  {
    timeout "$TIMEOUT_S" "$@"
  } >"$checkpoint_dir/$output" 2>&1 || {
    code=$?
    printf '\n[exit_code=%s]\n' "$code" >>"$checkpoint_dir/$output"
  }
}

run_ros() {
  local output="$1"
  shift
  local command="$*"
  {
    timeout "$TIMEOUT_S" docker exec -i "$CONTAINER" bash -lc "
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash
source /root/easy_handeye_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
$command
"
  } >"$checkpoint_dir/$output" 2>&1 || {
    code=$?
    printf '\n[exit_code=%s]\n' "$code" >>"$checkpoint_dir/$output"
  }
}

run_host git_status.txt git status -sb
run_host git_log.txt git log --oneline -20
run_host tmux_windows.txt tmux list-windows -t "$SESSION"
run_host running_processes.txt docker exec -i "$CONTAINER" bash -lc 'ps -eo pid,ppid,stat,comm,args --width 220 | grep -E "ros|piper|realsense|aruco|easy_handeye|image_proc|openpi|replay|move_group|trajectory" | grep -v grep || true'

run_ros ros_nodes.txt rosnode list
run_ros ros_topics.txt rostopic list
run_ros ros_params_aruco.yaml '{
  echo "# /piper_x_aruco_pose_node"
  rosparam get /piper_x_aruco_pose_node 2>/dev/null || true
  echo
  echo "# /aruco_simple"
  rosparam get /aruco_simple 2>/dev/null || true
  echo
  echo "# /piper_ctrl_single_node/auto_enable"
  rosparam get /piper_ctrl_single_node/auto_enable 2>/dev/null || echo missing
}'
run_ros camera_info.yaml "rostopic echo -n 1 /wrist_camera/color/camera_info"
run_ros joint_state_sample.yaml "rostopic echo -n 1 /joint_states_single"
run_ros robot_tf_sample.txt "rosrun tf tf_echo base_link gripper_base"
run_ros marker_pose_sample.yaml "rostopic echo -n 1 /aruco_simple/pose"
run_ros marker_tf_sample.txt "rosrun tf tf_echo wrist_camera_color_optical_frame aruco_marker_frame"

{
  MARKER_DICTIONARY="$MARKER_DICTIONARY" MARKER_ID="$MARKER_ID" MARKER_SIZE_M="$MARKER_SIZE_M" \
    ./tools/check_piper_x_d435i_handeye_readiness.sh
} >"$checkpoint_dir/readiness.txt" 2>&1 || {
  code=$?
  printf '\n[exit_code=%s]\n' "$code" >>"$checkpoint_dir/readiness.txt"
}

tmp_easy="/tmp/piper_x_easy_handeye_snapshot_$timestamp"
rm -rf "$tmp_easy"
mkdir -p "$checkpoint_dir/easy_handeye"
easy_status="$checkpoint_dir/easy_handeye_status.yaml"
docker exec -i "$CONTAINER" bash -lc '
set -e
rm -rf '"$tmp_easy"'
mkdir -p '"$tmp_easy"'/root_ros_easy_handeye '"$tmp_easy"'/easy_handeye_ws
if [ -d /root/.ros/easy_handeye ]; then
  find /root/.ros/easy_handeye -maxdepth 1 -type f \( -name "*piper_x_d435i*" -o -name "*piper_x*" \) -print -exec cp -a {} '"$tmp_easy"'/root_ros_easy_handeye/ \;
fi
if [ -d /root/easy_handeye_ws ]; then
  find /root/easy_handeye_ws -maxdepth 4 -type f \( -name "*piper_x_d435i*" -o -name "*piper_x*" \) -size -2M -print -exec cp -a --parents {} '"$tmp_easy"'/easy_handeye_ws/ \; 2>/dev/null || true
fi
' >"$checkpoint_dir/easy_handeye_copy_manifest.txt" 2>&1 || true
docker cp "$CONTAINER:$tmp_easy/." "$checkpoint_dir/easy_handeye/" >/dev/null 2>&1 || true
docker exec -i "$CONTAINER" bash -lc "rm -rf '$tmp_easy'" >/dev/null 2>&1 || true

saved_yaml_count="$(find "$checkpoint_dir/easy_handeye" -type f -name '*piper_x_d435i_wrist_eye_on_hand*.yaml' | wc -l | tr -d ' ')"
sample_file_count="$(find "$checkpoint_dir/easy_handeye" -type f \( -name '*sample*' -o -name '*samples*' \) | wc -l | tr -d ' ')"
marker_detected="false"
if grep -q "position:" "$checkpoint_dir/marker_pose_sample.yaml" 2>/dev/null; then
  marker_detected="true"
fi
auto_enable="$(docker exec -i "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash >/dev/null 2>&1; export ROS_MASTER_URI=http://localhost:11311; rosparam get /piper_ctrl_single_node/auto_enable 2>/dev/null || echo missing' 2>/dev/null || echo unknown)"
motion_publishers="none_detected"
if docker exec -i "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash >/dev/null 2>&1; export ROS_MASTER_URI=http://localhost:11311; rosnode list 2>/dev/null | grep -E "move_group|trajectory_bridge|openpi|demo_replay" >/dev/null' >/dev/null 2>&1; then
  motion_publishers="unexpected"
fi

cat >"$easy_status" <<EOF
calibration_samples_collected: $([ "$sample_file_count" -gt 0 ] && echo true || echo false)
calibration_computed: $([ "$saved_yaml_count" -gt 0 ] && echo true || echo false)
calibration_saved: $([ "$saved_yaml_count" -gt 0 ] && echo true || echo false)
saved_piper_x_yaml_count: $saved_yaml_count
sample_file_count: $sample_file_count
marker_detected_during_snapshot: $marker_detected
auto_enable: $auto_enable
motion_publishers_from_calibration_stack: $motion_publishers
EOF

cat >"$checkpoint_dir/README.md" <<EOF
# PiPER-X D435i Hand-Eye Runtime Snapshot

Created UTC: $timestamp

This read-only snapshot captures the current PiPER-X wrist D435i hand-eye
calibration runtime. It does not contain image streams, credentials, model
weights, or robot command logs.

## Calibration Contract

- Robot: AgileX PiPER-X
- Camera: Intel RealSense D435i, serial 243322074578
- Calibration mode: eye-on-hand
- Base frame: base_link
- End-effector frame: gripper_base
- Camera optical frame: wrist_camera_color_optical_frame
- Marker frame: aruco_marker_frame
- Marker dictionary: $MARKER_DICTIONARY
- Marker ID: $MARKER_ID
- Marker size: $MARKER_SIZE_M m
- Expected detector node: /piper_x_aruco_pose_node

The calibration marker is $MARKER_DICTIONARY. The OpenPI ArUco-touch task
profile may still use a different marker family and should not be conflated
with this physical calibration marker.

## Files

- git_status.txt: repository status at snapshot time
- git_log.txt: recent commit history
- tmux_windows.txt: calibration tmux window list
- ros_nodes.txt: ROS node list
- ros_topics.txt: ROS topic list
- ros_params_aruco.yaml: ArUco and auto-enable params
- readiness.txt: read-only readiness check output
- camera_info.yaml: one CameraInfo sample
- joint_state_sample.yaml: one /joint_states_single sample
- robot_tf_sample.txt: base_link -> gripper_base sample
- marker_pose_sample.yaml: /aruco_simple/pose sample if marker was visible
- marker_tf_sample.txt: wrist_camera_color_optical_frame -> aruco_marker_frame if visible
- running_processes.txt: relevant process list
- easy_handeye/: copied PiPER-X easy_handeye files when present
- easy_handeye_status.yaml: calibration/sample presence summary

## Safety

The snapshot helper did not enable the arm, publish motion commands, run OpenPI
execution, or replay a trajectory.
EOF

printf '%s\n' "$checkpoint_dir"
