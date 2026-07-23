#!/usr/bin/env bash
set -u

ROS_CONTAINER="${ROS_CONTAINER:-abot-piper-noetic}"
REMOTE_HOST="${REMOTE_HOST:-iliyas@master}"
REMOTE_FALLBACK_HOST="${REMOTE_FALLBACK_HOST:-iliyas@192.168.1.104}"
REFRESH_S="${REFRESH_S:-5}"
CHECK_REMOTE="${CHECK_REMOTE:-1}"
CHECK_PIPELINE_API="${CHECK_PIPELINE_API:-1}"

ok() { printf "%-18s OK\n" "$1"; }
down() { printf "%-18s DOWN%s\n" "$1" "${2:+ - $2}"; }

have_docker() {
  docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$ROS_CONTAINER"
}

ros_exec() {
  docker exec "$ROS_CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash >/dev/null 2>&1 || true; $*" >/dev/null 2>&1
}

check_http() {
  curl --noproxy '*' -fsS --max-time 1 "$1" >/dev/null 2>&1
}

remote_cmd() {
  ssh -o BatchMode=yes -o ConnectTimeout=2 "$REMOTE_HOST" "$1" >/dev/null 2>&1 ||
    ssh -o BatchMode=yes -o ConnectTimeout=2 "$REMOTE_FALLBACK_HOST" "$1" >/dev/null 2>&1
}

check_can() {
  if ip link show can0 >/dev/null 2>&1; then
    ip link show can0 | grep -q "state UP"
    return
  fi
  if have_docker; then
    docker exec "$ROS_CONTAINER" bash -lc 'ip link show can0 2>/dev/null | grep -q "state UP"' >/dev/null 2>&1
    return
  fi
  return 1
}

while true; do
  clear
  printf "ABot-Claw + PiPER read-only status dashboard\n"
  date
  printf "\n"

  check_can && ok "CAN" || down "CAN" "can0 missing or down"

  if have_docker; then
    ros_exec "rostopic list" && ok "ROS master" || down "ROS master"
    ros_exec "timeout 2 rostopic echo -n 1 /joint_states_single" && ok "PiPER state" || down "PiPER state"
    ros_exec "timeout 2 rostopic echo -n 1 /end_pose" && ok "PiPER end_pose" || down "PiPER end_pose"
    ros_exec "rosservice list | grep -q '^/joint_moveit_ctrl_arm$' && rosservice list | grep -q '^/joint_moveit_ctrl_endpose$' && rosservice list | grep -q '^/joint_moveit_ctrl_gripper$' && rosservice list | grep -q '^/joint_moveit_ctrl_piper$'" && ok "MoveIt" || down "MoveIt"
    ros_exec "rostopic list | grep -q '^/table_camera/color/image_raw$' && rostopic list | grep -q '^/table_camera/aligned_depth_to_color/image_raw$' && rostopic list | grep -q '^/table_camera/color/camera_info$'" && ok "RealSense" || down "RealSense"
  else
    down "ROS container" "$ROS_CONTAINER not running"
    down "ROS master"
    down "PiPER state"
    down "PiPER end_pose"
    down "MoveIt"
    down "RealSense"
  fi

  check_http "http://127.0.0.1:8891/health" && ok "8891" || down "8891"
  if [ "$CHECK_PIPELINE_API" = "1" ]; then
    check_http "http://127.0.0.1:8892/health" && ok "8892" || down "8892"
  fi
  if [ "$CHECK_REMOTE" = "1" ]; then
    remote_cmd "curl -fsS --max-time 1 http://127.0.0.1:8012/health" && ok "Spatial Memory" || down "Spatial Memory"
    remote_cmd "curl -fsS --max-time 1 http://127.0.0.1:8013/health" && ok "YOLO" || down "YOLO"
    remote_cmd "curl -fsS --max-time 1 http://127.0.0.1:8014/health" && ok "VLAC" || down "VLAC"
  fi

  printf "\nRead-only checks only. Refresh: %ss\n" "$REFRESH_S"
  sleep "$REFRESH_S"
done
