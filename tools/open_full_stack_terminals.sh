#!/usr/bin/env bash
set -euo pipefail

PIPELINE_DIR="${PIPELINE_DIR:-$HOME/piper-pipeline-testbed}"
ABOT_DIR="${ABOT_DIR:-$HOME/ABot-Claw}"
ROS_CONTAINER="${ROS_CONTAINER:-abot-piper-noetic}"
REMOTE_HOST="${REMOTE_HOST:-iliyas@master}"
REMOTE_FALLBACK_HOST="${REMOTE_FALLBACK_HOST:-iliyas@192.168.1.104}"
SESSION="${PIPER_OBS_SESSION:-piper_full_stack_observation}"
RUN_DIR="${PIPER_OBS_RUN_DIR:-/tmp/piper_full_stack_observation}"
PID_FILE="${PIPER_OBS_PID_FILE:-/tmp/piper_full_stack_observation.pids}"

SCOPE="all"
OBSERVE_ONLY=1
USE_TMUX="${PIPER_USE_TMUX:-auto}"

usage() {
  cat <<'USAGE'
Usage: tools/open_full_stack_terminals.sh [--local-only|--remote-only|--all] [--observe-only] [--start-missing] [--tmux|--gui]

Default: --all --observe-only
Observation mode never starts stopped services, never calls motion endpoints, and never enables physical motion.
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --local-only) SCOPE="local" ;;
    --remote-only) SCOPE="remote" ;;
    --all) SCOPE="all" ;;
    --observe-only) OBSERVE_ONLY=1 ;;
    --start-missing) OBSERVE_ONLY=0 ;;
    --tmux) USE_TMUX=1 ;;
    --gui) USE_TMUX=0 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
  shift
done

mkdir -p "$RUN_DIR"
: > "$PID_FILE"

term_emulator() {
  if command -v gnome-terminal >/dev/null 2>&1; then echo gnome-terminal; return 0; fi
  if command -v konsole >/dev/null 2>&1; then echo konsole; return 0; fi
  if command -v xfce4-terminal >/dev/null 2>&1; then echo xfce4-terminal; return 0; fi
  if command -v x-terminal-emulator >/dev/null 2>&1; then echo x-terminal-emulator; return 0; fi
  return 1
}

slugify() {
  printf "%s" "$1" | tr '[:upper:]' '[:lower:]' | sed -E 's/[^a-z0-9]+/_/g; s/^_//; s/_$//'
}

write_runner() {
  local title="$1"
  local workdir="$2"
  local command_text="$3"
  local slug
  slug="$(slugify "$title")"
  local file="$RUN_DIR/$slug.sh"
  local command_file="$RUN_DIR/$slug.command.sh"
  cat > "$command_file" <<EOF
#!/usr/bin/env bash
$command_text
EOF
  chmod +x "$command_file"
  cat > "$file" <<EOF
#!/usr/bin/env bash
set +e
printf '\\033]0;%s\\007' "$title"
cd "$workdir" 2>/dev/null || cd "$HOME"
echo "Title: $title"
echo "Working directory: \$(pwd)"
echo
echo "Command:"
cat <<'CMDTEXT'
$command_text
CMDTEXT
echo
bash "$command_file"
status=\$?
echo
echo "Process exited with status \$status."
echo "This terminal remains open for inspection."
exec bash
EOF
  chmod +x "$file"
  printf "%s" "$file"
}

launch_terminal() {
  local title="$1"
  local workdir="$2"
  local command_text="$3"
  local runner
  runner="$(write_runner "$title" "$workdir" "$command_text")"

  if [ "$USE_TMUX" = "1" ]; then
    if ! tmux has-session -t "$SESSION" 2>/dev/null; then
      tmux new-session -d -s "$SESSION" -n "$title" "$runner"
    else
      tmux new-window -t "$SESSION:" -n "$title" "$runner"
    fi
    return 0
  fi

  local emulator="${EMULATOR:-}"
  case "$emulator" in
    gnome-terminal)
      gnome-terminal --wait --title="$title" -- "$runner" &
      echo "$!" >> "$PID_FILE"
      ;;
    konsole)
      konsole --new-tab -p tabtitle="$title" -e "$runner" &
      echo "$!" >> "$PID_FILE"
      ;;
    xfce4-terminal)
      xfce4-terminal --title="$title" --command "$runner" &
      echo "$!" >> "$PID_FILE"
      ;;
    x-terminal-emulator)
      x-terminal-emulator -T "$title" -e "$runner" &
      echo "$!" >> "$PID_FILE"
      ;;
    *)
      echo "No terminal emulator available; using tmux session $SESSION" >&2
      USE_TMUX=1
      launch_terminal "$title" "$workdir" "$command_text"
      ;;
  esac
}

ros_prefix='source /opt/ros/noetic/setup.bash >/dev/null 2>&1; source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash >/dev/null 2>&1 || true'
ros_exec="docker exec $ROS_CONTAINER bash -lc"

service_running() {
  ss -ltn 2>/dev/null | grep -q ":$1 "
}

docker_running() {
  docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$1"
}

remote_ssh() {
  ssh -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE_HOST" "$@" ||
    ssh -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE_FALLBACK_HOST" "$@"
}

echo "Preflight inspection"
echo "Repository: $PIPELINE_DIR"
echo "ABot-Claw:  $ABOT_DIR"
echo "Mode:       $([ "$OBSERVE_ONLY" -eq 1 ] && echo observe-only || echo start-missing)"
echo
echo "Terminal emulators found:"
for candidate in gnome-terminal konsole xfce4-terminal x-terminal-emulator tmux; do
  command -v "$candidate" >/dev/null 2>&1 && echo "  $candidate: $(command -v "$candidate")"
done
echo
echo "Running tmux sessions:"
tmux list-sessions 2>/dev/null || true
echo
echo "Running Docker containers:"
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}' 2>/dev/null || true
echo
echo "Listening ports:"
ss -ltnp 2>/dev/null | grep -E ':(8891|8892|8888|11311|8012|8013|8014)\b' || true
echo
echo "Relevant ABot-Claw startup scripts:"
find "$ABOT_DIR" -maxdepth 3 -type f \( -name '*start*sh' -o -name '*realsense*sh' -o -name '*tmux*sh' \) 2>/dev/null | sort || true
echo

if [ "$USE_TMUX" = "auto" ]; then
  if [ -n "${DISPLAY:-}" ] && EMULATOR="$(term_emulator)"; then
    USE_TMUX=0
  else
    USE_TMUX=1
    EMULATOR=""
  fi
elif [ "$USE_TMUX" = "0" ]; then
  EMULATOR="$(term_emulator || true)"
else
  EMULATOR=""
fi

echo "Launch plan"
echo "  No PiPER motion commands will be sent."
echo "  hardware-demo will not be run."
echo "  physical_motion_enabled will not be changed."
if [ "$OBSERVE_ONLY" -eq 1 ]; then
  echo "  Stopped services will be reported, not started."
else
  echo "  Missing no-motion support services may be started; ROS/MoveIt/camera are still not duplicated."
fi
if [ "$USE_TMUX" = "1" ]; then
  echo "  Terminals will be created as tmux windows in session: $SESSION"
else
  echo "  Terminals will be opened with: $EMULATOR"
fi
echo

if [ "$USE_TMUX" = "1" ] && tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
fi

add_local() {
  launch_terminal "01 - PiPER CAN Monitor" "$PIPELINE_DIR" "ip -details -statistics link show can0 || { echo 'can0 is missing'; exit 1; }; if command -v candump >/dev/null 2>&1; then echo 'Passive CAN monitor only: candump can0'; candump can0; else echo 'candump is not installed'; fi"

  launch_terminal "02 - ROS Noetic / PiPER Driver" "$ABOT_DIR" "if tmux has-session -t abotclaw 2>/dev/null; then tmux capture-pane -f -t abotclaw:piper_driver -S -200; echo; echo 'Existing driver tmux session: tmux attach -t abotclaw'; else echo 'PiPER driver is not observed in tmux.'; if [ '$OBSERVE_ONLY' -eq 1 ]; then echo 'Observe-only mode: not starting driver. Startup scripts inspected in $ABOT_DIR.'; else echo 'Start command must be chosen from inspected ABot-Claw scripts; no automatic driver start here.'; fi; fi"

  launch_terminal "03 - MoveIt" "$PIPELINE_DIR" "if docker ps --format '{{.Names}}' | grep -qx '$ROS_CONTAINER'; then docker exec $ROS_CONTAINER bash -lc '$ros_prefix; rosservice list | grep joint_moveit_ctrl || true; echo; rosservice type /joint_moveit_ctrl_arm 2>/dev/null || true'; else echo '$ROS_CONTAINER is not running'; fi; if tmux has-session -t abotclaw 2>/dev/null; then echo; tmux capture-pane -f -t abotclaw:moveit_services -S -160 2>/dev/null || true; fi"

  launch_terminal "04 - PiPER State" "$PIPELINE_DIR" "if docker ps --format '{{.Names}}' | grep -qx '$ROS_CONTAINER'; then docker exec $ROS_CONTAINER bash -lc '$ros_prefix; echo joint_states_single sample; timeout 5 rostopic echo -n 1 /joint_states_single; echo; echo end_pose sample command: rostopic echo -n 1 /end_pose; timeout 5 rostopic echo -n 1 /end_pose || true; echo; echo Watching one joint state sample every 2s.; while true; do timeout 3 rostopic echo -n 1 /joint_states_single; sleep 2; done'; else echo '$ROS_CONTAINER is not running'; fi"

  launch_terminal "05 - RealSense D555" "$ABOT_DIR" "if docker ps --format '{{.Names}}' | grep -qx '$ROS_CONTAINER'; then docker exec $ROS_CONTAINER bash -lc '$ros_prefix; rostopic list | grep table_camera || true; echo; rostopic hz /table_camera/color/image_raw /table_camera/aligned_depth_to_color/image_raw /table_camera/color/camera_info'; else echo '$ROS_CONTAINER is not running'; fi; if tmux has-session -t abotclaw 2>/dev/null; then echo; tmux capture-pane -f -t abotclaw:realsense -S -160 2>/dev/null || true; fi"

  launch_terminal "06 - Camera Topics / ArUco Detection" "$PIPELINE_DIR" "if docker ps --format '{{.Names}}' | grep -qx '$ROS_CONTAINER'; then docker exec $ROS_CONTAINER bash -lc 'rm -rf /tmp/piper-pipeline-testbed-observe && mkdir -p /tmp/piper-pipeline-testbed-observe' >/dev/null; docker cp '$PIPELINE_DIR/piper-on-bunker' '$ROS_CONTAINER:/tmp/piper-pipeline-testbed-observe/' >/dev/null; docker exec $ROS_CONTAINER bash -lc '$ros_prefix; cd /tmp/piper-pipeline-testbed-observe; PYTHONPATH=piper-on-bunker/src:\$PYTHONPATH python3 piper-on-bunker/scripts/camera_only_aruco_check.py --dictionary DICT_4X4_50 --marker-id 0 --timeout 8'; else echo '$ROS_CONTAINER is not running'; fi"

  launch_terminal "07 - ABot-Claw 8891 API" "$ABOT_DIR" "if curl --noproxy '*' -fsS --max-time 2 http://localhost:8891/health; then echo; curl --noproxy '*' -fsS --max-time 2 http://localhost:8891/state; else echo '8891 is not running'; if [ '$OBSERVE_ONLY' -eq 1 ]; then echo 'Observe-only mode: not starting 8891.'; else echo 'Use existing ABot-Claw startup procedure; no move endpoints are called here.'; fi; fi; if tmux has-session -t abotclaw 2>/dev/null; then echo; tmux capture-pane -f -t abotclaw:action_8891 -S -160 2>/dev/null || true; fi"

  local api_cmd="if curl -fsS --max-time 2 http://127.0.0.1:8892/health; then echo; curl -fsS --max-time 2 http://127.0.0.1:8892/status; else echo '8892 is not running'; if [ '$OBSERVE_ONLY' -eq 1 ]; then echo 'Observe-only mode: not starting restricted API.'; echo 'Start command: docker exec $ROS_CONTAINER bash -lc \"source /opt/ros/noetic/setup.bash; cd /tmp/piper-pipeline-testbed-api; PYTHONPATH=piper-on-bunker/src:\\\$PYTHONPATH python3 piper-on-bunker/scripts/run_agent_api.py --config piper-on-bunker/config/piper_laptop_dry_run.yaml --port 8892\"'; else docker exec $ROS_CONTAINER bash -lc 'rm -rf /tmp/piper-pipeline-testbed-api && mkdir -p /tmp/piper-pipeline-testbed-api' >/dev/null; docker cp '$PIPELINE_DIR/piper-on-bunker' '$ROS_CONTAINER:/tmp/piper-pipeline-testbed-api/' >/dev/null; docker exec $ROS_CONTAINER bash -lc '$ros_prefix; cd /tmp/piper-pipeline-testbed-api; PYTHONPATH=piper-on-bunker/src:\$PYTHONPATH python3 piper-on-bunker/scripts/run_agent_api.py --config piper-on-bunker/config/piper_laptop_dry_run.yaml --port 8892'; fi; fi"
  launch_terminal "08 - Pipeline Restricted Agent API" "$PIPELINE_DIR" "$api_cmd"

  launch_terminal "09 - Live Dry Run" "$PIPELINE_DIR" "echo 'physical_motion_enabled: false'; echo 'will_call_service: false'; if docker ps --format '{{.Names}}' | grep -qx '$ROS_CONTAINER'; then docker exec $ROS_CONTAINER bash -lc 'rm -rf /tmp/piper-pipeline-testbed-dry-run && mkdir -p /tmp/piper-pipeline-testbed-dry-run' >/dev/null; docker cp '$PIPELINE_DIR/piper-on-bunker' '$ROS_CONTAINER:/tmp/piper-pipeline-testbed-dry-run/' >/dev/null; docker exec $ROS_CONTAINER bash -lc '$ros_prefix; cd /tmp/piper-pipeline-testbed-dry-run; PYTHONPATH=piper-on-bunker/src:\$PYTHONPATH python3 -m piper_on_bunker.cli live-dry-run --config piper-on-bunker/config/piper_laptop_dry_run.yaml'; else echo '$ROS_CONTAINER is not running'; fi"

  launch_terminal "14 - System Status Dashboard" "$PIPELINE_DIR" "'$PIPELINE_DIR/tools/system_status_dashboard.sh'"
}

add_remote() {
  launch_terminal "10 - Remote 5090 Services" "$PIPELINE_DIR" "ssh -o BatchMode=yes -o ConnectTimeout=5 '$REMOTE_HOST' 'cd ~/ABot-Claw-piper 2>/dev/null || cd ~; nvidia-smi || true; docker ps; echo; for p in 8012 8013 8014; do echo Port \$p; curl -fsS --max-time 2 http://127.0.0.1:\$p/health || curl -fsS --max-time 2 http://127.0.0.1:\$p/ || true; echo; done' || ssh -o BatchMode=yes -o ConnectTimeout=5 '$REMOTE_FALLBACK_HOST' 'cd ~/ABot-Claw-piper 2>/dev/null || cd ~; nvidia-smi || true; docker ps; echo; for p in 8012 8013 8014; do echo Port \$p; curl -fsS --max-time 2 http://127.0.0.1:\$p/health || curl -fsS --max-time 2 http://127.0.0.1:\$p/ || true; echo; done'"

  launch_terminal "11 - Remote VLAC Logs" "$PIPELINE_DIR" "ssh -o BatchMode=yes -o ConnectTimeout=5 '$REMOTE_HOST' 'name=\$(docker ps --format \"{{.Names}}\" | grep -Ei \"vlac|abot-vlac\" | head -n 1); [ -n \"\$name\" ] || name=\$(docker ps -a --format \"{{.Names}}\" | grep -Ei \"vlac|abot-vlac\" | head -n 1); if [ -z \"\$name\" ]; then echo \"No VLAC container found\"; exit 1; fi; docker ps -a --filter name=\$name; ss -ltn | grep :8014 || true; docker logs -f \$name' || ssh -o BatchMode=yes -o ConnectTimeout=5 '$REMOTE_FALLBACK_HOST' 'name=\$(docker ps --format \"{{.Names}}\" | grep -Ei \"vlac|abot-vlac\" | head -n 1); [ -n \"\$name\" ] || name=\$(docker ps -a --format \"{{.Names}}\" | grep -Ei \"vlac|abot-vlac\" | head -n 1); if [ -z \"\$name\" ]; then echo \"No VLAC container found\"; exit 1; fi; docker ps -a --filter name=\$name; ss -ltn | grep :8014 || true; docker logs -f \$name'"

  launch_terminal "12 - Remote YOLO Logs" "$PIPELINE_DIR" "ssh -o BatchMode=yes -o ConnectTimeout=5 '$REMOTE_HOST' 'name=\$(docker ps --format \"{{.Names}}\" | grep -Ei \"yolo|5090\" | head -n 1); [ -n \"\$name\" ] || name=\$(docker ps -a --format \"{{.Names}}\" | grep -Ei \"yolo|5090\" | head -n 1); if [ -z \"\$name\" ]; then echo \"No YOLO container found\"; exit 1; fi; docker ps -a --filter name=\$name; ss -ltn | grep :8013 || true; docker logs -f \$name' || ssh -o BatchMode=yes -o ConnectTimeout=5 '$REMOTE_FALLBACK_HOST' 'name=\$(docker ps --format \"{{.Names}}\" | grep -Ei \"yolo|5090\" | head -n 1); [ -n \"\$name\" ] || name=\$(docker ps -a --format \"{{.Names}}\" | grep -Ei \"yolo|5090\" | head -n 1); if [ -z \"\$name\" ]; then echo \"No YOLO container found\"; exit 1; fi; docker ps -a --filter name=\$name; ss -ltn | grep :8013 || true; docker logs -f \$name'"

  launch_terminal "13 - Remote Spatial Memory Logs" "$PIPELINE_DIR" "ssh -o BatchMode=yes -o ConnectTimeout=5 '$REMOTE_HOST' 'name=\$(docker ps --format \"{{.Names}}\" | grep -Ei \"spatial|memory\" | head -n 1); [ -n \"\$name\" ] || name=\$(docker ps -a --format \"{{.Names}}\" | grep -Ei \"spatial|memory\" | head -n 1); if [ -z \"\$name\" ]; then echo \"No Spatial Memory container found\"; exit 1; fi; docker ps -a --filter name=\$name; ss -ltn | grep :8012 || true; docker logs -f \$name' || ssh -o BatchMode=yes -o ConnectTimeout=5 '$REMOTE_FALLBACK_HOST' 'name=\$(docker ps --format \"{{.Names}}\" | grep -Ei \"spatial|memory\" | head -n 1); [ -n \"\$name\" ] || name=\$(docker ps -a --format \"{{.Names}}\" | grep -Ei \"spatial|memory\" | head -n 1); if [ -z \"\$name\" ]; then echo \"No Spatial Memory container found\"; exit 1; fi; docker ps -a --filter name=\$name; ss -ltn | grep :8012 || true; docker logs -f \$name'"
}

case "$SCOPE" in
  local) add_local ;;
  remote) add_remote ;;
  all) add_local; add_remote ;;
esac

echo
echo "Observation layout launched."
if [ "$USE_TMUX" = "1" ]; then
  echo "Attach with: tmux attach -t $SESSION"
fi
echo "Reopen with: $PIPELINE_DIR/tools/open_full_stack_terminals.sh --observe-only --all"
echo "Close only observation terminals with: $PIPELINE_DIR/tools/stop_observation_terminals.sh"
