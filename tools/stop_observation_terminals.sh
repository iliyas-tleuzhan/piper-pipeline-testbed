#!/usr/bin/env bash
set -euo pipefail

SESSION="${PIPER_OBS_SESSION:-piper_full_stack_observation}"
PID_FILE="${PIPER_OBS_PID_FILE:-/tmp/piper_full_stack_observation.pids}"

if command -v tmux >/dev/null 2>&1 && tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
  echo "Closed tmux observation session: $SESSION"
fi

if [ -f "$PID_FILE" ]; then
  while IFS= read -r pid; do
    case "$pid" in
      ''|*[!0-9]*) continue ;;
    esac
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      echo "Requested close for observation terminal process: $pid"
    fi
  done < "$PID_FILE"
  rm -f "$PID_FILE"
fi

echo "Only observation terminals were targeted. ROS, Docker, CAN, 8891, and robot services were not stopped."
