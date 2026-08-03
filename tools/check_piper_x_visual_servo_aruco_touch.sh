#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

python3 piper-on-bunker/scripts/run_piper_x_visual_servo_aruco_touch.py \
  --config "${1:-piper-on-bunker/config/piper_x_visual_servo_aruco_touch.yaml}" \
  --live
