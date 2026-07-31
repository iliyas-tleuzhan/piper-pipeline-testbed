#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
python3 piper-on-bunker/scripts/save_piper_x_moveit_taught_pose.py "$@"

