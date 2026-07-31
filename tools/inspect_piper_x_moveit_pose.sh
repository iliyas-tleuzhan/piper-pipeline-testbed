#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
python3 piper-on-bunker/scripts/inspect_piper_x_moveit_pose.py "$@"

