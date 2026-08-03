#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

python3 piper-on-bunker/scripts/check_piper_x_phase_0a_readiness.py "$@"
