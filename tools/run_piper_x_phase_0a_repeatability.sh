#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

python3 piper-on-bunker/scripts/run_piper_x_repeatability_diagnostic.py "$@"
