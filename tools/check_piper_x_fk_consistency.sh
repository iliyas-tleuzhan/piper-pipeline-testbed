#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${CONTAINER:-abot-piper-noetic}"
CAN_INTERFACE="${CAN_INTERFACE:-can0}"
OUTPUT_ROOT="${OUTPUT_ROOT:-piper-on-bunker/data/local/piper_x_fk_diagnostics}"
POSE_LABEL="${POSE_LABEL:-pose}"

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
out_dir="$OUTPUT_ROOT/$timestamp"
mkdir -p "$out_dir"

echo "Capturing one read-only PiPER-X FK diagnostic at the current stopped pose."
echo "This script does not enable the robot, move the robot, publish commands, run OpenPI, or run replay."
echo "Output directory: $out_dir"

PYTHONPATH="piper-on-bunker/src:${PYTHONPATH:-}" \
  python3 piper-on-bunker/scripts/capture_piper_x_fk_diagnostic.py \
    --container "$CONTAINER" \
    --can-interface "$CAN_INTERFACE" \
    --output "$out_dir/${POSE_LABEL}_${timestamp}.json"

cat >"$out_dir/README.md" <<EOF
# PiPER-X FK Diagnostic Capture

Captured at: $timestamp UTC

This is a read-only stopped-pose capture. The operator must manually reposition
the arm through the separate proven teleoperation setup between captures.

To analyze several poses:

\`\`\`bash
cd ~/piper-pipeline-testbed
PYTHONPATH=piper-on-bunker/src \\
  python3 piper-on-bunker/scripts/analyze_piper_x_fk_diagnostics.py \\
    piper-on-bunker/data/local/piper_x_fk_diagnostics/*/*.json
\`\`\`

No robot motion is commanded by this tool.
EOF

echo "$out_dir"
