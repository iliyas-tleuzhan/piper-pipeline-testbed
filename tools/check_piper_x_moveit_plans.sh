#!/usr/bin/env bash
set -u

cd "$(dirname "$0")/.."

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage: tools/check_piper_x_moveit_plans.sh

Runs live planning-only checks for pre_touch_test and full_touch, writes JSON
reports under piper-on-bunker/logs/moveit_aruco_touch/plan_checks/, and never
requests physical execution.
EOF
  exit 0
fi

CONFIG=${CONFIG:-piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml}
OUT_DIR=${OUT_DIR:-piper-on-bunker/logs/moveit_aruco_touch/plan_checks/$(date -u +%Y%m%dT%H%M%SZ)}

mkdir -p "$OUT_DIR"

run_plan() {
  local sequence=$1
  local output="$OUT_DIR/${sequence}.json"
  echo "=== ${sequence} planning-only ==="
  python3 piper-on-bunker/scripts/run_moveit_aruco_touch.py \
    --config "$CONFIG" \
    --live \
    --planning-only \
    --sequence "$sequence" \
    --publish-plans-to-rviz >"$output"
  local status=$?
  cat "$output"
  echo
  echo "${sequence}_exit_code: ${status}"
  echo
  return 0
}

run_plan pre_touch_test
run_plan full_touch

python3 - "$OUT_DIR" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
print("=== Summary ===")
print(f"report_dir: {root}")
for name in ["pre_touch_test", "full_touch"]:
    path = root / f"{name}.json"
    try:
        text = path.read_text(encoding="utf-8")
        start = text.find("{")
        depth = 0
        end = -1
        for index, char in enumerate(text[start:], start=start):
            if char == "{":
                depth += 1
            elif char == "}":
                depth -= 1
                if depth == 0:
                    end = index + 1
                    break
        data = json.loads(text[start:end])
    except Exception as exc:
        print(f"{name}: unreadable ({exc})")
        continue
    outputs = data.get("outputs") or {}
    plans = outputs.get("plans") or outputs.get("partial_plans") or []
    print(f"{name}: success={data.get('success')} state={data.get('state')} reason={data.get('failure_reason')}")
    for plan in plans:
        metrics = plan.get("metrics") or {}
        print(
            "  {name}: points={points} duration_s={duration:.3f} "
            "max_delta={max_delta:.6f} max_adjacent={max_adjacent:.6f} "
            "continuity={continuity:.6f} target_error={target_error:.6f}".format(
                name=plan.get("name"),
                points=plan.get("trajectory_points", 0),
                duration=float(plan.get("estimated_duration_s") or 0.0),
                max_delta=float(plan.get("maximum_joint_delta_rad") or 0.0),
                max_adjacent=float(plan.get("maximum_adjacent_joint_delta_rad") or 0.0),
                continuity=float(metrics.get("maximum_continuity_error_rad") or 0.0),
                target_error=float(metrics.get("maximum_target_error_rad") or 0.0),
            )
        )
PY

echo "No physical execution was requested by this helper."
