#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

SOURCE_CONFIG=${SOURCE_CONFIG:-piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml}
LOCAL_CONFIG=${LOCAL_CONFIG:-piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.local.yaml}

python3 - "$SOURCE_CONFIG" "$LOCAL_CONFIG" <<'PY'
import sys
from pathlib import Path
import yaml

source = Path(sys.argv[1])
target = Path(sys.argv[2])
data = yaml.safe_load(source.read_text(encoding="utf-8"))
motion = data.setdefault("motion", {})
motion["motion_profiles"] = {
    "transit": {"velocity_scaling": 0.25, "acceleration_scaling": 0.20},
    "approach": {"velocity_scaling": 0.15, "acceleration_scaling": 0.12},
    "touch": {"velocity_scaling": 0.10, "acceleration_scaling": 0.08},
    "retract": {"velocity_scaling": 0.15, "acceleration_scaling": 0.12},
}
motion["velocity_scaling"] = 0.15
motion["acceleration_scaling"] = 0.12
motion["max_segment_duration_s"] = 120.0
motion["max_mission_duration_s"] = 300.0
motion["min_effective_joint_velocity_rad_s"] = 0.0
motion["segment_duration_limits_s"] = {
    "move_staging": 120.0,
    "move_pre_touch": 120.0,
    "move_touch": 120.0,
    "move_retract": 120.0,
    "move_staging_final": 120.0,
    "move_home": 120.0,
    "move_home_final": 120.0,
}
data.setdefault("safety", {})["physical_execution_enabled_by_default"] = True
target.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
print(f"created_local_execution_config: {target}")
print("physical_execution_enabled_by_default: true")
print("motion_profiles: transit=0.25/0.20 approach=0.15/0.12 touch=0.10/0.08 retract=0.15/0.12")
print("duration_limits: segment=120s mission=300s")
print("min_effective_joint_velocity_rad_s: 0.0")
print("committed_source_config_unchanged: true")
PY
