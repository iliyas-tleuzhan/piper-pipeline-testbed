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
data.setdefault("safety", {})["physical_execution_enabled_by_default"] = True
target.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
print(f"created_local_execution_config: {target}")
print("physical_execution_enabled_by_default: true")
print("committed_source_config_unchanged: true")
PY
