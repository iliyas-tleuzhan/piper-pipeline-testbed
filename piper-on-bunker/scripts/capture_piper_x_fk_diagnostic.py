#!/usr/bin/env python3
"""Read-only PiPER-X FK diagnostic capture for stopped operator-selected poses."""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path


ROS_SETUP = (
    "source /opt/ros/noetic/setup.bash; "
    "source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash; "
    "source /root/easy_handeye_ws/devel/setup.bash; "
    "export ROS_MASTER_URI=http://localhost:11311; "
    "export ROS_HOSTNAME=localhost; "
)


def run_container(container: str, command: str, timeout_s: float = 6.0) -> dict:
    proc = subprocess.run(
        ["docker", "exec", "-i", container, "bash", "-lc", ROS_SETUP + command],
        text=True,
        capture_output=True,
        timeout=timeout_s,
        check=False,
    )
    return {"returncode": proc.returncode, "stdout": proc.stdout, "stderr": proc.stderr}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--container", default="abot-piper-noetic")
    parser.add_argument("--output", required=True, help="Output JSON path")
    parser.add_argument("--can-interface", default="can0")
    args = parser.parse_args()

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    urdf = run_container(args.container, "P=$(rospack find piper_description)/urdf/piper_description.urdf; echo $P; sha256sum $P", 5)
    urdf_candidates = run_container(
        args.container,
        r"""P=$(rospack find piper_description)/urdf
find "$P" -maxdepth 1 -type f \( -name "*.urdf" -o -name "*.xacro" \) -print0 | sort -z | xargs -0 sha256sum""",
        6,
    )
    revisions = run_container(
        args.container,
        r"""python3 - <<'PY'
import json, subprocess
from pathlib import Path
items = {}
for name, path in {
    "piper_ros": "/root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/src/piper_ros",
    "piper_sdk_checkout": "/root/piper_sdk",
}.items():
    p = Path(path)
    if (p / ".git").exists():
        items[name] = subprocess.run(["git", "-C", str(p), "rev-parse", "HEAD"], text=True, capture_output=True, check=False).stdout.strip() or None
    else:
        items[name] = None
try:
    import importlib.metadata
    items["piper_sdk_distribution_version"] = importlib.metadata.version("piper_sdk")
except Exception as exc:
    items["piper_sdk_distribution_version_error"] = repr(exc)
try:
    import piper_sdk
    items["piper_sdk_module_file"] = getattr(piper_sdk, "__file__", None)
    items["piper_sdk_module_version"] = getattr(piper_sdk, "__version__", None)
except Exception as exc:
    items["piper_sdk_module_error"] = repr(exc)
print(json.dumps(items, indent=2, sort_keys=True))
PY""",
        8,
    )
    sdk = run_container(
        args.container,
        rf"""python3 - <<'PY'
import inspect, json, time
from piper_sdk import C_PiperInterface_V2
result = {{}}
result["constructor_signature"] = str(inspect.signature(C_PiperInterface_V2))
for name in ["SearchPiperFirmwareVersion", "GetPiperFirmwareVersion", "EnableFkCal", "GetFK", "GetArmEndPoseMsgs"]:
    attr = getattr(C_PiperInterface_V2, name, None)
    result[name + "_signature"] = None if attr is None else str(inspect.signature(attr))
try:
    p = C_PiperInterface_V2("{args.can_interface}", judge_flag=False, can_auto_init=True)
    result["created"] = True
    try:
        result["firmware_before_search"] = p.GetPiperFirmwareVersion()
    except Exception as exc:
        result["firmware_before_search_error"] = repr(exc)
    try:
        p.SearchPiperFirmwareVersion()
        result["firmware_search_called"] = True
    except Exception as exc:
        result["firmware_search_error"] = repr(exc)
    time.sleep(2.0)
    try:
        result["firmware"] = p.GetPiperFirmwareVersion()
    except Exception as exc:
        result["firmware_error"] = repr(exc)
    try:
        p.EnableFkCal()
        result["enable_fk_cal_called"] = True
    except Exception as exc:
        result["enable_fk_cal_error"] = repr(exc)
    time.sleep(1.0)
    for mode in ["feedback", "control"]:
        try:
            result["sdk_fk_" + mode] = p.GetFK(mode)
        except Exception as exc:
            result["sdk_fk_" + mode + "_error"] = repr(exc)
    try:
        result["sdk_end_pose"] = str(p.GetArmEndPoseMsgs())
    except Exception as exc:
        result["sdk_end_pose_error"] = repr(exc)
except Exception as exc:
    result["created"] = False
    result["create_error"] = repr(exc)
print(json.dumps(result, indent=2, sort_keys=True))
PY""",
        12,
    )
    report = {
        "created_unix_s": time.time(),
        "safety": {
            "read_only": True,
            "robot_enable_command_sent": False,
            "motion_commands_published": False,
            "notes": "This helper reads ROS topics, TF, firmware query, and SDK FK only.",
        },
        "ros": {
            "joint_states_single": run_container(args.container, "rostopic echo -n 1 /joint_states_single", 6),
            "joint_states": run_container(args.container, "rostopic echo -n 1 /joint_states", 6),
            "end_pose": run_container(args.container, "rostopic echo -n 1 /end_pose", 6),
            "base_to_link6": run_container(args.container, "timeout 4 rosrun tf tf_echo base_link link6", 6),
            "base_to_gripper_base": run_container(args.container, "timeout 4 rosrun tf tf_echo base_link gripper_base", 6),
            "base_to_gripper_tcp": run_container(args.container, "timeout 4 rosrun tf tf_echo base_link gripper_tcp", 6),
            "robot_description_hash": run_container(args.container, "rosparam get /robot_description | sha256sum", 6),
            "urdf_path_and_hash": urdf,
            "urdf_candidate_hashes": urdf_candidates,
            "piper_x_handeye_model_params": run_container(args.container, "rosparam get /piper_x_handeye_model 2>/dev/null || true", 6),
        },
        "runtime_revisions": revisions,
        "sdk": sdk,
    }
    out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
