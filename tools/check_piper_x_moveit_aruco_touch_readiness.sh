#!/usr/bin/env bash
set -euo pipefail

CONTAINER=${CONTAINER:-abot-piper-noetic}
cd "$(dirname "$0")/.."

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "container: not_ready ($CONTAINER not running)"
  echo "mock_ready: true"
  echo "live_read_only_ready: false"
  echo "live_planning_ready: false"
  echo "physical_execution_blocked: true"
  echo "physical_execution_ready: false"
  exit 0
fi

docker exec -i "$CONTAINER" bash -lc '
source /opt/ros/noetic/setup.bash 2>/dev/null || true
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash 2>/dev/null || true
source /root/easy_handeye_ws/devel/setup.bash 2>/dev/null || true
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

ok_topic() {
  local topic=$1
  timeout 3 rostopic echo -n 1 "$topic" >/tmp/topic_sample 2>/tmp/topic_err && echo "$topic: ok" || echo "$topic: not_ready ($(cat /tmp/topic_err 2>/dev/null))"
}

echo "camera image:"
ok_topic /wrist_camera/color/image_rect_color
echo "aruco debug image:"
ok_topic /aruco_simple/debug_image
echo "marker pose:"
python3 - <<'PY'
import rospy
from geometry_msgs.msg import PoseStamped

max_age_s = 0.5
rospy.init_node("piper_x_moveit_marker_readiness", anonymous=True, disable_signals=True)
try:
    msg = rospy.wait_for_message("/aruco_simple/pose", PoseStamped, timeout=1.0)
    age_s = rospy.Time.now().to_sec() - msg.header.stamp.to_sec()
    if age_s <= max_age_s:
        print(f"/aruco_simple/pose: ok age_s={age_s:.3f}")
    else:
        print(f"/aruco_simple/pose: stale age_s={age_s:.3f} max_age_s={max_age_s:.3f}")
except Exception as exc:
    print(f"/aruco_simple/pose: not_ready ({exc})")
PY
echo "joint state:"
ok_topic /piper_x/joint_states
echo "moveit joint state:"
ok_topic /joint_states

echo "raw CAN:"
python3 - <<'PY'
import subprocess
try:
    text = subprocess.check_output(["ip", "-details", "-statistics", "link", "show", "can0"], text=True, stderr=subprocess.STDOUT)
except Exception as exc:
    print(f"can0: not_ready ({exc})")
else:
    flags = text.split(">", 1)[0]
    if "state ERROR-ACTIVE" in text and "UP" in flags and "LOWER_UP" in flags:
        print("can0: ok UP LOWER_UP ERROR-ACTIVE")
    else:
        print("can0: not_ready " + text.replace("\n", " ")[:240])
PY

echo "passive SocketCAN feedback status:"
python3 - <<'PY'
import json
import rospy
from std_msgs.msg import String

rospy.init_node("piper_x_passive_feedback_readiness", anonymous=True, disable_signals=True)
try:
    msg = rospy.wait_for_message("/piper_x/feedback_status", String, timeout=1.0)
    payload = json.loads(msg.data)
    print("feedback_adapter_type:", payload.get("adapter_type"))
    print("passive_socketcan_connected:", payload.get("connected"))
    print("passive_socketcan_feedback_valid:", payload.get("feedback_valid"))
    print("passive_socketcan_feedback_age_s:", payload.get("feedback_age_s"))
    print("passive_socketcan_source_id:", payload.get("source_id"))
    print("passive_socketcan_joint_mapping_version:", payload.get("joint_mapping_version"))
    print("passive_socketcan_dependency_commit:", payload.get("dependency_commit"))
    print("passive_socketcan_source_can_ids:", payload.get("source_can_ids"))
    print("passive_socketcan_raw_joint_values:", payload.get("raw_joint_values"))
    print("passive_socketcan_joint_values:", payload.get("positions_rad"))
    print("passive_socketcan_tx_frames_sent_by_bridge:", payload.get("tx_frames_sent_by_bridge"))
    print("passive_socketcan_no_motion_commands_sent:", payload.get("no_motion_commands_sent"))
    if payload.get("error"):
        print("passive_socketcan_error:", payload.get("error"))
except Exception as exc:
    print(f"passive_socketcan_feedback_status: not_ready ({exc})")
PY

echo "joint-state publishers:"
python3 - <<'PY'
import subprocess
for topic in ["/piper_x/joint_states", "/joint_states", "/joint_states_single"]:
    try:
        out = subprocess.check_output(["rostopic", "info", topic], text=True, stderr=subprocess.STDOUT)
    except Exception as exc:
        print(f"{topic}: unavailable ({exc})")
        continue
    pubs = []
    in_pub = False
    for line in out.splitlines():
        if line.startswith("Publishers:"):
            in_pub = True
            continue
        if line.startswith("Subscribers:"):
            in_pub = False
        if in_pub and line.strip().startswith("*"):
            pubs.append(line.strip())
    print(f"{topic}: publishers={pubs}")
PY

echo "moveit execution controller:"
echo "/move_group/moveit_controller_manager: $(rosparam get /move_group/moveit_controller_manager 2>/dev/null || echo unavailable)"
echo "/move_group/controller_list:"
rosparam get /move_group/controller_list 2>/dev/null || echo unavailable
for t in /arm_controllers/follow_joint_trajectory/goal /arm_controllers/follow_joint_trajectory/result /arm_controllers/follow_joint_trajectory/status /arm_controllers/follow_joint_trajectory/cancel; do
  rostopic info "$t" >/dev/null 2>&1 && echo "$t: ok" || echo "$t: not_ready"
done
echo "piper_x_pyagxarm_trajectory_controller:"
if rosnode list 2>/dev/null | grep -qx /piper_x_moveit_sdk_trajectory_controller; then
  echo "node: ok"
  echo "controller_type: $(rosparam get /piper_x_moveit_sdk_trajectory_controller/controller_type 2>/dev/null || echo unavailable)"
  echo "backend: $(rosparam get /piper_x_moveit_sdk_trajectory_controller/backend 2>/dev/null || echo unavailable)"
  echo "feedback_topic: $(rosparam get /piper_x_moveit_sdk_trajectory_controller/feedback_topic 2>/dev/null || echo unavailable)"
  echo "can_interface: $(rosparam get /piper_x_moveit_sdk_trajectory_controller/can_interface 2>/dev/null || echo unavailable)"
  echo "arm_model: $(rosparam get /piper_x_moveit_sdk_trajectory_controller/arm_model 2>/dev/null || echo unavailable)"
  echo "firmware_profile: $(rosparam get /piper_x_moveit_sdk_trajectory_controller/firmware_profile 2>/dev/null || echo unavailable)"
  echo "motion_mode: $(rosparam get /piper_x_moveit_sdk_trajectory_controller/motion_mode 2>/dev/null || echo unavailable)"
  echo "command_primitive: $(rosparam get /piper_x_moveit_sdk_trajectory_controller/command_primitive 2>/dev/null || echo unavailable)"
  echo "dependency_repo: $(rosparam get /piper_x_moveit_sdk_trajectory_controller/dependency_repo 2>/dev/null || echo unavailable)"
  echo "dependency_commit: $(rosparam get /piper_x_moveit_sdk_trajectory_controller/dependency_commit 2>/dev/null || echo unavailable)"
  echo "connects_on_first_goal: $(rosparam get /piper_x_moveit_sdk_trajectory_controller/connects_on_first_goal 2>/dev/null || echo unavailable)"
  echo "motion_commanded_at_startup: $(rosparam get /piper_x_moveit_sdk_trajectory_controller/motion_commanded_at_startup 2>/dev/null || echo unavailable)"
  echo "speed_percent: $(rosparam get /piper_x_moveit_sdk_trajectory_controller/speed_percent 2>/dev/null || echo unavailable)"
  echo "command_rate_hz: $(rosparam get /piper_x_moveit_sdk_trajectory_controller/command_rate_hz 2>/dev/null || echo unavailable)"
  echo "endpoint_tolerance_rad: $(rosparam get /piper_x_moveit_sdk_trajectory_controller/endpoint_tolerance_rad 2>/dev/null || echo unavailable)"
else
  echo "node: not_ready"
fi

echo "aruco dictionary: $(rosparam get /aruco_simple/dictionary 2>/dev/null || echo unknown)"
echo "aruco marker_id: $(rosparam get /aruco_simple/marker_id 2>/dev/null || echo unknown)"
echo "aruco marker_size: $(rosparam get /aruco_simple/marker_size 2>/dev/null || echo unknown)"

echo "move_group nodes:"
rosnode list 2>/dev/null | grep move_group || true
echo "moveit planning interfaces:"
if rosnode list 2>/dev/null | grep -qx /move_group; then
  echo "/move_group: ok"
else
  echo "/move_group: not_ready"
fi
for t in /move_group/goal /move_group/result /move_group/status /execute_trajectory/goal /execute_trajectory/result /execute_trajectory/status; do
  rostopic info "$t" >/dev/null 2>&1 && echo "$t: ok" || echo "$t: not_ready"
done
echo "legacy moveit_ctrl services:"
for s in /joint_moveit_ctrl_arm /joint_moveit_ctrl_endpose /joint_moveit_ctrl_gripper /joint_moveit_ctrl_piper; do
  echo "$s: $(rosservice type "$s" 2>/dev/null || echo not_expected_for_move_group_backend)"
done
echo "robot_description_sha256: $(rosparam get /robot_description 2>/dev/null | sha256sum | awk '\''{print $1}'\'' || echo unavailable)"
echo "robot_description_name: $(python3 - <<PY
import subprocess
import xml.etree.ElementTree as ET
import yaml
try:
    raw = subprocess.check_output(["rosparam", "get", "/robot_description"], text=True, stderr=subprocess.DEVNULL)
    text = yaml.safe_load(raw)
    print(ET.fromstring(text).attrib.get("name", "unknown"))
except Exception:
    print("unavailable")
PY
)"
echo "piper_x_moveit_model_source: $(rosparam get /piper_x_moveit/model_source 2>/dev/null || echo unavailable)"
echo "piper_x_moveit_model_verified: $(rosparam get /piper_x_moveit/model_verified 2>/dev/null || echo unavailable)"
echo "piper_x_moveit_model_bounds_override: $(rosparam get /piper_x_moveit/model_bounds_override 2>/dev/null || echo unavailable)"
echo "piper_x_physical_execution_blocked_reason: $(rosparam get /piper_x_moveit/physical_execution_blocked_reason 2>/dev/null || echo unavailable)"
'

python3 piper-on-bunker/scripts/inspect_piper_x_moveit_pose.py \
  --config piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml \
  --positions "0,0,0,0,0,0" >/tmp/piper_x_moveit_pose_offline_check.json

python3 - <<'PY'
from pathlib import Path

import yaml

manifest = Path("piper-on-bunker/data/local/moveit_aruco_touch/taught_poses.yaml")
required = ["staging", "pre_touch", "touch", "retract"]
diagnostic = ["home"]
expected_joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
if not manifest.exists():
    for pose in required:
        print(f"taught_{pose}: missing")
    for pose in diagnostic:
        print(f"taught_{pose}: missing_diagnostic_optional")
    print("staging_test_planning_ready: False")
    print("fixed_touch_planning_ready: False")
    print("home_transit_diagnostic_ready: False")
    raise SystemExit(0)

try:
    data = yaml.safe_load(manifest.read_text(encoding="utf-8")) or {}
except Exception as exc:
    for pose in required:
        print(f"taught_{pose}: invalid_manifest ({exc!r})")
    raise SystemExit(0)

poses = data.get("poses") or {}
status = {}
for pose in required + diagnostic:
    payload = poses.get(pose)
    if not payload:
        label = "missing" if pose in required else "missing_diagnostic_optional"
        print(f"taught_{pose}: {label}")
        status[pose] = False
        continue
    names = list(payload.get("joint_names") or [])
    positions = list(payload.get("positions") or [])
    if names != expected_joints:
        print(f"taught_{pose}: invalid_joint_schema ({names})")
        status[pose] = False
    elif len(positions) != 6:
        print(f"taught_{pose}: invalid_position_count ({len(positions)})")
        status[pose] = False
    else:
        meta = payload.get("metadata") or {}
        required = {
            "feedback_source_id": "piper_x_passive_socketcan_feedback_v1",
            "joint_mapping_version": "piper_x_lora_feedback_2a5_2a6_2a7_raw001deg_to_rad_v1",
            "dependency_commit": "521c9c5fdfd9ee63bd96c0f9342fca6b2398092e",
        }
        missing = [k for k in required if k not in meta]
        mismatch = [k for k, v in required.items() if k in meta and str(meta.get(k)) != v]
        if missing or mismatch:
            print(f"taught_{pose}: invalidated_requires_recapture (missing={missing}, mismatch={mismatch})")
            status[pose] = False
        else:
            print(f"taught_{pose}: exists")
            status[pose] = True
print(f"staging_test_planning_ready: {all(status.get(p) for p in ['staging', 'pre_touch', 'retract'])}")
print(f"fixed_touch_planning_ready: {all(status.get(p) for p in ['staging', 'pre_touch', 'touch', 'retract'])}")
print(f"home_transit_diagnostic_ready: {all(status.get(p) for p in ['home', 'pre_touch', 'retract'])}")
PY

python3 - <<'PY'
from pathlib import Path

import yaml

paths = [
    Path("piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.local.yaml"),
    Path("piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml"),
]
selected = None
data = {}
for path in paths:
    if path.exists():
        selected = path
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        break
enabled = bool(((data.get("safety") or {}).get("physical_execution_enabled_by_default")))
if selected and selected.name.endswith(".local.yaml"):
    source = "ignored local config"
else:
    source = "committed config default"
print(f"physical_execution_config: {selected if selected else 'missing'}")
print(f"physical_execution_enabled: {str(enabled).lower()} ({source})")
PY
echo "rejected_handeye_transform_used_for_targeting: false"
echo "mock_ready: true"
echo "live_read_only_ready: requires camera image, aruco debug image, /piper_x/joint_states, passive SocketCAN valid feedback, and no stale normal-PiPER joint-state authority; marker pose additionally requires marker ID 6 visible"
echo "live_planning_ready: see staging_test_planning_ready, fixed_touch_planning_ready, and home_transit_diagnostic_ready"
echo "physical_execution_ready_requires: ignored local execution config, fresh passive /joint_states relay, arm_controllers FollowJointTrajectory action server, new verified-source taught poses, marker ID 6 visible, successful planning-only review, and explicit confirmation"
