from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.policies.vlac_shadow_policy import (
    DEFAULT_ENDPOINT,
    SHADOW_BANNER,
    EndEffectorStateSI,
    VlacShadowPolicyClient,
    build_action_preview_request,
    encode_image_bytes,
    encode_image_file,
    quaternion_to_rpy_rad,
)


def _read_state_json(path: Path) -> EndEffectorStateSI:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if "position" in raw and "orientation" in raw:
        pos = raw["position"]
        ori = raw["orientation"]
        roll, pitch, yaw = quaternion_to_rpy_rad(ori["x"], ori["y"], ori["z"], ori["w"])
        return EndEffectorStateSI(float(pos["x"]), float(pos["y"]), float(pos["z"]), roll, pitch, yaw, float(raw.get("gripper_m", 0.0)))
    return EndEffectorStateSI(
        x_m=float(raw["x_m"]),
        y_m=float(raw["y_m"]),
        z_m=float(raw["z_m"]),
        roll_rad=float(raw["roll_rad"]),
        pitch_rad=float(raw["pitch_rad"]),
        yaw_rad=float(raw["yaw_rad"]),
        gripper_m=float(raw.get("gripper_m", 0.0)),
    )


def _read_live_state(timeout_s: float) -> EndEffectorStateSI:
    try:
        import rospy
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState
    except Exception as exc:
        raise RuntimeError("live ROS state requires rospy, geometry_msgs, and sensor_msgs") from exc
    if not rospy.get_node_uri():
        rospy.init_node("vlac_shadow_client", anonymous=True, disable_signals=True)
    end_pose = rospy.wait_for_message("/end_pose", PoseStamped, timeout=timeout_s)
    joint_state = rospy.wait_for_message("/joint_states_single", JointState, timeout=timeout_s)
    gripper = 0.0
    if "gripper" in joint_state.name:
        gripper = float(joint_state.position[list(joint_state.name).index("gripper")])
    pose = end_pose.pose
    roll, pitch, yaw = quaternion_to_rpy_rad(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    return EndEffectorStateSI(pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw, gripper)


def _read_live_color_image(timeout_s: float) -> str:
    try:
        import cv2
        import rospy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
    except Exception as exc:
        raise RuntimeError("live ROS image capture requires rospy, cv_bridge, sensor_msgs, and cv2") from exc
    if not rospy.get_node_uri():
        rospy.init_node("vlac_shadow_client", anonymous=True, disable_signals=True)
    msg = rospy.wait_for_message("/table_camera/color/image_raw", Image, timeout=timeout_s)
    image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
    ok, encoded = cv2.imencode(".jpg", image)
    if not ok:
        raise RuntimeError("failed to JPEG-encode live color image")
    return encode_image_bytes(encoded.tobytes())


def _default_log_path() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    directory = REPO_ROOT / "logs" / "vla_shadow"
    directory.mkdir(parents=True, exist_ok=True)
    return directory / f"{stamp}_vlac_shadow.json"


def main() -> int:
    parser = argparse.ArgumentParser(description="VLAC shadow-only action preview client")
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--image", action="append", help="PNG/JPEG path. May be repeated one to three times.")
    parser.add_argument("--state-json", type=Path)
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--state-format", default="si_json", choices=["si_json", "legacy_xyz_0p001mm_rpy_0p001deg"])
    args = parser.parse_args()

    print(SHADOW_BANNER)
    if args.image:
        images = [encode_image_file(path) for path in args.image]
    else:
        images = [_read_live_color_image(args.timeout)]
    state = _read_state_json(args.state_json) if args.state_json else _read_live_state(args.timeout)
    payload = build_action_preview_request(images, args.instruction, state, state_format=args.state_format)
    result = VlacShadowPolicyClient(args.endpoint, args.timeout).preview_action(payload)
    result["shadow_banner"] = SHADOW_BANNER
    result["input_state_si"] = state.to_dict()
    result["execution_allowed"] = False
    output_path = args.output_json or _default_log_path()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    print(f"Saved shadow result: {output_path}")
    return 0 if result.get("success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
