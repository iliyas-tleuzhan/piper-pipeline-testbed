#!/usr/bin/env python3
"""Read-only PiPER-X ArUco live shadow observation/protocol test."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "piper-on-bunker" / "src"
sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.perception.aruco_touch_diagnostics import ArucoDiagnosticConfig, detect_aruco_touch_diagnostics
from piper_on_bunker.policies.openpi_piper_policy import ShadowOpenPIPiperClient
from piper_on_bunker.profiles.piper_x_aruco import make_wrist_only_openpi_observation, load_piper_x_profile


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--instruction", default="Touch the center of the ArUco marker and retract.")
    parser.add_argument("--profile", default="piper-on-bunker/config/openpi_piper_x_touch_aruco.yaml")
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--output-json")
    args = parser.parse_args()

    import cv2
    import numpy as np
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, JointState

    profile = load_piper_x_profile(args.profile)
    rospy.init_node("run_openpi_piper_x_aruco_live_shadow", anonymous=True, disable_signals=True)
    image_msg = rospy.wait_for_message(profile.raw["camera"]["wrist_image_topic"], Image, timeout=args.timeout)
    state_msg = rospy.wait_for_message(profile.raw["feedback"]["state_topic"], JointState, timeout=args.timeout)
    bridge = CvBridge()
    if image_msg.encoding == "rgb8":
        image_rgb = np.asarray(bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8"), dtype=np.uint8)
    else:
        image_rgb = cv2.cvtColor(bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8"), cv2.COLOR_BGR2RGB)
    state = list(state_msg.position[:6]) + [float(profile.raw["gripper"].get("fixed_target_value") or 0.0)]
    observation, obs_meta = make_wrist_only_openpi_observation(
        wrist_image_rgb=image_rgb,
        state=state,
        prompt=args.instruction,
    )
    diagnostics = detect_aruco_touch_diagnostics(
        observation["observation/wrist_image"],
        ArucoDiagnosticConfig(profile.raw["marker"]["dictionary"], int(profile.raw["marker"]["id"]), profile.raw["marker"].get("side_length_m")),
    )
    response = ShadowOpenPIPiperClient(action_horizon=3, piper_compatible=False).infer(observation)
    result = {
        "schema_version": "piper_x_aruco_live_shadow.v1",
        "execution_allowed": False,
        "physical_motion_performed": False,
        "robot_profile_id": profile.robot_profile_id,
        "task_id": profile.task_id,
        "observation_metadata": obs_meta,
        "aruco": diagnostics,
        "policy_response": response.raw,
        "checked_unix_s": time.time(),
    }
    if args.output_json:
        with Path(args.output_json).open("w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, sort_keys=True)
            f.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
