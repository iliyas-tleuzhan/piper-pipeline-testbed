#!/usr/bin/env python3
"""Read-only PiPER-X ArUco collection environment preflight."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "piper-on-bunker" / "src"
sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.perception.aruco_touch_diagnostics import ArucoDiagnosticConfig, detect_aruco_touch_diagnostics
from piper_on_bunker.profiles.piper_x_aruco import load_piper_x_profile


def _can_status(interface: str) -> dict[str, Any]:
    try:
        proc = subprocess.run(["ip", "-details", "link", "show", interface], check=False, text=True, capture_output=True)
        text = proc.stdout + proc.stderr
        return {
            "interface": interface,
            "command_returncode": proc.returncode,
            "up": "<UP" in text or ",UP," in text,
            "error_active": "ERROR-ACTIVE" in text,
            "raw": text,
        }
    except Exception as exc:
        return {"interface": interface, "error": str(exc), "up": False, "error_active": False}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", default="piper-on-bunker/config/openpi_piper_x_touch_aruco.yaml")
    parser.add_argument("--can-interface", default="can0")
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--skip-ros", action="store_true")
    parser.add_argument("--marker-id", type=int)
    parser.add_argument("--marker-size-m", type=float)
    args = parser.parse_args()

    profile = load_piper_x_profile(args.profile)
    blockers: list[str] = []
    report: dict[str, Any] = {
        "schema_version": "piper_x_aruco_collection_preflight.v1",
        "robot_profile_id": profile.robot_profile_id,
        "repository_commit": subprocess.run(["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True, capture_output=True).stdout.strip(),
        "can": _can_status(args.can_interface),
        "manifests": {},
        "ready_for_collection": False,
    }
    if not report["can"].get("up"):
        blockers.append(f"{args.can_interface} is not UP")

    for key in (
        "required_hardware_manifest",
        "required_camera_manifest",
        "required_camera_mount_manifest",
        "required_target_manifest",
    ):
        path = REPO_ROOT / profile.raw[key]
        exists = path.exists()
        report["manifests"][key] = {"path": str(path), "exists": exists}
        if not exists:
            blockers.append(f"missing {key}: {path}")

    if not args.skip_ros:
        try:
            import rospy
            from sensor_msgs.msg import CameraInfo, Image, JointState
            from cv_bridge import CvBridge
            import cv2

            rospy.init_node("inspect_piper_x_aruco_collection_environment", anonymous=True, disable_signals=True)
            wrist_topic = profile.raw["camera"]["wrist_image_topic"]
            state_topic = profile.raw["feedback"]["state_topic"]
            info_topic = profile.raw["camera"]["camera_info_topic"]
            image_msg = rospy.wait_for_message(wrist_topic, Image, timeout=args.timeout)
            state_msg = rospy.wait_for_message(state_topic, JointState, timeout=args.timeout)
            info_msg = rospy.wait_for_message(info_topic, CameraInfo, timeout=args.timeout)
            bridge = CvBridge()
            if image_msg.encoding == "rgb8":
                image_rgb = np.asarray(bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8"), dtype=np.uint8)
            else:
                image_rgb = cv2.cvtColor(bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8"), cv2.COLOR_BGR2RGB)
            marker_id = int(args.marker_id if args.marker_id is not None else profile.raw["marker"]["id"])
            marker_size = args.marker_size_m if args.marker_size_m is not None else profile.raw["marker"].get("side_length_m")
            aruco = detect_aruco_touch_diagnostics(
                image_rgb,
                ArucoDiagnosticConfig(profile.raw["marker"]["dictionary"], marker_id, marker_size),
            )
            report["ros"] = {
                "wrist_image_topic": wrist_topic,
                "wrist_image_shape": list(image_rgb.shape),
                "wrist_image_encoding": image_msg.encoding,
                "state_topic": state_topic,
                "state_dim": len(state_msg.position),
                "camera_info_topic": info_topic,
                "camera_info_width": info_msg.width,
                "camera_info_height": info_msg.height,
                "aruco": aruco,
                "gripper_fixed_hold_value": profile.raw["gripper"].get("fixed_target_value"),
            }
            if not aruco.get("aruco_visible"):
                blockers.append("configured ArUco marker is not visible")
            if len(state_msg.position) < 6:
                blockers.append("state source does not expose at least six joints")
        except Exception as exc:
            blockers.append(f"ROS read-only checks failed: {exc}")

    if profile.raw["joint_limits"]["status"] == "unresolved":
        blockers.append("PiPER-X joint limits are unresolved; collection is still allowed, physical execution is not")
    if profile.raw["marker"].get("side_length_m") is None and args.marker_size_m is None:
        blockers.append("marker side length is not configured; pose diagnostics will be unavailable")

    report["blockers"] = blockers
    report["ready_for_collection"] = not [b for b in blockers if not b.startswith("PiPER-X joint limits")]
    report["checked_unix_s"] = time.time()
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ready_for_collection"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
