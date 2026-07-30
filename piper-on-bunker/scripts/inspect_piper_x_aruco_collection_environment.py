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


def _stamp_s(msg: Any) -> float:
    try:
        return float(msg.header.stamp.to_sec())
    except Exception:
        return time.time()


def _named_positions(msg: Any, required_names: tuple[str, ...]) -> dict[str, float]:
    names = list(getattr(msg, "name", []))
    positions = list(getattr(msg, "position", []))
    by_name = {str(name): float(position) for name, position in zip(names, positions)}
    missing = [name for name in required_names if name not in by_name]
    if missing:
        raise ValueError(f"missing required joint names: {missing}")
    extras = [name for name in names if name in required_names]
    if len(extras) != len(required_names):
        raise ValueError(f"expected exactly {len(required_names)} named targets, got {len(extras)}")
    return {name: by_name[name] for name in required_names}


def build_state_7d(msg: Any, joint_order: tuple[str, ...]) -> list[float]:
    by_name = _named_positions(msg, joint_order)
    return [by_name[name] for name in joint_order]


def build_action_7d(msg: Any, joint_order: tuple[str, ...], fixed_gripper: float) -> list[float]:
    arm_targets = _named_positions(msg, joint_order[:6])
    return [arm_targets[name] for name in joint_order[:6]] + [float(fixed_gripper)]


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


def _topic_publishers(topic: str) -> list[str]:
    try:
        import rosgraph

        master = rosgraph.Master("/inspect_piper_x_aruco_collection_environment")
        publishers, _, _ = master.getSystemState()
        return sorted(node for published_topic, nodes in publishers if published_topic == topic for node in nodes)
    except Exception:
        return []


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", default="piper-on-bunker/config/openpi_piper_x_touch_aruco.yaml")
    parser.add_argument("--can-interface", default="can0")
    parser.add_argument("--action-source", default="ros_command_topic", choices=["ros_command_topic", "socketcan_command_frames", "pyagxarm_leader_feedback"])
    parser.add_argument("--command-topic")
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--skip-ros", action="store_true")
    parser.add_argument("--marker-id", type=int)
    parser.add_argument("--marker-size-m", type=float)
    parser.add_argument("--fixed-gripper-target", type=float)
    parser.add_argument("--max-image-age-s", type=float, default=0.5)
    parser.add_argument("--max-state-age-s", type=float, default=0.5)
    parser.add_argument("--max-action-age-s", type=float, default=0.5)
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
    if not report["can"].get("error_active"):
        blockers.append(f"{args.can_interface} is not ERROR-ACTIVE")

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
            command_topic = args.command_topic or profile.raw["command_labels"]["ros_command_topic"]
            info_topic = profile.raw["camera"]["camera_info_topic"]
            source_cfg = profile.raw["collection"]["verified_action_sources"][args.action_source]
            if source_cfg.get("approved_for_collection") is not True:
                blockers.append(f"action source {args.action_source} is not approved: {source_cfg.get('note')}")

            image_msg = rospy.wait_for_message(wrist_topic, Image, timeout=args.timeout)
            state_msg = rospy.wait_for_message(state_topic, JointState, timeout=args.timeout)
            command_msg = rospy.wait_for_message(command_topic, JointState, timeout=args.timeout)
            command_msg_2 = None
            try:
                command_msg_2 = rospy.wait_for_message(command_topic, JointState, timeout=min(args.timeout, 1.0))
            except Exception:
                pass
            info_msg = rospy.wait_for_message(info_topic, CameraInfo, timeout=args.timeout)
            receive_time = time.time()
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
            joint_order = profile.joint_order
            state_7d = build_state_7d(state_msg, joint_order)
            fixed_gripper = (
                float(args.fixed_gripper_target)
                if args.fixed_gripper_target is not None
                else float(state_7d[6])
            )
            action_7d = build_action_7d(command_msg, joint_order, fixed_gripper)
            image_stamp = _stamp_s(image_msg)
            state_stamp = _stamp_s(state_msg)
            action_stamp = _stamp_s(command_msg)
            source_stamps = [image_stamp, state_stamp, action_stamp]
            image_age = receive_time - image_stamp
            state_age = receive_time - state_stamp
            action_age = receive_time - action_stamp
            command_rate_hz = None
            if command_msg_2 is not None:
                dt = _stamp_s(command_msg_2) - action_stamp
                command_rate_hz = (1.0 / dt) if dt > 0 else None
            command_publishers = _topic_publishers(command_topic)
            report["ros"] = {
                "wrist_image_topic": wrist_topic,
                "wrist_image_shape": list(image_rgb.shape),
                "wrist_image_encoding": image_msg.encoding,
                "wrist_image_age_s": image_age,
                "state_topic": state_topic,
                "state_dim": len(state_msg.position),
                "state_schema": {"joint_order": list(joint_order), "state": state_7d},
                "state_age_s": state_age,
                "action_source": args.action_source,
                "command_topic": command_topic,
                "command_topic_publishers": command_publishers,
                "command_rate_hz_estimate": command_rate_hz,
                "action": action_7d,
                "action_age_s": action_age,
                "max_image_state_action_skew_s": max(source_stamps) - min(source_stamps),
                "action_label_warning": (
                    "Fresh JointState command labels prove topic shape only. Semantic correctness still requires "
                    "the one-joint-at-a-time operator check."
                ),
                "camera_info_topic": info_topic,
                "camera_info_width": info_msg.width,
                "camera_info_height": info_msg.height,
                "aruco": aruco,
                "gripper_fixed_hold_value": fixed_gripper,
            }
            if not aruco.get("aruco_visible"):
                blockers.append("configured ArUco marker is not visible")
            if not command_publishers:
                blockers.append(f"command topic has no visible publishers: {command_topic}")
            if image_age > args.max_image_age_s:
                blockers.append(f"wrist image is stale: {image_age:.3f}s")
            if state_age > args.max_state_age_s:
                blockers.append(f"state is stale: {state_age:.3f}s")
            if action_age > args.max_action_age_s:
                blockers.append(f"action command is stale: {action_age:.3f}s")
            if max(source_stamps) - min(source_stamps) > profile.max_timestamp_skew_s:
                blockers.append("image/state/action timestamp skew exceeds profile limit")
        except Exception as exc:
            blockers.append(f"ROS read-only checks failed: {exc}")

    if profile.raw["joint_limits"]["status"] == "unresolved":
        blockers.append("PiPER-X joint limits are unresolved; collection is still allowed, physical execution is not")
    if profile.raw["marker"].get("side_length_m") is None and args.marker_size_m is None:
        blockers.append("marker side length is not configured; pose diagnostics will be unavailable")
    blockers.append(
        "semantic command-label correctness requires one-joint-at-a-time operator verification before recording"
    )

    report["blockers"] = blockers
    allowed_warnings = (
        "PiPER-X joint limits",
        "semantic command-label correctness",
    )
    report["ready_for_collection"] = not [b for b in blockers if not b.startswith(allowed_warnings)]
    report["checked_unix_s"] = time.time()
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ready_for_collection"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
