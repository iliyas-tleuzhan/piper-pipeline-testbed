#!/usr/bin/env python3
"""Passively record PiPER-X wrist-camera ArUco-touch demonstrations."""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
import uuid
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "piper-on-bunker" / "src"
sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.data.image_transforms import resize_with_pad_rgb
from piper_on_bunker.data.piper_x_aruco_episode import INCOMPLETE_MARKER, append_frame, write_json_atomic
from piper_on_bunker.perception.aruco_touch_diagnostics import ArucoDiagnosticConfig, detect_aruco_touch_diagnostics
from piper_on_bunker.profiles.piper_x_aruco import (
    PIPER_X_DEFAULT_PROMPT,
    make_wrist_only_openpi_observation,
    load_piper_x_profile,
    stable_json_hash,
)


class Latest:
    def __init__(self) -> None:
        self.stamp_s: float | None = None
        self.value: Any = None

    def update(self, value: Any, stamp_s: float | None = None) -> None:
        self.value = value
        self.stamp_s = time.time() if stamp_s is None else float(stamp_s)

    def age_s(self) -> float:
        return 999.0 if self.stamp_s is None else time.time() - self.stamp_s


def _joint_state_to_7d(msg: Any, joint_order: tuple[str, ...], fixed_gripper: float) -> list[float]:
    names = list(getattr(msg, "name", []))
    positions = list(getattr(msg, "position", []))
    by_name = {name: float(pos) for name, pos in zip(names, positions)}
    values = [by_name[name] for name in joint_order[:6]]
    values.append(float(fixed_gripper if joint_order[6] not in by_name else by_name[joint_order[6]]))
    return values


def _capture_gripper_from_state(msg: Any, gripper_name: str) -> float:
    names = list(getattr(msg, "name", []))
    positions = list(getattr(msg, "position", []))
    by_name = {str(name): float(pos) for name, pos in zip(names, positions)}
    if gripper_name not in by_name:
        raise ValueError(
            f"state message does not contain {gripper_name!r}; pass --fixed-gripper-target explicitly"
        )
    return float(by_name[gripper_name])


def _command_msg_to_action(msg: Any, joint_order: tuple[str, ...], fixed_gripper: float) -> list[float]:
    names = list(getattr(msg, "name", []))
    positions = list(getattr(msg, "position", []))
    if not names or len(positions) < 6:
        raise ValueError("command topic must publish sensor_msgs/JointState with names and positions")
    by_name = {name: float(pos) for name, pos in zip(names, positions)}
    missing = [name for name in joint_order[:6] if name not in by_name]
    if missing:
        raise ValueError(f"command topic missing required arm joint names: {missing}")
    action = [by_name[name] for name in joint_order[:6]]
    action.append(float(fixed_gripper))
    return action


def _ros_image_to_rgb(msg: Any) -> np.ndarray:
    import cv2
    from cv_bridge import CvBridge

    bridge = CvBridge()
    encoding = getattr(msg, "encoding", "")
    if encoding in {"rgb8", "rgba8"}:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
    else:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    return np.asarray(cv_img, dtype=np.uint8)


def _streams_ready(
    *,
    latest_wrist: Latest,
    latest_state: Latest,
    latest_action: Latest,
    max_image_age_s: float,
    max_state_age_s: float,
    max_action_age_s: float,
) -> tuple[bool, list[str], dict[str, float]]:
    ages = {
        "wrist_image_age_s": latest_wrist.age_s(),
        "state_age_s": latest_state.age_s(),
        "action_age_s": latest_action.age_s(),
    }
    failures: list[str] = []
    if latest_wrist.value is None:
        failures.append("missing wrist image")
    if latest_state.value is None:
        failures.append("missing state")
    if latest_action.value is None:
        failures.append("missing action")
    if latest_wrist.value is not None and ages["wrist_image_age_s"] > max_image_age_s:
        failures.append(f"stale wrist image: {ages['wrist_image_age_s']:.3f}s")
    if latest_state.value is not None and ages["state_age_s"] > max_state_age_s:
        failures.append(f"stale state: {ages['state_age_s']:.3f}s")
    if latest_action.value is not None and ages["action_age_s"] > max_action_age_s:
        failures.append(f"stale action: {ages['action_age_s']:.3f}s")
    return not failures, failures, ages


def _finalize_episode(episode_dir: Path, summary: dict[str, Any], *, clean: bool) -> None:
    write_json_atomic(episode_dir / "summary.json", summary)
    incomplete = episode_dir / INCOMPLETE_MARKER
    if clean and incomplete.exists():
        incomplete.unlink()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--instruction", default=PIPER_X_DEFAULT_PROMPT)
    parser.add_argument("--profile", default="piper-on-bunker/config/openpi_piper_x_touch_aruco.yaml")
    parser.add_argument("--episode-root")
    parser.add_argument("--fps", type=float)
    parser.add_argument("--wrist-image-topic")
    parser.add_argument("--state-topic")
    parser.add_argument("--action-source", default="ros_command_topic", choices=["ros_command_topic", "socketcan_command_frames", "pyagxarm_leader_feedback"])
    parser.add_argument("--command-topic")
    parser.add_argument("--fixed-gripper-target", type=float)
    parser.add_argument("--fixed-gripper-source", choices=["startup_state", "explicit"], default="startup_state")
    parser.add_argument("--max-image-age-s", type=float, default=0.5)
    parser.add_argument("--max-state-age-s", type=float, default=0.5)
    parser.add_argument("--max-action-age-s", type=float, default=0.5)
    parser.add_argument("--marker-id", type=int)
    parser.add_argument("--marker-size-m", type=float)
    parser.add_argument("--operator-notes")
    args = parser.parse_args()

    profile = load_piper_x_profile(args.profile)
    source_cfg = profile.raw["collection"]["verified_action_sources"][args.action_source]
    if source_cfg.get("approved_for_collection") is not True:
        raise SystemExit(f"action source {args.action_source} is not verified for PiPER-X collection: {source_cfg.get('note')}")

    import rospy
    from sensor_msgs.msg import Image, JointState

    rospy.init_node("record_openpi_piper_x_aruco_episode", anonymous=True, disable_signals=True)
    fps = float(args.fps or profile.raw["collection"]["fps"])
    period = 1.0 / fps
    output_root = Path(args.episode_root or profile.raw["output_dataset_root"])
    wrist_topic = args.wrist_image_topic or profile.raw["camera"]["wrist_image_topic"]
    state_topic = args.state_topic or profile.raw["feedback"]["state_topic"]
    command_topic = args.command_topic or profile.raw["command_labels"]["ros_command_topic"]
    marker_id = int(args.marker_id if args.marker_id is not None else profile.raw["marker"]["id"])
    marker_size = args.marker_size_m if args.marker_size_m is not None else profile.raw["marker"].get("side_length_m")
    marker_cfg = ArucoDiagnosticConfig(profile.raw["marker"]["dictionary"], marker_id, marker_size)

    latest_wrist = Latest()
    latest_state = Latest()
    latest_action = Latest()
    joint_order = profile.joint_order
    fixed_gripper: float | None = float(args.fixed_gripper_target) if args.fixed_gripper_target is not None else None
    fixed_gripper_source = "explicit_cli" if args.fixed_gripper_target is not None else "startup_state"
    if args.fixed_gripper_source == "explicit" and fixed_gripper is None:
        raise SystemExit("--fixed-gripper-source explicit requires --fixed-gripper-target")

    def on_image(msg: Any) -> None:
        latest_wrist.update(_ros_image_to_rgb(msg), getattr(msg.header.stamp, "to_sec", lambda: time.time())())

    def on_state(msg: Any) -> None:
        nonlocal fixed_gripper, fixed_gripper_source
        if fixed_gripper is None:
            fixed_gripper = _capture_gripper_from_state(msg, joint_order[6])
            fixed_gripper_source = f"startup_state:{joint_order[6]}"
        latest_state.update(_joint_state_to_7d(msg, joint_order, fixed_gripper), getattr(msg.header.stamp, "to_sec", lambda: time.time())())

    def on_command(msg: Any) -> None:
        if fixed_gripper is None:
            return
        latest_action.update(_command_msg_to_action(msg, joint_order, fixed_gripper), getattr(msg.header.stamp, "to_sec", lambda: time.time())())

    rospy.Subscriber(wrist_topic, Image, on_image, queue_size=1)
    rospy.Subscriber(state_topic, JointState, on_state, queue_size=20)
    rospy.Subscriber(command_topic, JointState, on_command, queue_size=20)

    deadline = time.time() + max(0.1, args.max_image_age_s, args.max_state_age_s, args.max_action_age_s, 1.0)
    ready = False
    failures: list[str] = []
    ages: dict[str, float] = {}
    while time.time() < deadline and not rospy.is_shutdown():
        ready, failures, ages = _streams_ready(
            latest_wrist=latest_wrist,
            latest_state=latest_state,
            latest_action=latest_action,
            max_image_age_s=args.max_image_age_s,
            max_state_age_s=args.max_state_age_s,
            max_action_age_s=args.max_action_age_s,
        )
        if ready:
            break
        time.sleep(0.02)
    if not ready or fixed_gripper is None:
        raise SystemExit(
            "refusing to create episode; required streams are missing or stale: "
            + ", ".join(failures or ["fixed gripper target unavailable"])
        )

    episode_id = f"piper_x_aruco_{int(time.time())}_{uuid.uuid4().hex[:8]}"
    episode_dir = output_root / episode_id
    (episode_dir / "images" / "wrist").mkdir(parents=True, exist_ok=True)
    (episode_dir / "images" / "exterior").mkdir(parents=True, exist_ok=True)
    (episode_dir / INCOMPLETE_MARKER).write_text("recording\n", encoding="utf-8")

    metadata = {
        "schema_version": "piper_x_aruco_raw_episode.v1",
        "episode_id": episode_id,
        "robot_profile_id": profile.robot_profile_id,
        "robot_model": profile.robot_model,
        "task_id": profile.task_id,
        "prompt": args.instruction,
        "joint_order": list(joint_order),
        "action_semantics": profile.action_semantics,
        "state_units": dict(profile.raw["state_units"]),
        "action_units": dict(profile.raw["action_units"]),
        "control_frequency_hz": fps,
        "camera_schema": dict(profile.raw["camera"]["schema"]),
        "camera_mount_id": profile.camera_mount_id,
        "image_preprocessing_id": profile.preprocessing_id,
        "fixed_gripper_target": fixed_gripper,
        "fixed_gripper_target_source": fixed_gripper_source,
        "gripper_mode": profile.gripper_mode,
        "action_source": args.action_source,
        "action_source_topic": command_topic,
        "wrist_image_topic": wrist_topic,
        "state_topic": state_topic,
        "operator_notes": args.operator_notes,
        "profile_hash": stable_json_hash(profile.raw),
    }
    write_json_atomic(episode_dir / "episode_metadata.json", metadata)

    stop = {"requested": False}

    def request_stop(_signum: int, _frame: Any) -> None:
        stop["requested"] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    frames = 0
    invalid = 0
    clean_finalization = False
    exit_code = 1
    try:
        next_tick = time.time()
        while not rospy.is_shutdown() and not stop["requested"]:
            now = time.time()
            if now < next_tick:
                time.sleep(min(0.005, next_tick - now))
                continue
            next_tick += period
            if latest_wrist.value is None or latest_state.value is None or latest_action.value is None:
                invalid += 1
                continue
            ready, failures, ages = _streams_ready(
                latest_wrist=latest_wrist,
                latest_state=latest_state,
                latest_action=latest_action,
                max_image_age_s=args.max_image_age_s,
                max_state_age_s=args.max_state_age_s,
                max_action_age_s=args.max_action_age_s,
            )
            if not ready:
                invalid += 1
                continue
            source_times = [latest_wrist.stamp_s, latest_state.stamp_s, latest_action.stamp_s]
            max_skew = max(source_times) - min(source_times)  # type: ignore[arg-type]
            if max_skew > profile.max_timestamp_skew_s:
                invalid += 1
                continue

            obs, obs_meta = make_wrist_only_openpi_observation(
                wrist_image_rgb=latest_wrist.value,
                state=latest_state.value,
                prompt=args.instruction,
            )
            wrist_path = episode_dir / "images" / "wrist" / f"{frames:06d}.npy"
            exterior_path = episode_dir / "images" / "exterior" / f"{frames:06d}.npy"
            np.save(wrist_path, obs["observation/wrist_image"])
            np.save(exterior_path, obs["observation/exterior_image"])
            diagnostics = detect_aruco_touch_diagnostics(obs["observation/wrist_image"], marker_cfg)
            frame = {
                "frame_index": frames,
                "sample_time_s": now,
                "source_timestamps_s": {
                    "wrist_image": latest_wrist.stamp_s,
                    "state": latest_state.stamp_s,
                    "action": latest_action.stamp_s,
                },
                "max_source_skew_s": max_skew,
                "source_ages_s": ages,
                "state": list(map(float, latest_state.value)),
                "action": list(map(float, latest_action.value)),
                "prompt": args.instruction,
                "wrist_image_path": str(wrist_path.relative_to(episode_dir)),
                "exterior_image_path": str(exterior_path.relative_to(episode_dir)),
                **diagnostics,
            }
            frame.update(obs_meta)
            append_frame(episode_dir, frame)
            frames += 1
        clean_finalization = True
        exit_code = 0 if frames > 0 else 3
    finally:
        summary = {
            "schema_version": "piper_x_aruco_episode_summary.v1",
            "episode_id": episode_id,
            "episode_dir": str(episode_dir),
            "frames_recorded": frames,
            "invalid_sample_ticks": invalid,
            "zero_valid_frames": frames == 0,
            "clean_finalization": clean_finalization,
            "completed_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
        _finalize_episode(episode_dir, summary, clean=clean_finalization and frames > 0)
        print(json.dumps(summary, indent=2, sort_keys=True))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
