#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
from datetime import datetime, timezone
from pathlib import Path
from time import monotonic, sleep, time


PIPER_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PIPER_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.data.episode_collection import PiperCanCommandDecoder
from piper_on_bunker.data.episode_collection import joint_state_to_state_vector
from piper_on_bunker.data.episode_collection import make_episode_id
from piper_on_bunker.data.episode_collection import resize_rgb
from piper_on_bunker.data.episode_collection import ros_command_to_sample
from piper_on_bunker.data.episode_collection import write_episode_metadata
from piper_on_bunker.data.episode_collection import write_frame_record
from piper_on_bunker.openclaw.semantic_plan import plan_manipulation_task


def _default_phase_prompt(instruction: str, phase_id: str) -> str:
    plan = plan_manipulation_task(instruction)
    for phase in plan.phases:
        if phase.phase_id == phase_id:
            return phase.policy_prompt
    raise SystemExit(f"Unknown phase id {phase_id!r}; expected one of {[phase.phase_id for phase in plan.phases]}")


class LatestRosBuffers:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.state = None
        self.exterior = None
        self.wrist = None

    def set_state(self, msg) -> None:
        with self.lock:
            self.state = msg

    def set_exterior(self, msg) -> None:
        with self.lock:
            self.exterior = msg

    def set_wrist(self, msg) -> None:
        with self.lock:
            self.wrist = msg

    def snapshot(self):
        with self.lock:
            return self.state, self.exterior, self.wrist


def _image_msg_to_rgb(msg):
    from cv_bridge import CvBridge

    return CvBridge().imgmsg_to_cv2(msg, desired_encoding="rgb8")


def _stamp_s(msg) -> float:
    try:
        return float(msg.header.stamp.to_sec())
    except Exception:
        return 0.0


def _save_frame_images(episode_dir: Path, frame_index: int, exterior, wrist):
    import cv2

    image_dir = episode_dir / "images"
    image_dir.mkdir(parents=True, exist_ok=True)
    exterior_path = image_dir / f"exterior_{frame_index:06d}.jpg"
    wrist_path = image_dir / f"wrist_{frame_index:06d}.jpg"
    cv2.imwrite(str(exterior_path), cv2.cvtColor(exterior, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(wrist_path), cv2.cvtColor(wrist, cv2.COLOR_RGB2BGR))
    return exterior_path.relative_to(episode_dir).as_posix(), wrist_path.relative_to(episode_dir).as_posix()


def _missing_buffers(buffers: LatestRosBuffers, *, no_wrist: bool) -> list[str]:
    state_msg, exterior_msg, wrist_msg = buffers.snapshot()
    missing = []
    if state_msg is None:
        missing.append("joint_state")
    if exterior_msg is None:
        missing.append("exterior_image")
    if wrist_msg is None and not no_wrist:
        missing.append("wrist_image")
    return missing


def _wait_for_observations(buffers: LatestRosBuffers, *, no_wrist: bool, timeout_s: float) -> None:
    deadline = monotonic() + float(timeout_s)
    while monotonic() < deadline:
        missing = _missing_buffers(buffers, no_wrist=no_wrist)
        if not missing:
            return
        sleep(0.05)
    missing = _missing_buffers(buffers, no_wrist=no_wrist)
    raise RuntimeError(
        "timed out waiting for required recorder observations: "
        + ", ".join(missing)
        + ". Check camera/joint publishers before recording."
    )


def _record_sample(
    *,
    episode_dir: Path,
    frame_index: int,
    command,
    buffers: LatestRosBuffers,
    instruction: str,
    phase_id: str,
    phase_prompt: str,
    max_skew_s: float,
    no_wrist: bool,
) -> bool:
    state_msg, exterior_msg, wrist_msg = buffers.snapshot()
    if state_msg is None or exterior_msg is None or (wrist_msg is None and not no_wrist):
        raise RuntimeError("record_sample called before required observations were ready")

    state, state_stamp_s, state_age_s = joint_state_to_state_vector(state_msg)
    exterior_stamp_s = _stamp_s(exterior_msg)
    wrist_stamp_s = exterior_stamp_s if no_wrist else _stamp_s(wrist_msg)
    command_stamp_s = float(command.stamp_s)
    timestamp_s = command_stamp_s
    skew_s = max(
        abs(timestamp_s - state_stamp_s),
        abs(timestamp_s - exterior_stamp_s),
        abs(timestamp_s - wrist_stamp_s),
    )
    valid = skew_s <= max_skew_s

    exterior = resize_rgb(_image_msg_to_rgb(exterior_msg))
    wrist = exterior.copy() if no_wrist else resize_rgb(_image_msg_to_rgb(wrist_msg))
    exterior_path, wrist_path = _save_frame_images(episode_dir, frame_index, exterior, wrist)
    record = {
        "frame_index": frame_index,
        "valid": bool(valid),
        "invalid_reason": None if valid else f"timestamp skew {skew_s:.6f}s exceeds max_skew_s={max_skew_s:.6f}s",
        "instruction": instruction,
        "phase_id": phase_id,
        "phase_prompt": phase_prompt,
        "timestamp_s": timestamp_s,
        "command_timestamp_s": command_stamp_s,
        "state_timestamp_s": state_stamp_s,
        "exterior_image_timestamp_s": exterior_stamp_s,
        "wrist_image_timestamp_s": wrist_stamp_s,
        "timestamp_skew_s": skew_s,
        "state_age_s": state_age_s,
        "state": [float(value) for value in state],
        "action": [float(value) for value in [*command.arm_action_rad, command.gripper_action_m]],
        "action_source": command.source,
        "gripper_action_source": command.gripper_action_source,
        "raw_command": command.raw,
        "exterior_image": exterior_path,
        "wrist_image": wrist_path,
    }
    write_frame_record(episode_dir, record)
    return valid


def main() -> int:
    parser = argparse.ArgumentParser(description="Record PiPER OpenPI episodes from teleop command streams.")
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--phase-id", required=True, choices=["approach", "grasp", "transport", "release"])
    parser.add_argument("--phase-prompt")
    parser.add_argument("--output-root", default="piper-on-bunker/data/local/openpi_episodes")
    parser.add_argument("--episode-id")
    parser.add_argument("--action-source", choices=["ros", "can"], default="can")
    parser.add_argument("--command-topic", default="/piper_joint_commands")
    parser.add_argument("--can-interface", default="can0")
    parser.add_argument("--joint-state-topic", default="/joint_states_single")
    parser.add_argument("--exterior-image-topic", default="/table_camera/color/image_raw")
    parser.add_argument("--wrist-image-topic", default="/cam_left_wrist")
    parser.add_argument("--no-wrist", action="store_true")
    parser.add_argument("--max-skew-s", type=float, default=0.08)
    parser.add_argument("--max-duration-s", type=float, default=0.0, help="0 means record until Ctrl-C.")
    parser.add_argument("--max-frames", type=int, default=0, help="0 means unlimited.")
    parser.add_argument("--observation-timeout-s", type=float, default=10.0)
    parser.add_argument("--gripper-raw-closed", type=float)
    parser.add_argument("--gripper-raw-open", type=float)
    parser.add_argument("--operator-notes", default="")
    args = parser.parse_args()

    try:
        import rospy
        from sensor_msgs.msg import Image
        from sensor_msgs.msg import JointState
    except Exception as exc:
        raise RuntimeError("episode recording must run inside the ROS Noetic environment") from exc

    if not rospy.get_node_uri():
        rospy.init_node("openpi_piper_episode_recorder", anonymous=True, disable_signals=True)

    phase_prompt = args.phase_prompt or _default_phase_prompt(args.instruction, args.phase_id)
    episode_id = args.episode_id or make_episode_id(f"piper_{args.phase_id}")
    episode_dir = Path(args.output_root) / episode_id
    buffers = LatestRosBuffers()

    rospy.Subscriber(args.joint_state_topic, JointState, buffers.set_state, queue_size=1)
    rospy.Subscriber(args.exterior_image_topic, Image, buffers.set_exterior, queue_size=1)
    if not args.no_wrist:
        rospy.Subscriber(args.wrist_image_topic, Image, buffers.set_wrist, queue_size=1)

    _wait_for_observations(buffers, no_wrist=args.no_wrist, timeout_s=args.observation_timeout_s)

    metadata = {
        "schema_version": "piper_openpi_raw_episode.v1",
        "episode_id": episode_id,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "instruction": args.instruction,
        "phase_id": args.phase_id,
        "phase_prompt": phase_prompt,
        "action_source": args.action_source,
        "joint_order": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"],
        "state_units": {"arm": "rad", "gripper": "total_jaw_opening_m"},
        "action_units": {"arm": "rad", "gripper": "total_jaw_opening_m"},
        "topics": {
            "joint_state": args.joint_state_topic,
            "ros_command": args.command_topic,
            "exterior_image": args.exterior_image_topic,
            "wrist_image": "" if args.no_wrist else args.wrist_image_topic,
        },
        "can_interface": args.can_interface if args.action_source == "can" else None,
        "max_skew_s": args.max_skew_s,
        "operator_notes": args.operator_notes,
        "gripper_note": (
            "gripper action uses current feedback unless calibrated CAN gripper raw open/closed values are provided"
        ),
    }
    write_episode_metadata(episode_dir, metadata)

    stop = {"requested": False}

    def _stop(*_):
        stop["requested"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    frame_count = 0
    valid_count = 0
    dropped_count = 0
    start = monotonic()

    if args.action_source == "ros":
        def on_command(msg):
            nonlocal frame_count, valid_count, dropped_count
            if stop["requested"]:
                return
            if _missing_buffers(buffers, no_wrist=args.no_wrist):
                dropped_count += 1
                return
            state_msg, _, _ = buffers.snapshot()
            current_gripper = 0.0
            if state_msg is not None:
                current_gripper = joint_state_to_state_vector(state_msg)[0][6]
            command = ros_command_to_sample(msg, current_gripper_m=current_gripper)
            try:
                valid = _record_sample(
                    episode_dir=episode_dir,
                    frame_index=frame_count,
                    command=command,
                    buffers=buffers,
                    instruction=args.instruction,
                    phase_id=args.phase_id,
                    phase_prompt=phase_prompt,
                    max_skew_s=args.max_skew_s,
                    no_wrist=args.no_wrist,
                )
                frame_count += 1
                valid_count += int(valid)
            except RuntimeError:
                dropped_count += 1

        rospy.Subscriber(args.command_topic, JointState, on_command, queue_size=100)
        while not stop["requested"] and (args.max_duration_s <= 0 or monotonic() - start < args.max_duration_s):
            if args.max_frames and frame_count >= args.max_frames:
                break
            sleep(0.05)
    else:
        import can

        decoder = PiperCanCommandDecoder(
            gripper_raw_closed=args.gripper_raw_closed,
            gripper_raw_open=args.gripper_raw_open,
        )
        bus = can.interface.Bus(channel=args.can_interface, interface="socketcan")
        try:
            while not stop["requested"] and (args.max_duration_s <= 0 or monotonic() - start < args.max_duration_s):
                if args.max_frames and frame_count >= args.max_frames:
                    break
                msg = bus.recv(timeout=0.25)
                if msg is None:
                    continue
                state_msg, _, _ = buffers.snapshot()
                current_gripper = 0.0
                if state_msg is not None:
                    current_gripper = joint_state_to_state_vector(state_msg)[0][6]
                command = decoder.decode(
                    int(msg.arbitration_id),
                    bytes(msg.data),
                    stamp_s=time(),
                    current_gripper_m=current_gripper,
                )
                if command is None:
                    continue
                if _missing_buffers(buffers, no_wrist=args.no_wrist):
                    dropped_count += 1
                    continue
                try:
                    valid = _record_sample(
                        episode_dir=episode_dir,
                        frame_index=frame_count,
                        command=command,
                        buffers=buffers,
                        instruction=args.instruction,
                        phase_id=args.phase_id,
                        phase_prompt=phase_prompt,
                        max_skew_s=args.max_skew_s,
                        no_wrist=args.no_wrist,
                    )
                    frame_count += 1
                    valid_count += int(valid)
                except RuntimeError:
                    dropped_count += 1
        finally:
            bus.shutdown()

    summary = {
        "episode_id": episode_id,
        "episode_dir": str(episode_dir),
        "frames_recorded": frame_count,
        "valid_frames": valid_count,
        "invalid_frames": frame_count - valid_count,
        "dropped_command_samples": dropped_count,
        "completed_utc": datetime.now(timezone.utc).isoformat(),
    }
    (episode_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if frame_count > 0 and valid_count > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
