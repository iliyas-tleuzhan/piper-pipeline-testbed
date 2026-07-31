#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time

from _bootstrap import add_repo_src_to_syspath

add_repo_src_to_syspath()

from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_AGX_URDF_COMMIT
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_CONFIGURED_ARM_MODEL
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_CONFIGURED_FIRMWARE_PROFILE
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_DECODER_COMMIT
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_DECODER_REPO
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_FEEDBACK_CAN_IDS
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_FEEDBACK_SOURCE_ID
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_JOINT_MAPPING_VERSION
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_JOINT_NAMES
from piper_on_bunker.hardware.piper_x_feedback import PassivePiperXSocketcanDecoder
from piper_on_bunker.hardware.piper_x_feedback import PiperXFeedbackConfig
from piper_on_bunker.hardware.piper_x_feedback import status_json


def _open_bus(can_interface: str):
    import can

    return can.interface.Bus(channel=can_interface, interface="socketcan")


def main() -> int:
    parser = argparse.ArgumentParser(description="RX-only PiPER-X SocketCAN joint feedback bridge; publishes no commands.")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--joint-topic", default="/piper_x/joint_states")
    parser.add_argument("--status-topic", default="/piper_x/feedback_status")
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--max-age-s", type=float, default=0.5)
    parser.add_argument("--frame-set-window-s", type=float, default=0.10)
    parser.add_argument("--once", action="store_true", help="Read until one complete fresh frame set is decoded, print JSON, and exit.")
    args = parser.parse_args()

    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    rospy.init_node("piper_x_passive_socketcan_joint_state_bridge", anonymous=False, disable_signals=True)
    cfg = PiperXFeedbackConfig(max_age_s=args.max_age_s, frame_set_window_s=args.frame_set_window_s)
    decoder = PassivePiperXSocketcanDecoder(config=cfg)
    status_pub = rospy.Publisher(args.status_topic, String, queue_size=1, latch=True)
    joint_pub = rospy.Publisher(args.joint_topic, JointState, queue_size=1)

    rospy.set_param("~adapter_type", "passive_socketcan")
    rospy.set_param("~no_motion_commands_sent", True)
    rospy.set_param("~tx_frames_sent_by_bridge", 0)
    rospy.set_param("~feedback_source_id", PIPER_X_FEEDBACK_SOURCE_ID)
    rospy.set_param("~joint_mapping_version", PIPER_X_JOINT_MAPPING_VERSION)
    rospy.set_param("~dependency_repo", PIPER_X_DECODER_REPO)
    rospy.set_param("~dependency_commit", PIPER_X_DECODER_COMMIT)
    rospy.set_param("~agx_urdf_commit", PIPER_X_AGX_URDF_COMMIT)
    rospy.set_param("~arm_model", PIPER_X_CONFIGURED_ARM_MODEL)
    rospy.set_param("~firmware_profile", PIPER_X_CONFIGURED_FIRMWARE_PROFILE)
    rospy.set_param("~firmware_read_from_hardware", False)
    rospy.set_param("~source_can_ids", [hex(v) for v in PIPER_X_FEEDBACK_CAN_IDS])

    try:
        bus = _open_bus(args.can)
    except Exception as exc:
        text = status_json(None, f"SocketCAN open failed: {exc!r}")
        status_pub.publish(String(data=text))
        print(text)
        return 2

    publish_period_s = 1.0 / args.rate_hz if args.rate_hz > 0.0 else 0.0
    last_publish_s = 0.0
    last_valid_payload: dict | None = None

    def handle_message(message, *, force_publish: bool = False) -> dict | None:
        nonlocal last_publish_s, last_valid_payload
        timestamp = float(getattr(message, "timestamp", 0.0) or time.time())
        updated = decoder.update_from_can(int(message.arbitration_id), bytes(message.data), timestamp)
        if not updated:
            return None
        sample = decoder.sample(now_s=time.time())
        now_s = time.time()
        if not force_publish and publish_period_s > 0.0 and (now_s - last_publish_s) < publish_period_s:
            return sample.to_status_dict()
        msg = JointState()
        msg.header.stamp = rospy.Time.from_sec(sample.stamp_s)
        msg.name = list(PIPER_X_JOINT_NAMES)
        msg.position = list(sample.positions_rad)
        msg.velocity = []
        msg.effort = []
        joint_pub.publish(msg)
        payload = sample.to_status_dict()
        status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        last_valid_payload = payload
        last_publish_s = now_s
        return payload

    deadline = time.time() + args.max_age_s if args.once else None
    last_error = "waiting for complete passive SocketCAN frame set"
    try:
        while not rospy.is_shutdown():
            timeout = min(0.05, max(0.0, deadline - time.time())) if deadline is not None else 0.05
            message = bus.recv(timeout=timeout)
            if message is not None:
                try:
                    payload = handle_message(message, force_publish=args.once)
                except ValueError as exc:
                    last_error = str(exc)
                    if (
                        not last_error.startswith("no fresh complete PiPER-X feedback frame set")
                        or last_valid_payload is None
                    ):
                        status_pub.publish(String(data=status_json(None, last_error)))
                else:
                    if payload is not None:
                        if args.once:
                            print(json.dumps(payload, indent=2, sort_keys=True))
                            return 0
            if args.once and deadline is not None and time.time() >= deadline:
                text = status_json(None, last_error)
                status_pub.publish(String(data=text))
                print(text)
                return 2
    finally:
        shutdown = getattr(bus, "shutdown", None)
        if callable(shutdown):
            shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
