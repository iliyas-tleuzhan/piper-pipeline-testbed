#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib
import json
import sys
import time
from typing import Any

from _bootstrap import add_repo_src_to_syspath

add_repo_src_to_syspath()

from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_AGX_URDF_COMMIT
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_CONFIGURED_ARM_MODEL
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_CONFIGURED_FIRMWARE_PROFILE
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_FEEDBACK_SOURCE_ID
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_JOINT_MAPPING_VERSION
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_JOINT_NAMES
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_TELEOP_COMMIT
from piper_on_bunker.hardware.piper_x_feedback import PiperXFeedbackConfig
from piper_on_bunker.hardware.piper_x_feedback import PiperXFeedbackEvidence
from piper_on_bunker.hardware.piper_x_feedback import ensure_no_command_methods_called
from piper_on_bunker.hardware.piper_x_feedback import status_json
from piper_on_bunker.hardware.piper_x_feedback import validate_piper_x_feedback


def _build_arm(module_name: str, class_name: str, can: str, arm_model: str, firmware_profile: str) -> Any:
    module = importlib.import_module(module_name)
    cls = getattr(module, class_name)
    attempts = (
        {"can": can, "arm_model": arm_model, "firmware": firmware_profile, "readonly": True},
        {"can": can, "arm_model": arm_model, "readonly": True},
        {"can": can},
        {"can_interface": can},
        {"channel": can},
        {},
    )
    errors: list[str] = []
    for kwargs in attempts:
        try:
            return cls(**kwargs)
        except TypeError as exc:
            errors.append(f"{kwargs}: {exc}")
    raise RuntimeError(f"could not construct {module_name}.{class_name} with safe kwargs: {errors}")


def _read_raw(arm: Any, method_name: str, calls: list[str]) -> Any:
    method = getattr(arm, method_name, None)
    if method is None:
        raise RuntimeError(f"read-only feedback method unavailable: {method_name}")
    calls.append(method_name)
    return method()


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only PiPER-X pyAgxArm joint feedback bridge; publishes no commands.")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--module", default="pyagxarm", help="Runtime module that exposes the pinned PiPER-X read-only arm class.")
    parser.add_argument("--class-name", default="AgxArm")
    parser.add_argument("--feedback-method", default="get_leader_joint_angles")
    parser.add_argument("--joint-topic", default="/piper_x/joint_states")
    parser.add_argument("--moveit-joint-topic", default="", help="Optional explicit relay topic, normally /joint_states after validation.")
    parser.add_argument("--status-topic", default="/piper_x/feedback_status")
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--allow-genuine-zero", action="store_true")
    parser.add_argument("--once", action="store_true", help="Read once, print status JSON, and exit.")
    args = parser.parse_args()

    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    rospy.init_node("piper_x_pyagxarm_joint_state_bridge", anonymous=False, disable_signals=True)
    cfg = PiperXFeedbackConfig(reject_unproven_all_zero=not args.allow_genuine_zero)
    status_pub = rospy.Publisher(args.status_topic, String, queue_size=1, latch=True)
    joint_pub = rospy.Publisher(args.joint_topic, JointState, queue_size=1)
    moveit_pub = rospy.Publisher(args.moveit_joint_topic, JointState, queue_size=1) if args.moveit_joint_topic else None

    rospy.set_param("~no_motion_commands_sent", True)
    rospy.set_param("~feedback_source_id", PIPER_X_FEEDBACK_SOURCE_ID)
    rospy.set_param("~joint_mapping_version", PIPER_X_JOINT_MAPPING_VERSION)
    rospy.set_param("~teleop_repo_commit", PIPER_X_TELEOP_COMMIT)
    rospy.set_param("~agx_urdf_commit", PIPER_X_AGX_URDF_COMMIT)
    rospy.set_param("~arm_model", PIPER_X_CONFIGURED_ARM_MODEL)
    rospy.set_param("~firmware_profile", PIPER_X_CONFIGURED_FIRMWARE_PROFILE)
    rospy.set_param("~feedback_method", args.feedback_method)

    calls: list[str] = []
    try:
        arm = _build_arm(args.module, args.class_name, args.can, PIPER_X_CONFIGURED_ARM_MODEL, PIPER_X_CONFIGURED_FIRMWARE_PROFILE)
    except Exception as exc:
        status_pub.publish(String(data=status_json(None, f"pyAgxArm unavailable: {exc!r}")))
        print(status_json(None, f"pyAgxArm unavailable: {exc!r}"))
        return 2 if args.once else _spin_failed(status_pub, f"pyAgxArm unavailable: {exc!r}")

    def read_publish_once() -> dict[str, Any]:
        raw = _read_raw(arm, args.feedback_method, calls)
        ensure_no_command_methods_called(calls)
        now = time.time()
        sample = validate_piper_x_feedback(
            raw,
            PiperXFeedbackEvidence(
                connected=True,
                feedback_valid=True,
                source_update_counter=len(calls),
                source_timestamp_s=now,
                real_feedback_packet=True,
                communication_ready=True,
                no_motion_commands_sent=True,
            ),
            config=cfg,
            now_s=now,
        )
        msg = JointState()
        msg.header.stamp = rospy.Time.from_sec(sample.stamp_s)
        msg.name = list(PIPER_X_JOINT_NAMES)
        msg.position = list(sample.positions_rad)
        msg.velocity = []
        msg.effort = []
        joint_pub.publish(msg)
        if moveit_pub is not None:
            moveit_pub.publish(msg)
        payload = sample.to_status_dict(now_s=now)
        status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        return payload

    if args.once:
        print(json.dumps(read_publish_once(), indent=2, sort_keys=True))
        return 0

    rate = rospy.Rate(args.rate_hz)
    while not rospy.is_shutdown():
        try:
            read_publish_once()
        except Exception as exc:
            status_pub.publish(String(data=status_json(None, repr(exc))))
            rospy.logwarn("PiPER-X read-only feedback bridge rejected sample: %r", exc)
        rate.sleep()
    return 0


def _spin_failed(status_pub: Any, error: str) -> int:
    import rospy
    from std_msgs.msg import String

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        status_pub.publish(String(data=status_json(None, error)))
        rate.sleep()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
