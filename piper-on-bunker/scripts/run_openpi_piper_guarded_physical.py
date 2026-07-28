#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PIPER_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PIPER_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.control.phase_orchestrator import PhaseOrchestrator
from piper_on_bunker.control.piper_joint_phase_executor import ExecutionConfig
from piper_on_bunker.control.piper_joint_phase_executor import PiperJointPhaseExecutor
from piper_on_bunker.hardware.live_openpi_observation import read_live_openpi_observation
from piper_on_bunker.policies.checkpoint_metadata import load_checkpoint_metadata
from piper_on_bunker.policies.checkpoint_metadata import validate_checkpoint_metadata
from piper_on_bunker.policies.openpi_piper_policy import OpenPIPiperClient
from piper_on_bunker.policies.openpi_piper_policy import PIPER_JOINT_NAMES
from piper_on_bunker.policies.openpi_piper_policy import make_observation
from piper_on_bunker.policies.openpi_piper_policy import validate_openpi_response


def _checkpoint_preflight_report(path: str | None) -> dict | None:
    if not path:
        return None
    try:
        metadata = load_checkpoint_metadata(path)
        eligibility = validate_checkpoint_metadata(metadata, require_gripper=True)
        return {
            "metadata_path": path,
            "checkpoint": metadata.get("checkpoint"),
            "piper_compatible": bool(metadata.get("piper_compatible", False)),
            "eligible_for_physical_execution": eligibility.eligible,
            "failures": list(eligibility.failures),
        }
    except ValueError as exc:
        return {
            "metadata_path": path,
            "eligible_for_physical_execution": False,
            "failures": [str(exc)],
        }


def _make_joint_state_publisher(topic: str):
    try:
        import rospy
        from sensor_msgs.msg import JointState
    except Exception as exc:
        raise RuntimeError("physical OpenPI execution requires rospy and sensor_msgs") from exc
    if not rospy.get_node_uri():
        rospy.init_node("openpi_piper_guarded_physical", anonymous=True, disable_signals=True)
    publisher = rospy.Publisher(topic, JointState, queue_size=1)

    def publish(sample):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(PIPER_JOINT_NAMES[:6])
        msg.position = [float(value) for value in sample[:6]]
        publisher.publish(msg)

    return publish


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Guarded physical runner for PiPER-compatible OpenPI direct joint checkpoints."
    )
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--phase-id", default="approach")
    parser.add_argument("--checkpoint-metadata")
    parser.add_argument("--host", default="192.168.1.104")
    parser.add_argument("--port", type=int, default=8017)
    parser.add_argument("--policy-frequency-hz", type=float, default=20.0)
    parser.add_argument("--command-topic", default="/piper_joint_commands")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--joint-topic", default="/joint_states_single")
    parser.add_argument("--exterior-image-topic", default="/table_camera/color/image_raw")
    parser.add_argument("--wrist-image-topic", default="/cam_left_wrist")
    parser.add_argument("--no-wrist", action="store_true", help="Use a zero wrist image if the wrist camera is unavailable.")
    parser.add_argument("--execute", action="store_true", help="Required for physical publishing.")
    args = parser.parse_args()

    orchestrator = PhaseOrchestrator(policy_client=OpenPIPiperClient(args.host, args.port), frequency_hz=args.policy_frequency_hz)
    plan = orchestrator.plan_task(args.instruction)
    phase = next((item for item in plan.phases if item.phase_id == args.phase_id), None)
    if phase is None:
        raise SystemExit(f"Unknown phase id: {args.phase_id}")

    wrist_topic = None if args.no_wrist else args.wrist_image_topic
    live = read_live_openpi_observation(
        timeout_s=args.timeout,
        joint_topic=args.joint_topic,
        exterior_image_topic=args.exterior_image_topic,
        wrist_image_topic=wrist_topic,
    )

    if not args.execute:
        print(
            json.dumps(
                {
                    "plan": plan.to_dict(),
                    "phase": phase.to_dict(),
                    "live_observation": live.to_summary(),
                    "checkpoint_metadata": _checkpoint_preflight_report(args.checkpoint_metadata),
                    "checkpoint_metadata_required_for_execute": True,
                    "execution_allowed": False,
                    "physical_motion_performed": False,
                    "reason": "preflight only; pass --execute with a validated PiPER-compatible checkpoint to publish commands",
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0

    if not args.checkpoint_metadata:
        raise SystemExit("--checkpoint-metadata is required with --execute")

    try:
        metadata = load_checkpoint_metadata(args.checkpoint_metadata)
        eligibility = validate_checkpoint_metadata(metadata, require_gripper=True)
        eligibility.require()
    except ValueError as exc:
        raise SystemExit(f"Physical OpenPI PiPER execution refused: {exc}") from exc

    observation = make_observation(live.exterior_image, live.wrist_image, live.state, phase.policy_prompt)
    payload = orchestrator.policy_client.infer_phase(observation)
    response = validate_openpi_response(
        payload,
        require_piper_compatible=True,
        expected_frequency_hz=args.policy_frequency_hz,
    )
    if response.metadata.checkpoint != metadata.get("checkpoint"):
        raise SystemExit(
            f"Policy service checkpoint {response.metadata.checkpoint!r} does not match metadata "
            f"{metadata.get('checkpoint')!r}"
        )

    executor = PiperJointPhaseExecutor(
        ExecutionConfig(
            physical_motion_permission=True,
            gripper_physical_enabled=True,
        )
    )
    result = executor.execute_response(
        response,
        current_state=live.state,
        state_age_s=live.state_age_s,
        camera_age_s=live.camera_age_s,
        execute=True,
        publisher=_make_joint_state_publisher(args.command_topic),
    )
    print(
        json.dumps(
            {
                "plan": plan.to_dict(),
                "phase": phase.to_dict(),
                "execution": result.__dict__,
                "checkpoint": response.metadata.checkpoint,
                "physical_motion_performed": bool(result.published_commands),
            },
            indent=2,
            default=str,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
