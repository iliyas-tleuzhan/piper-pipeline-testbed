#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

from _bootstrap import add_repo_src_to_syspath

add_repo_src_to_syspath()

from piper_on_bunker.calibration.piper_x_repeatability import (
    base_manifest,
    build_readiness_report,
    classify_phase0a,
    load_phase0a_config,
    phase0a_lock,
    plan_repeatability_segments,
    result_json,
    run_observation_mode,
    validate_taught_pose_for_phase0a,
)
from piper_on_bunker.calibration.piper_x_repeatability_artifacts import Phase0AArtifactWriter
from piper_on_bunker.calibration.piper_x_repeatability_contract import PHASE_0A_CONFIRMATION_TOKEN
from piper_on_bunker.calibration.piper_x_repeatability_metrics import (
    endpoint_settling_metrics,
    joint_repeatability_metrics,
    physical_repeatability_metrics,
)
from piper_on_bunker.manipulation.moveit_aruco_touch import (
    RosMoveItJointSequenceBackend,
    load_taught_poses,
    load_touch_config,
)
from piper_on_bunker.models import utc_now


def _rospy_available() -> bool:
    try:
        import rospy  # noqa: F401
    except Exception:
        return False
    return True


def _rerun_live_in_noetic_container(argv: list[str]) -> int | None:
    if _rospy_available() or os.environ.get("PIPER_PHASE0A_NO_DOCKER_REEXEC") == "1":
        return None
    container = os.environ.get("ROS_CONTAINER", "abot-piper-noetic")
    live_modes = {"observe", "repeat_pose"}
    if "--mode" not in argv or argv[argv.index("--mode") + 1] not in live_modes:
        return None
    cmd = [
        "docker",
        "exec",
        "-i",
        container,
        "bash",
        "-lc",
        "source /opt/ros/noetic/setup.bash && "
        "source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash 2>/dev/null || true; "
        "export ROS_PACKAGE_PATH=/root/piper-pipeline-testbed/piper-on-bunker/ros:/tmp/piper_x_moveit_ros:${ROS_PACKAGE_PATH:-}; "
        "cd /root/piper-pipeline-testbed && "
        "python3 piper-on-bunker/scripts/run_piper_x_repeatability_diagnostic.py "
        + " ".join(subprocess.list2cmdline([arg]) for arg in argv[1:]),
    ]
    print(f"Host Python cannot import rospy; re-running Phase 0A diagnostic inside {container}.", file=sys.stderr)
    try:
        return subprocess.call(cmd)
    except FileNotFoundError:
        return None


class RosJointStateReader:
    def __init__(self, topic: str, max_age_s: float, max_velocity_rad_s: float) -> None:
        import rospy
        from piper_on_bunker.manipulation.moveit_aruco_touch import extract_named_joint_state

        self.rospy = rospy
        self.topic = topic
        self.max_age_s = float(max_age_s)
        self.max_velocity_rad_s = float(max_velocity_rad_s)
        self.extract_named_joint_state = extract_named_joint_state
        if not rospy.get_node_uri():
            rospy.init_node("piper_x_phase_0a_repeatability", anonymous=True, disable_signals=True)

    def read_joint_state(self):
        from sensor_msgs.msg import JointState

        msg = self.rospy.wait_for_message(self.topic, JointState, timeout=self.max_age_s)
        return self.extract_named_joint_state(
            names=list(msg.name),
            positions=list(msg.position),
            velocities=list(msg.velocity),
            stamp_s=float(msg.header.stamp.to_sec()),
            now_s=float(self.rospy.Time.now().to_sec()),
            max_age_s=self.max_age_s,
            max_abs_velocity_rad_s=self.max_velocity_rad_s,
            source_topic=self.topic,
        )


def _planning_only(config, args) -> dict:
    touch_config = load_touch_config(config.moveit_touch_config)
    poses = load_taught_poses(config.taught_pose_manifest)
    validate_taught_pose_for_phase0a(config, args.pose, poses, touch_config.joint_limits)
    backend = RosMoveItJointSequenceBackend(touch_config, joint_topic=config.joint_state_topic)
    plans = plan_repeatability_segments(config, pose_name=args.pose, backend=backend, touch_config=touch_config, poses=poses, cycles=args.cycles)
    return {
        "mode": "repeat_pose",
        "planning_only": True,
        "execution_enabled": False,
        "pose": args.pose,
        "cycles": args.cycles,
        "plans": [plan.__dict__ for plan in plans],
        "readiness": build_readiness_report(config),
        "motion_commanded": False,
    }


def _execute_repeat_pose(config, args) -> dict:
    if args.confirm != PHASE_0A_CONFIRMATION_TOKEN:
        raise RuntimeError(f"physical Phase 0A execution requires --confirm {PHASE_0A_CONFIRMATION_TOKEN}")
    if not config.physical_execution_enabled_by_default:
        raise RuntimeError("physical execution is disabled in the committed Phase 0A config")
    touch_config = load_touch_config(config.moveit_touch_config)
    poses = load_taught_poses(config.taught_pose_manifest)
    target_pose = validate_taught_pose_for_phase0a(config, args.pose, poses, touch_config.joint_limits)
    backend = RosMoveItJointSequenceBackend(touch_config, joint_topic=config.joint_state_topic)
    diag_id = args.diagnostic_id or f"{config.diagnostic_id_prefix}_{int(time.time())}"
    artifact_dir = Path(args.artifact_dir or Path(config.output_root) / diag_id)
    writer = Phase0AArtifactWriter(artifact_dir)
    writer.write_checklist()
    writer.write_physical_measurement_template(args.cycles)
    manifest = base_manifest(config, diag_id, mode="repeat_pose")
    manifest["test"].update({"requested_cycles": args.cycles, "pose_names": [config.staging_pose_name, args.pose], "execution_enabled": True})
    cycles = []
    final_positions = []
    aborted = False
    try:
        with phase0a_lock(config.lock_path):
            plans = plan_repeatability_segments(config, pose_name=args.pose, backend=backend, touch_config=touch_config, poses=poses, cycles=args.cycles)
            plan_iter = iter(plans)
            for cycle_index in range(args.cycles):
                to_target = next(plan_iter)
                started = time.time()
                if not backend.execute_plan(to_target, touch_config.execution_timeout_s):
                    aborted = True
                    backend.stop()
                    cycles.append({"cycle_index": cycle_index + 1, "controller_result": "aborted", "endpoint_reached": False})
                    writer.append_cycle(cycles[-1])
                    break
                state = backend.read_joint_state()
                elapsed = time.time() - started
                errors = [float(actual) - float(target) for actual, target in zip(state.positions, target_pose.positions)]
                cycle = {
                    "cycle_index": cycle_index + 1,
                    "controller_result": "succeeded",
                    "endpoint_reached": max(abs(v) for v in errors) <= config.endpoint_tolerance_rad,
                    "target_joint_values": list(target_pose.positions),
                    "final_measured_joint_values": list(state.positions),
                    "per_joint_signed_error": dict(zip(state.joint_names, errors)),
                    "per_joint_abs_error": dict(zip(state.joint_names, [abs(v) for v in errors])),
                    "maximum_joint_error_rad": max(abs(v) for v in errors),
                    "rms_joint_error_rad": (sum(v * v for v in errors) / len(errors)) ** 0.5,
                    "settling_time_s": elapsed,
                    "command_duration_s": to_target.estimated_duration_s,
                    "execution_duration_s": elapsed,
                    "stale_feedback_events": 0,
                    "timeout": False,
                }
                cycles.append(cycle)
                final_positions.append(list(state.positions))
                writer.append_cycle(cycle)
                return_plan = next(plan_iter)
                if not backend.execute_plan(return_plan, touch_config.execution_timeout_s):
                    aborted = True
                    backend.stop()
                    break
    except KeyboardInterrupt:
        aborted = True
        backend.stop()
    except Exception:
        aborted = True
        backend.stop()
        raise
    joint_metrics = joint_repeatability_metrics(joint_names=list(target_pose.joint_names), target_positions=list(target_pose.positions), final_positions_by_cycle=final_positions)
    settling = endpoint_settling_metrics(cycles)
    physical = physical_repeatability_metrics([])
    classification, passed, blockers = classify_phase0a(
        completed_cycles=len(final_positions),
        joint_metrics=joint_metrics,
        settling=settling,
        physical_metrics=physical,
        thresholds=config.thresholds,
        aborted=aborted,
    )
    manifest["completed_at"] = utc_now()
    manifest["test"]["completed_cycles"] = len(final_positions)
    manifest["results"].update(
        {
            "joint_repeatability": joint_metrics,
            "endpoint_settling": settling,
            "physical_measurements": physical,
            "blockers": blockers,
            "classification": classification,
            "phase_0a_passed": passed,
        }
    )
    writer.write_manifest(manifest)
    writer.write_joint_summary(joint_metrics)
    writer.write_report({"diagnostic_id": diag_id, "classification": classification, "phase_0a_passed": passed, "completed_cycles": len(final_positions), "blockers": blockers})
    return {"artifact_dir": str(artifact_dir), "manifest": manifest, "motion_commanded": bool(final_positions)}


def main() -> int:
    container_status = _rerun_live_in_noetic_container(sys.argv)
    if container_status is not None:
        return container_status
    parser = argparse.ArgumentParser(description="Run non-destructive PiPER-X Phase 0A repeatability diagnostics.")
    parser.add_argument("--config", default="piper-on-bunker/config/piper_x_phase_0a_repeatability.yaml")
    parser.add_argument("--mode", choices=["observe", "repeat_pose"], required=True)
    parser.add_argument("--duration-s", type=float, default=None)
    parser.add_argument("--pose", default="staging")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--artifact-dir", default="")
    parser.add_argument("--diagnostic-id", default="")
    parser.add_argument("--no-physical-measurement", action="store_true", default=False)
    args = parser.parse_args()
    config = load_phase0a_config(args.config)
    try:
        if args.mode == "observe":
            if args.execute:
                raise RuntimeError("observation mode never accepts --execute")
            reader = RosJointStateReader(config.joint_state_topic, config.max_joint_state_age_s, config.max_abs_velocity_rad_s)
            result = run_observation_mode(config, reader, duration_s=args.duration_s, artifact_dir=args.artifact_dir or None)
        else:
            if args.cycles < 1:
                raise RuntimeError("--cycles must be >= 1")
            if args.execute:
                result = _execute_repeat_pose(config, args)
            else:
                result = _planning_only(config, args)
        print(result_json(result))
        return 0
    except Exception as exc:
        print(json.dumps({"success": False, "reason": repr(exc), "motion_commanded": False}, indent=2, sort_keys=True))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
