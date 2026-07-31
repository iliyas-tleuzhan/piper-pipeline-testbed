#!/usr/bin/env python3
"""Plan/report a PiPER-X wrist-depth ArUco alignment touch step.

This path is intentionally guarded. It estimates the marker center/depth and
the gripper-frame correction, but physical execution requires a local config
with a measured gripper-tip offset and a verified eye-in-hand transform.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "piper-on-bunker" / "src"
if SRC_ROOT.exists():
    sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.manipulation.piper_x_visual_servo_aruco_touch import camera_geometry_from_ros_info
from piper_on_bunker.manipulation.piper_x_visual_servo_aruco_touch import estimate_depth_touch_step
from piper_on_bunker.manipulation.piper_x_visual_servo_aruco_touch import load_visual_servo_touch_config
from piper_on_bunker.manipulation.piper_x_visual_servo_aruco_touch import quaternion_xyzw_to_matrix
from piper_on_bunker.manipulation.piper_x_visual_servo_aruco_touch import split_alignment_and_forward_steps


def _rerun_in_container(argv: list[str]) -> int:
    cmd = [
        "docker",
        "exec",
        "-i",
        "abot-piper-noetic",
        "bash",
        "-lc",
        "source /opt/ros/noetic/setup.bash && "
        "source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash 2>/dev/null || true && "
        "export ROS_MASTER_URI=http://localhost:11311 ROS_HOSTNAME=localhost PYTHONPATH=/root/piper-pipeline-testbed/piper-on-bunker/src:${PYTHONPATH:-} && "
        + " ".join(_shell_quote(arg) for arg in ["python3", "/root/piper-pipeline-testbed/piper-on-bunker/scripts/run_piper_x_visual_servo_aruco_touch.py", *argv]),
    ]
    return subprocess.call(cmd)


def _shell_quote(value: str) -> str:
    return "'" + value.replace("'", "'\"'\"'") + "'"


def _image_to_rgb(msg: Any) -> Any:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge

    bridge = CvBridge()
    if msg.encoding == "rgb8":
        return np.asarray(bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"), dtype=np.uint8)
    return cv2.cvtColor(bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"), cv2.COLOR_BGR2RGB)


def _depth_to_array(msg: Any) -> Any:
    import numpy as np
    from cv_bridge import CvBridge

    return np.asarray(CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough"))


def _capture_estimate(config: Any) -> tuple[dict[str, Any], Any]:
    import rospy
    from sensor_msgs.msg import CameraInfo, Image

    color_msg = rospy.wait_for_message(config.color_image_topic, Image, timeout=config.max_image_age_s)
    depth_msg = rospy.wait_for_message(config.depth_image_topic, Image, timeout=config.max_depth_age_s)
    info_msg = rospy.wait_for_message(config.camera_info_topic, CameraInfo, timeout=config.max_image_age_s)
    now_s = float(rospy.Time.now().to_sec())
    color_age_s = now_s - float(color_msg.header.stamp.to_sec())
    depth_age_s = now_s - float(depth_msg.header.stamp.to_sec())
    if color_age_s > config.max_image_age_s:
        raise RuntimeError(f"stale color image: {color_age_s:.3f}s")
    if depth_age_s > config.max_depth_age_s:
        raise RuntimeError(f"stale depth image: {depth_age_s:.3f}s")
    matrix, dist = camera_geometry_from_ros_info(info_msg, config=config)
    estimate = estimate_depth_touch_step(
        image_rgb=_image_to_rgb(color_msg),
        depth_image=_depth_to_array(depth_msg),
        depth_encoding=str(depth_msg.encoding),
        camera_matrix=matrix,
        dist_coeffs=dist,
        config=config,
    )
    return {
        "motion_commanded": False,
        "color_topic": config.color_image_topic,
        "depth_topic": config.depth_image_topic,
        "camera_info_topic": config.camera_info_topic,
        "color_age_s": color_age_s,
        "depth_age_s": depth_age_s,
        "camera_frame": config.camera_frame,
        "gripper_frame": config.gripper_frame,
        "handeye_source": config.handeye_source,
        "handeye_verified": config.handeye_verified,
        "gripper_tip_offset_source": config.gripper_tip_offset_source,
        "estimate": estimate.as_dict(),
    }, estimate


def _live_estimate(config_path: str) -> dict[str, Any]:
    import rospy

    config = load_visual_servo_touch_config(config_path)
    if not rospy.get_node_uri():
        rospy.init_node("piper_x_visual_servo_aruco_touch", anonymous=True, disable_signals=True)
    report, _ = _capture_estimate(config)
    return {
        "config": config_path,
        "mode": "live_read_only_estimate",
        **report,
        "physical_execution_supported_by_this_command": False,
        "next_required_setup": [
            "measure gripper tip offset in gripper_base coordinates",
            "provide a validated gripper_base -> wrist_camera_color_optical_frame transform in a local config",
            "verify the marker is centered and depth is live before any execution command is used",
        ],
    }


def _pose_translated_in_gripper_frame(pose: Any, delta_gripper_m: list[float]) -> Any:
    from copy import deepcopy

    waypoint = deepcopy(pose)
    q = pose.orientation
    rotation = quaternion_xyzw_to_matrix([q.x, q.y, q.z, q.w])
    delta = rotation @ __import__("numpy").asarray(delta_gripper_m, dtype=float).reshape(3)
    waypoint.position.x = float(pose.position.x + delta[0])
    waypoint.position.y = float(pose.position.y + delta[1])
    waypoint.position.z = float(pose.position.z + delta[2])
    return waypoint


def _execute_cartesian_delta(group: Any, config: Any, *, name: str, delta_gripper_m: list[float]) -> dict[str, Any]:
    current = group.get_current_pose(config.end_effector_link).pose
    waypoint = _pose_translated_in_gripper_frame(current, delta_gripper_m)
    plan, fraction = group.compute_cartesian_path(
        [waypoint],
        float(config.cartesian_eef_step_m),
        0.0,
    )
    if float(fraction) < float(config.cartesian_fraction_threshold):
        raise RuntimeError(f"{name} Cartesian fraction {fraction:.3f} below {config.cartesian_fraction_threshold:.3f}")
    ok = bool(group.execute(plan, wait=True))
    group.stop()
    if not ok:
        raise RuntimeError(f"{name} MoveIt execute returned false")
    return {"name": name, "delta_gripper_m": [float(v) for v in delta_gripper_m], "cartesian_fraction": float(fraction)}


def _execution_blockers(config: Any, estimate: Any, *, allow_not_centered: bool) -> list[str]:
    blockers = list(estimate.execution_blockers)
    if allow_not_centered:
        blockers = [v for v in blockers if v != "marker is not centered in wrist image"]
    if not config.physical_execution_enabled_by_default:
        if "physical execution disabled in config" not in blockers:
            blockers.append("physical execution disabled in config")
    return blockers


def _live_align_then_depth_touch(config_path: str, confirm: str) -> dict[str, Any]:
    import moveit_commander
    import rospy

    if confirm != "ALIGN_DEPTH_TOUCH":
        raise RuntimeError("physical visual-depth execution requires --confirm ALIGN_DEPTH_TOUCH")
    config = load_visual_servo_touch_config(config_path)
    if not rospy.get_node_uri():
        rospy.init_node("piper_x_visual_servo_aruco_touch_execute", anonymous=True, disable_signals=True)
    moveit_commander.roscpp_initialize([])
    group = moveit_commander.MoveGroupCommander(config.planning_group)
    group.set_end_effector_link(config.end_effector_link)
    group.set_max_velocity_scaling_factor(0.05)
    group.set_max_acceleration_scaling_factor(0.05)

    actions: list[dict[str, Any]] = []
    last_report: dict[str, Any] | None = None
    for index in range(config.max_alignment_iterations):
        report, estimate = _capture_estimate(config)
        last_report = report
        blockers = _execution_blockers(config, estimate, allow_not_centered=True)
        if blockers:
            raise RuntimeError(f"alignment blocked: {blockers}")
        if estimate.image_aligned:
            actions.append({"name": "alignment_complete", "iteration": index, "pixel_error_uv": report["estimate"]["pixel_error_uv"]})
            break
        align_step, _ = split_alignment_and_forward_steps(estimate)
        if align_step is None:
            raise RuntimeError("alignment step unavailable")
        actions.append(_execute_cartesian_delta(group, config, name=f"align_{index + 1}", delta_gripper_m=align_step))
    else:
        raise RuntimeError("marker did not align within max_alignment_iterations")

    final_report, final_estimate = _capture_estimate(config)
    blockers = _execution_blockers(config, final_estimate, allow_not_centered=False)
    if blockers:
        raise RuntimeError(f"forward touch blocked: {blockers}")
    _, forward_step = split_alignment_and_forward_steps(final_estimate)
    if forward_step is None:
        raise RuntimeError("forward depth step unavailable")
    actions.append(_execute_cartesian_delta(group, config, name="depth_forward_touch", delta_gripper_m=forward_step))
    return {
        "config": config_path,
        "mode": "align_then_depth_touch",
        "motion_commanded": True,
        "actions": actions,
        "pre_forward_estimate": final_report,
        "last_alignment_estimate": last_report,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="piper-on-bunker/config/piper_x_visual_servo_aruco_touch.yaml")
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--mode", choices=["check", "align_then_depth_touch"], default="check")
    args = parser.parse_args(argv)

    if args.execute and args.mode != "align_then_depth_touch":
        print(
            json.dumps(
                {
                    "success": False,
                    "motion_commanded": False,
                    "reason": "execution is intentionally not implemented in this guarded setup command; use this tool to verify depth/offsets first",
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 2
    if not args.live:
        parser.error("--live is required for the current read-only estimator")
    try:
        import rospy  # noqa: F401
    except Exception:
        print("Host Python cannot import rospy; re-running live visual-servo check inside abot-piper-noetic.", file=sys.stderr)
        return _rerun_in_container(sys.argv[1:])
    try:
        if args.execute:
            report = _live_align_then_depth_touch(args.config, args.confirm)
        else:
            report = _live_estimate(args.config)
    except Exception as exc:
        print(json.dumps({"success": False, "motion_commanded": False, "reason": repr(exc)}, indent=2, sort_keys=True))
        return 1
    print(json.dumps({"success": True, **report}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
