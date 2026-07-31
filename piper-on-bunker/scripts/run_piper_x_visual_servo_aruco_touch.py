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
import threading
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "piper-on-bunker" / "src"
if SRC_ROOT.exists():
    sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.manipulation.piper_x_visual_servo_aruco_touch import camera_geometry_from_ros_info
from piper_on_bunker.manipulation.piper_x_visual_servo_aruco_touch import depth_roi_m
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


def _pose_translated_in_world(pose: Any, delta_world_m: list[float]) -> Any:
    from copy import deepcopy

    waypoint = deepcopy(pose)
    waypoint.position.x = float(pose.position.x + float(delta_world_m[0]))
    waypoint.position.y = float(pose.position.y + float(delta_world_m[1]))
    waypoint.position.z = float(pose.position.z + float(delta_world_m[2]))
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


def _execute_world_delta(group: Any, config: Any, *, name: str, delta_world_m: list[float]) -> dict[str, Any]:
    current = group.get_current_pose(config.end_effector_link).pose
    waypoint = _pose_translated_in_world(current, delta_world_m)
    plan, fraction = group.compute_cartesian_path([waypoint], float(config.cartesian_eef_step_m), 0.0)
    if float(fraction) < float(config.cartesian_fraction_threshold):
        raise RuntimeError(f"{name} Cartesian fraction {fraction:.3f} below {config.cartesian_fraction_threshold:.3f}")
    ok = bool(group.execute(plan, wait=True))
    group.stop()
    if not ok:
        raise RuntimeError(f"{name} MoveIt execute returned false")
    return {"name": name, "delta_world_m": [float(v) for v in delta_world_m], "cartesian_fraction": float(fraction)}


def _center_depth_m_from_msg(msg: Any, config: Any) -> float | None:
    depth = _depth_to_array(msg)
    return depth_roi_m(
        depth,
        u=float(msg.width) / 2.0,
        v=float(msg.height) / 2.0,
        encoding=str(msg.encoding),
        roi_px=config.depth_roi_px,
    )


def _capture_center_depth(config: Any) -> dict[str, Any]:
    import rospy
    from sensor_msgs.msg import Image

    msg = rospy.wait_for_message(config.depth_image_topic, Image, timeout=config.max_depth_age_s)
    now_s = float(rospy.Time.now().to_sec())
    age_s = now_s - float(msg.header.stamp.to_sec())
    if age_s > config.max_depth_age_s:
        raise RuntimeError(f"stale depth image: {age_s:.3f}s")
    return {
        "depth_topic": config.depth_image_topic,
        "depth_age_s": age_s,
        "center_depth_m": _center_depth_m_from_msg(msg, config),
        "marker_required": False,
    }


def _execute_monitored_forward(group: Any, config: Any, *, distance_m: float) -> dict[str, Any]:
    import rospy
    from sensor_msgs.msg import Image

    stop_event = threading.Event()
    done_event = threading.Event()
    monitor: dict[str, Any] = {
        "triggered": False,
        "last_center_depth_m": None,
        "samples": 0,
        "stop_depth_m": config.continuous_forward_stop_depth_m,
    }

    def watch_depth() -> None:
        while not done_event.is_set() and not rospy.is_shutdown():
            try:
                msg = rospy.wait_for_message(config.depth_image_topic, Image, timeout=max(0.05, config.max_depth_age_s))
                depth_m = _center_depth_m_from_msg(msg, config)
            except Exception as exc:
                monitor["last_error"] = repr(exc)
                continue
            monitor["samples"] += 1
            monitor["last_center_depth_m"] = depth_m
            if depth_m is not None and depth_m > 0.0 and depth_m <= config.continuous_forward_stop_depth_m:
                monitor["triggered"] = True
                stop_event.set()
                try:
                    group.stop()
                except Exception as exc:
                    monitor["stop_error"] = repr(exc)
                return

    current = group.get_current_pose(config.end_effector_link).pose
    axis = _normalized_forward(config, 1.0)
    waypoint_count = max(1, int(float(distance_m) / max(config.simple_forward_step_m, 1e-6)))
    waypoints = []
    for index in range(1, waypoint_count + 1):
        mag = min(float(distance_m), index * config.simple_forward_step_m)
        waypoints.append(_pose_translated_in_world(current, [axis[0] * mag, axis[1] * mag, axis[2] * mag]))
    plan, fraction = group.compute_cartesian_path(waypoints, float(config.cartesian_eef_step_m), 0.0)
    if float(fraction) <= 0.0:
        raise RuntimeError("continuous forward Cartesian planner returned zero usable path")

    thread = threading.Thread(target=watch_depth, daemon=True)
    thread.start()
    ok = False
    try:
        ok = bool(group.execute(plan, wait=True))
    finally:
        done_event.set()
        try:
            group.stop()
        except Exception:
            pass
        thread.join(timeout=1.0)
    return {
        "name": "continuous_forward_until_depth",
        "planned_forward_distance_m": float(distance_m),
        "cartesian_fraction": float(fraction),
        "cartesian_fraction_warning": None
        if float(fraction) >= float(config.cartesian_fraction_threshold)
        else f"executed partial Cartesian path fraction {fraction:.3f}; continuing command did not fail on fraction",
        "moveit_execute_returned": ok,
        "depth_stop_triggered": bool(monitor["triggered"]),
        "depth_monitor": monitor,
    }


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
    index = 0
    while not rospy.is_shutdown():
        index += 1
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


def _live_simple_up_then_forward(config_path: str, confirm: str) -> dict[str, Any]:
    import moveit_commander
    import rospy

    if confirm != "SIMPLE_UP_FORWARD":
        raise RuntimeError("simple up/forward execution requires --confirm SIMPLE_UP_FORWARD")
    config = load_visual_servo_touch_config(config_path)
    if not rospy.get_node_uri():
        rospy.init_node("piper_x_simple_up_forward_aruco_touch", anonymous=True, disable_signals=True)
    moveit_commander.roscpp_initialize([])
    group = moveit_commander.MoveGroupCommander(config.planning_group)
    group.set_end_effector_link(config.end_effector_link)
    group.set_max_velocity_scaling_factor(0.04)
    group.set_max_acceleration_scaling_factor(0.04)
    actions: list[dict[str, Any]] = []
    report, estimate = _capture_estimate(config)
    blockers = _execution_blockers(config, estimate, allow_not_centered=True)
    if blockers:
        raise RuntimeError(f"simple up/forward blocked: {blockers}")
    if estimate.pixel_error_uv is None:
        raise RuntimeError("marker pixel error unavailable")
    # Image v decreases when marker is above center. Move upward in world Z and
    # then stop. This is deliberately simple and avoids gripper-frame hand-eye
    # deltas for the alignment stage.
    if estimate.pixel_error_uv[1] < -config.image_center_tolerance_px:
        actions.append(_execute_world_delta(group, config, name="simple_up", delta_world_m=[0.0, 0.0, config.simple_up_step_m]))
    elif estimate.pixel_error_uv[1] > config.image_center_tolerance_px:
        actions.append(_execute_world_delta(group, config, name="simple_down", delta_world_m=[0.0, 0.0, -config.simple_up_step_m]))
    else:
        actions.append({"name": "vertical_alignment_already_in_tolerance", "pixel_error_uv": list(estimate.pixel_error_uv)})
    post_up_report, post_up_estimate = _capture_estimate(config)
    blockers = _execution_blockers(config, post_up_estimate, allow_not_centered=True)
    if blockers:
        raise RuntimeError(f"forward step blocked after up/down move: {blockers}")
    depth = float(post_up_estimate.depth_m or 0.0)
    forward_mag = min(config.simple_forward_step_m, max(0.0, depth - config.contact_clearance_m))
    axis = config.simple_forward_axis_world
    norm = sum(v * v for v in axis) ** 0.5
    if norm <= 0.0:
        raise RuntimeError("simple_forward_axis_world must be nonzero")
    forward = [float(v) / norm * forward_mag for v in axis]
    actions.append(_execute_world_delta(group, config, name="simple_forward", delta_world_m=forward))
    return {
        "config": config_path,
        "mode": "simple_up_then_forward",
        "motion_commanded": True,
        "actions": actions,
        "initial_estimate": report,
        "pre_forward_estimate": post_up_report,
    }


def _normalized_forward(config: Any, magnitude_m: float) -> list[float]:
    axis = config.simple_forward_axis_world
    norm = sum(v * v for v in axis) ** 0.5
    if norm <= 0.0:
        raise RuntimeError("simple_forward_axis_world must be nonzero")
    return [float(v) / norm * float(magnitude_m) for v in axis]


def _live_continuous_simple_up_forward(config_path: str, confirm: str) -> dict[str, Any]:
    import moveit_commander
    import rospy

    if confirm != "CONTINUOUS_SIMPLE_UP_FORWARD":
        raise RuntimeError("continuous simple up/forward requires --confirm CONTINUOUS_SIMPLE_UP_FORWARD")
    config = load_visual_servo_touch_config(config_path)
    if not rospy.get_node_uri():
        rospy.init_node("piper_x_continuous_simple_up_forward", anonymous=True, disable_signals=True)
    moveit_commander.roscpp_initialize([])
    group = moveit_commander.MoveGroupCommander(config.planning_group)
    group.set_end_effector_link(config.end_effector_link)
    group.set_max_velocity_scaling_factor(0.04)
    group.set_max_acceleration_scaling_factor(0.04)

    actions: list[dict[str, Any]] = []
    estimates: list[dict[str, Any]] = []

    for index in range(config.max_alignment_iterations):
        report, estimate = _capture_estimate(config)
        estimates.append(report)
        blockers = _execution_blockers(config, estimate, allow_not_centered=True)
        if blockers:
            raise RuntimeError(f"continuous vertical alignment blocked: {blockers}")
        if estimate.pixel_error_uv is None:
            raise RuntimeError("marker pixel error unavailable")
        vertical_error = float(estimate.pixel_error_uv[1])
        if abs(vertical_error) <= config.image_center_tolerance_px:
            actions.append({"name": "vertical_alignment_complete", "iteration": index, "pixel_error_uv": list(estimate.pixel_error_uv)})
            break
        vertical_step = config.simple_up_step_m if vertical_error < 0.0 else -config.simple_up_step_m
        action = _execute_world_delta(group, config, name=f"vertical_align_step_{index}", delta_world_m=[0.0, 0.0, vertical_step])
        action["vertical_component_m"] = vertical_step
        action["pixel_error_uv"] = list(estimate.pixel_error_uv)
        actions.append(action)

    center_depth_report = _capture_center_depth(config)
    estimates.append({"forward_depth_precheck": center_depth_report})
    depth = float(center_depth_report["center_depth_m"] or 0.0)
    if depth > 0.0 and depth <= config.continuous_forward_stop_depth_m:
        actions.append({"name": "forward_stop_depth_already_reached", "depth_m": depth})
        total_forward = 0.0
    else:
        forward_distance = max(config.simple_forward_step_m, depth + 0.05)
        action = _execute_monitored_forward(group, config, distance_m=forward_distance)
        actions.append(action)
        total_forward = float(action["planned_forward_distance_m"])

    return {
        "config": config_path,
        "mode": "continuous_simple_up_forward",
        "motion_commanded": True,
        "actions": actions,
        "estimates": estimates,
        "total_forward_m": total_forward,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="piper-on-bunker/config/piper_x_visual_servo_aruco_touch.yaml")
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument(
        "--mode",
        choices=["check", "align_then_depth_touch", "simple_up_then_forward", "continuous_simple_up_forward"],
        default="check",
    )
    args = parser.parse_args(argv)

    if args.execute and args.mode == "check":
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
        if args.execute and args.mode == "simple_up_then_forward":
            report = _live_simple_up_then_forward(args.config, args.confirm)
        elif args.execute and args.mode == "continuous_simple_up_forward":
            report = _live_continuous_simple_up_forward(args.config, args.confirm)
        elif args.execute:
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
