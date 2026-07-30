#!/usr/bin/env python3
"""Publish DICT_4X4_50 ArUco pose/TF for PiPER-X wrist hand-eye calibration."""

from __future__ import annotations

import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "piper-on-bunker" / "src"
if SRC_ROOT.exists():
    sys.path.insert(0, str(SRC_ROOT))

import numpy as np

from piper_on_bunker.perception.piper_x_aruco_pose import PiperXArucoPoseConfig
from piper_on_bunker.perception.piper_x_aruco_pose import camera_info_is_valid
from piper_on_bunker.perception.piper_x_aruco_pose import camera_geometry_from_info
from piper_on_bunker.perception.piper_x_aruco_pose import detect_piper_x_aruco_markers_only
from piper_on_bunker.perception.piper_x_aruco_pose import detect_piper_x_aruco_pose
from piper_on_bunker.perception.piper_x_aruco_pose import render_debug_image_rgb


def preserve_image_header(debug_msg, source_msg):
    debug_msg.header = source_msg.header
    return debug_msg


def rgb_array_to_image_msg(image_rgb, source_msg, image_cls):
    msg = image_cls()
    preserve_image_header(msg, source_msg)
    msg.height = int(image_rgb.shape[0])
    msg.width = int(image_rgb.shape[1])
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = int(image_rgb.shape[1]) * 3
    msg.data = image_rgb.tobytes()
    return msg


def main() -> int:
    if any(arg in {"-h", "--help"} for arg in sys.argv[1:]):
        print(
            "usage: piper_x_aruco_pose_node.py "
            "_image_topic:=/wrist_camera/color/image_rect_color "
            "_camera_info_topic:=/wrist_camera/color/camera_info "
            "_image_geometry_mode:=rectified "
            "_dictionary:=DICT_ARUCO_ORIGINAL _marker_id:=6 _marker_size_m:=0.100"
        )
        return 0

    import rospy
    import tf2_ros
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from sensor_msgs.msg import CameraInfo, Image

    rospy.init_node("piper_x_aruco_pose_node")
    image_topic = rospy.get_param("~image_topic", "/wrist_camera/color/image_rect_color")
    camera_info_topic = rospy.get_param("~camera_info_topic", "/wrist_camera/color/camera_info")
    pose_topic = rospy.get_param("~pose_topic", "/aruco_simple/pose")
    debug_image_topic = rospy.get_param("~debug_image_topic", "/aruco_simple/debug_image")
    dictionary = rospy.get_param("~dictionary", "DICT_ARUCO_ORIGINAL")
    marker_id = int(rospy.get_param("~marker_id", 6))
    marker_size_m = float(rospy.get_param("~marker_size_m", 0.100))
    camera_frame = rospy.get_param("~camera_frame", "wrist_camera_color_optical_frame")
    marker_frame = rospy.get_param("~marker_frame", "aruco_marker_frame")
    image_geometry_mode = rospy.get_param("~image_geometry_mode", "rectified")
    log_period_s = float(rospy.get_param("~log_period_s", 10.0))
    config = PiperXArucoPoseConfig(dictionary, marker_id, marker_size_m, camera_frame, marker_frame)
    try:
        camera_geometry_from_info(k=[1, 0, 0, 0, 1, 0, 0, 0, 1], d=[0, 0, 0, 0, 0], p=[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0], mode=image_geometry_mode, image_topic=image_topic)
    except ValueError as exc:
        rospy.logerr("Refusing ArUco node startup: %s", exc)
        return 2

    bridge = CvBridge()
    latest_info = {"matrix": None, "dist": None, "width": 0, "height": 0, "geometry_error": "no CameraInfo received"}
    last_log = 0.0
    last_visible = False
    pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=1)
    debug_pub = rospy.Publisher(debug_image_topic, Image, queue_size=1)
    tf_pub = tf2_ros.TransformBroadcaster()

    rospy.set_param("~dictionary", dictionary)
    rospy.set_param("~marker_id", marker_id)
    rospy.set_param("~marker_size_m", marker_size_m)
    rospy.set_param("~camera_frame", camera_frame)
    rospy.set_param("~marker_frame", marker_frame)
    rospy.set_param("~image_geometry_mode", image_geometry_mode)
    rospy.set_param("/aruco_simple/dictionary", dictionary)
    rospy.set_param("/aruco_simple/marker_id", marker_id)
    rospy.set_param("/aruco_simple/marker_size", marker_size_m)
    rospy.set_param("/aruco_simple/camera_frame", camera_frame)
    rospy.set_param("/aruco_simple/marker_frame", marker_frame)
    rospy.set_param("/aruco_simple/debug_image_topic", debug_image_topic)
    rospy.set_param("/aruco_simple/image_geometry_mode", image_geometry_mode)

    def on_info(msg: CameraInfo) -> None:
        latest_info["width"] = int(msg.width)
        latest_info["height"] = int(msg.height)
        try:
            matrix, dist = camera_geometry_from_info(k=msg.K, d=msg.D, p=msg.P, mode=image_geometry_mode, image_topic=image_topic)
            latest_info["matrix"] = matrix.tolist()
            latest_info["dist"] = dist.tolist()
            latest_info["geometry_error"] = None
        except ValueError as exc:
            latest_info["matrix"] = None
            latest_info["dist"] = None
            latest_info["geometry_error"] = str(exc)

    def maybe_log(message: str, visible: bool = False) -> None:
        nonlocal last_log, last_visible
        now = time.monotonic()
        if visible != last_visible or now - last_log >= log_period_s:
            (rospy.loginfo if visible else rospy.logwarn)(message)
            last_log = now
            last_visible = visible

    def on_image(msg: Image) -> None:
        matrix = latest_info["matrix"]
        try:
            import cv2

            if msg.encoding == "rgb8":
                image_rgb = np.asarray(bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"), dtype=np.uint8)
            else:
                image_rgb = cv2.cvtColor(bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"), cv2.COLOR_BGR2RGB)
        except Exception as exc:
            maybe_log(f"Refusing ArUco pose: image conversion failed: {exc}")
            return
        if camera_info_is_valid(matrix, latest_info["width"], latest_info["height"]):
            result = detect_piper_x_aruco_pose(image_rgb, matrix, latest_info["dist"], config)
        else:
            result = detect_piper_x_aruco_markers_only(image_rgb, config, reason=latest_info["geometry_error"] or "invalid or missing CameraInfo")

        debug_rgb = render_debug_image_rgb(image_rgb, result, config, matrix, latest_info["dist"])
        debug_pub.publish(rgb_array_to_image_msg(debug_rgb, msg, Image))
        if not result.visible:
            maybe_log(f"ArUco marker {marker_id} not published: {result.reason}")
            return

        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        frame_id = msg.header.frame_id or camera_frame
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(result.tvec[0])
        pose.pose.position.y = float(result.tvec[1])
        pose.pose.position.z = float(result.tvec[2])
        pose.pose.orientation.x = float(result.quaternion_xyzw[0])
        pose.pose.orientation.y = float(result.quaternion_xyzw[1])
        pose.pose.orientation.z = float(result.quaternion_xyzw[2])
        pose.pose.orientation.w = float(result.quaternion_xyzw[3])
        pose_pub.publish(pose)

        tf_msg = TransformStamped()
        tf_msg.header = pose.header
        tf_msg.child_frame_id = marker_frame
        tf_msg.transform.translation.x = pose.pose.position.x
        tf_msg.transform.translation.y = pose.pose.position.y
        tf_msg.transform.translation.z = pose.pose.position.z
        tf_msg.transform.rotation = pose.pose.orientation
        tf_pub.sendTransform(tf_msg)
        maybe_log(
            f"Published ArUco marker {marker_id} pose using {dictionary}, marker_size_m={marker_size_m:.3f}",
            visible=True,
        )

    rospy.Subscriber(camera_info_topic, CameraInfo, on_info, queue_size=1)
    rospy.Subscriber(image_topic, Image, on_image, queue_size=1)
    rospy.loginfo(
        "PiPER-X ArUco pose node ready: image=%s camera_info=%s geometry=%s pose=%s dictionary=%s marker_id=%d marker_size_m=%.3f",
        image_topic,
        camera_info_topic,
        image_geometry_mode,
        pose_topic,
        dictionary,
        marker_id,
        marker_size_m,
    )
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
