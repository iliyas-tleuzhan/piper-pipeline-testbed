#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${CONTAINER:-abot-piper-noetic}"
MARKER_ID="${MARKER_ID:-6}"
MARKER_DICTIONARY="${MARKER_DICTIONARY:-DICT_ARUCO_ORIGINAL}"

docker exec -i "$CONTAINER" bash -lc '
source /opt/ros/noetic/setup.bash
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash
source /root/easy_handeye_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
python3 - <<PY
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

topic = "/wrist_camera/color/image_rect_color"
debug_topic = "/aruco_simple/debug_image"
dictionary_name = "'"$MARKER_DICTIONARY"'"
marker_id = int("'"$MARKER_ID"'")
rospy.init_node("check_piper_x_d435i_aruco_image", anonymous=True, disable_signals=True)
msg = rospy.wait_for_message(topic, Image, timeout=5)
bridge = CvBridge()
if msg.encoding == "rgb8":
    rgb = np.asarray(bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"), dtype=np.uint8)
else:
    rgb = cv2.cvtColor(bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"), cv2.COLOR_BGR2RGB)
gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
if hasattr(cv2.aruco, "ArucoDetector"):
    corners, ids, rejected = cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
else:
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary)
detected = [] if ids is None else [int(v) for v in np.asarray(ids).reshape(-1)]
print("topic:", topic)
print("encoding:", msg.encoding)
print("shape:", list(rgb.shape))
print("dictionary:", dictionary_name)
print("required_marker_id:", marker_id)
print("detected_marker_ids:", detected)
print("rejected_candidates:", len(rejected) if rejected is not None else 0)
print("marker_id_visible:", marker_id in detected)
try:
    debug_msg = rospy.wait_for_message(debug_topic, Image, timeout=5)
    print("debug_topic:", debug_topic)
    print("debug_topic_live:", True)
    print("debug_header_frame_id:", debug_msg.header.frame_id)
except Exception as exc:
    print("debug_topic:", debug_topic)
    print("debug_topic_live:", False)
    print("debug_error:", exc)
    raise SystemExit(3)
raise SystemExit(0 if marker_id in detected else 2)
PY
'
