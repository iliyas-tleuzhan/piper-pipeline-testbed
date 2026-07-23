from __future__ import annotations

import threading
import time
from datetime import datetime, timezone
from typing import Optional

from piper_on_bunker.models import Observation, Target
from piper_on_bunker.perception.marker_detector import MarkerDetector


class ExternalFixedCamera:
    def __init__(
        self,
        camera_name: str = "table_camera",
        color_topic: str = "/table_camera/color/image_raw",
        depth_topic: str = "/table_camera/aligned_depth_to_color/image_raw",
        camera_info_topic: str = "/table_camera/color/camera_info",
        timeout_s: float = 2.0,
        max_color_depth_delta_s: float = 0.08,
        marker_id: Optional[int] = None,
        aruco_dictionary: str = "DICT_4X4_50",
    ) -> None:
        try:
            import rospy
            from cv_bridge import CvBridge
            from sensor_msgs.msg import CameraInfo, Image
        except Exception as exc:
            raise RuntimeError("ExternalFixedCamera requires ROS, sensor_msgs, and cv_bridge") from exc
        self.rospy = rospy
        if not rospy.get_node_uri():
            rospy.init_node("piper_pipeline_camera", anonymous=True, disable_signals=True)
        self.camera_name = camera_name
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.timeout_s = timeout_s
        self.max_color_depth_delta_s = max_color_depth_delta_s
        self.bridge = CvBridge()
        self.detector = MarkerDetector(marker_id=marker_id, dictionary_name=aruco_dictionary)
        self.lock = threading.Lock()
        self.color_msg = None
        self.depth_msg = None
        self.info_msg = None
        self.color_image = None
        self.depth_image = None
        self.depth_encoding = None
        self.last_color_time = None
        self.last_depth_time = None
        self.rospy.Subscriber(color_topic, Image, self._color_cb, queue_size=1)
        self.rospy.Subscriber(depth_topic, Image, self._depth_cb, queue_size=1)
        self.rospy.Subscriber(camera_info_topic, CameraInfo, self._info_cb, queue_size=1)

    def _color_cb(self, msg) -> None:
        with self.lock:
            self.color_msg = msg
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.last_color_time = time.time()

    def _depth_cb(self, msg) -> None:
        with self.lock:
            self.depth_msg = msg
            self.depth_encoding = msg.encoding
            encoding = "passthrough"
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
            self.last_depth_time = time.time()

    def _info_cb(self, msg) -> None:
        with self.lock:
            self.info_msg = msg

    def capture_observation(self) -> Observation:
        deadline = time.time() + self.timeout_s
        while time.time() < deadline and not self.rospy.is_shutdown():
            with self.lock:
                ready = self.color_msg is not None and self.depth_msg is not None and self.info_msg is not None
                if ready:
                    color_age = time.time() - float(self.last_color_time)
                    depth_age = time.time() - float(self.last_depth_time)
                    color_stamp = float(self.color_msg.header.stamp.to_sec())
                    depth_stamp = float(self.depth_msg.header.stamp.to_sec())
                    stamp_delta = abs(color_stamp - depth_stamp)
                    if color_age <= self.timeout_s and depth_age <= self.timeout_s and stamp_delta <= self.max_color_depth_delta_s:
                        color_image = self.color_image.copy()
                        depth_image = self.depth_image.copy()
                        return Observation(
                            camera_name=self.camera_name,
                            frame_id=self.color_msg.header.frame_id,
                            timestamp=datetime.now(timezone.utc).isoformat(),
                            metadata={
                                "color_topic": self.color_topic,
                                "depth_topic": self.depth_topic,
                                "camera_info_topic": self.camera_info_topic,
                                "color_stamp": color_stamp,
                                "depth_stamp": depth_stamp,
                                "color_depth_delta_s": stamp_delta,
                                "color_age_s": color_age,
                                "depth_age_s": depth_age,
                                "depth_encoding": self.depth_encoding,
                                "camera_matrix": list(self.info_msg.K),
                                "width": int(self.info_msg.width),
                                "height": int(self.info_msg.height),
                                "_color_image": color_image,
                                "_depth_image": depth_image,
                            },
                        )
            time.sleep(0.02)
        raise RuntimeError("timed out waiting for synchronized table_camera color/depth/camera_info")

    def detect_target(self, observation: Observation, label: str) -> Optional[Target]:
        return self.detector.detect(observation, label)
