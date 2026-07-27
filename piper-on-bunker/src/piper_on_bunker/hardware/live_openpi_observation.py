from __future__ import annotations

from dataclasses import dataclass
from time import time

import numpy as np

from piper_on_bunker.hardware.joint_state import DEFAULT_ARM_JOINT_NAMES
from piper_on_bunker.hardware.joint_state import map_joint_state


@dataclass(frozen=True)
class LiveOpenPIObservation:
    exterior_image: np.ndarray
    wrist_image: np.ndarray
    state: np.ndarray
    state_age_s: float
    exterior_image_age_s: float
    wrist_image_age_s: float | None
    source_topics: dict[str, str]

    @property
    def camera_age_s(self) -> float:
        ages = [self.exterior_image_age_s]
        if self.wrist_image_age_s is not None:
            ages.append(self.wrist_image_age_s)
        return max(ages)

    def to_summary(self) -> dict:
        return {
            "state": [float(value) for value in self.state],
            "state_age_s": float(self.state_age_s),
            "exterior_image_shape": list(self.exterior_image.shape),
            "exterior_image_age_s": float(self.exterior_image_age_s),
            "wrist_image_shape": list(self.wrist_image.shape),
            "wrist_image_age_s": None if self.wrist_image_age_s is None else float(self.wrist_image_age_s),
            "source_topics": dict(self.source_topics),
        }


def _stamp_to_seconds(stamp) -> float:
    try:
        return float(stamp.to_sec())
    except Exception:
        return 0.0


def _message_age_s(stamp) -> float:
    stamp_s = _stamp_to_seconds(stamp)
    if stamp_s <= 0.0:
        return float("inf")
    return max(0.0, time() - stamp_s)


def _image_msg_to_rgb_array(msg) -> np.ndarray:
    try:
        from cv_bridge import CvBridge
    except Exception as exc:
        raise RuntimeError("live OpenPI observation capture requires cv_bridge") from exc
    image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="rgb8")
    array = np.asarray(image, dtype=np.uint8)
    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError(f"expected RGB image with shape [H, W, 3], got {array.shape}")
    return array


def read_live_openpi_observation(
    *,
    timeout_s: float = 5.0,
    joint_topic: str = "/joint_states_single",
    exterior_image_topic: str = "/table_camera/color/image_raw",
    wrist_image_topic: str | None = "/cam_left_wrist",
    expected_joint_names=DEFAULT_ARM_JOINT_NAMES,
) -> LiveOpenPIObservation:
    try:
        import rospy
        from sensor_msgs.msg import Image
        from sensor_msgs.msg import JointState
    except Exception as exc:
        raise RuntimeError("live OpenPI observation capture requires rospy and sensor_msgs") from exc

    if not rospy.get_node_uri():
        rospy.init_node("openpi_piper_live_shadow", anonymous=True, disable_signals=True)

    joint_msg = rospy.wait_for_message(joint_topic, JointState, timeout=timeout_s)
    mapped = map_joint_state(
        joint_msg.name,
        joint_msg.position,
        _stamp_to_seconds(joint_msg.header.stamp),
        expected_joint_names,
    )
    gripper = 0.0 if mapped.gripper_position is None else float(mapped.gripper_position)
    state = np.asarray([*mapped.arm_positions, gripper], dtype=np.float32)

    exterior_msg = rospy.wait_for_message(exterior_image_topic, Image, timeout=timeout_s)
    exterior = _image_msg_to_rgb_array(exterior_msg)
    exterior_age = _message_age_s(exterior_msg.header.stamp)

    wrist = np.zeros_like(exterior)
    wrist_age = None
    if wrist_image_topic:
        try:
            wrist_msg = rospy.wait_for_message(wrist_image_topic, Image, timeout=timeout_s)
            wrist = _image_msg_to_rgb_array(wrist_msg)
            wrist_age = _message_age_s(wrist_msg.header.stamp)
        except Exception:
            wrist = np.zeros_like(exterior)
            wrist_age = None

    return LiveOpenPIObservation(
        exterior_image=exterior,
        wrist_image=wrist,
        state=state,
        state_age_s=float(mapped.age_s),
        exterior_image_age_s=float(exterior_age),
        wrist_image_age_s=wrist_age,
        source_topics={
            "joint_state": joint_topic,
            "exterior_image": exterior_image_topic,
            "wrist_image": wrist_image_topic or "",
        },
    )
