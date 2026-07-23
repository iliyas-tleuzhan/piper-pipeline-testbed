from __future__ import annotations

import math
from typing import Optional

from piper_on_bunker.models import Pose
from piper_on_bunker.safety import validate_quaternion


def require_base_pose(pose: Optional[Pose]) -> Pose:
    if pose is None:
        raise ValueError("target has no valid base-frame transform")
    return pose


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_rotate(q, v):
    q_conj = (-q[0], -q[1], -q[2], q[3])
    rotated = quaternion_multiply(quaternion_multiply(q, (v[0], v[1], v[2], 0.0)), q_conj)
    return rotated[:3]


class StaticTransform:
    def __init__(self, source_frame: str, target_frame: str, translation_m, quaternion_xyzw) -> None:
        if not source_frame or not target_frame:
            raise ValueError("source and target frames are required")
        if translation_m is None or quaternion_xyzw is None:
            raise ValueError("translation and quaternion must be configured")
        if len(translation_m) != 3 or len(quaternion_xyzw) != 4:
            raise ValueError("translation must have 3 values and quaternion must have 4 values")
        if not all(math.isfinite(float(value)) for value in list(translation_m) + list(quaternion_xyzw)):
            raise ValueError("transform contains non-finite values")
        q_pose = validate_quaternion(Pose(0.0, 0.0, 0.0, *[float(value) for value in quaternion_xyzw]))
        self.source_frame = source_frame
        self.target_frame = target_frame
        self.translation_m = tuple(float(value) for value in translation_m)
        self.quaternion_xyzw = (q_pose.qx, q_pose.qy, q_pose.qz, q_pose.qw)

    def transform_pose(self, pose: Pose) -> Pose:
        if pose.frame_id != self.source_frame:
            raise ValueError(f"expected source frame {self.source_frame}, got {pose.frame_id}")
        rx, ry, rz = quaternion_rotate(self.quaternion_xyzw, (pose.x, pose.y, pose.z))
        q = quaternion_multiply(self.quaternion_xyzw, (pose.qx, pose.qy, pose.qz, pose.qw))
        return validate_quaternion(
            Pose(
                x=rx + self.translation_m[0],
                y=ry + self.translation_m[1],
                z=rz + self.translation_m[2],
                qx=q[0],
                qy=q[1],
                qz=q[2],
                qw=q[3],
                frame_id=self.target_frame,
            )
        )
