from __future__ import annotations

import math
from datetime import datetime, timezone
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
    def __init__(self, source_frame: str, target_frame: str, translation_m, quaternion_xyzw, recorded_at: Optional[str] = None, max_age_s: Optional[float] = None) -> None:
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
        self.recorded_at = recorded_at
        self.max_age_s = max_age_s

    @property
    def age_s(self) -> Optional[float]:
        if not self.recorded_at:
            return None
        try:
            text = self.recorded_at.replace("Z", "+00:00")
            recorded = datetime.fromisoformat(text)
            if recorded.tzinfo is None:
                recorded = recorded.replace(tzinfo=timezone.utc)
            return (datetime.now(timezone.utc) - recorded).total_seconds()
        except Exception:
            return float("inf")

    def validate_fresh(self) -> None:
        age = self.age_s
        if age is not None and self.max_age_s is not None and age > float(self.max_age_s):
            raise ValueError(f"transform is stale: age_s={age:.3f} max_age_s={float(self.max_age_s):.3f}")

    def transform_pose(self, pose: Pose) -> Pose:
        self.validate_fresh()
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


class TransformResolver:
    def __init__(self, transforms: Optional[dict] = None, prefer_live_tf: bool = True) -> None:
        self.transforms = transforms or {}
        self.prefer_live_tf = prefer_live_tf

    def transform_pose(self, pose: Pose, target_frame: str) -> Pose:
        if self.prefer_live_tf:
            live = self._try_live_tf(pose, target_frame)
            if live is not None:
                return live
        static = self._static_for(pose.frame_id, target_frame)
        if static is None:
            raise ValueError(f"no transform from {pose.frame_id} to {target_frame}")
        return static.transform_pose(pose)

    def _static_for(self, source_frame: str, target_frame: str) -> Optional[StaticTransform]:
        for raw in self.transforms.values():
            if not isinstance(raw, dict):
                continue
            src = raw.get("source_frame")
            dst = raw.get("target_frame")
            if src == source_frame and dst == target_frame:
                return StaticTransform(
                    source_frame=src,
                    target_frame=dst,
                    translation_m=raw.get("translation_m"),
                    quaternion_xyzw=raw.get("quaternion_xyzw"),
                    recorded_at=raw.get("recorded_at"),
                    max_age_s=raw.get("max_age_s"),
                )
        return None

    def _try_live_tf(self, pose: Pose, target_frame: str) -> Optional[Pose]:
        try:
            import rospy
            import tf2_ros

            if not rospy.get_node_uri():
                return None
            buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(buffer)
            transform = buffer.lookup_transform(target_frame, pose.frame_id, rospy.Time(0), rospy.Duration(0.2))
            t = transform.transform.translation
            q = transform.transform.rotation
            static = StaticTransform(
                pose.frame_id,
                target_frame,
                [t.x, t.y, t.z],
                [q.x, q.y, q.z, q.w],
            )
            return static.transform_pose(pose)
        except Exception:
            return None
