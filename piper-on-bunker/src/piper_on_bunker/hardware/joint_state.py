from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional


DEFAULT_ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


@dataclass
class MappedJointState:
    names: List[str]
    arm_joint_names: List[str]
    arm_positions: List[float]
    gripper_position: Optional[float]
    stamp_s: float
    age_s: float


def map_joint_state(names: Iterable[str], positions: Iterable[float], stamp_s: float, expected_joint_names=None) -> MappedJointState:
    names = list(names)
    positions = [float(value) for value in positions]
    expected = list(expected_joint_names or DEFAULT_ARM_JOINT_NAMES)
    if len(names) != len(positions):
        raise ValueError("joint name and position arrays have different lengths")
    if len(set(names)) != len(names):
        raise ValueError("joint state contains duplicate joint names")
    by_name = dict(zip(names, positions))
    missing = [name for name in expected if name not in by_name]
    if missing:
        raise ValueError("joint state is missing expected arm joints: " + ", ".join(missing))
    arm_positions = [float(by_name[name]) for name in expected]
    if not all(math.isfinite(value) for value in arm_positions):
        raise ValueError("joint state contains non-finite arm joint values")
    gripper = None
    for gripper_name in ("gripper", "gripper_joint", "joint7"):
        if gripper_name in by_name:
            gripper = float(by_name[gripper_name])
            if not math.isfinite(gripper):
                raise ValueError("joint state contains non-finite gripper value")
            break
    return MappedJointState(
        names=names,
        arm_joint_names=expected,
        arm_positions=arm_positions,
        gripper_position=gripper,
        stamp_s=float(stamp_s),
        age_s=max(0.0, time.time() - float(stamp_s)) if stamp_s else float("inf"),
    )


def within_joint_tolerance(commanded, measured, tolerance_rad: float) -> bool:
    if len(commanded) != len(measured):
        return False
    return all(abs(float(a) - float(b)) <= tolerance_rad for a, b in zip(commanded, measured))
