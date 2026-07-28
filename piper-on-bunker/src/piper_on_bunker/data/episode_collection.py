from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from time import time
from typing import Any
import json
import math
import uuid

import numpy as np

from piper_on_bunker.hardware.joint_state import DEFAULT_ARM_JOINT_NAMES
from piper_on_bunker.hardware.joint_state import map_joint_state


PIPER_COMMAND_CAN_IDS = {0x151, 0x155, 0x156, 0x157, 0x159}
RAW_UNITS_PER_DEGREE = 1000.0


@dataclass
class CommandSample:
    source: str
    stamp_s: float
    arm_action_rad: list[float]
    gripper_action_m: float
    raw: dict[str, Any]
    gripper_action_source: str


def raw_joint_to_rad(value: int | float) -> float:
    return math.radians(float(value) / RAW_UNITS_PER_DEGREE)


def piper_raw_joints_to_rad(joints_raw) -> list[float]:
    values = [int(value) for value in joints_raw]
    if len(values) != 6:
        raise ValueError("PiPER raw command must contain exactly six arm joints")
    return [raw_joint_to_rad(value) for value in values]


def convert_gripper_raw_to_m(
    raw_angle: int | float,
    *,
    raw_closed: float,
    raw_open: float,
    closed_m: float = 0.0,
    open_m: float = 0.06,
) -> float:
    if raw_open == raw_closed:
        raise ValueError("gripper raw open and closed calibration values must differ")
    fraction = (float(raw_angle) - float(raw_closed)) / (float(raw_open) - float(raw_closed))
    fraction = min(1.0, max(0.0, fraction))
    return float(closed_m) + fraction * (float(open_m) - float(closed_m))


class PiperCanCommandDecoder:
    def __init__(
        self,
        *,
        gripper_raw_closed: float | None = None,
        gripper_raw_open: float | None = None,
        gripper_closed_m: float = 0.0,
        gripper_open_m: float = 0.06,
    ) -> None:
        self.joints_raw: list[int | None] = [None] * 6
        self.gripper_raw: dict[str, int] | None = None
        self.gripper_raw_closed = gripper_raw_closed
        self.gripper_raw_open = gripper_raw_open
        self.gripper_closed_m = float(gripper_closed_m)
        self.gripper_open_m = float(gripper_open_m)

    def decode(self, arbitration_id: int, data: bytes, *, stamp_s: float, current_gripper_m: float) -> CommandSample | None:
        if arbitration_id not in PIPER_COMMAND_CAN_IDS:
            return None
        if arbitration_id == 0x155 and len(data) == 8:
            self.joints_raw[0] = _decode_i32_be(data[0:4])
            self.joints_raw[1] = _decode_i32_be(data[4:8])
            return None
        elif arbitration_id == 0x156 and len(data) == 8:
            self.joints_raw[2] = _decode_i32_be(data[0:4])
            self.joints_raw[3] = _decode_i32_be(data[4:8])
            return None
        elif arbitration_id == 0x157 and len(data) == 8:
            self.joints_raw[4] = _decode_i32_be(data[0:4])
            self.joints_raw[5] = _decode_i32_be(data[4:8])
        elif arbitration_id == 0x159 and len(data) == 8:
            self.gripper_raw = {
                "angle": _decode_i32_be(data[0:4]),
                "effort": int.from_bytes(data[4:6], byteorder="big", signed=False),
                "code": data[6],
            }
        else:
            return None

        if not all(value is not None for value in self.joints_raw):
            return None
        joints_raw = [int(value) for value in self.joints_raw]
        gripper_action = float(current_gripper_m)
        gripper_source = "current_feedback_hold"
        if self.gripper_raw is not None and self.gripper_raw_closed is not None and self.gripper_raw_open is not None:
            gripper_action = convert_gripper_raw_to_m(
                self.gripper_raw["angle"],
                raw_closed=self.gripper_raw_closed,
                raw_open=self.gripper_raw_open,
                closed_m=self.gripper_closed_m,
                open_m=self.gripper_open_m,
            )
            gripper_source = "can_raw_linear_calibration"
        return CommandSample(
            source="socketcan_piper_command",
            stamp_s=float(stamp_s),
            arm_action_rad=piper_raw_joints_to_rad(joints_raw),
            gripper_action_m=gripper_action,
            raw={"joints_raw": joints_raw, "gripper_raw": self.gripper_raw, "can_id": int(arbitration_id)},
            gripper_action_source=gripper_source,
        )


def _decode_i32_be(data: bytes | bytearray | memoryview) -> int:
    if len(data) != 4:
        raise ValueError("expected exactly four bytes")
    return int.from_bytes(bytes(data), byteorder="big", signed=True)


def make_episode_id(prefix: str = "piper_openpi") -> str:
    stamp = int(time())
    return f"{prefix}_{stamp}_{uuid.uuid4().hex[:8]}"


def resize_rgb(image: np.ndarray, size: int = 224) -> np.ndarray:
    try:
        import cv2
    except Exception as exc:
        raise RuntimeError("episode collection requires cv2") from exc
    rgb = np.asarray(image, dtype=np.uint8)
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f"expected RGB image [H,W,3], got {rgb.shape}")
    return cv2.resize(rgb, (size, size), interpolation=cv2.INTER_AREA)


def write_episode_metadata(path: Path, metadata: dict[str, Any]) -> None:
    path.mkdir(parents=True, exist_ok=True)
    (path / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")


def write_frame_record(path: Path, record: dict[str, Any]) -> None:
    with (path / "frames.jsonl").open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, sort_keys=True) + "\n")


def joint_state_to_state_vector(msg, *, expected_joint_names=DEFAULT_ARM_JOINT_NAMES) -> tuple[list[float], float, float]:
    stamp_s = float(msg.header.stamp.to_sec())
    mapped = map_joint_state(msg.name, msg.position, stamp_s, expected_joint_names)
    gripper = 0.0 if mapped.gripper_position is None else float(mapped.gripper_position)
    return [*mapped.arm_positions, gripper], float(mapped.stamp_s), float(mapped.age_s)


def ros_command_to_sample(msg, *, current_gripper_m: float) -> CommandSample:
    stamp_s = float(msg.header.stamp.to_sec()) if msg.header.stamp else time()
    names = list(msg.name)
    positions = [float(value) for value in msg.position]
    by_name = dict(zip(names, positions))
    missing = [name for name in DEFAULT_ARM_JOINT_NAMES if name not in by_name]
    if missing:
        if len(positions) >= 6:
            arm = positions[:6]
        else:
            raise ValueError("ROS command is missing PiPER arm joints: " + ", ".join(missing))
    else:
        arm = [by_name[name] for name in DEFAULT_ARM_JOINT_NAMES]
    gripper_source = "current_feedback_hold"
    gripper = float(current_gripper_m)
    for name in ("gripper", "gripper_joint", "joint7"):
        if name in by_name:
            gripper = float(by_name[name])
            gripper_source = f"ros_command_{name}"
            break
    return CommandSample(
        source="ros_joint_state_command",
        stamp_s=stamp_s,
        arm_action_rad=arm,
        gripper_action_m=gripper,
        raw={"names": names, "positions": positions},
        gripper_action_source=gripper_source,
    )
