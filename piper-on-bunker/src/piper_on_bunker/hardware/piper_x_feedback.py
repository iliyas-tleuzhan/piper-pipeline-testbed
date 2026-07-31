from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Any


PIPER_X_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
PIPER_X_FEEDBACK_SOURCE_ID = "piper_x_passive_socketcan_feedback_v1"
PIPER_X_JOINT_MAPPING_VERSION = "piper_x_lora_feedback_2a5_2a6_2a7_raw001deg_to_rad_v1"
PIPER_X_DECODER_REPO = "/home/dase-hw101/Iliyas/piper-lora-teleop-bridge"
PIPER_X_DECODER_COMMIT = "521c9c5fdfd9ee63bd96c0f9342fca6b2398092e"
PIPER_X_AGX_URDF_COMMIT = "f6642ce0d7872c686f29c99e9e10cd23d1d49313"
PIPER_X_CONFIGURED_ARM_MODEL = "agilex_piper_x"
PIPER_X_CONFIGURED_FIRMWARE_PROFILE = "unresolved_passive_feedback_only"
PIPER_X_FEEDBACK_CAN_IDS = (0x2A5, 0x2A6, 0x2A7)
RAW_UNITS_PER_DEGREE = 1000.0


@dataclass(frozen=True)
class PiperXFeedbackConfig:
    expected_joint_names: list[str] = field(default_factory=lambda: list(PIPER_X_JOINT_NAMES))
    source_id: str = PIPER_X_FEEDBACK_SOURCE_ID
    joint_mapping_version: str = PIPER_X_JOINT_MAPPING_VERSION
    dependency_repo: str = PIPER_X_DECODER_REPO
    dependency_commit: str = PIPER_X_DECODER_COMMIT
    arm_model: str = PIPER_X_CONFIGURED_ARM_MODEL
    firmware_profile: str = PIPER_X_CONFIGURED_FIRMWARE_PROFILE
    units: str = "rad"
    max_age_s: float = 0.5
    frame_set_window_s: float = 0.10
    reject_unproven_all_zero: bool = True


@dataclass(frozen=True)
class PiperXFeedbackEvidence:
    connected: bool
    feedback_valid: bool
    source_update_counter: int | None = None
    source_timestamp_s: float | None = None
    source_can_ids: list[int] = field(default_factory=list)
    raw_joint_values: list[int] = field(default_factory=list)
    adapter_type: str = "passive_socketcan"
    dependency_repo: str = PIPER_X_DECODER_REPO
    dependency_commit: str = PIPER_X_DECODER_COMMIT
    module_source_path: str = "piper_on_bunker.hardware.piper_x_feedback"
    firmware_read_from_hardware: bool = False
    tx_frames_sent_by_bridge: int = 0
    sdk_error: str | None = None
    real_feedback_packet: bool = False
    complete_frame_set: bool = False
    communication_ready: bool = False
    no_motion_commands_sent: bool = True


@dataclass(frozen=True)
class PiperXFeedbackSample:
    joint_names: list[str]
    positions_rad: list[float]
    stamp_s: float
    source_id: str
    joint_mapping_version: str
    dependency_repo: str
    dependency_commit: str
    arm_model: str
    firmware_profile: str
    units: str
    evidence: PiperXFeedbackEvidence

    def to_status_dict(self, now_s: float | None = None) -> dict[str, Any]:
        now = time.time() if now_s is None else float(now_s)
        return {
            "adapter_type": self.evidence.adapter_type,
            "connected": self.evidence.connected,
            "feedback_valid": self.evidence.feedback_valid,
            "feedback_age_s": max(0.0, now - float(self.stamp_s)),
            "source_update_counter": self.evidence.source_update_counter,
            "source_can_ids": [hex(v) for v in self.evidence.source_can_ids],
            "raw_joint_values": list(self.evidence.raw_joint_values),
            "joint_names": list(self.joint_names),
            "positions_rad": [float(v) for v in self.positions_rad],
            "source_id": self.source_id,
            "joint_mapping_version": self.joint_mapping_version,
            "dependency_repo": self.dependency_repo,
            "dependency_commit": self.dependency_commit,
            "module_source_path": self.evidence.module_source_path,
            "arm_model": self.arm_model,
            "firmware_profile": self.firmware_profile,
            "firmware_read_from_hardware": self.evidence.firmware_read_from_hardware,
            "units": self.units,
            "sdk_error": self.evidence.sdk_error,
            "real_feedback_packet": self.evidence.real_feedback_packet,
            "complete_frame_set": self.evidence.complete_frame_set,
            "communication_ready": self.evidence.communication_ready,
            "tx_frames_sent_by_bridge": self.evidence.tx_frames_sent_by_bridge,
            "no_motion_commands_sent": self.evidence.no_motion_commands_sent,
        }


@dataclass
class _JointFrame:
    first_index: int
    values_raw: tuple[int, int]
    timestamp_s: float
    receive_counter: int


class PassivePiperXSocketcanDecoder:
    """RX-only decoder for broadcast PiPER-X joint feedback frames.

    The conversion semantics are copied from the local working
    `piper-lora-teleop-bridge` decoder: feedback frames `0x2A5`, `0x2A6`,
    and `0x2A7` carry two signed big-endian int32 joint values each, in Piper
    raw units of `0.001 degrees`.
    """

    FRAME_TO_FIRST_INDEX = {0x2A5: 0, 0x2A6: 2, 0x2A7: 4}

    def __init__(self, *, config: PiperXFeedbackConfig | None = None) -> None:
        self.config = config or PiperXFeedbackConfig()
        self.frames: dict[int, _JointFrame] = {}
        self.receive_counter = 0
        self._last_complete_counter = 0

    @staticmethod
    def decode_i32_be(data: bytes | bytearray | memoryview) -> int:
        if len(data) != 4:
            raise ValueError("expected exactly 4 bytes for int32")
        return int.from_bytes(bytes(data), byteorder="big", signed=True)

    @staticmethod
    def raw_to_rad(value: int | float) -> float:
        return math.radians(float(value) / RAW_UNITS_PER_DEGREE)

    def update_from_can(self, arbitration_id: int, data: bytes, timestamp_s: float) -> bool:
        can_id = int(arbitration_id)
        if can_id not in self.FRAME_TO_FIRST_INDEX:
            return False
        if len(data) != 8:
            raise ValueError(f"feedback frame 0x{can_id:X} must contain 8 bytes")
        self.receive_counter += 1
        self.frames[can_id] = _JointFrame(
            first_index=self.FRAME_TO_FIRST_INDEX[can_id],
            values_raw=(self.decode_i32_be(data[:4]), self.decode_i32_be(data[4:8])),
            timestamp_s=float(timestamp_s),
            receive_counter=self.receive_counter,
        )
        return True

    def complete(self, now_s: float) -> bool:
        required = set(PIPER_X_FEEDBACK_CAN_IDS)
        if set(self.frames) != required:
            return False
        timestamps = [self.frames[can_id].timestamp_s for can_id in required]
        if max(timestamps) - min(timestamps) > self.config.frame_set_window_s:
            return False
        if any(float(now_s) - stamp > self.config.max_age_s for stamp in timestamps):
            return False
        return True

    def sample(self, now_s: float | None = None) -> PiperXFeedbackSample:
        now = time.time() if now_s is None else float(now_s)
        if not self.complete(now):
            raise ValueError("incomplete or stale PiPER-X feedback frame set")
        latest_counter = max(frame.receive_counter for frame in self.frames.values())
        if latest_counter <= self._last_complete_counter:
            raise ValueError("no new PiPER-X feedback packet since last sample")
        self._last_complete_counter = latest_counter
        raw = [0] * 6
        latest_timestamp = 0.0
        for can_id in PIPER_X_FEEDBACK_CAN_IDS:
            frame = self.frames[can_id]
            raw[frame.first_index] = frame.values_raw[0]
            raw[frame.first_index + 1] = frame.values_raw[1]
            latest_timestamp = max(latest_timestamp, frame.timestamp_s)
        positions = [self.raw_to_rad(value) for value in raw]
        return validate_piper_x_feedback(
            positions,
            PiperXFeedbackEvidence(
                connected=True,
                feedback_valid=True,
                source_update_counter=latest_counter,
                source_timestamp_s=latest_timestamp,
                source_can_ids=list(PIPER_X_FEEDBACK_CAN_IDS),
                raw_joint_values=raw,
                real_feedback_packet=True,
                complete_frame_set=True,
                communication_ready=True,
                tx_frames_sent_by_bridge=0,
                no_motion_commands_sent=True,
            ),
            config=self.config,
            now_s=now,
        )


def validate_piper_x_feedback(
    values_rad: Any,
    evidence: PiperXFeedbackEvidence,
    *,
    config: PiperXFeedbackConfig | None = None,
    now_s: float | None = None,
) -> PiperXFeedbackSample:
    cfg = config or PiperXFeedbackConfig()
    if evidence.sdk_error:
        raise ValueError(f"feedback decoder error: {evidence.sdk_error}")
    if evidence.adapter_type != "passive_socketcan":
        raise ValueError(f"unaudited PiPER-X feedback adapter: {evidence.adapter_type}")
    if evidence.tx_frames_sent_by_bridge != 0:
        raise ValueError("passive PiPER-X feedback bridge transmitted CAN frames")
    if not evidence.no_motion_commands_sent:
        raise ValueError("read-only feedback source reported motion command use")
    if not evidence.connected or not evidence.communication_ready:
        raise ValueError("PiPER-X feedback source is not communication-ready")
    if not evidence.feedback_valid:
        raise ValueError("PiPER-X feedback source did not report valid feedback")
    if not evidence.real_feedback_packet or not evidence.complete_frame_set:
        raise ValueError("PiPER-X feedback is missing a complete real packet set")
    if sorted(evidence.source_can_ids) != list(PIPER_X_FEEDBACK_CAN_IDS):
        raise ValueError(f"unexpected PiPER-X feedback CAN IDs: {evidence.source_can_ids}")
    if evidence.source_update_counter is None or evidence.source_timestamp_s is None:
        raise ValueError("PiPER-X feedback evidence requires a real receive counter and timestamp")

    values = [float(v) for v in values_rad]
    if len(values) != 6:
        raise ValueError(f"expected six PiPER-X joints, got {len(values)}")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("PiPER-X feedback contains non-finite joint values")
    if cfg.reject_unproven_all_zero and max(abs(value) for value in values) == 0.0:
        if not evidence.raw_joint_values or not evidence.complete_frame_set:
            raise ValueError("all-zero feedback is refused without a complete source frame set")

    stamp = float(evidence.source_timestamp_s)
    now = time.time() if now_s is None else float(now_s)
    if now - stamp > cfg.max_age_s:
        raise ValueError(f"stale PiPER-X feedback: age {now - stamp:.3f}s exceeds {cfg.max_age_s:.3f}s")
    return PiperXFeedbackSample(
        joint_names=list(cfg.expected_joint_names),
        positions_rad=values,
        stamp_s=stamp,
        source_id=cfg.source_id,
        joint_mapping_version=cfg.joint_mapping_version,
        dependency_repo=cfg.dependency_repo,
        dependency_commit=cfg.dependency_commit,
        arm_model=cfg.arm_model,
        firmware_profile=cfg.firmware_profile,
        units=cfg.units,
        evidence=evidence,
    )


def status_json(sample: PiperXFeedbackSample | None, error: str | None = None) -> str:
    if sample is None:
        payload: dict[str, Any] = {
            "adapter_type": "passive_socketcan",
            "connected": False,
            "feedback_valid": False,
            "feedback_age_s": None,
            "source_id": PIPER_X_FEEDBACK_SOURCE_ID,
            "joint_mapping_version": PIPER_X_JOINT_MAPPING_VERSION,
            "dependency_repo": PIPER_X_DECODER_REPO,
            "dependency_commit": PIPER_X_DECODER_COMMIT,
            "arm_model": PIPER_X_CONFIGURED_ARM_MODEL,
            "firmware_profile": PIPER_X_CONFIGURED_FIRMWARE_PROFILE,
            "firmware_read_from_hardware": False,
            "source_can_ids": [hex(v) for v in PIPER_X_FEEDBACK_CAN_IDS],
            "tx_frames_sent_by_bridge": 0,
            "no_motion_commands_sent": True,
            "error": error,
        }
    else:
        payload = sample.to_status_dict()
        payload["error"] = error
    return json.dumps(payload, sort_keys=True)


def validate_single_joint_state_authority(
    publishers: list[str],
    *,
    expected_substring: str = "piper_x_passive_socketcan_joint_state_bridge",
) -> None:
    if len(publishers) != 1:
        raise ValueError(f"expected exactly one /joint_states publisher, got {publishers}")
    if expected_substring not in publishers[0]:
        raise ValueError(f"/joint_states publisher is not the PiPER-X feedback bridge: {publishers[0]}")
