from __future__ import annotations

from dataclasses import dataclass
from time import monotonic
from typing import Any

import numpy as np


PIPER_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper")
PIPER_ACTION_SEMANTICS = "absolute_piper_joint_targets"
PIPER_STATE_KEYS = ("observation/exterior_image", "observation/wrist_image", "observation/state", "prompt")
OPENPI_COMMIT = "15a9616a00943ada6c20a0f158e3adb39df2ccac"
OPENPI_BASE_CHECKPOINT = "gs://openpi-assets/checkpoints/pi05_base"


@dataclass(frozen=True)
class OpenPIPiperMetadata:
    checkpoint: str
    piper_compatible: bool
    action_horizon: int
    action_dim: int
    joint_names: tuple[str, ...]
    control_frequency_hz: float
    action_semantics: str
    units: dict[str, str]
    normalization_metadata: dict[str, Any]

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "OpenPIPiperMetadata":
        return cls(
            checkpoint=str(payload.get("checkpoint", "")),
            piper_compatible=bool(payload.get("piper_compatible", False)),
            action_horizon=int(payload.get("action_horizon", 0)),
            action_dim=int(payload.get("action_dim", 0)),
            joint_names=tuple(payload.get("joint_names", ())),
            control_frequency_hz=float(payload.get("control_frequency_hz", 0.0)),
            action_semantics=str(payload.get("action_semantics", "")),
            units=dict(payload.get("units", {})),
            normalization_metadata=dict(payload.get("normalization_metadata", {})),
        )


@dataclass(frozen=True)
class OpenPIPiperResponse:
    actions: np.ndarray
    metadata: OpenPIPiperMetadata
    inference_duration_s: float
    received_monotonic_s: float
    raw: dict[str, Any]


def make_observation(exterior_image, wrist_image, state, prompt: str) -> dict[str, Any]:
    state_array = np.asarray(state, dtype=np.float32)
    if state_array.shape != (7,):
        raise ValueError(f"PiPER observation/state must have shape (7,), got {state_array.shape}")
    return {
        "observation/exterior_image": np.asarray(exterior_image, dtype=np.uint8),
        "observation/wrist_image": np.asarray(wrist_image, dtype=np.uint8),
        "observation/state": state_array,
        "prompt": str(prompt),
    }


def validate_openpi_response(
    payload: dict[str, Any],
    *,
    require_piper_compatible: bool,
    expected_frequency_hz: float,
    max_policy_response_age_s: float = 1.0,
    now_monotonic_s: float | None = None,
) -> OpenPIPiperResponse:
    now = monotonic() if now_monotonic_s is None else now_monotonic_s
    metadata_payload = dict(payload.get("metadata") or payload)
    metadata = OpenPIPiperMetadata.from_payload(metadata_payload)
    actions = np.asarray(payload.get("actions"), dtype=np.float64)
    if actions.ndim != 2:
        raise ValueError(f"OpenPI actions must be 2D [horizon, 7], got {actions.shape}")
    if metadata.action_horizon and actions.shape[0] != metadata.action_horizon:
        raise ValueError(f"action horizon mismatch: actions={actions.shape[0]} metadata={metadata.action_horizon}")
    if actions.shape[1] != 7 or metadata.action_dim != 7:
        raise ValueError(f"PiPER action dimension must be 7, got actions={actions.shape} metadata={metadata.action_dim}")
    if tuple(metadata.joint_names) != PIPER_JOINT_NAMES:
        raise ValueError(f"joint names differ: {metadata.joint_names} != {PIPER_JOINT_NAMES}")
    if metadata.action_semantics != PIPER_ACTION_SEMANTICS:
        raise ValueError(f"wrong action semantics: {metadata.action_semantics}")
    if abs(metadata.control_frequency_hz - expected_frequency_hz) > 1e-6:
        raise ValueError(
            f"control frequency mismatch: {metadata.control_frequency_hz} != {expected_frequency_hz}"
        )
    if not np.all(np.isfinite(actions)):
        raise ValueError("OpenPI actions contain non-finite values")
    if require_piper_compatible and not metadata.piper_compatible:
        raise ValueError("checkpoint is not marked PiPER-compatible; physical execution refused")
    if require_piper_compatible and not metadata.normalization_metadata:
        raise ValueError("PiPER-compatible checkpoint metadata is missing normalization metadata")
    response_age = float(payload.get("policy_response_age_s", 0.0))
    if response_age > max_policy_response_age_s:
        raise ValueError(f"stale policy response: {response_age:.3f}s > {max_policy_response_age_s:.3f}s")
    return OpenPIPiperResponse(
        actions=actions,
        metadata=metadata,
        inference_duration_s=float(payload.get("inference_duration_s", 0.0)),
        received_monotonic_s=now,
        raw=payload,
    )


class OpenPIPiperClient:
    """Thin client wrapper around the official openpi-client websocket protocol."""

    def __init__(self, host: str, port: int, *, client_factory=None) -> None:
        self.host = host
        self.port = int(port)
        self._client_factory = client_factory
        self._client = None

    def _connect(self):
        if self._client is None:
            if self._client_factory is not None:
                self._client = self._client_factory(self.host, self.port)
            else:
                from openpi_client import websocket_client_policy

                self._client = websocket_client_policy.WebsocketClientPolicy(host=self.host, port=self.port)
        return self._client

    def infer_phase(self, observation: dict[str, Any]) -> dict[str, Any]:
        missing = [key for key in PIPER_STATE_KEYS if key not in observation]
        if missing:
            raise ValueError(f"OpenPI PiPER observation missing keys: {missing}")
        return self._connect().infer(observation)


class ShadowOpenPIPiperClient:
    """Deterministic protocol stub for tests and no-hardware shadow runs."""

    def __init__(self, *, horizon: int = 10, frequency_hz: float = 20.0, piper_compatible: bool = False) -> None:
        self.horizon = int(horizon)
        self.frequency_hz = float(frequency_hz)
        self.piper_compatible = bool(piper_compatible)

    def infer_phase(self, observation: dict[str, Any]) -> dict[str, Any]:
        state = np.asarray(observation["observation/state"], dtype=np.float64)
        actions = np.repeat(state.reshape(1, 7), self.horizon, axis=0)
        return {
            "actions": actions,
            "action_horizon": self.horizon,
            "action_dim": 7,
            "action_semantics": PIPER_ACTION_SEMANTICS,
            "joint_names": list(PIPER_JOINT_NAMES),
            "control_frequency_hz": self.frequency_hz,
            "inference_duration_s": 0.0,
            "checkpoint": OPENPI_BASE_CHECKPOINT,
            "piper_compatible": self.piper_compatible,
            "units": {"arm": "rad", "gripper": "total_jaw_opening_m"},
            "normalization_metadata": {} if not self.piper_compatible else {"asset_id": "piper_single_arm_v1"},
        }

