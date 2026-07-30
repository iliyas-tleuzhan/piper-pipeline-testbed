"""OpenPI transform scaffold for PiPER-X wrist-camera ArUco touch data.

This file is intentionally small and depends on official OpenPI extension
points at training time. It is not imported by runtime hardware code.
"""

from __future__ import annotations

import numpy as np


PIPER_X_ARUCO_REPACK_CONFIG = {
    "observation/wrist_image": "observation/wrist_image",
    "observation/exterior_image": "observation/exterior_image",
    "observation/state": "observation/state",
    "prompt": "prompt",
    "action": "action",
}


def piper_x_wrist_only_transform(row: dict) -> dict:
    """Map converted PiPER-X rows to the OpenPI observation/action contract."""

    wrist = np.asarray(row["observation/wrist_image"], dtype=np.uint8)
    exterior = np.asarray(row["observation/exterior_image"], dtype=np.uint8)
    state = np.asarray(row["observation/state"], dtype=np.float32)
    action = np.asarray(row["action"], dtype=np.float32)
    if wrist.shape != (224, 224, 3):
        raise ValueError(f"wrist image must be 224x224x3, got {wrist.shape}")
    if exterior.shape != (224, 224, 3):
        raise ValueError(f"exterior placeholder must be 224x224x3, got {exterior.shape}")
    if np.any(exterior):
        raise ValueError("PiPER-X exterior image must be the deterministic zero placeholder")
    if state.shape != (7,):
        raise ValueError(f"state must be 7D, got {state.shape}")
    if action.shape != (7,):
        raise ValueError(f"action must be 7D, got {action.shape}")
    return {
        "observation/exterior_image": exterior,
        "observation/wrist_image": wrist,
        "observation/state": state,
        "prompt": str(row["prompt"]),
        "action": action,
    }
