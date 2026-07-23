from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, List, Optional


@dataclass
class StandardAction:
    translation_delta_m: List[Optional[float]]
    rotation_delta_rad: List[Optional[float]]
    gripper_command: Optional[float]
    model_name: str
    raw_action: Any
    action_frame: str = "unknown"
    confidence: str = "failed"
    execution_allowed: bool = False

    def to_dict(self) -> dict:
        data = asdict(self)
        data["execution_allowed"] = False
        return data


def empty_standard_action(model_name: str, raw_action: Any, confidence: str = "failed") -> StandardAction:
    return StandardAction(
        translation_delta_m=[None, None, None],
        rotation_delta_rad=[None, None, None],
        gripper_command=None,
        model_name=model_name,
        raw_action=raw_action,
        action_frame="unknown",
        confidence=confidence,
        execution_allowed=False,
    )
