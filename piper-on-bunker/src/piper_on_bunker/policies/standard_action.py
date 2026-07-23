from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, List, Optional


@dataclass
class StandardAction:
    raw_translation_m: List[Optional[float]]
    raw_rotation_rad: List[Optional[float]]
    gripper_command_raw: Optional[float]
    model_name: str
    raw_action: Any
    action_semantics: str = "unknown_songling_convention"
    parse_reliability: str = "failed"
    model_confidence: Optional[float] = None
    unverified_adapter_assumption: Optional[str] = None
    execution_allowed: bool = False

    def to_dict(self) -> dict:
        data = asdict(self)
        data["execution_allowed"] = False
        return data


def empty_standard_action(model_name: str, raw_action: Any, parse_reliability: str = "failed") -> StandardAction:
    return StandardAction(
        raw_translation_m=[None, None, None],
        raw_rotation_rad=[None, None, None],
        gripper_command_raw=None,
        model_name=model_name,
        raw_action=raw_action,
        action_semantics="unknown_songling_convention",
        parse_reliability=parse_reliability,
        model_confidence=None,
        unverified_adapter_assumption=None,
        execution_allowed=False,
    )
