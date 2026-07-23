from __future__ import annotations

import base64
import json
import math
import re
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import requests

from piper_on_bunker.policies.standard_action import StandardAction, empty_standard_action


SHADOW_BANNER = "SHADOW MODE - NO ROBOT ACTION WILL BE EXECUTED"
MODEL_NAME = "vlac-2b"
REMOTE_MODEL_ID = "InternRobotics/VLAC"
DEFAULT_ENDPOINT = "http://192.168.1.104:8016/action-preview"
ACTION_SEMANTICS = "unknown_songling_convention"
UNVERIFIED_ADAPTER_NOTE = "Raw VLAC action values are preserved without proving they are relative PiPER deltas."
UNVERIFIED_GRIPPER_NOTE = "VLAC gripper input units are unverified. The client sends the live PiPER gripper state unchanged unless the remote service overrides it with a placeholder."


@dataclass
class EndEffectorStateSI:
    x_m: float
    y_m: float
    z_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    gripper_m: float = 0.0

    def validate(self) -> None:
        values = asdict(self)
        bad = [name for name, value in values.items() if not math.isfinite(float(value))]
        if bad:
            raise ValueError("non-finite end-effector state fields: " + ", ".join(bad))

    def to_dict(self) -> dict:
        self.validate()
        return {name: float(value) for name, value in asdict(self).items()}


def quaternion_to_rpy_rad(qx: float, qy: float, qz: float, qw: float) -> List[float]:
    values = [float(qx), float(qy), float(qz), float(qw)]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion contains non-finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm == 0:
        raise ValueError("quaternion norm is zero")
    qx, qy, qz, qw = [value / norm for value in values]
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]


def si_to_model_state(state: EndEffectorStateSI, state_format: str = "si_json") -> Any:
    state.validate()
    if state_format == "si_json":
        return state.to_dict()
    if state_format == "legacy_xyz_0p001mm_rpy_0p001deg":
        return [
            state.x_m * 1_000_000.0,
            state.y_m * 1_000_000.0,
            state.z_m * 1_000_000.0,
            math.degrees(state.roll_rad) * 1000.0,
            math.degrees(state.pitch_rad) * 1000.0,
            math.degrees(state.yaw_rad) * 1000.0,
            state.gripper_m,
        ]
    raise ValueError(f"unknown VLAC state format: {state_format}")


def encode_image_file(path: str | Path, max_bytes: int = 8_000_000) -> str:
    data = Path(path).read_bytes()
    return encode_image_bytes(data, max_bytes=max_bytes)


def encode_image_bytes(data: bytes, max_bytes: int = 8_000_000) -> str:
    if not data:
        raise ValueError("image is empty")
    if len(data) > max_bytes:
        raise ValueError(f"image exceeds request limit: {len(data)} > {max_bytes}")
    mime = _detect_image_mime(data)
    if mime is None:
        raise ValueError("image is not a supported PNG or JPEG")
    return "data:%s;base64,%s" % (mime, base64.b64encode(data).decode("ascii"))


def validate_images(images: Iterable[str], max_images: int = 3, max_decoded_bytes: int = 8_000_000) -> List[str]:
    values = list(images)
    if not 1 <= len(values) <= max_images:
        raise ValueError("VLAC shadow request requires one to three images")
    for image in values:
        if not isinstance(image, str) or not image:
            raise ValueError("image entries must be non-empty strings")
        payload = image.split(",", 1)[1] if image.startswith("data:") and "," in image else image
        try:
            decoded = base64.b64decode(payload, validate=True)
        except Exception as exc:
            raise ValueError("image entry is not valid base64") from exc
        if len(decoded) > max_decoded_bytes:
            raise ValueError("decoded image exceeds request limit")
        if _detect_image_mime(decoded) is None:
            raise ValueError("decoded image is not a supported PNG or JPEG")
    return values


def _detect_image_mime(data: bytes) -> Optional[str]:
    if data.startswith(b"\x89PNG\r\n\x1a\n"):
        return "image/png"
    if data.startswith(b"\xff\xd8\xff"):
        return "image/jpeg"
    return None


def build_action_preview_request(
    images: Iterable[str],
    task_description: str,
    state_si: EndEffectorStateSI,
    history: Optional[list] = None,
    state_format: str = "si_json",
) -> dict:
    instruction = task_description.strip()
    if not instruction:
        raise ValueError("task_description must be non-empty")
    model_units = si_to_model_state(state_si, state_format)
    return {
        "images": validate_images(images),
        "task_description": instruction,
        "end_effector_state": state_si.to_dict(),
        "input_state_model_units": model_units,
        "input_state_model_units_note": UNVERIFIED_GRIPPER_NOTE,
        "state_format": state_format,
        "history": history or [],
        "execution_allowed": False,
    }


def parse_vlac_actions(raw_model_output: Any) -> List[StandardAction]:
    raw_text = raw_model_output if isinstance(raw_model_output, str) else json.dumps(raw_model_output, sort_keys=True)
    vlac_prompt_action = _parse_vlac_prompt_action(raw_text, raw_model_output)
    if vlac_prompt_action is not None:
        return [vlac_prompt_action]
    parsed = _extract_json(raw_model_output)
    if parsed is not None:
        if isinstance(parsed, list) and all(isinstance(value, (int, float)) for value in parsed) and len(parsed) >= 6:
            return [_numbers_to_action([float(value) for value in parsed], raw_model_output)]
        actions = parsed if isinstance(parsed, list) else [parsed]
        converted = [_dict_to_action(item, raw_model_output) for item in actions if isinstance(item, dict)]
        return [action for action in converted if action is not None]
    numbers = [float(value) for value in re.findall(r"[-+]?(?:\d+\.\d+|\d+)(?:[eE][-+]?\d+)?", raw_text)]
    if len(numbers) >= 6:
        return [_numbers_to_action(numbers, raw_model_output)]
    return []


def _parse_vlac_prompt_action(raw_text: str, raw_model_output: Any) -> Optional[StandardAction]:
    pattern = re.compile(
        r"x:\s*([-+0-9.eE]+)\s*mm,\s*y:\s*([-+0-9.eE]+)\s*mm,\s*z:\s*([-+0-9.eE]+)\s*mm,\s*"
        r"roll:\s*([-+0-9.eE]+)\s*degrees,\s*pitch:\s*([-+0-9.eE]+)\s*degrees,\s*yaw:\s*([-+0-9.eE]+)\s*degrees,\s*open:\s*([-+0-9.eE]+)"
    )
    match = pattern.search(raw_text)
    if match is None:
        return None
    values = [float(value) for value in match.groups()]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-finite VLAC prompt action value")
    return StandardAction(
        raw_translation_m=[values[0] / 1000.0, values[1] / 1000.0, values[2] / 1000.0],
        raw_rotation_rad=[math.radians(values[3]), math.radians(values[4]), math.radians(values[5])],
        gripper_command_raw=values[6],
        model_name=MODEL_NAME,
        raw_action=raw_model_output,
        action_semantics=ACTION_SEMANTICS,
        parse_reliability="exact_grammar_match",
        model_confidence=None,
        unverified_adapter_assumption=UNVERIFIED_ADAPTER_NOTE,
        execution_allowed=False,
    )


def _extract_json(raw: Any) -> Optional[Any]:
    if isinstance(raw, (dict, list)):
        return raw
    if not isinstance(raw, str):
        return None
    text = raw.strip()
    candidates = [text]
    for start, end in (("[", "]"), ("{", "}")):
        if start in text and end in text:
            candidates.append(text[text.find(start) : text.rfind(end) + 1])
    for candidate in candidates:
        try:
            return json.loads(candidate)
        except Exception:
            continue
    return None


def _dict_to_action(item: Dict[str, Any], raw_model_output: Any) -> Optional[StandardAction]:
    tx = _number(item, "raw_translation_m", "translation_delta_m", "dx_m", "dx", "delta_x", "x")
    ty = _number(item, "raw_translation_m", "translation_delta_m", "dy_m", "dy", "delta_y", "y", index=1)
    tz = _number(item, "raw_translation_m", "translation_delta_m", "dz_m", "dz", "delta_z", "z", index=2)
    rr = _number(item, "raw_rotation_rad", "rotation_delta_rad", "droll_rad", "droll", "roll", "rx")
    rp = _number(item, "raw_rotation_rad", "rotation_delta_rad", "dpitch_rad", "dpitch", "pitch", "ry", index=1)
    ry = _number(item, "raw_rotation_rad", "rotation_delta_rad", "dyaw_rad", "dyaw", "yaw", "rz", index=2)
    gripper = _number(item, "gripper_command_raw", "gripper", "gripper_command")
    if all(value is None for value in (tx, ty, tz, rr, rp, ry, gripper)):
        return None
    parse_reliability = str(item.get("parse_reliability") or item.get("parse_confidence") or "assumed_numeric_layout")
    return StandardAction(
        raw_translation_m=[tx, ty, tz],
        raw_rotation_rad=[rr, rp, ry],
        gripper_command_raw=gripper,
        model_name=MODEL_NAME,
        raw_action=raw_model_output,
        action_semantics=ACTION_SEMANTICS,
        parse_reliability=parse_reliability if parse_reliability in {"exact_grammar_match", "assumed_numeric_layout", "failed"} else "assumed_numeric_layout",
        model_confidence=None,
        unverified_adapter_assumption=UNVERIFIED_ADAPTER_NOTE,
        execution_allowed=False,
    )


def _number(item: Dict[str, Any], *names: str, index: int = 0) -> Optional[float]:
    for name in names:
        if name in item and item[name] is not None:
            raw = item[name]
            if isinstance(raw, list):
                if len(raw) <= index or raw[index] is None:
                    continue
                value = float(raw[index])
            else:
                value = float(raw)
            if not math.isfinite(value):
                raise ValueError(f"non-finite action field: {name}")
            return value
    return None


def _numbers_to_action(numbers: List[float], raw_model_output: Any) -> StandardAction:
    return StandardAction(
        raw_translation_m=[numbers[0], numbers[1], numbers[2]],
        raw_rotation_rad=[numbers[3], numbers[4], numbers[5]],
        gripper_command_raw=numbers[6] if len(numbers) > 6 else None,
        model_name=MODEL_NAME,
        raw_action=raw_model_output,
        action_semantics=ACTION_SEMANTICS,
        parse_reliability="assumed_numeric_layout",
        model_confidence=None,
        unverified_adapter_assumption=UNVERIFIED_ADAPTER_NOTE,
        execution_allowed=False,
    )


class VlacShadowPolicyClient:
    def __init__(self, endpoint: str = DEFAULT_ENDPOINT, timeout_s: float = 30.0) -> None:
        self.endpoint = endpoint
        self.timeout_s = timeout_s

    def preview_action(self, request_payload: dict) -> dict:
        request_payload = dict(request_payload)
        request_payload["execution_allowed"] = False
        started = time.monotonic()
        try:
            response = requests.post(self.endpoint, json=request_payload, timeout=self.timeout_s)
            latency_ms = (time.monotonic() - started) * 1000.0
            response.raise_for_status()
            payload = response.json()
        except requests.Timeout as exc:
            return self._failure("request timed out", repr(exc), started)
        except requests.RequestException as exc:
            return self._failure("service unavailable", repr(exc), started)
        except ValueError as exc:
            return self._failure("invalid JSON response", repr(exc), started)
        payload["execution_allowed"] = False
        payload.setdefault("mode", "shadow_only")
        payload.setdefault("model", REMOTE_MODEL_ID)
        payload.setdefault("latency_ms", latency_ms)
        payload.setdefault("model_confidence", None)
        raw = payload.get("raw_model_output", payload.get("raw_output", payload))
        parsed = parse_vlac_actions(raw)
        if "parsed_actions" not in payload or not payload["parsed_actions"]:
            payload["parsed_actions"] = [action.to_dict() for action in parsed]
        payload["standard_actions"] = [action.to_dict() for action in parsed] if parsed else [empty_standard_action(MODEL_NAME, raw).to_dict()]
        payload["parse_reliability"] = payload.get("parse_reliability") or (parsed[0].parse_reliability if parsed else "failed")
        payload.setdefault("warnings", [])
        return payload

    def _failure(self, message: str, detail: str, started: float) -> dict:
        return {
            "success": False,
            "model": REMOTE_MODEL_ID,
            "mode": "shadow_only",
            "execution_allowed": False,
            "message": message,
            "detail": detail,
            "parsed_actions": [],
            "standard_actions": [empty_standard_action(MODEL_NAME, detail).to_dict()],
            "parse_reliability": "failed",
            "model_confidence": None,
            "warnings": [message],
            "latency_ms": (time.monotonic() - started) * 1000.0,
        }
