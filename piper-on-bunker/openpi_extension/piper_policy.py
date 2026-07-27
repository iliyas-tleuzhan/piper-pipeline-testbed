from __future__ import annotations

import dataclasses

import numpy as np

from openpi import transforms
from openpi.models import model as _model


PIPER_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper")


def _parse_image(image) -> np.ndarray:
    image = np.asarray(image)
    if np.issubdtype(image.dtype, np.floating):
        image = (255 * image).astype(np.uint8)
    if image.shape[0] == 3:
        image = np.moveaxis(image, 0, -1)
    return image


@dataclasses.dataclass(frozen=True)
class PiperInputs(transforms.DataTransformFn):
    model_type: _model.ModelType

    def __call__(self, data: dict) -> dict:
        exterior = _parse_image(data["observation/exterior_image"])
        wrist = _parse_image(data["observation/wrist_image"])
        state = np.asarray(data["observation/state"], dtype=np.float32)
        if state.shape != (7,):
            raise ValueError(f"PiPER state must be 7D, got {state.shape}")
        inputs = {
            "state": state,
            "image": {
                "base_0_rgb": exterior,
                "left_wrist_0_rgb": wrist,
                "right_wrist_0_rgb": np.zeros_like(exterior),
            },
            "image_mask": {
                "base_0_rgb": np.True_,
                "left_wrist_0_rgb": np.True_,
                "right_wrist_0_rgb": np.False_,
            },
        }
        if "actions" in data:
            actions = np.asarray(data["actions"], dtype=np.float32)
            if actions.shape[-1] != 7:
                raise ValueError(f"PiPER actions must have final dimension 7, got {actions.shape}")
            inputs["actions"] = actions
        if "prompt" in data:
            inputs["prompt"] = data["prompt"]
        return inputs


@dataclasses.dataclass(frozen=True)
class PiperOutputs(transforms.DataTransformFn):
    def __call__(self, data: dict) -> dict:
        return {
            "actions": np.asarray(data["actions"][..., :7]),
            "action_semantics": "absolute_piper_joint_targets",
            "joint_names": list(PIPER_JOINT_NAMES),
            "units": {"arm": "rad", "gripper": "total_jaw_opening_m"},
        }

