from __future__ import annotations

from typing import Dict

from .action_schema import (
    ACTION_DIM,
    ACTION_FEATURE_KEY,
    CAMERA_FEATURE_KEY,
    PIPER_ACTION_NAMES,
    PIPER_JOINT_NAMES,
    STATE_DIM,
    STATE_FEATURE_KEY,
)


def build_lerobot_feature_spec(image_height: int, image_width: int, use_videos: bool = False) -> Dict[str, dict]:
    image_dtype = "video" if use_videos else "image"
    return {
        CAMERA_FEATURE_KEY: {
            "dtype": image_dtype,
            "shape": (image_height, image_width, 3),
            "names": ["height", "width", "channels"],
        },
        STATE_FEATURE_KEY: {
            "dtype": "float32",
            "shape": (STATE_DIM,),
            "names": list(PIPER_JOINT_NAMES),
        },
        ACTION_FEATURE_KEY: {
            "dtype": "float32",
            "shape": (ACTION_DIM,),
            "names": list(PIPER_ACTION_NAMES),
        },
    }
