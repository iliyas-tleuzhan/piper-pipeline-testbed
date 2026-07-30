"""Image transforms shared by OpenPI collection, conversion, and inference."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


OPENPI_PADDED_RGB_224_V1 = "openpi_resize_with_pad_rgb_224_v1"


@dataclass(frozen=True)
class ResizePadResult:
    image: np.ndarray
    scale: float
    pad_left: int
    pad_top: int
    resized_width: int
    resized_height: int


def require_rgb_uint8(image: np.ndarray) -> np.ndarray:
    arr = np.asarray(image)
    if arr.ndim != 3 or arr.shape[2] != 3:
        raise ValueError(f"expected RGB image with shape HxWx3, got {arr.shape}")
    if arr.dtype != np.uint8:
        raise ValueError(f"expected uint8 RGB image, got {arr.dtype}")
    return arr


def resize_with_pad_rgb(image: np.ndarray, size: int = 224, pad_value: int = 0) -> ResizePadResult:
    """Resize an RGB uint8 image without stretching, then zero-pad to ``size``.

    The function preserves channel order exactly. The caller is responsible for
    converting ROS/OpenCV BGR images to RGB before calling it.
    """

    arr = require_rgb_uint8(image)
    if size <= 0:
        raise ValueError("size must be positive")

    height, width = arr.shape[:2]
    scale = min(size / float(width), size / float(height))
    resized_width = max(1, int(round(width * scale)))
    resized_height = max(1, int(round(height * scale)))

    try:
        import cv2

        resized = cv2.resize(arr, (resized_width, resized_height), interpolation=cv2.INTER_AREA)
    except Exception:
        from PIL import Image

        resized = np.asarray(
            Image.fromarray(arr).resize((resized_width, resized_height), resample=Image.Resampling.BILINEAR)
        )

    output = np.full((size, size, 3), int(pad_value), dtype=np.uint8)
    pad_left = (size - resized_width) // 2
    pad_top = (size - resized_height) // 2
    output[pad_top : pad_top + resized_height, pad_left : pad_left + resized_width] = resized
    return ResizePadResult(
        image=output,
        scale=scale,
        pad_left=pad_left,
        pad_top=pad_top,
        resized_width=resized_width,
        resized_height=resized_height,
    )
