"""ArUco marker diagnostics for PiPER-X wrist-camera data collection."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np


@dataclass(frozen=True)
class ArucoDiagnosticConfig:
    dictionary: str = "DICT_4X4_50"
    marker_id: int = 6
    marker_size_m: float | None = None


@dataclass(frozen=True)
class CameraIntrinsics:
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    calibration_id: str


def _base_result(config: ArucoDiagnosticConfig, reason: str | None = None) -> dict[str, Any]:
    return {
        "aruco_visible": False,
        "aruco_dictionary": config.dictionary,
        "aruco_id": config.marker_id,
        "aruco_center_u": None,
        "aruco_center_v": None,
        "aruco_corners": None,
        "aruco_pixel_area": None,
        "aruco_rvec": None,
        "aruco_tvec": None,
        "aruco_pose_valid": False,
        "camera_calibration_id": None,
        "aruco_failure_reason": reason,
    }


def detect_aruco_touch_diagnostics(
    image_rgb: np.ndarray,
    config: ArucoDiagnosticConfig,
    intrinsics: CameraIntrinsics | None = None,
) -> dict[str, Any]:
    arr = np.asarray(image_rgb)
    if arr.ndim != 3 or arr.shape[2] != 3:
        return _base_result(config, f"expected RGB image HxWx3, got {arr.shape}")
    if arr.dtype != np.uint8:
        return _base_result(config, f"expected uint8 image, got {arr.dtype}")

    try:
        import cv2
    except Exception as exc:
        return _base_result(config, f"opencv unavailable: {exc}")
    if not hasattr(cv2, "aruco"):
        return _base_result(config, "opencv aruco module unavailable")

    dictionary_id = getattr(cv2.aruco, config.dictionary, None)
    if dictionary_id is None:
        return _base_result(config, f"unknown aruco dictionary {config.dictionary}")

    gray = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary)
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)

    if ids is None or len(ids) == 0:
        return _base_result(config, "marker not visible")
    flat_ids = [int(v) for v in np.asarray(ids).reshape(-1)]
    if config.marker_id not in flat_ids:
        result = _base_result(config, "configured marker id not detected")
        result["detected_marker_ids"] = flat_ids
        return result

    index = flat_ids.index(config.marker_id)
    marker_corners = np.asarray(corners[index], dtype=np.float64).reshape(4, 2)
    center = marker_corners.mean(axis=0)
    area = float(abs(_polygon_area(marker_corners)))
    result = _base_result(config, None)
    result.update(
        {
            "aruco_visible": True,
            "aruco_center_u": float(center[0]),
            "aruco_center_v": float(center[1]),
            "aruco_corners": marker_corners.tolist(),
            "aruco_pixel_area": area,
            "detected_marker_ids": flat_ids,
        }
    )

    if intrinsics is not None and config.marker_size_m is not None:
        try:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [marker_corners.astype(np.float32)],
                float(config.marker_size_m),
                np.asarray(intrinsics.camera_matrix, dtype=np.float64),
                np.asarray(intrinsics.dist_coeffs, dtype=np.float64),
            )
            result["aruco_rvec"] = np.asarray(rvecs[0][0], dtype=float).tolist()
            result["aruco_tvec"] = np.asarray(tvecs[0][0], dtype=float).tolist()
            result["aruco_pose_valid"] = True
            result["camera_calibration_id"] = intrinsics.calibration_id
        except Exception as exc:
            result["aruco_failure_reason"] = f"pose estimation failed: {exc}"
    return result


def marker_visibility_summary(frames: list[Mapping[str, Any]], configured_marker_id: int) -> dict[str, Any]:
    total = len(frames)
    visible = [frame for frame in frames if frame.get("aruco_visible") is True]
    wrong_ids = [
        frame.get("detected_marker_ids")
        for frame in frames
        if frame.get("detected_marker_ids") and configured_marker_id not in frame.get("detected_marker_ids", [])
    ]
    return {
        "total_frames": total,
        "visible_frames": len(visible),
        "visible_fraction": (len(visible) / total) if total else 0.0,
        "marker_visible_at_start": bool(frames and frames[0].get("aruco_visible") is True),
        "wrong_marker_detections": wrong_ids,
    }


def _polygon_area(points: np.ndarray) -> float:
    x = points[:, 0]
    y = points[:, 1]
    return float(0.5 * np.sum(x * np.roll(y, -1) - y * np.roll(x, -1)))
