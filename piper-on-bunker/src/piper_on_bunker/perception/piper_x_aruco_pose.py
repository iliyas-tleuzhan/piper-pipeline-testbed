"""Read-only OpenCV ArUco pose support for PiPER-X wrist calibration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class PiperXArucoPoseConfig:
    dictionary: str = "DICT_4X4_50"
    marker_id: int = 6
    marker_size_m: float = 0.100
    camera_frame: str = "wrist_camera_color_optical_frame"
    marker_frame: str = "aruco_marker_frame"


@dataclass(frozen=True)
class PiperXArucoPoseResult:
    visible: bool
    reason: str | None
    center_uv: tuple[float, float] | None = None
    corners: list[list[float]] | None = None
    all_corners: list[list[list[float]]] | None = None
    pixel_area: float | None = None
    rvec: list[float] | None = None
    tvec: list[float] | None = None
    quaternion_xyzw: list[float] | None = None
    detected_marker_ids: list[int] | None = None


def camera_info_is_valid(camera_matrix: Any, width: int | None = None, height: int | None = None) -> bool:
    matrix = np.asarray(camera_matrix, dtype=float).reshape(-1)
    if matrix.size != 9:
        return False
    if not np.all(np.isfinite(matrix)):
        return False
    if matrix[0] <= 0.0 or matrix[4] <= 0.0:
        return False
    if width is not None and int(width) <= 0:
        return False
    if height is not None and int(height) <= 0:
        return False
    return True


def detect_piper_x_aruco_pose(
    image_rgb: np.ndarray,
    camera_matrix: Any,
    dist_coeffs: Any,
    config: PiperXArucoPoseConfig = PiperXArucoPoseConfig(),
) -> PiperXArucoPoseResult:
    arr = np.asarray(image_rgb)
    if arr.ndim != 3 or arr.shape[2] != 3:
        return PiperXArucoPoseResult(False, f"expected RGB image HxWx3, got {arr.shape}")
    if arr.dtype != np.uint8:
        return PiperXArucoPoseResult(False, f"expected uint8 image, got {arr.dtype}")
    if not camera_info_is_valid(camera_matrix):
        return PiperXArucoPoseResult(False, "invalid CameraInfo intrinsics")
    if config.marker_size_m <= 0.0:
        return PiperXArucoPoseResult(False, "marker size must be positive")

    try:
        import cv2
    except Exception as exc:
        return PiperXArucoPoseResult(False, f"opencv unavailable: {exc}")
    if not hasattr(cv2, "aruco"):
        return PiperXArucoPoseResult(False, "opencv aruco module unavailable")

    dictionary_id = getattr(cv2.aruco, config.dictionary, None)
    if dictionary_id is None:
        return PiperXArucoPoseResult(False, f"unknown aruco dictionary {config.dictionary}")

    corners, ids = detect_marker_corners(arr, config.dictionary)
    if ids is None or len(ids) == 0:
        return PiperXArucoPoseResult(False, "marker not visible")

    flat_ids = [int(v) for v in np.asarray(ids).reshape(-1)]
    all_corners = [np.asarray(corner, dtype=np.float64).reshape(4, 2).tolist() for corner in corners]
    if config.marker_id not in flat_ids:
        return PiperXArucoPoseResult(
            False,
            "configured marker id not detected",
            all_corners=all_corners,
            detected_marker_ids=flat_ids,
        )

    index = flat_ids.index(config.marker_id)
    marker_corners = np.asarray(corners[index], dtype=np.float64).reshape(4, 2)
    camera_matrix_arr = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    dist_coeffs_arr = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
    if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [marker_corners.astype(np.float32)],
            float(config.marker_size_m),
            camera_matrix_arr,
            dist_coeffs_arr,
        )
        rvec = np.asarray(rvecs[0][0], dtype=float)
        tvec = np.asarray(tvecs[0][0], dtype=float)
    else:
        half = float(config.marker_size_m) / 2.0
        object_points = np.asarray(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )
        ok, rvec_out, tvec_out = cv2.solvePnP(
            object_points,
            marker_corners.astype(np.float64),
            camera_matrix_arr,
            dist_coeffs_arr,
            flags=cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE") else cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return PiperXArucoPoseResult(False, "solvePnP failed", detected_marker_ids=flat_ids)
        rvec = np.asarray(rvec_out, dtype=float).reshape(3)
        tvec = np.asarray(tvec_out, dtype=float).reshape(3)
    quat = _rvec_to_quaternion_xyzw(rvec)
    center = marker_corners.mean(axis=0)
    return PiperXArucoPoseResult(
        visible=True,
        reason=None,
        center_uv=(float(center[0]), float(center[1])),
        corners=marker_corners.tolist(),
        all_corners=all_corners,
        pixel_area=float(abs(_polygon_area(marker_corners))),
        rvec=rvec.tolist(),
        tvec=tvec.tolist(),
        quaternion_xyzw=quat,
        detected_marker_ids=flat_ids,
    )


def detect_piper_x_aruco_markers_only(
    image_rgb: np.ndarray,
    config: PiperXArucoPoseConfig = PiperXArucoPoseConfig(),
    reason: str = "pose unavailable",
) -> PiperXArucoPoseResult:
    arr = np.asarray(image_rgb)
    if arr.ndim != 3 or arr.shape[2] != 3:
        return PiperXArucoPoseResult(False, f"expected RGB image HxWx3, got {arr.shape}")
    if arr.dtype != np.uint8:
        return PiperXArucoPoseResult(False, f"expected uint8 image, got {arr.dtype}")
    try:
        corners, ids = detect_marker_corners(arr, config.dictionary)
    except Exception as exc:
        return PiperXArucoPoseResult(False, str(exc))
    if ids is None or len(ids) == 0:
        return PiperXArucoPoseResult(False, "marker not visible")
    flat_ids = [int(v) for v in np.asarray(ids).reshape(-1)]
    all_corners = [np.asarray(corner, dtype=np.float64).reshape(4, 2).tolist() for corner in corners]
    if config.marker_id not in flat_ids:
        return PiperXArucoPoseResult(False, "configured marker id not detected", all_corners=all_corners, detected_marker_ids=flat_ids)
    index = flat_ids.index(config.marker_id)
    marker_corners = np.asarray(corners[index], dtype=np.float64).reshape(4, 2)
    center = marker_corners.mean(axis=0)
    return PiperXArucoPoseResult(
        False,
        reason,
        center_uv=(float(center[0]), float(center[1])),
        corners=marker_corners.tolist(),
        all_corners=all_corners,
        pixel_area=float(abs(_polygon_area(marker_corners))),
        detected_marker_ids=flat_ids,
    )


def detect_marker_corners(image_rgb: np.ndarray, dictionary_name: str) -> tuple[list[np.ndarray], np.ndarray | None]:
    import cv2

    arr = np.asarray(image_rgb)
    dictionary_id = getattr(cv2.aruco, dictionary_name, None)
    if dictionary_id is None:
        raise ValueError(f"unknown aruco dictionary {dictionary_name}")
    gray = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary)
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    return list(corners), ids


def render_debug_image_rgb(
    image_rgb: np.ndarray,
    result: PiperXArucoPoseResult,
    config: PiperXArucoPoseConfig,
    camera_matrix: Any | None = None,
    dist_coeffs: Any | None = None,
) -> np.ndarray:
    import cv2

    debug = np.asarray(image_rgb).copy()
    detected_ids = result.detected_marker_ids or []
    if result.all_corners and detected_ids:
        draw_corners = [np.asarray(corner, dtype=np.float32).reshape(1, 4, 2) for corner in result.all_corners]
        cv2.aruco.drawDetectedMarkers(debug, draw_corners, np.asarray(detected_ids, dtype=np.int32).reshape(-1, 1))

    if result.corners is not None:
        selected = np.asarray(result.corners, dtype=np.int32).reshape(4, 2)
        cv2.polylines(debug, [selected], isClosed=True, color=(255, 255, 0), thickness=4)
        if result.center_uv is not None:
            cv2.circle(debug, (int(result.center_uv[0]), int(result.center_uv[1])), 6, (255, 0, 0), -1)

    pose_available = result.visible and result.rvec is not None and result.tvec is not None
    if pose_available and camera_matrix is not None and camera_info_is_valid(camera_matrix):
        try:
            axis_length = float(config.marker_size_m) * 0.5
            cv2.drawFrameAxes(
                debug,
                np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3),
                np.asarray(dist_coeffs if dist_coeffs is not None else [0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64).reshape(-1),
                np.asarray(result.rvec, dtype=np.float64).reshape(3, 1),
                np.asarray(result.tvec, dtype=np.float64).reshape(3, 1),
                axis_length,
            )
        except Exception:
            pass

    if result.visible:
        status = f"{config.dictionary} id={config.marker_id} size={config.marker_size_m:.3f}m DETECTED"
    elif result.corners is not None:
        status = f"{config.dictionary} marker {config.marker_id} detected, pose unavailable"
    else:
        status = f"{config.dictionary} marker {config.marker_id} not detected"
    reason = "" if result.visible or not result.reason else f" ({result.reason})"
    cv2.rectangle(debug, (8, 8), (min(debug.shape[1] - 1, 620), 72), (0, 0, 0), -1)
    cv2.putText(debug, status, (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
    if reason:
        cv2.putText(debug, reason[:70], (16, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1, cv2.LINE_AA)
    return debug


def _rvec_to_quaternion_xyzw(rvec: np.ndarray) -> list[float]:
    import cv2

    rot, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rot[2, 1] - rot[1, 2]) * s
        y = (rot[0, 2] - rot[2, 0]) * s
        z = (rot[1, 0] - rot[0, 1]) * s
    else:
        idx = int(np.argmax(np.diag(rot)))
        if idx == 0:
            s = 2.0 * np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
            w = (rot[2, 1] - rot[1, 2]) / s
            x = 0.25 * s
            y = (rot[0, 1] + rot[1, 0]) / s
            z = (rot[0, 2] + rot[2, 0]) / s
        elif idx == 1:
            s = 2.0 * np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
            w = (rot[0, 2] - rot[2, 0]) / s
            x = (rot[0, 1] + rot[1, 0]) / s
            y = 0.25 * s
            z = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
            w = (rot[1, 0] - rot[0, 1]) / s
            x = (rot[0, 2] + rot[2, 0]) / s
            y = (rot[1, 2] + rot[2, 1]) / s
            z = 0.25 * s
    quat = np.asarray([x, y, z, w], dtype=float)
    quat /= np.linalg.norm(quat)
    return quat.tolist()


def _polygon_area(points: np.ndarray) -> float:
    x = points[:, 0]
    y = points[:, 1]
    return float(0.5 * np.sum(x * np.roll(y, -1) - y * np.roll(x, -1)))
