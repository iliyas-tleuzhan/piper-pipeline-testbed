import numpy as np
import pytest

from piper_on_bunker.perception.aruco_touch_diagnostics import ArucoDiagnosticConfig
from piper_on_bunker.perception.aruco_touch_diagnostics import detect_aruco_touch_diagnostics
from piper_on_bunker.perception.aruco_touch_diagnostics import marker_visibility_summary


cv2 = pytest.importorskip("cv2")
pytestmark = pytest.mark.skipif(not hasattr(cv2, "aruco"), reason="opencv aruco module unavailable")


def _marker_image(marker_id=6, dictionary_name="DICT_4X4_50"):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 120)
    else:
        marker = cv2.aruco.drawMarker(dictionary, marker_id, 120)
    image = np.full((224, 224, 3), 255, dtype=np.uint8)
    image[52:172, 52:172, :] = marker[:, :, None]
    return image


def test_synthetic_aruco_detection():
    image = _marker_image(6)
    result = detect_aruco_touch_diagnostics(image, ArucoDiagnosticConfig(marker_id=6))
    assert result["aruco_visible"] is True
    assert abs(result["aruco_center_u"] - 111.5) < 2.0
    assert result["aruco_pixel_area"] > 10000


def test_wrong_marker_id_rejected():
    image = _marker_image(6)
    result = detect_aruco_touch_diagnostics(image, ArucoDiagnosticConfig(marker_id=7))
    assert result["aruco_visible"] is False
    assert result["aruco_failure_reason"] == "configured marker id not detected"


def test_marker_visibility_summary():
    summary = marker_visibility_summary(
        [
            {"aruco_visible": True, "detected_marker_ids": [6]},
            {"aruco_visible": False, "detected_marker_ids": [7]},
        ],
        configured_marker_id=6,
    )
    assert summary["visible_fraction"] == 0.5
    assert summary["marker_visible_at_start"] is True
    assert summary["wrong_marker_detections"] == [[7]]
