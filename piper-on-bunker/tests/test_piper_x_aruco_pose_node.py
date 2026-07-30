import numpy as np
import pytest

from piper_on_bunker.perception.piper_x_aruco_pose import PiperXArucoPoseConfig
from piper_on_bunker.perception.piper_x_aruco_pose import camera_info_is_valid
from piper_on_bunker.perception.piper_x_aruco_pose import detect_piper_x_aruco_pose


cv2 = pytest.importorskip("cv2")
pytestmark = pytest.mark.skipif(not hasattr(cv2, "aruco"), reason="opencv aruco module unavailable")


def _marker_image(marker_id=6, dictionary_name="DICT_4X4_50"):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 160)
    else:
        marker = cv2.aruco.drawMarker(dictionary, marker_id, 160)
    image = np.full((300, 300, 3), 255, dtype=np.uint8)
    image[70:230, 70:230, :] = marker[:, :, None]
    return image


def _camera_matrix():
    return [260.0, 0.0, 150.0, 0.0, 260.0, 150.0, 0.0, 0.0, 1.0]


def test_detects_dict_4x4_50_marker_id_6_with_pose():
    result = detect_piper_x_aruco_pose(
        _marker_image(6),
        _camera_matrix(),
        [0.0, 0.0, 0.0, 0.0, 0.0],
        PiperXArucoPoseConfig(marker_id=6, marker_size_m=0.100),
    )
    assert result.visible is True
    assert result.tvec is not None
    assert result.quaternion_xyzw is not None
    assert result.detected_marker_ids == [6]


def test_wrong_marker_id_is_not_publishable():
    result = detect_piper_x_aruco_pose(
        _marker_image(6),
        _camera_matrix(),
        [0.0, 0.0, 0.0, 0.0, 0.0],
        PiperXArucoPoseConfig(marker_id=7, marker_size_m=0.100),
    )
    assert result.visible is False
    assert result.reason == "configured marker id not detected"
    assert result.tvec is None


def test_invalid_camera_info_is_rejected_before_pose():
    assert camera_info_is_valid([0.0] * 9, width=640, height=480) is False
    result = detect_piper_x_aruco_pose(
        _marker_image(6),
        [0.0] * 9,
        [0.0, 0.0, 0.0, 0.0, 0.0],
        PiperXArucoPoseConfig(marker_id=6, marker_size_m=0.100),
    )
    assert result.visible is False
    assert result.reason == "invalid CameraInfo intrinsics"


def test_marker_loss_returns_no_pose_to_prevent_stale_tf_republish():
    image = np.full((300, 300, 3), 255, dtype=np.uint8)
    result = detect_piper_x_aruco_pose(
        image,
        _camera_matrix(),
        [0.0, 0.0, 0.0, 0.0, 0.0],
        PiperXArucoPoseConfig(marker_id=6, marker_size_m=0.100),
    )
    assert result.visible is False
    assert result.tvec is None
    assert result.quaternion_xyzw is None
