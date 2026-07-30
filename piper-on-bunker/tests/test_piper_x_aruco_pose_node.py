import numpy as np
import pytest
import importlib.util
from pathlib import Path

from piper_on_bunker.perception.piper_x_aruco_pose import PiperXArucoPoseConfig
from piper_on_bunker.perception.piper_x_aruco_pose import camera_info_is_valid
from piper_on_bunker.perception.piper_x_aruco_pose import detect_piper_x_aruco_pose
from piper_on_bunker.perception.piper_x_aruco_pose import render_debug_image_rgb


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


def test_debug_image_draws_detected_marker_border():
    image = _marker_image(6)
    result = detect_piper_x_aruco_pose(
        image,
        _camera_matrix(),
        [0.0, 0.0, 0.0, 0.0, 0.0],
        PiperXArucoPoseConfig(marker_id=6, marker_size_m=0.100),
    )
    debug = render_debug_image_rgb(image, result, PiperXArucoPoseConfig(marker_id=6, marker_size_m=0.100), _camera_matrix(), [0.0] * 5)
    assert debug.shape == image.shape
    assert np.count_nonzero(debug != image) > 0
    assert np.any(np.all(debug == [255, 255, 0], axis=2))


def test_debug_axes_drawing_path_runs_when_pose_exists():
    image = _marker_image(6)
    result = detect_piper_x_aruco_pose(
        image,
        _camera_matrix(),
        [0.0, 0.0, 0.0, 0.0, 0.0],
        PiperXArucoPoseConfig(marker_id=6, marker_size_m=0.100),
    )
    debug = render_debug_image_rgb(image, result, PiperXArucoPoseConfig(marker_id=6, marker_size_m=0.100), _camera_matrix(), [0.0] * 5)
    assert debug.dtype == np.uint8


def test_debug_image_draws_missing_marker_status():
    image = np.full((300, 300, 3), 255, dtype=np.uint8)
    result = detect_piper_x_aruco_pose(
        image,
        _camera_matrix(),
        [0.0, 0.0, 0.0, 0.0, 0.0],
        PiperXArucoPoseConfig(marker_id=6, marker_size_m=0.100),
    )
    debug = render_debug_image_rgb(image, result, PiperXArucoPoseConfig(marker_id=6, marker_size_m=0.100), _camera_matrix(), [0.0] * 5)
    assert np.count_nonzero(debug[:80, :, :] != image[:80, :, :]) > 0


def test_debug_header_timestamp_and_frame_are_preserved():
    path = Path("piper-on-bunker/scripts/piper_x_aruco_pose_node.py")
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    class Msg:
        pass

    source = Msg()
    debug = Msg()
    source.header = {"stamp": 123.4, "frame_id": "wrist_camera_color_optical_frame"}
    assert module.preserve_image_header(debug, source) is debug
    assert debug.header is source.header


def test_rgb_array_to_image_msg_preserves_header_and_shape():
    path = Path("piper-on-bunker/scripts/piper_x_aruco_pose_node.py")
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    class Msg:
        pass

    source = Msg()
    source.header = {"stamp": 123.4, "frame_id": "wrist_camera_color_optical_frame"}
    image = np.zeros((10, 20, 3), dtype=np.uint8)
    msg = module.rgb_array_to_image_msg(image, source, Msg)
    assert msg.header is source.header
    assert msg.height == 10
    assert msg.width == 20
    assert msg.encoding == "rgb8"
    assert msg.step == 60
    assert len(msg.data) == 600
