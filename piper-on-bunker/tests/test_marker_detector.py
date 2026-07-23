import pytest

from piper_on_bunker.models import Observation
from piper_on_bunker.perception.marker_detector import MarkerDetector


def make_marker_observation(marker_id=0, depth_value=500, depth_encoding="16UC1", depth_dtype=None, camera_matrix=None, depth_shape=(160, 160)):
    np = pytest.importorskip("numpy")
    cv2 = pytest.importorskip("cv2")
    if not hasattr(cv2, "aruco"):
        pytest.skip("OpenCV aruco module unavailable")
    dtype = depth_dtype or (np.uint16 if depth_encoding == "16UC1" else np.float32)
    color = np.full((160, 160, 3), 255, dtype=np.uint8)
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), marker_id, 80)
    color[40:120, 40:120] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    depth = np.full(depth_shape, depth_value, dtype=dtype)
    return Observation(
        camera_name="test",
        frame_id="table_camera_color_optical_frame",
        timestamp="now",
        metadata={
            "_color_image": color,
            "_depth_image": depth,
            "depth_encoding": depth_encoding,
            "camera_matrix": camera_matrix or [100.0, 0, 80.0, 0, 100.0, 80.0, 0, 0, 1],
        },
    )


def test_successful_aruco_detection_16uc1_deprojects_center():
    target = MarkerDetector(marker_id=0).detect(make_marker_observation(depth_value=500, depth_encoding="16UC1"), "marked_button")
    assert target is not None
    assert target.pixel == (80, 80)
    assert target.depth_m == pytest.approx(0.5)
    assert target.camera_pose.x == pytest.approx(0.0, abs=0.01)
    assert target.camera_pose.y == pytest.approx(0.0, abs=0.01)
    assert target.camera_pose.z == pytest.approx(0.5)


def test_successful_aruco_detection_32fc1_depth():
    target = MarkerDetector(marker_id=7).detect(make_marker_observation(marker_id=7, depth_value=0.42, depth_encoding="32FC1"), "marked_button")
    assert target is not None
    assert target.depth_m == pytest.approx(0.42)


def test_marker_id_not_found():
    assert MarkerDetector(marker_id=3).detect(make_marker_observation(marker_id=2), "marked_button") is None


def test_invalid_zero_depth_returns_no_target():
    assert MarkerDetector().detect(make_marker_observation(depth_value=0), "marked_button") is None


def test_nan_depth_returns_no_target():
    np = pytest.importorskip("numpy")
    assert MarkerDetector().detect(make_marker_observation(depth_value=np.nan, depth_encoding="32FC1"), "marked_button") is None


def test_out_of_range_depth_returns_no_target():
    assert MarkerDetector(max_depth_m=1.0).detect(make_marker_observation(depth_value=2500), "marked_button") is None


def test_mismatched_image_dimensions_returns_no_target():
    assert MarkerDetector().detect(make_marker_observation(depth_shape=(80, 80)), "marked_button") is None


def test_missing_intrinsics_returns_no_target():
    obs = make_marker_observation()
    obs.metadata.pop("camera_matrix")
    assert MarkerDetector().detect(obs, "marked_button") is None


def test_unsupported_depth_encoding_raises():
    with pytest.raises(RuntimeError):
        MarkerDetector().detect(make_marker_observation(depth_encoding="8UC1", depth_value=1), "marked_button")
