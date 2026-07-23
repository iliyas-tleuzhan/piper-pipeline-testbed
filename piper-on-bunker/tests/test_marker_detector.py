import pytest

from piper_on_bunker.models import Observation
from piper_on_bunker.perception.marker_detector import MarkerDetector


def test_invalid_depth_returns_no_target():
    np = pytest.importorskip("numpy")
    cv2 = pytest.importorskip("cv2")
    if not hasattr(cv2, "aruco"):
        pytest.skip("OpenCV aruco module unavailable")
    color = np.zeros((120, 120, 3), dtype=np.uint8)
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 0, 60)
    color[30:90, 30:90] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    depth = np.zeros((120, 120), dtype=np.uint16)
    obs = Observation(
        camera_name="test",
        frame_id="table_camera_color_optical_frame",
        timestamp="now",
        metadata={"_color_image": color, "_depth_image": depth, "camera_matrix": [100.0, 0, 60.0, 0, 100.0, 60.0, 0, 0, 1]},
    )
    assert MarkerDetector().detect(obs, "marked_button") is None
