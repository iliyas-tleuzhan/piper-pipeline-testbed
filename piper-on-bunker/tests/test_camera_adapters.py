import pytest

from piper_on_bunker.perception.mock_camera import MockCamera


def test_mock_camera_detects_target():
    cam = MockCamera()
    obs = cam.capture_observation()
    target = cam.detect_target(obs, "marked_button")
    assert target and target.base_pose


def test_camera_unavailable():
    with pytest.raises(RuntimeError):
        MockCamera(unavailable=True).capture_observation()
