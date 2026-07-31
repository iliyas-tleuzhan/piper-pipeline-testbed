import numpy as np
import pytest

from piper_on_bunker.manipulation.piper_x_visual_servo_aruco_touch import VisualServoTouchConfig
from piper_on_bunker.manipulation.piper_x_visual_servo_aruco_touch import clamp_step
from piper_on_bunker.manipulation.piper_x_visual_servo_aruco_touch import deproject_pixel
from piper_on_bunker.manipulation.piper_x_visual_servo_aruco_touch import depth_roi_m
from piper_on_bunker.manipulation.piper_x_visual_servo_aruco_touch import estimate_depth_touch_step
from piper_on_bunker.manipulation.piper_x_visual_servo_aruco_touch import load_visual_servo_touch_config
from piper_on_bunker.manipulation.piper_x_visual_servo_aruco_touch import split_alignment_and_forward_steps
from piper_on_bunker.manipulation.piper_x_visual_servo_aruco_touch import transform_camera_point_to_gripper

cv2 = pytest.importorskip("cv2")
pytestmark = pytest.mark.skipif(not hasattr(cv2, "aruco"), reason="opencv aruco module unavailable")


def _marker_image(marker_id=6, dictionary_name="DICT_ARUCO_ORIGINAL", offset=(0, 0)):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 120)
    else:
        marker = cv2.aruco.drawMarker(dictionary, marker_id, 120)
    image = np.full((240, 320, 3), 255, dtype=np.uint8)
    y0 = 60 + int(offset[1])
    x0 = 100 + int(offset[0])
    image[y0 : y0 + 120, x0 : x0 + 120, :] = marker[:, :, None]
    return image


def _config(**overrides):
    data = {
        "profile_id": "test",
        "task_id": "test",
        "marker": {"dictionary": "DICT_ARUCO_ORIGINAL", "id": 6, "size_m": 0.100},
        "camera": {
            "color_image_topic": "/wrist_camera/color/image_rect_color",
            "depth_image_topic": "/wrist_camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/wrist_camera/color/camera_info",
            "image_geometry_mode": "rectified",
        },
        "frames": {"camera_frame": "wrist_camera_color_optical_frame", "gripper_frame": "gripper_base"},
        "moveit": {"planning_group": "arm", "end_effector_link": "gripper_base"},
        "handeye": {
            "source": "test",
            "verified": True,
            "translation_xyz_m": [0.01, 0.0, 0.02],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "gripper": {"tip_offset_source": "measured_test", "tip_offset_from_gripper_base_xyz_m": [0.0, 0.0, 0.05]},
        "depth": {"roi_px": 7, "min_depth_m": 0.03, "max_depth_m": 0.8},
        "alignment": {
            "image_center_tolerance_px": 24.0,
            "max_lateral_step_m": 0.01,
            "max_forward_step_m": 0.02,
            "contact_clearance_m": 0.003,
        },
        "safety": {
            "max_image_age_s": 0.5,
            "max_depth_age_s": 0.5,
            "max_joint_state_age_s": 0.5,
            "require_verified_handeye_for_execution": True,
            "physical_execution_enabled_by_default": True,
        },
    }
    for key, value in overrides.items():
        data[key] = value
    return VisualServoTouchConfig.from_mapping(data)


def _camera_matrix():
    return np.asarray([[260.0, 0.0, 160.0], [0.0, 260.0, 120.0], [0.0, 0.0, 1.0]], dtype=float)


def test_depth_roi_uses_median_and_converts_16uc1():
    depth = np.full((20, 20), 500, dtype=np.uint16)
    depth[10, 10] = 0
    assert depth_roi_m(depth, u=10, v=10, encoding="16UC1", roi_px=5) == pytest.approx(0.5)


def test_deproject_pixel_to_camera_point():
    assert deproject_pixel(_camera_matrix(), u=160, v=120, depth_m=0.4) == pytest.approx([0.0, 0.0, 0.4])
    assert deproject_pixel(_camera_matrix(), u=186, v=120, depth_m=0.4) == pytest.approx([0.04, 0.0, 0.4])


def test_transform_camera_point_to_gripper_applies_eye_in_hand_translation():
    point = transform_camera_point_to_gripper(
        [0.0, 0.0, 0.4],
        translation_gripper_camera_m=[0.01, 0.0, 0.02],
        quaternion_gripper_camera_xyzw=[0.0, 0.0, 0.0, 1.0],
    )
    assert point == pytest.approx([0.01, 0.0, 0.42])


def test_clamp_step_limits_lateral_and_forward_components():
    assert clamp_step([0.05, -0.03, 0.2], max_lateral_step_m=0.01, max_forward_step_m=0.02) == pytest.approx([0.01, -0.01, 0.02])


def test_estimator_reports_depth_and_blocks_when_marker_not_centered():
    depth = np.full((240, 320), 400, dtype=np.uint16)
    estimate = estimate_depth_touch_step(
        image_rgb=_marker_image(offset=(35, 0)),
        depth_image=depth,
        depth_encoding="16UC1",
        camera_matrix=_camera_matrix(),
        dist_coeffs=[0.0] * 5,
        config=_config(),
    )
    assert estimate.marker_visible is True
    assert estimate.depth_m == pytest.approx(0.4)
    assert estimate.image_aligned is False
    assert "marker is not centered in wrist image" in estimate.execution_blockers


def test_estimator_requires_verified_handeye_and_measured_tip_offset():
    cfg = _config(
        handeye={
            "source": "rejected",
            "verified": False,
            "translation_xyz_m": [0.0, 0.0, 0.0],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        gripper={"tip_offset_source": "UNMEASURED_LOCAL_VALUE_REQUIRED", "tip_offset_from_gripper_base_xyz_m": [0.0, 0.0, 0.0]},
    )
    estimate = estimate_depth_touch_step(
        image_rgb=_marker_image(),
        depth_image=np.full((240, 320), 400, dtype=np.uint16),
        depth_encoding="16UC1",
        camera_matrix=_camera_matrix(),
        dist_coeffs=[0.0] * 5,
        config=cfg,
    )
    assert estimate.execution_allowed is False
    assert "eye-in-hand calibration is not verified" in estimate.execution_blockers
    assert "gripper tip offset is not measured" in estimate.execution_blockers


def test_split_alignment_and_forward_steps_stops_lateral_before_depth_touch():
    estimate = estimate_depth_touch_step(
        image_rgb=_marker_image(offset=(35, 0)),
        depth_image=np.full((240, 320), 400, dtype=np.uint16),
        depth_encoding="16UC1",
        camera_matrix=_camera_matrix(),
        dist_coeffs=[0.0] * 5,
        config=_config(),
    )
    align, forward = split_alignment_and_forward_steps(estimate)
    assert align is not None
    assert forward is not None
    assert align[2] == pytest.approx(0.0)
    assert abs(align[0]) > 0.0 or abs(align[1]) > 0.0
    assert forward[0] == pytest.approx(0.0)
    assert forward[1] == pytest.approx(0.0)
    assert forward[2] == pytest.approx(0.02)


def test_committed_config_is_execution_blocked_by_default():
    cfg = load_visual_servo_touch_config("piper-on-bunker/config/piper_x_visual_servo_aruco_touch.yaml")
    assert cfg.physical_execution_enabled_by_default is False
    assert cfg.handeye_verified is False
    assert cfg.gripper_tip_offset_source.startswith("UNMEASURED")
