from __future__ import annotations

import numpy as np

from piper_on_bunker.policies.lap_policy import (
    LapRuntimeConfig,
    LapStateSnapshot,
    action_to_target_pose,
    build_lap_request,
    clamp_translation,
    euler_to_rot6d,
    normalize_gripper_width,
    quaternion_to_rpy_rad,
    resize_with_pad_rgb,
)


def _config() -> LapRuntimeConfig:
    return LapRuntimeConfig(
        host="127.0.0.1",
        port=8016,
        color_image_topic="/color",
        joint_state_topic="/joint",
        end_pose_topic="/pose",
        axis_map=[0, 1, 2],
        translation_scale=1.0,
        max_translation_per_action_m=0.02,
        preserve_orientation=True,
        max_speed_scaling=0.05,
        max_acceleration_scaling=0.05,
        workspace_bounds_m={"x": [0.05, 0.60], "y": [-0.35, 0.35], "z": [0.02, 0.55]},
    )


def _snapshot() -> LapStateSnapshot:
    return LapStateSnapshot(
        image_rgb=np.zeros((10, 20, 3), dtype=np.uint8),
        image_stamp_s=1.0,
        pose_position_m=[0.2, 0.0, 0.2],
        pose_quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
        pose_rpy_rad=[0.0, 0.0, 0.0],
        joint_positions_rad=[0.0] * 6,
        gripper_m=0.03,
        joint_stamp_s=1.0,
        end_pose_stamp_s=1.0,
    )


def test_quaternion_to_rpy_identity():
    assert quaternion_to_rpy_rad(0.0, 0.0, 0.0, 1.0) == [0.0, 0.0, 0.0]


def test_euler_to_rot6d_size():
    rot6d = euler_to_rot6d([0.0, 0.0, 0.0])
    assert len(rot6d) == 6


def test_normalize_gripper_width():
    assert normalize_gripper_width(0.03) == 0.5


def test_resize_with_pad_rgb():
    image = np.zeros((10, 20, 3), dtype=np.uint8)
    resized = resize_with_pad_rgb(image, size=224)
    assert resized.shape == (224, 224, 3)


def test_build_lap_request_shape():
    request = build_lap_request(_snapshot(), "Move toward the red cup.")
    obs = request["observation"]
    assert np.asarray(obs["base_0_rgb"]).shape == (224, 224, 3)
    assert len(obs["cartesian_position"]) == 9
    assert len(obs["joint_position"]) == 6
    assert len(obs["gripper_position"]) == 1
    assert len(obs["state"]) == 10


def test_clamp_translation():
    delta = clamp_translation(np.array([0.04, 0.0, 0.0], dtype=float), 0.02)
    assert np.isclose(np.linalg.norm(delta), 0.02)


def test_action_to_target_pose_clamps_workspace():
    converted = action_to_target_pose(
        _snapshot(),
        {"actions": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]},
        _config(),
        max_actions=1,
    )
    target = converted["target_pose"]["position"]
    assert target[0] <= 0.60
