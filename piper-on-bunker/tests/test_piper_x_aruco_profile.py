import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from piper_on_bunker.data.image_transforms import resize_with_pad_rgb
from piper_on_bunker.policies.openpi_piper_policy import PIPER_JOINT_NAMES
from piper_on_bunker.policies.robot_checkpoint_metadata import ROBOT_CHECKPOINT_SCHEMA_V2
from piper_on_bunker.policies.robot_checkpoint_metadata import validate_robot_profile_checkpoint
from piper_on_bunker.profiles.piper_x_aruco import (
    PIPER_X_ACTION_SEMANTICS,
    PIPER_X_JOINT_ORDER,
    PIPER_X_PROFILE_ID,
    load_piper_x_profile,
    make_wrist_only_openpi_observation,
    validate_fixed_gripper_channel,
)


PROFILE = Path("piper-on-bunker/config/openpi_piper_x_touch_aruco.yaml")


def _profile():
    return load_piper_x_profile(PROFILE)


def test_piper_x_profile_is_separate_from_old_piper():
    profile = _profile()
    assert profile.robot_profile_id == PIPER_X_PROFILE_ID
    assert tuple(PIPER_JOINT_NAMES) == PIPER_X_JOINT_ORDER
    assert profile.action_semantics == "absolute_piper_x_joint_targets"
    assert profile.raw["joint_limits"]["status"] == "unresolved"
    assert profile.raw["physical_execution"]["allowed"] is False


def test_wrist_only_observation_zeroes_exterior_image():
    wrist = np.zeros((20, 40, 3), dtype=np.uint8)
    wrist[:, :, 0] = 255
    obs, meta = make_wrist_only_openpi_observation(wrist_image_rgb=wrist, state=np.arange(7), prompt="touch")
    assert obs["observation/wrist_image"].shape == (224, 224, 3)
    assert obs["observation/exterior_image"].shape == (224, 224, 3)
    assert np.all(obs["observation/exterior_image"] == 0)
    assert meta["camera_schema"]["observation/exterior_image"] == "zero_image_placeholder"
    assert obs["observation/state"].shape == (7,)


def test_padded_resize_preserves_rgb_and_aspect():
    img = np.zeros((10, 20, 3), dtype=np.uint8)
    img[:, :, 0] = 17
    img[:, :, 1] = 29
    result = resize_with_pad_rgb(img, size=100)
    assert result.image.shape == (100, 100, 3)
    assert result.resized_width == 100
    assert result.resized_height == 50
    content = result.image[result.pad_top : result.pad_top + result.resized_height]
    assert int(content[:, :, 0].max()) == 17
    assert int(content[:, :, 1].max()) == 29
    assert int(content[:, :, 2].max()) == 0


def test_fixed_gripper_channel_validation():
    actions = np.zeros((4, 7))
    actions[:, 6] = 0.123
    validate_fixed_gripper_channel(actions, fixed_target=0.123, tolerance=1e-6)
    actions[2, 6] = 0.2
    with pytest.raises(ValueError, match="fixed gripper"):
        validate_fixed_gripper_channel(actions, fixed_target=0.123, tolerance=1e-6)


def test_legacy_piper_metadata_cannot_authorize_piper_x():
    profile = _profile()
    metadata = {"checkpoint": "dev://old", "piper_compatible": True}
    result = validate_robot_profile_checkpoint(
        metadata,
        expected_robot_profile_id=profile.robot_profile_id,
        expected_robot_model=profile.robot_model,
        expected_task_id=profile.task_id,
        expected_joint_order=profile.joint_order,
        expected_action_semantics=profile.action_semantics,
        expected_state_units=profile.raw["state_units"],
        expected_action_units=profile.raw["action_units"],
        expected_control_frequency_hz=profile.control_frequency_hz,
        expected_camera_schema=profile.raw["camera"]["schema"],
        expected_camera_mount_id=profile.camera_mount_id,
        expected_image_preprocessing_id=profile.preprocessing_id,
        expected_gripper_mode=profile.gripper_mode,
        expected_normalization_asset_id=profile.raw["normalization_asset_id"],
        expected_openpi_commit=profile.raw["openpi_commit"],
    )
    assert result.eligible is False
    assert "legacy PiPER" in result.failures[0]


def test_robot_profile_metadata_requires_exact_match():
    profile = _profile()
    metadata = {
        "schema_version": ROBOT_CHECKPOINT_SCHEMA_V2,
        "checkpoint": "dev://trained",
        "robot_profile_id": profile.robot_profile_id,
        "robot_model": profile.robot_model,
        "task_id": profile.task_id,
        "action_semantics": profile.action_semantics,
        "action_dim": 7,
        "joint_order": list(profile.joint_order),
        "state_units": dict(profile.raw["state_units"]),
        "action_units": dict(profile.raw["action_units"]),
        "control_frequency_hz": profile.control_frequency_hz,
        "camera_schema": dict(profile.raw["camera"]["schema"]),
        "camera_mount_id": profile.camera_mount_id,
        "image_preprocessing_id": profile.preprocessing_id,
        "gripper_mode": profile.gripper_mode,
        "dataset_id": "dataset",
        "dataset_hash": "hash",
        "dataset_manifest_hash": "manifest",
        "normalization_asset_id": profile.raw["normalization_asset_id"],
        "normalization": {"hash": "norm"},
        "openpi_commit": profile.raw["openpi_commit"],
        "offline_validation": {"passed": True},
        "hardware_verification": {"passed": True},
        "piper_x_compatible": True,
        "physical_execution_allowed": True,
    }
    result = validate_robot_profile_checkpoint(
        metadata,
        expected_robot_profile_id=profile.robot_profile_id,
        expected_robot_model=profile.robot_model,
        expected_task_id=profile.task_id,
        expected_joint_order=profile.joint_order,
        expected_action_semantics=profile.action_semantics,
        expected_state_units=profile.raw["state_units"],
        expected_action_units=profile.raw["action_units"],
        expected_control_frequency_hz=profile.control_frequency_hz,
        expected_camera_schema=profile.raw["camera"]["schema"],
        expected_camera_mount_id=profile.camera_mount_id,
        expected_image_preprocessing_id=profile.preprocessing_id,
        expected_gripper_mode=profile.gripper_mode,
        expected_normalization_asset_id=profile.raw["normalization_asset_id"],
        expected_openpi_commit=profile.raw["openpi_commit"],
    )
    assert result.eligible is True
    metadata["camera_mount_id"] = "wrong"
    result = validate_robot_profile_checkpoint(
        metadata,
        expected_robot_profile_id=profile.robot_profile_id,
        expected_robot_model=profile.robot_model,
        expected_task_id=profile.task_id,
        expected_joint_order=profile.joint_order,
        expected_action_semantics=profile.action_semantics,
        expected_state_units=profile.raw["state_units"],
        expected_action_units=profile.raw["action_units"],
        expected_control_frequency_hz=profile.control_frequency_hz,
        expected_camera_schema=profile.raw["camera"]["schema"],
        expected_camera_mount_id=profile.camera_mount_id,
        expected_image_preprocessing_id=profile.preprocessing_id,
        expected_gripper_mode=profile.gripper_mode,
        expected_normalization_asset_id=profile.raw["normalization_asset_id"],
        expected_openpi_commit=profile.raw["openpi_commit"],
    )
    assert result.eligible is False
    assert any("camera_mount_id" in failure for failure in result.failures)
