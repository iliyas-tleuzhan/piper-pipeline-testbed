"""PiPER-X ArUco OpenPI training configuration scaffold.

The future training entry is named ``pi05_piper_x_touch_aruco``. Do not run it
until a real PiPER-X dataset, normalization stats, and manifests exist.
"""

OPENPI_COMMIT = "15a9616a00943ada6c20a0f158e3adb39df2ccac"
BASE_CHECKPOINT = "gs://openpi-assets/checkpoints/pi05_base"
TRAINING_CONFIG_NAME = "pi05_piper_x_touch_aruco"
ROBOT_PROFILE_ID = "agilex_piper_x_single_arm_wrist_aruco_v1"
NORMALIZATION_ASSET_ID = "piper_x_touch_aruco_v1"


PIPER_X_TOUCH_ARUCO_CONFIG = {
    "config_name": TRAINING_CONFIG_NAME,
    "base_checkpoint": BASE_CHECKPOINT,
    "robot_profile_id": ROBOT_PROFILE_ID,
    "task_id": "touch_aruco_marker_vertical_v1",
    "dataset_robot_type": "piper_x_single_arm",
    "normalization_asset_id": NORMALIZATION_ASSET_ID,
    "image_preprocessing_id": "openpi_resize_with_pad_rgb_224_v1",
    "observation_keys": [
        "observation/exterior_image",
        "observation/wrist_image",
        "observation/state",
        "prompt",
    ],
    "action_key": "action",
    "action_dim": 7,
    "action_semantics": "absolute_piper_x_joint_targets",
    "joint_order": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"],
    "control_frequency_hz": 20.0,
    "physical_execution_allowed": False,
}
