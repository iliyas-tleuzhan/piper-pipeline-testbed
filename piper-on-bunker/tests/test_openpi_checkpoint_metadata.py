import pytest

from piper_on_bunker.policies.checkpoint_metadata import load_checkpoint_metadata
from piper_on_bunker.policies.checkpoint_metadata import validate_checkpoint_metadata
from piper_on_bunker.policies.openpi_piper_policy import OPENPI_COMMIT
from piper_on_bunker.policies.openpi_piper_policy import PIPER_ACTION_SEMANTICS
from piper_on_bunker.policies.openpi_piper_policy import PIPER_JOINT_NAMES


def _metadata():
    return {
        "checkpoint": "checkpoints/pi05_piper_single_arm/step_30000",
        "piper_compatible": True,
        "openpi_commit": OPENPI_COMMIT,
        "action_semantics": PIPER_ACTION_SEMANTICS,
        "joint_names": list(PIPER_JOINT_NAMES),
        "action_dim": 7,
        "action_horizon": 10,
        "control_frequency_hz": 20.0,
        "units": {"arm": "rad", "gripper": "total_jaw_opening_m"},
        "normalization_metadata": {"asset_id": "piper_single_arm_v1", "statistics_hash": "sha256:abc"},
        "dataset_provenance": {"dataset_id": "piper_cup", "episode_count": 10, "dataset_hash": "sha256:def"},
        "camera_schema": {
            "observation/exterior_image": {"shape": [224, 224, 3], "dtype": "uint8"},
            "observation/wrist_image": {"shape": [224, 224, 3], "dtype": "uint8"},
        },
        "offline_validation": {"passed": True, "report_hash": "sha256:123"},
        "gripper_hardware_verification": {"passed": True, "report_hash": "sha256:456"},
    }


def test_complete_checkpoint_metadata_is_eligible():
    eligibility = validate_checkpoint_metadata(_metadata())
    assert eligibility.eligible


def test_base_or_foreign_checkpoint_is_not_eligible():
    metadata = _metadata()
    metadata["piper_compatible"] = False
    metadata["normalization_metadata"] = {}
    eligibility = validate_checkpoint_metadata(metadata)
    assert not eligibility.eligible
    assert any("piper_compatible" in item for item in eligibility.failures)


def test_gripper_gate_can_be_relaxed_only_for_arm_only_validation():
    metadata = _metadata()
    metadata["gripper_hardware_verification"] = {"passed": False}
    assert not validate_checkpoint_metadata(metadata).eligible
    assert validate_checkpoint_metadata(metadata, require_gripper=False).eligible


def test_require_raises_with_failures():
    with pytest.raises(ValueError, match="not eligible"):
        validate_checkpoint_metadata({}).require()


def test_missing_metadata_path_has_clear_error(tmp_path):
    with pytest.raises(ValueError, match="does not exist"):
        load_checkpoint_metadata(tmp_path / "missing.json")
