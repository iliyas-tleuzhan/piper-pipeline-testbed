import numpy as np
import pytest

from piper_on_bunker.policies.openpi_piper_policy import PIPER_ACTION_SEMANTICS
from piper_on_bunker.policies.openpi_piper_policy import PIPER_JOINT_NAMES
from piper_on_bunker.policies.openpi_piper_policy import validate_openpi_response


def _payload(piper_compatible=False):
    return {
        "actions": np.zeros((3, 7)),
        "action_horizon": 3,
        "action_dim": 7,
        "action_semantics": PIPER_ACTION_SEMANTICS,
        "joint_names": list(PIPER_JOINT_NAMES),
        "control_frequency_hz": 20.0,
        "checkpoint": "gs://openpi-assets/checkpoints/pi05_base",
        "piper_compatible": piper_compatible,
        "units": {"arm": "rad", "gripper": "total_jaw_opening_m"},
        "normalization_metadata": {"asset_id": "piper_single_arm_v1"} if piper_compatible else {},
    }


def test_foreign_checkpoint_rejected_for_physical_use():
    with pytest.raises(ValueError, match="PiPER-compatible"):
        validate_openpi_response(_payload(False), require_piper_compatible=True, expected_frequency_hz=20.0)


def test_piper_checkpoint_metadata_accepts_shadow_or_physical():
    response = validate_openpi_response(_payload(True), require_piper_compatible=True, expected_frequency_hz=20.0)
    assert response.actions.shape == (3, 7)
    assert response.metadata.piper_compatible is True


def test_wrong_action_semantics_rejected():
    payload = _payload(True)
    payload["action_semantics"] = "cartesian_delta"
    with pytest.raises(ValueError, match="semantics"):
        validate_openpi_response(payload, require_piper_compatible=False, expected_frequency_hz=20.0)

