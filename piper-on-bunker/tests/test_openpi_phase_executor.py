import numpy as np
import pytest

from piper_on_bunker.control.piper_joint_phase_executor import ExecutionConfig
from piper_on_bunker.control.piper_joint_phase_executor import PiperJointPhaseExecutor
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


def test_shadow_mode_publishes_nothing():
    response = validate_openpi_response(_payload(True), require_piper_compatible=True, expected_frequency_hz=20.0)
    executor = PiperJointPhaseExecutor()
    result = executor.execute_response(response, current_state=np.zeros(7), state_age_s=0, camera_age_s=0, execute=False)
    assert result.shadow_mode is True
    assert result.published_commands == 0


def test_execute_requires_permission():
    response = validate_openpi_response(_payload(True), require_piper_compatible=True, expected_frequency_hz=20.0)
    executor = PiperJointPhaseExecutor(ExecutionConfig(physical_motion_permission=False))
    with pytest.raises(ValueError, match="permission"):
        executor.execute_response(response, current_state=np.zeros(7), state_age_s=0, camera_age_s=0, execute=True)


def test_stale_state_rejected():
    response = validate_openpi_response(_payload(True), require_piper_compatible=True, expected_frequency_hz=20.0)
    executor = PiperJointPhaseExecutor()
    with pytest.raises(ValueError, match="stale robot state"):
        executor.execute_response(response, current_state=np.zeros(7), state_age_s=99, camera_age_s=0, execute=False)
