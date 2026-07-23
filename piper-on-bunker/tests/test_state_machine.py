import pytest

from piper_on_bunker.arm_state_machine import ArmStateMachine
from piper_on_bunker.models import ArmMode


def test_state_machine_nominal_path():
    sm = ArmStateMachine()
    sm.transition(ArmMode.NAVIGATION_VIEW)
    sm.transition(ArmMode.TASK_INSPECTION)
    sm.transition(ArmMode.PRE_MANIPULATION)
    sm.transition(ArmMode.MANIPULATION)
    sm.transition(ArmMode.RETRACTING)
    assert sm.mode == ArmMode.RETRACTING


def test_state_machine_rejects_bad_transition():
    sm = ArmStateMachine(ArmMode.ESTOP)
    with pytest.raises(ValueError):
        sm.transition(ArmMode.IDLE)
