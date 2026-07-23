from __future__ import annotations

from piper_on_bunker.models import ArmMode


ALLOWED_TRANSITIONS: dict[ArmMode, set[ArmMode]] = {
    ArmMode.IDLE: {ArmMode.STOWED, ArmMode.NAVIGATION_VIEW, ArmMode.FAULT, ArmMode.ESTOP},
    ArmMode.STOWED: {ArmMode.IDLE, ArmMode.NAVIGATION_VIEW, ArmMode.TASK_INSPECTION, ArmMode.FAULT, ArmMode.ESTOP},
    ArmMode.NAVIGATION_VIEW: {ArmMode.ACTIVE_SCAN, ArmMode.TASK_INSPECTION, ArmMode.STOWED, ArmMode.FAULT, ArmMode.ESTOP},
    ArmMode.ACTIVE_SCAN: {ArmMode.TASK_INSPECTION, ArmMode.NAVIGATION_VIEW, ArmMode.FAULT, ArmMode.ESTOP},
    ArmMode.TASK_INSPECTION: {ArmMode.PRE_MANIPULATION, ArmMode.NAVIGATION_VIEW, ArmMode.FAULT, ArmMode.ESTOP},
    ArmMode.PRE_MANIPULATION: {ArmMode.MANIPULATION, ArmMode.RETRACTING, ArmMode.FAULT, ArmMode.ESTOP},
    ArmMode.MANIPULATION: {ArmMode.RETRACTING, ArmMode.VERIFYING, ArmMode.FAULT, ArmMode.ESTOP},
    ArmMode.VERIFYING: {ArmMode.RETRACTING, ArmMode.NAVIGATION_VIEW, ArmMode.FAULT, ArmMode.ESTOP},
    ArmMode.RETRACTING: {ArmMode.VERIFYING, ArmMode.NAVIGATION_VIEW, ArmMode.STOWED, ArmMode.FAULT, ArmMode.ESTOP},
    ArmMode.FAULT: {ArmMode.RETRACTING, ArmMode.STOWED, ArmMode.ESTOP},
    ArmMode.ESTOP: set(),
}


class ArmStateMachine:
    def __init__(self, initial: ArmMode = ArmMode.IDLE) -> None:
        self.mode = initial

    def can_transition(self, target: ArmMode) -> bool:
        return target == self.mode or target in ALLOWED_TRANSITIONS[self.mode]

    def transition(self, target: ArmMode) -> ArmMode:
        if not self.can_transition(target):
            raise ValueError(f"Cannot transition from {self.mode.value} to {target.value}")
        self.mode = target
        return self.mode
