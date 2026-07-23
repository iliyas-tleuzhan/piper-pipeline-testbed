from __future__ import annotations

from piper_on_bunker.hardware.mock_arm import MockArm
from piper_on_bunker.models import Pose, SkillResult, StatusCode
from piper_on_bunker.resource_manager import ResourceManager


class DualArmMock:
    def __init__(self) -> None:
        self.front_arm = MockArm()
        self.rear_arm = MockArm()
        self.resources = ResourceManager()

    def configure_for_travel(self, direction: str):
        if direction == "forward":
            self.front_arm.move_to_named_pose("simulated_front_nav_view")
            self.rear_arm.move_to_named_pose("stowed")
        elif direction == "reverse":
            self.front_arm.move_to_named_pose("stowed")
            self.rear_arm.move_to_named_pose("simulated_rear_nav_view")
        else:
            raise ValueError("direction must be forward or reverse")
        return {"front_arm": self.front_arm.current_pose_name, "rear_arm": self.rear_arm.current_pose_name}

    def manipulate(self, arm_name: str, target: Pose, owner: str = "dual_arm_mock") -> SkillResult:
        if arm_name not in {"front", "rear"}:
            return SkillResult.build(False, StatusCode.INVALID_COMMAND, "arm_name must be front or rear", __import__("time").monotonic())
        resource = arm_name + "_arm"
        if not self.resources.acquire(resource, owner):
            return SkillResult.build(False, StatusCode.RESOURCE_BUSY, resource + " is busy", __import__("time").monotonic())
        try:
            arm = self.front_arm if arm_name == "front" else self.rear_arm
            other = self.rear_arm if arm_name == "front" else self.front_arm
            other.move_to_named_pose("stowed")
            move = arm.move_to_pose(target)
            if not move.success:
                return move
            return arm.press(target, 0.01)
        finally:
            self.resources.release(resource, owner)
