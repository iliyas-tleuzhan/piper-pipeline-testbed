from __future__ import annotations

from time import monotonic

from piper_on_bunker.models import Pose, SkillResult, StatusCode


class PiperRosArm:
    def __init__(self, physical_motion_enabled: bool = False) -> None:
        if not physical_motion_enabled:
            raise RuntimeError("PiperRosArm is disabled unless physical_motion_enabled is true")
        try:
            import rospy  # noqa: F401
        except Exception as exc:
            raise RuntimeError("ROS/rospy is required for PiperRosArm; use mock or replay mode on this laptop") from exc

    def get_state(self) -> dict:
        return {"adapter": "piper_ros"}

    def move_to_named_pose(self, name: str) -> SkillResult:
        return SkillResult.build(False, StatusCode.HARDWARE_DISABLED, "ROS implementation is an interface placeholder", monotonic())

    def move_to_pose(self, pose: Pose) -> SkillResult:
        return SkillResult.build(False, StatusCode.HARDWARE_DISABLED, "ROS implementation is an interface placeholder", monotonic())

    def press(self, pose: Pose, depth_m: float) -> SkillResult:
        return SkillResult.build(False, StatusCode.HARDWARE_DISABLED, "ROS implementation is an interface placeholder", monotonic())

    def retract(self) -> SkillResult:
        return SkillResult.build(False, StatusCode.HARDWARE_DISABLED, "ROS implementation is an interface placeholder", monotonic())

    def stop(self) -> SkillResult:
        return SkillResult.build(True, StatusCode.ESTOP, "stop requested", monotonic())
