from piper_on_bunker.hardware.mock_arm import MockArm
from piper_on_bunker.models import Pose, StatusCode


def test_mock_arm_moves_and_presses():
    arm = MockArm()
    assert arm.move_to_named_pose("scan_center").success
    assert arm.press(Pose(0.3, 0, 0.1), 0.01).success
    assert arm.get_state()["press_count"] == 1


def test_mock_arm_failure_injection():
    arm = MockArm(failures={"planning_failure"})
    result = arm.move_to_named_pose("planning_failure")
    assert not result.success
    assert result.status_code == StatusCode.PLANNING_FAILURE
