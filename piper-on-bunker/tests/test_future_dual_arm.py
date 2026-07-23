from piper_on_bunker.hardware.dual_arm_mock import DualArmMock
from piper_on_bunker.hardware.mock_base import MockBase
from piper_on_bunker.models import Pose


def test_future_dual_arm_role_switching_contract():
    base = MockBase(direction="forward")
    dual = DualArmMock()
    roles = dual.configure_for_travel(base.get_navigation_direction())
    assert roles["front_arm"] == "simulated_front_nav_view"
    assert roles["rear_arm"] == "stowed"
    assert base.request_navigation_pause()
    assert base.is_base_stopped()
    assert base.unlock_base()


def test_reverse_travel_role_switching():
    dual = DualArmMock()
    roles = dual.configure_for_travel("reverse")
    assert roles["front_arm"] == "stowed"
    assert roles["rear_arm"] == "simulated_rear_nav_view"


def test_selected_arm_manipulates_other_stowed():
    dual = DualArmMock()
    result = dual.manipulate("front", Pose(0.3, 0.0, 0.1))
    assert result.success
    assert dual.front_arm.press_count == 1
    assert dual.rear_arm.current_pose_name == "stowed"
