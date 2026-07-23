from piper_on_bunker.hardware.mock_base import MockBase


def test_future_dual_arm_role_switching_contract():
    base = MockBase(direction="forward")
    front_arm = "NAVIGATION_VIEW" if base.get_navigation_direction() == "forward" else "STOWED"
    rear_arm = "STOWED" if base.get_navigation_direction() == "forward" else "NAVIGATION_VIEW"
    assert front_arm == "NAVIGATION_VIEW"
    assert rear_arm == "STOWED"
    assert base.request_navigation_pause()
    assert base.is_base_stopped()
    assert base.unlock_base()
