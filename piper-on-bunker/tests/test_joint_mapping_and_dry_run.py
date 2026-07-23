from piper_on_bunker.hardware.joint_state import map_joint_state
from piper_on_bunker.hardware.mock_arm import MockArm
from piper_on_bunker.hardware.piper_ros_arm import PiperRosArm
from piper_on_bunker.mission_supervisor import MissionSupervisor
from piper_on_bunker.models import Pose, StatusCode
from piper_on_bunker.perception.mock_camera import MockCamera


def test_joint_mapping_handles_shuffled_order_and_gripper():
    mapped = map_joint_state(
        ["joint3", "gripper", "joint1", "joint6", "joint2", "joint5", "joint4"],
        [3, 0.02, 1, 6, 2, 5, 4],
        9999999999.0,
        ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    )
    assert mapped.arm_positions == [1, 2, 3, 4, 5, 6]
    assert mapped.gripper_position == 0.02


def test_joint_mapping_rejects_missing_or_duplicate_joints():
    try:
        map_joint_state(["joint1", "joint1"], [0, 1], 1.0)
        assert False
    except ValueError as exc:
        assert "duplicate" in str(exc)
    try:
        map_joint_state(["joint1"], [0], 1.0)
        assert False
    except ValueError as exc:
        assert "missing" in str(exc)


def test_piper_ros_dry_run_builds_request_without_service_call():
    arm = PiperRosArm.__new__(PiperRosArm)
    arm.physical_motion_enabled = False
    arm.safety = {"max_speed_scaling": 0.1}
    arm.last_preview = None
    result = arm._call_joint_moveit("/joint_moveit_ctrl_endpose", [0] * 6, [0.3, 0, 0.1, 0, 0, 0, 1], 0.0, __import__("time").monotonic(), "move_to_pose")
    assert result.success
    assert result.outputs["dry_run"]
    assert result.outputs["moveit_request_preview"]["will_call_service"] is False


def test_no_recovery_motion_after_controller_failure():
    arm = MockArm(failures={"controller_failure"})
    supervisor = MissionSupervisor(arm, MockCamera())
    result = supervisor.run_button_mission()
    assert not result.success
    assert result.status_code == StatusCode.CONTROLLER_FAILURE
    press_index = [i for i, action in enumerate(arm.actions) if action[0] == "press"][0]
    assert all(action[0] not in {"retract", "move_to_named_pose"} for action in arm.actions[press_index + 1 :] if action[0] != "stop")
