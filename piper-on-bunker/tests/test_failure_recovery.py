from piper_on_bunker.hardware.mock_arm import MockArm
from piper_on_bunker.mission_supervisor import MissionSupervisor
from piper_on_bunker.models import StatusCode
from piper_on_bunker.perception.mock_camera import MockCamera


def test_target_not_found_failure():
    result = MissionSupervisor(MockArm(), MockCamera(target_found=False)).run_button_mission()
    assert not result.success
    assert result.status_code == StatusCode.TARGET_NOT_FOUND


def test_camera_unavailable_failure():
    result = MissionSupervisor(MockArm(), MockCamera(unavailable=True)).run_button_mission()
    assert not result.success
    assert result.status_code == StatusCode.CAMERA_UNAVAILABLE


def test_verification_failure():
    supervisor = MissionSupervisor(MockArm(), MockCamera())
    supervisor.verification_should_pass = False
    result = supervisor.run_button_mission()
    assert not result.success
    assert result.status_code == StatusCode.VERIFICATION_FAILURE


def test_injected_failures():
    for failure, code in [
        ("ik_failure", StatusCode.IK_FAILURE),
        ("planning_failure", StatusCode.PLANNING_FAILURE),
        ("pose_timeout", StatusCode.POSE_TIMEOUT),
        ("controller_failure", StatusCode.CONTROLLER_FAILURE),
    ]:
        supervisor = MissionSupervisor(MockArm(failures={failure}), MockCamera())
        result = supervisor.run_button_mission()
        assert not result.success
        assert result.status_code == code


def test_estop():
    supervisor = MissionSupervisor(MockArm(), MockCamera())
    result = supervisor.stop_motion()
    assert result.status_code == StatusCode.ESTOP
