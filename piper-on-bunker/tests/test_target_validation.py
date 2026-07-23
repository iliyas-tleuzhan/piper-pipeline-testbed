from piper_on_bunker.hardware.mock_arm import MockArm
from piper_on_bunker.mission_supervisor import MissionSupervisor
from piper_on_bunker.models import StatusCode
from piper_on_bunker.perception.mock_camera import MockCamera


def test_invalid_transform_is_rejected():
    supervisor = MissionSupervisor(MockArm(), MockCamera(invalid_transform=True))
    assert supervisor.detect_target().success
    result = supervisor.estimate_target_pose()
    assert not result.success
    assert result.status_code == StatusCode.INVALID_TRANSFORM
