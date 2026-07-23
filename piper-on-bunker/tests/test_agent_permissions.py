from piper_on_bunker.agent.command_mapper import CommandMapper
from piper_on_bunker.hardware.mock_arm import MockArm
from piper_on_bunker.mission_supervisor import MissionSupervisor
from piper_on_bunker.models import StatusCode
from piper_on_bunker.perception.mock_camera import MockCamera


def test_agent_allows_high_level_mission():
    mapper = CommandMapper(MissionSupervisor(MockArm(), MockCamera()))
    result = mapper.map_command("run_button_mission")()
    assert not result.success
    assert result.status_code == StatusCode.WAITING_FOR_VERIFICATION


def test_agent_denies_raw_commands():
    mapper = CommandMapper(MissionSupervisor(MockArm(), MockCamera()))
    result = mapper.map_command("python shell ros publish raw motor disable safety")()
    assert not result.success
    assert result.status_code == StatusCode.INVALID_COMMAND
