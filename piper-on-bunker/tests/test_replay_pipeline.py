from piper_on_bunker.factory import build_supervisor_from_path
from piper_on_bunker.models import StatusCode


def test_replay_mission():
    supervisor = build_supervisor_from_path("piper-on-bunker/config/tabletop_replay.yaml")
    result = supervisor.run_button_mission()
    assert not result.success
    assert result.status_code == StatusCode.WAITING_FOR_VERIFICATION


def test_replay_mission_with_manual_acknowledgement():
    supervisor = build_supervisor_from_path("piper-on-bunker/config/tabletop_replay.yaml")
    supervisor.acknowledge_manual_verification()
    assert supervisor.run_button_mission().success
