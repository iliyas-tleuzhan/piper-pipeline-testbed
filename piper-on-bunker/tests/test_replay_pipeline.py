from piper_on_bunker.factory import build_supervisor_from_path


def test_replay_mission():
    supervisor = build_supervisor_from_path("piper-on-bunker/config/tabletop_replay.yaml")
    assert supervisor.run_button_mission().success
