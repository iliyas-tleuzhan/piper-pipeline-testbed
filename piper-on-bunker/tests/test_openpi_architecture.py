from pathlib import Path


FORBIDDEN = (
    "moveit_commander",
    "moveit_msgs",
    "FollowJointTrajectory",
    "lap_policy",
    "run_lap",
    "MoveGroupCommander",
    "OMPL",
)


def test_openpi_runtime_imports_no_lap_or_moveit():
    root = Path("piper-on-bunker/src/piper_on_bunker")
    paths = [
        *root.joinpath("control").glob("*.py"),
        *root.joinpath("openclaw").glob("*.py"),
        root / "policies" / "openpi_piper_policy.py",
    ]
    text = "\n".join(path.read_text(encoding="utf-8") for path in paths)
    for forbidden in FORBIDDEN:
        assert forbidden not in text


def test_default_openpi_config_excludes_lap_and_moveit():
    text = Path("piper-on-bunker/config/openpi_piper.yaml").read_text(encoding="utf-8").lower()
    assert "lap" not in text
    assert "moveit" not in text


def test_mission_supervisor_does_not_import_moveit_by_default():
    text = Path("piper-on-bunker/src/piper_on_bunker/mission_supervisor.py").read_text(encoding="utf-8")
    top_level = text.split("class MissionSupervisor", 1)[0]
    assert "moveit_policy" not in top_level
