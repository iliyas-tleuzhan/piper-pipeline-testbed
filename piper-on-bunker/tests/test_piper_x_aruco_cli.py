import subprocess


SCRIPTS = [
    "piper-on-bunker/scripts/record_openpi_piper_x_aruco_episode.py",
    "piper-on-bunker/scripts/inspect_piper_x_aruco_collection_environment.py",
    "piper-on-bunker/scripts/inspect_openpi_piper_x_aruco_episode.py",
    "piper-on-bunker/scripts/convert_openpi_piper_x_aruco_to_lerobot.py",
    "piper-on-bunker/scripts/label_openpi_episode_outcome.py",
    "piper-on-bunker/scripts/run_openpi_piper_x_aruco_live_shadow.py",
    "piper-on-bunker/scripts/make_openpi_piper_x_aruco_smoke_checkpoint_metadata.py",
]


def test_new_piper_x_clis_have_help():
    for script in SCRIPTS:
        proc = subprocess.run(["python3", script, "--help"], text=True, capture_output=True, check=False)
        assert proc.returncode == 0, proc.stderr
        assert "usage:" in proc.stdout


def test_read_only_preflight_skip_ros_reports_blockers_without_motion():
    proc = subprocess.run(
        [
            "python3",
            "piper-on-bunker/scripts/inspect_piper_x_aruco_collection_environment.py",
            "--skip-ros",
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert proc.returncode in {0, 2}
    assert "ready_for_collection" in proc.stdout
    assert "enable" not in proc.stdout.lower()
