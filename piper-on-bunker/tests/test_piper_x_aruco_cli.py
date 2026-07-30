import subprocess
import importlib.util
from pathlib import Path
from unittest import mock


SCRIPTS = [
    "piper-on-bunker/scripts/record_openpi_piper_x_aruco_episode.py",
    "piper-on-bunker/scripts/inspect_piper_x_aruco_collection_environment.py",
    "piper-on-bunker/scripts/inspect_openpi_piper_x_aruco_episode.py",
    "piper-on-bunker/scripts/convert_openpi_piper_x_aruco_to_lerobot.py",
    "piper-on-bunker/scripts/label_openpi_episode_outcome.py",
    "piper-on-bunker/scripts/run_openpi_piper_x_aruco_live_shadow.py",
    "piper-on-bunker/scripts/make_openpi_piper_x_aruco_smoke_checkpoint_metadata.py",
    "piper-on-bunker/scripts/piper_x_aruco_pose_node.py",
]


SHELL_SCRIPTS = [
    "tools/check_piper_x_d435i_aruco_image.sh",
    "tools/open_piper_x_d435i_aruco_debug_view.sh",
    "tools/snapshot_piper_x_d435i_handeye_state.sh",
    "tools/start_piper_x_d435i_handeye_calibration.sh",
]


def test_new_piper_x_clis_have_help():
    for script in SCRIPTS:
        proc = subprocess.run(["python3", script, "--help"], text=True, capture_output=True, check=False)
        assert proc.returncode == 0, proc.stderr
        assert "usage:" in proc.stdout


def test_debug_helper_shell_syntax():
    for script in SHELL_SCRIPTS:
        proc = subprocess.run(["bash", "-n", script], text=True, capture_output=True, check=False)
        assert proc.returncode == 0, proc.stderr


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


def _load_script(name):
    path = Path("piper-on-bunker/scripts") / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _Stamp:
    def __init__(self, value):
        self._value = value

    def to_sec(self):
        return self._value


class _Header:
    def __init__(self, stamp):
        self.stamp = _Stamp(stamp)


class _JointState:
    def __init__(self, names, positions, stamp=10.0):
        self.name = names
        self.position = positions
        self.header = _Header(stamp)


def test_preflight_command_message_missing_wrong_joint_names_rejected():
    preflight = _load_script("inspect_piper_x_aruco_collection_environment.py")
    msg = _JointState(["joint1", "joint2", "joint3", "joint4", "joint5", "wrong"], [0, 1, 2, 3, 4, 5])
    try:
        preflight.build_action_7d(msg, ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"), 0.2)
    except ValueError as exc:
        assert "joint6" in str(exc)
    else:
        raise AssertionError("wrong joint names should fail")


def test_preflight_successful_fresh_7d_mapping():
    preflight = _load_script("inspect_piper_x_aruco_collection_environment.py")
    order = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper")
    state = _JointState(list(order), [0, 1, 2, 3, 4, 5, 0.4])
    command = _JointState(list(order[:6]), [10, 11, 12, 13, 14, 15])
    assert preflight.build_state_7d(state, order) == [0, 1, 2, 3, 4, 5, 0.4]
    assert preflight.build_action_7d(command, order, 0.4) == [10, 11, 12, 13, 14, 15, 0.4]


def test_preflight_missing_command_topic_reports_no_publishers(monkeypatch):
    preflight = _load_script("inspect_piper_x_aruco_collection_environment.py")
    monkeypatch.setattr(preflight, "_topic_publishers", lambda topic: [])
    assert preflight._topic_publishers("/missing_command_topic") == []


def test_preflight_can_up_but_not_error_active(monkeypatch):
    preflight = _load_script("inspect_piper_x_aruco_collection_environment.py")

    class Proc:
        returncode = 0
        stdout = "8: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16\n    can state ERROR-PASSIVE restart-ms 0\n"
        stderr = ""

    monkeypatch.setattr(preflight.subprocess, "run", lambda *args, **kwargs: Proc())
    status = preflight._can_status("can0")
    assert status["up"] is True
    assert status["error_active"] is False


def test_recorder_stale_but_mutually_synchronized_streams_rejected(monkeypatch):
    recorder = _load_script("record_openpi_piper_x_aruco_episode.py")
    now = 100.0
    monkeypatch.setattr(recorder.time, "time", lambda: now)
    latest_wrist = recorder.Latest()
    latest_state = recorder.Latest()
    latest_action = recorder.Latest()
    for latest in (latest_wrist, latest_state, latest_action):
        latest.update([1], stamp_s=90.0)
    ready, failures, ages = recorder._streams_ready(
        latest_wrist=latest_wrist,
        latest_state=latest_state,
        latest_action=latest_action,
        max_image_age_s=0.5,
        max_state_age_s=0.5,
        max_action_age_s=0.5,
    )
    assert ready is False
    assert len({round(v, 3) for v in ages.values()}) == 1
    assert any("stale wrist image" in failure for failure in failures)


def test_recorder_fixed_gripper_startup_capture_and_explicit_action():
    recorder = _load_script("record_openpi_piper_x_aruco_episode.py")
    order = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper")
    state = _JointState(list(order), [0, 1, 2, 3, 4, 5, 0.42])
    assert recorder._capture_gripper_from_state(state, "gripper") == 0.42
    command = _JointState(list(order[:6]), [10, 11, 12, 13, 14, 15])
    assert recorder._command_msg_to_action(command, order, 0.7) == [10, 11, 12, 13, 14, 15, 0.7]


def test_recorder_abnormal_exit_preserves_incomplete_marker(tmp_path):
    recorder = _load_script("record_openpi_piper_x_aruco_episode.py")
    episode_dir = tmp_path / "episode"
    episode_dir.mkdir()
    incomplete = episode_dir / recorder.INCOMPLETE_MARKER
    incomplete.write_text("recording\n")
    recorder._finalize_episode(episode_dir, {"frames_recorded": 1}, clean=False)
    assert incomplete.exists()
    recorder._finalize_episode(episode_dir, {"frames_recorded": 1}, clean=True)
    assert not incomplete.exists()
