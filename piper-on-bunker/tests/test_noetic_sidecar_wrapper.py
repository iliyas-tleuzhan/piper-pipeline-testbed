from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import stat
import subprocess

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
WRAPPER = REPO_ROOT / "tools" / "run_in_noetic_container.sh"


def _write_fake_docker(tmp_path: Path) -> Path:
    docker_path = tmp_path / "docker"
    docker_path.write_text(
        """#!/usr/bin/env python3
import json
import os
import subprocess
import sys

log_path = os.environ.get("FAKE_DOCKER_LOG")
if log_path:
    with open(log_path, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(sys.argv[1:]) + "\\n")

if len(sys.argv) < 2 or sys.argv[1] != "run":
    sys.exit(0)

cmd = sys.argv[-1]
prefix = "source /opt/ros/noetic/setup.bash && "
if cmd.startswith(prefix):
    cmd = cmd[len(prefix):]

completed = subprocess.run(["bash", "-lc", cmd])
sys.exit(completed.returncode)
""",
        encoding="utf-8",
    )
    docker_path.chmod(docker_path.stat().st_mode | stat.S_IXUSR)
    return docker_path


def _env(tmp_path: Path) -> dict[str, str]:
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    _write_fake_docker(fake_bin)
    abot_root = tmp_path / "ABot-Claw"
    abot_root.mkdir()
    env = os.environ.copy()
    env["PATH"] = str(fake_bin) + os.pathsep + env["PATH"]
    env["ABOT_ROOT"] = str(abot_root)
    env["ROS_NOETIC_IMAGE"] = "fake-noetic:latest"
    env["FAKE_DOCKER_LOG"] = str(tmp_path / "docker.log")
    return env


def _read_logged_args(tmp_path: Path) -> list[list[str]]:
    log_path = tmp_path / "docker.log"
    return [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines() if line.strip()]


def test_wrapper_noninteractive_execution_preserves_exit_code(tmp_path):
    env = _env(tmp_path)
    result = subprocess.run(
        [str(WRAPPER), "python3", "-c", "print('wrapper_ok')"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0
    assert "wrapper_ok" in result.stdout
    args = _read_logged_args(tmp_path)[0]
    assert "-i" in args
    assert "-t" not in args


def test_wrapper_piped_stdin_reaches_container_command(tmp_path):
    env = _env(tmp_path)
    result = subprocess.run(
        [str(WRAPPER), "python3", "-c", "import sys; print(sys.stdin.read().strip())"],
        cwd=REPO_ROOT,
        env=env,
        input="hello from pipe\n",
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0
    assert "hello from pipe" in result.stdout


def test_wrapper_preserves_arguments_with_spaces(tmp_path):
    env = _env(tmp_path)
    result = subprocess.run(
        [str(WRAPPER), "python3", "-c", "import sys; print(sys.argv[1])", "value with spaces"],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0
    assert "value with spaces" in result.stdout


def test_wrapper_adds_tty_for_interactive_terminal(tmp_path):
    script_cmd = shutil.which("script")
    if not script_cmd:
        pytest.skip("script utility unavailable")
    env = _env(tmp_path)
    command = (
        f"cd {REPO_ROOT} && "
        f"PATH={env['PATH']} ABOT_ROOT={env['ABOT_ROOT']} ROS_NOETIC_IMAGE={env['ROS_NOETIC_IMAGE']} "
        f"FAKE_DOCKER_LOG={env['FAKE_DOCKER_LOG']} "
        f"{WRAPPER} python3 -c \"import os; print(int(os.isatty(0)), int(os.isatty(1)))\""
    )
    result = subprocess.run(
        [script_cmd, "-qfec", command, "/dev/null"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0
    assert "1 1" in result.stdout
    args = _read_logged_args(tmp_path)[0]
    assert "-i" in args
    assert "-t" in args
