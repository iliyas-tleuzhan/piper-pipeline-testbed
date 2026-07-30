from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from piper_on_bunker.diagnostics.piper_x_fk_compare import analyze_captures
from piper_on_bunker.diagnostics.piper_x_fk_compare import parse_diagnostic_file


REPO_ROOT = Path(__file__).resolve().parents[2]
DIAG_DIR = REPO_ROOT / "handeye_failure_diagnostics"


def _captures():
    return [parse_diagnostic_file(path) for path in sorted(DIAG_DIR.glob("pose_*.txt"), key=lambda path: (path.stat().st_mtime_ns, str(path)))]


def test_pose_diagnostics_preserve_joint_relay_copy_evidence():
    pose = parse_diagnostic_file(DIAG_DIR / "pose_1.txt")

    assert pose.joint_states_single[:6] == pose.relayed_joint_states[:6]
    assert pose.controller_end_pose.translation.tolist() == pytest.approx([0.017566, 0.000001, 0.574498])
    assert pose.base_to_marker is not None
    assert pose.base_to_marker.translation.tolist() == pytest.approx([0.017, 0.032, 0.756], abs=1e-3)


def test_fk_mismatch_is_configuration_dependent_and_not_constant_tcp():
    report = analyze_captures(_captures())

    assert report["pose_count"] == 3
    assert report["all_first_six_joints_copied_exactly"] is True
    assert report["fk_verified"] is False
    assert report["constant_gripper_to_controller_transform"]["verified"] is False
    assert report["constant_gripper_to_controller_transform"]["max_translation_residual_m"] > 0.02
    assert report["constant_gripper_to_controller_transform"]["max_angular_residual_deg"] > 5.0
    assert report["fixed_marker_false_motion"]["visible_pose_count"] == 2
    assert report["fixed_marker_false_motion"]["max_pairwise_displacement_m"] == pytest.approx(0.2739945, rel=1e-5)


def test_analyzer_cli_writes_machine_readable_report(tmp_path):
    output = tmp_path / "report.json"
    cmd = [
        sys.executable,
        str(REPO_ROOT / "piper-on-bunker/scripts/analyze_piper_x_fk_diagnostics.py"),
        str(DIAG_DIR / "pose_1.txt"),
        str(DIAG_DIR / "pose_2.txt"),
        str(DIAG_DIR / "pose_3.txt"),
        "--output-json",
        str(output),
    ]
    proc = subprocess.run(cmd, cwd=REPO_ROOT, text=True, capture_output=True, check=True)
    stdout_report = json.loads(proc.stdout)
    file_report = json.loads(output.read_text(encoding="utf-8"))

    assert stdout_report == file_report
    assert file_report["fk_verified"] is False
    assert "configuration-dependent" in file_report["rejection_reason"]
