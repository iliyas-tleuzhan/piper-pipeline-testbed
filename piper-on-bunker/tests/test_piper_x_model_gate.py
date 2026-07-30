from __future__ import annotations

from pathlib import Path

from piper_on_bunker.diagnostics.piper_x_model_gate import official_standard_piper_urdf_for_firmware
from piper_on_bunker.diagnostics.piper_x_model_gate import select_urdf_for_calibration
from piper_on_bunker.diagnostics.piper_x_model_gate import sha256_file


def test_official_standard_piper_firmware_rule_is_documented():
    assert official_standard_piper_urdf_for_firmware("S-V1.6-2") == "piper_description_old.urdf"
    assert official_standard_piper_urdf_for_firmware("S-V1.6-3") == "piper_description.urdf"
    assert official_standard_piper_urdf_for_firmware("S-V1.7-0") == "piper_description.urdf"
    assert official_standard_piper_urdf_for_firmware("unknown") is None


def test_unresolved_model_fails_closed(tmp_path):
    urdf = tmp_path / "piper_x_description.urdf"
    urdf.write_text("<robot name='piper_x'/>\n", encoding="utf-8")
    selection = select_urdf_for_calibration(
        physical_model_id="",
        firmware_version="",
        robot_urdf_path=str(urdf),
        fk_verified=False,
    )
    assert selection.selected is False
    assert "physical model ID is unresolved" in selection.blockers
    assert "firmware version is unresolved" in selection.blockers
    assert "FK verification has not passed across multiple stopped poses" in selection.blockers


def test_piper_x_requires_explicit_piper_x_urdf_and_fk_verification(tmp_path):
    wrong = tmp_path / "piper_description.urdf"
    wrong.write_text("<robot name='piper'/>\n", encoding="utf-8")
    wrong_selection = select_urdf_for_calibration(
        physical_model_id="agilex_piper_x",
        firmware_version="S-V1.6-3",
        robot_urdf_path=str(wrong),
        fk_verified=True,
    )
    assert wrong_selection.selected is False
    assert any("PiPER-X calibration requires" in blocker for blocker in wrong_selection.blockers)

    right = tmp_path / "piper_x_description.urdf"
    right.write_text("<robot name='piper_x'/>\n", encoding="utf-8")
    right_selection = select_urdf_for_calibration(
        physical_model_id="agilex_piper_x",
        firmware_version="S-V1.6-3",
        robot_urdf_path=str(right),
        expected_sha256=sha256_file(right),
        fk_verified=True,
    )
    assert right_selection.selected is True
    assert right_selection.selected_urdf_path == str(right)
    assert right_selection.selected_urdf_sha256 == sha256_file(right)


def test_urdf_hash_mismatch_fails_closed(tmp_path):
    urdf = Path(tmp_path / "piper_x_description.urdf")
    urdf.write_text("<robot name='piper_x'/>\n", encoding="utf-8")
    selection = select_urdf_for_calibration(
        physical_model_id="agilex_piper_x",
        firmware_version="S-V1.6-3",
        robot_urdf_path=str(urdf),
        expected_sha256="0" * 64,
        fk_verified=True,
    )
    assert selection.selected is False
    assert any("hash mismatch" in blocker for blocker in selection.blockers)
