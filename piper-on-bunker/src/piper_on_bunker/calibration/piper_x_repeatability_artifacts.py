from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import yaml


CHECKLIST_ITEMS = {
    "bunker_stationary_powered_safely": "Bunker stationary and powered safely",
    "piper_x_base_bolts_tight": "PiPER-X base bolts tight",
    "arm_mounting_plate_rigid": "Arm mounting plate rigid",
    "no_bunker_to_arm_base_movement": "No movement between Bunker frame and arm base",
    "wrist_camera_bracket_rigid": "Wrist camera bracket rigid",
    "d435i_mounting_screws_tight": "D435i mounting screws tight",
    "camera_usb_cable_strain_relieved": "Camera USB cable strain relieved",
    "cable_does_not_pull_camera": "Cable does not pull camera during arm movement",
    "gripper_or_tool_rigid": "Gripper/tool rigid",
    "no_visible_joint_damage": "No visible joint damage",
    "no_unusual_noise": "No unusual noise",
    "no_obvious_backlash": "No obvious backlash",
    "inactive_second_arm_outside_workspace": "Inactive second arm outside workspace",
    "environment_cleared": "Environment cleared",
    "stop_command_ready": "Stop command ready",
    "operator_standing_clear": "Operator standing clear",
    "physical_measurement_reference_fixed": "Physical measurement reference fixed to Bunker or environment",
}


CSV_FIELDS = [
    "cycle_index",
    "method",
    "reference_frame",
    "reference_description",
    "x_mm",
    "y_mm",
    "z_mm",
    "estimated_measurement_uncertainty_mm",
    "notes",
]


def checklist_template(identity: dict[str, str] | None = None) -> dict[str, Any]:
    return {
        "schema_version": "piper_x.phase_0a_operator_checklist.v1",
        "operator": "",
        "timestamp": "",
        "identity": dict(identity or {}),
        "items": {
            key: {"description": description, "checked": False, "notes": ""}
            for key, description in CHECKLIST_ITEMS.items()
        },
    }


def validate_checklist(path: str | Path, expected_identity: dict[str, str]) -> tuple[bool, list[str], dict[str, Any]]:
    checklist_path = Path(path)
    if not checklist_path.exists():
        return False, [f"checklist missing: {checklist_path}"], {}
    with checklist_path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    blockers: list[str] = []
    if not data.get("operator"):
        blockers.append("checklist operator missing")
    if not data.get("timestamp"):
        blockers.append("checklist timestamp missing")
    identity = data.get("identity") or {}
    for key, expected in expected_identity.items():
        if expected == "unknown":
            blockers.append(f"local identity {key} is unknown")
        elif str(identity.get(key)) != str(expected):
            blockers.append(f"checklist identity mismatch for {key}: expected {expected}, got {identity.get(key)}")
    items = data.get("items") or {}
    for key in CHECKLIST_ITEMS:
        payload = items.get(key)
        if not isinstance(payload, dict):
            blockers.append(f"checklist item missing: {key}")
        elif payload.get("checked") is not True:
            blockers.append(f"checklist item not checked: {key}")
    return not blockers, blockers, data


class Phase0AArtifactWriter:
    def __init__(self, artifact_dir: str | Path) -> None:
        self.artifact_dir = Path(artifact_dir)
        self.artifact_dir.mkdir(parents=True, exist_ok=True)

    def write_manifest(self, payload: dict[str, Any]) -> Path:
        path = self.artifact_dir / "manifest.yaml"
        with path.open("w", encoding="utf-8") as fh:
            yaml.safe_dump(payload, fh, sort_keys=True)
        return path

    def read_manifest(self) -> dict[str, Any]:
        with (self.artifact_dir / "manifest.yaml").open("r", encoding="utf-8") as fh:
            return yaml.safe_load(fh) or {}

    def append_cycle(self, payload: dict[str, Any]) -> Path:
        path = self.artifact_dir / "cycles.jsonl"
        with path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(payload, sort_keys=True) + "\n")
        return path

    def read_cycles(self) -> list[dict[str, Any]]:
        path = self.artifact_dir / "cycles.jsonl"
        if not path.exists():
            return []
        out = []
        with path.open("r", encoding="utf-8") as fh:
            for line in fh:
                if line.strip():
                    out.append(json.loads(line))
        return out

    def write_joint_summary(self, payload: dict[str, Any]) -> Path:
        path = self.artifact_dir / "joint_summary.json"
        with path.open("w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2, sort_keys=True)
            fh.write("\n")
        return path

    def write_physical_measurement_template(self, cycles: int) -> Path:
        path = self.artifact_dir / "physical_measurements.csv"
        if path.exists():
            return path
        with path.open("w", encoding="utf-8", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
            writer.writeheader()
            for index in range(int(cycles)):
                writer.writerow(
                    {
                        "cycle_index": index + 1,
                        "method": "manual_xyz_mm",
                        "reference_frame": "bunker_fixed_reference",
                        "reference_description": "",
                        "x_mm": "",
                        "y_mm": "",
                        "z_mm": "",
                        "estimated_measurement_uncertainty_mm": "",
                        "notes": "",
                    }
                )
        return path

    def write_checklist_templates(self, identity: dict[str, str]) -> dict[str, str]:
        md_path = self.artifact_dir / "operator_checklist.md"
        yaml_path = self.artifact_dir / "operator_checklist.yaml"
        md_lines = ["# PiPER-X Phase 0A Mechanical Checklist", "", "Fill operator_checklist.yaml before physical execution.", ""]
        md_lines.extend(f"- [ ] {description}" for description in CHECKLIST_ITEMS.values())
        md_path.write_text("\n".join(md_lines) + "\n", encoding="utf-8")
        if not yaml_path.exists():
            with yaml_path.open("w", encoding="utf-8") as fh:
                yaml.safe_dump(checklist_template(identity), fh, sort_keys=True)
        return {"operator_checklist_md": str(md_path), "operator_checklist_yaml": str(yaml_path)}

    def write_report(self, payload: dict[str, Any]) -> Path:
        path = self.artifact_dir / "final_report.md"
        phase = payload.get("phase_0a") or {}
        lines = [
            "# PiPER-X Phase 0A Repeatability Report",
            "",
            f"- diagnostic_id: `{payload.get('diagnostic_id', 'unknown')}`",
            f"- classification: `{payload.get('classification', 'unknown')}`",
            f"- software_repeatability_passed: `{phase.get('joint_repeatability_passed', False)}`",
            f"- controller_settling_passed: `{phase.get('controller_settling_passed', False)}`",
            f"- physical_repeatability_status: `{(phase.get('physical_repeatability') or {}).get('status', 'UNKNOWN')}`",
            f"- ready_for_joint_zero_fk_investigation: `{phase.get('ready_for_joint_zero_fk_investigation', False)}`",
            f"- completed_cycles: `{payload.get('completed_cycles', 0)}`",
            "",
            "## Motion Reporting",
            "",
        ]
        motion = payload.get("motion_reporting") or {}
        if motion:
            lines.extend(f"- {key}: `{value}`" for key, value in sorted(motion.items()))
        else:
            lines.append("- unavailable")
        lines.extend(["", "## Blockers", ""])
        blockers = payload.get("blockers") or []
        lines.extend(f"- {blocker}" for blocker in blockers) if blockers else lines.append("- none")
        lines.extend(
            [
                "",
                "## Interpretation",
                "",
                "Phase 0A only evaluates repeatability and readiness to proceed. It does not validate joint zero, FK, TCP, or hand-eye calibration.",
                "Physical repeatability UNKNOWN is not the same as physical repeatability ACCEPTABLE.",
                "",
            ]
        )
        path.write_text("\n".join(lines), encoding="utf-8")
        return path
