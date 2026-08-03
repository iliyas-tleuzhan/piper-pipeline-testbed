from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import yaml


CHECKLIST_TEXT = """# PiPER-X Phase 0A Mechanical Checklist

Mark this manually before physical repeatability execution. Do not edit this
file to claim completion unless the operator actually checked the item.

- [ ] Bunker stationary and powered safely.
- [ ] PiPER-X base bolts tight.
- [ ] Arm mounting plate rigid.
- [ ] No movement between Bunker frame and arm base.
- [ ] Wrist camera bracket rigid.
- [ ] D435i mounting screws tight.
- [ ] Camera USB cable strain relieved.
- [ ] Cable does not pull camera during arm movement.
- [ ] Gripper/tool rigid.
- [ ] No visible joint damage.
- [ ] No unusual noise.
- [ ] No obvious backlash.
- [ ] Inactive second arm outside workspace.
- [ ] Environment cleared.
- [ ] Stop command ready.
- [ ] Operator standing clear.
- [ ] Physical measurement reference fixed to Bunker or environment.
"""


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


class Phase0AArtifactWriter:
    def __init__(self, artifact_dir: str | Path) -> None:
        self.artifact_dir = Path(artifact_dir)
        self.artifact_dir.mkdir(parents=True, exist_ok=True)

    def write_manifest(self, payload: dict[str, Any]) -> Path:
        path = self.artifact_dir / "manifest.yaml"
        with path.open("w", encoding="utf-8") as fh:
            yaml.safe_dump(payload, fh, sort_keys=True)
        return path

    def append_cycle(self, payload: dict[str, Any]) -> Path:
        path = self.artifact_dir / "cycles.jsonl"
        with path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(payload, sort_keys=True) + "\n")
        return path

    def write_joint_summary(self, payload: dict[str, Any]) -> Path:
        path = self.artifact_dir / "joint_summary.json"
        with path.open("w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2, sort_keys=True)
            fh.write("\n")
        return path

    def write_physical_measurement_template(self, cycles: int) -> Path:
        path = self.artifact_dir / "physical_measurements.csv"
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

    def write_checklist(self) -> Path:
        path = self.artifact_dir / "operator_checklist.md"
        path.write_text(CHECKLIST_TEXT, encoding="utf-8")
        return path

    def write_report(self, payload: dict[str, Any]) -> Path:
        path = self.artifact_dir / "final_report.md"
        lines = [
            "# PiPER-X Phase 0A Repeatability Report",
            "",
            f"- diagnostic_id: `{payload.get('diagnostic_id', 'unknown')}`",
            f"- classification: `{payload.get('classification', 'unknown')}`",
            f"- phase_0a_passed: `{payload.get('phase_0a_passed', False)}`",
            f"- completed_cycles: `{payload.get('completed_cycles', 0)}`",
            "",
            "## Blockers",
            "",
        ]
        blockers = payload.get("blockers") or []
        if blockers:
            lines.extend(f"- {blocker}" for blocker in blockers)
        else:
            lines.append("- none")
        lines.extend(
            [
                "",
                "## Interpretation",
                "",
                "Phase 0A only evaluates repeatability and readiness to proceed. It does not validate joint zero, FK, TCP, or hand-eye calibration.",
                "",
            ]
        )
        path.write_text("\n".join(lines), encoding="utf-8")
        return path
