#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path


PIPER_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PIPER_ROOT.parent
SRC_ROOT = PIPER_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.control.phase_orchestrator import PhaseOrchestrator
from piper_on_bunker.hardware.live_openpi_observation import read_live_openpi_observation


def _default_log_path() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    directory = PIPER_ROOT / "logs" / "openpi_live_shadow"
    directory.mkdir(parents=True, exist_ok=True)
    return directory / f"{stamp}_openpi_piper_live_shadow.json"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run OpenClaw -> OpenPI PiPER semantic phases with live ROS observations in shadow mode."
    )
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--phase-id", help="Run only one semantic phase instead of the full manipulation plan.")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--joint-topic", default="/joint_states_single")
    parser.add_argument("--exterior-image-topic", default="/table_camera/color/image_raw")
    parser.add_argument("--wrist-image-topic", default="/cam_left_wrist")
    parser.add_argument("--no-wrist", action="store_true", help="Use a zero wrist image if the wrist camera is unavailable.")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--execute", action="store_true", help="Always refused; this command is shadow-only.")
    args = parser.parse_args()

    if args.execute:
        raise SystemExit("run_openpi_piper_live_shadow.py is shadow-only and never publishes robot commands.")

    orchestrator = PhaseOrchestrator()
    plan = orchestrator.plan_task(args.instruction)
    phases = plan.phases
    if args.phase_id:
        phase = next((item for item in plan.phases if item.phase_id == args.phase_id), None)
        if phase is None:
            raise SystemExit(f"Unknown phase id {args.phase_id!r}; expected one of {[item.phase_id for item in plan.phases]}")
        phases = [phase]
    phase_results = []
    observations = []
    wrist_topic = None if args.no_wrist else args.wrist_image_topic

    for phase in phases:
        observation = read_live_openpi_observation(
            timeout_s=args.timeout,
            joint_topic=args.joint_topic,
            exterior_image_topic=args.exterior_image_topic,
            wrist_image_topic=wrist_topic,
        )
        observations.append({"phase_id": phase.phase_id, **observation.to_summary()})
        result = orchestrator.run_phase_shadow(
            phase,
            exterior_image=observation.exterior_image,
            wrist_image=observation.wrist_image,
            state=observation.state,
            state_age_s=observation.state_age_s,
            camera_age_s=observation.camera_age_s,
        )
        phase_results.append(result)

    output = {
        "instruction": args.instruction,
        "plan": plan.to_dict(),
        "phase_results": phase_results,
        "observations": observations,
        "shadow_mode": True,
        "execution_allowed": False,
        "physical_motion_performed": False,
    }
    output_path = args.output_json or _default_log_path()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(output, indent=2, sort_keys=True))
    print(f"Saved live OpenPI shadow result: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
