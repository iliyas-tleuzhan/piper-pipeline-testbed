#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json

from piper_on_bunker.control.phase_orchestrator import PhaseOrchestrator


def main() -> int:
    parser = argparse.ArgumentParser(description="Run one OpenPI PiPER semantic phase in shadow mode.")
    parser.add_argument("--instruction", default="Move the red cup onto the paper.")
    parser.add_argument("--phase-id", default="approach")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if args.execute:
        raise SystemExit("Physical learned execution is intentionally disabled until PiPER checkpoint validation passes.")
    orchestrator = PhaseOrchestrator()
    plan = orchestrator.plan_task(args.instruction)
    phase = next((item for item in plan.phases if item.phase_id == args.phase_id), None)
    if phase is None:
        raise SystemExit(f"Unknown phase id: {args.phase_id}")
    print(json.dumps(orchestrator.run_phase_shadow(phase), indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

