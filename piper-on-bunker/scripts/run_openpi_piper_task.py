#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json

from piper_on_bunker.control.phase_orchestrator import PhaseOrchestrator


def main() -> int:
    parser = argparse.ArgumentParser(description="Run an OpenClaw -> OpenPI PiPER task in shadow mode by default.")
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--execute", action="store_true", help="Refused until a PiPER-compatible checkpoint is configured.")
    args = parser.parse_args()
    if args.execute:
        raise SystemExit("Physical learned execution is intentionally disabled until PiPER checkpoint validation passes.")
    orchestrator = PhaseOrchestrator()
    plan = orchestrator.plan_task(args.instruction)
    phase_results = [orchestrator.run_phase_shadow(phase) for phase in plan.phases]
    print(json.dumps({"plan": plan.to_dict(), "phase_results": phase_results, "shadow_mode": True}, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

