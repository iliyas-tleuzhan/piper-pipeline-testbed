#!/usr/bin/env python3
"""Label a PiPER-X ArUco episode outcome after passive recording."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "piper-on-bunker" / "src"
sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.data.piper_x_aruco_episode import label_episode_outcome


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode_dir")
    parser.add_argument("--status", required=True, choices=["success", "failure", "aborted"])
    parser.add_argument("--contact-confirmed", action="store_true")
    parser.add_argument("--marker-touched", action="store_true")
    parser.add_argument("--retraction-completed", action="store_true")
    parser.add_argument("--failure-reason")
    parser.add_argument("--operator-notes")
    args = parser.parse_args()

    outcome = label_episode_outcome(
        args.episode_dir,
        status=args.status,
        contact_confirmed=args.contact_confirmed or None,
        marker_touched=args.marker_touched or None,
        retraction_completed=args.retraction_completed or None,
        failure_reason=args.failure_reason,
        operator_notes=args.operator_notes,
    )
    import json

    print(json.dumps(outcome, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
