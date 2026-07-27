#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PIPER_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PIPER_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.policies.checkpoint_metadata import load_checkpoint_metadata
from piper_on_bunker.policies.checkpoint_metadata import validate_checkpoint_metadata


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate PiPER OpenPI checkpoint metadata for physical eligibility.")
    parser.add_argument("metadata_or_checkpoint_dir")
    parser.add_argument("--no-gripper", action="store_true", help="Allow arm-only phases without gripper verification.")
    args = parser.parse_args()

    metadata = load_checkpoint_metadata(args.metadata_or_checkpoint_dir)
    eligibility = validate_checkpoint_metadata(metadata, require_gripper=not args.no_gripper)
    print(
        json.dumps(
            {
                "eligible_for_physical_execution": eligibility.eligible,
                "failures": list(eligibility.failures),
                "checkpoint": metadata.get("checkpoint"),
                "piper_compatible": metadata.get("piper_compatible", False),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if eligibility.eligible else 2


if __name__ == "__main__":
    raise SystemExit(main())
