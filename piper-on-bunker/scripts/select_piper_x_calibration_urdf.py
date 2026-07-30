#!/usr/bin/env python3
"""Select a PiPER-X hand-eye calibration URDF, failing closed by default."""

from __future__ import annotations

import argparse
import json

from piper_on_bunker.diagnostics.piper_x_model_gate import select_urdf_for_calibration


def _bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on", "passed"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--physical-model-id", default="")
    parser.add_argument("--firmware-version", default="")
    parser.add_argument("--robot-urdf-path", default="")
    parser.add_argument("--expected-sha256", default="")
    parser.add_argument("--fk-verified", default="false")
    args = parser.parse_args()

    selection = select_urdf_for_calibration(
        physical_model_id=args.physical_model_id,
        firmware_version=args.firmware_version,
        robot_urdf_path=args.robot_urdf_path,
        expected_sha256=args.expected_sha256 or None,
        fk_verified=_bool(args.fk_verified),
    )
    print(json.dumps(selection.to_dict(), indent=2, sort_keys=True))
    return 0 if selection.selected else 2


if __name__ == "__main__":
    raise SystemExit(main())
