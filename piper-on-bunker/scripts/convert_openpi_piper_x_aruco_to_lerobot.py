#!/usr/bin/env python3
"""Convert raw PiPER-X ArUco episodes into an OpenPI/LeRobot scaffold."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "piper-on-bunker" / "src"
sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.data.piper_x_aruco_episode import convert_to_lerobot_scaffold
from piper_on_bunker.profiles.piper_x_aruco import load_piper_x_profile


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes-root", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--profile", default="piper-on-bunker/config/openpi_piper_x_touch_aruco.yaml")
    args = parser.parse_args()

    profile = load_piper_x_profile(args.profile)
    manifest = convert_to_lerobot_scaffold(args.episodes_root, args.output_dir, profile=profile.raw)
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
