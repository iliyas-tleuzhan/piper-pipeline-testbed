#!/usr/bin/env python3
"""Inspect PiPER-X ArUco raw episodes and optionally create episode splits."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "piper-on-bunker" / "src"
sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.data.piper_x_aruco_episode import inspect_episode, write_episode_splits
from piper_on_bunker.profiles.piper_x_aruco import load_piper_x_profile


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("path", help="Episode directory or dataset root")
    parser.add_argument("--profile", default="piper-on-bunker/config/openpi_piper_x_touch_aruco.yaml")
    parser.add_argument("--json-report")
    parser.add_argument("--write-splits")
    parser.add_argument("--split-seed", type=int, default=7)
    args = parser.parse_args()

    profile = load_piper_x_profile(args.profile).raw
    path = Path(args.path)
    episode_dirs = [path] if (path / "summary.json").exists() else sorted(p for p in path.iterdir() if p.is_dir())
    reports = [inspect_episode(ep, expected_profile=profile).report for ep in episode_dirs]
    dataset_report = {
        "schema_version": "piper_x_aruco_inspection_report.v1",
        "path": str(path),
        "episode_count": len(reports),
        "passed_episodes": sum(1 for report in reports if report["passed"]),
        "failed_episodes": sum(1 for report in reports if not report["passed"]),
        "episodes": reports,
    }
    if args.write_splits:
        dataset_report["splits"] = write_episode_splits(path, args.write_splits, seed=args.split_seed)
    if args.json_report:
        with Path(args.json_report).open("w", encoding="utf-8") as f:
            json.dump(dataset_report, f, indent=2, sort_keys=True)
            f.write("\n")

    print(f"PiPER-X ArUco inspection: {dataset_report['passed_episodes']}/{dataset_report['episode_count']} passed")
    for report in reports:
        status = "PASS" if report["passed"] else "FAIL"
        print(f"- {status} {report['episode_dir']}")
        for blocker in report["blockers"][:8]:
            print(f"  blocker: {blocker}")
    print(json.dumps(dataset_report, indent=2, sort_keys=True))
    return 0 if dataset_report["failed_episodes"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
