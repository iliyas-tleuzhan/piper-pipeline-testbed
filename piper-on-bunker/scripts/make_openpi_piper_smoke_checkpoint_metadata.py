#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any


PIPER_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PIPER_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.policies.openpi_piper_policy import OPENPI_BASE_CHECKPOINT
from piper_on_bunker.policies.openpi_piper_policy import OPENPI_COMMIT
from piper_on_bunker.policies.openpi_piper_policy import PIPER_ACTION_SEMANTICS
from piper_on_bunker.policies.openpi_piper_policy import PIPER_JOINT_NAMES


EXPECTED_PHASE_ORDER = ("approach", "grasp", "transport", "release")


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _iter_jsonl(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                yield json.loads(line)


def _hash_episode_files(episode_dirs: list[Path]) -> str:
    digest = hashlib.sha256()
    for episode_dir in sorted(episode_dirs):
        digest.update(str(episode_dir.name).encode("utf-8"))
        for name in ("metadata.json", "summary.json", "frames.jsonl"):
            path = episode_dir / name
            digest.update(name.encode("utf-8"))
            digest.update(path.read_bytes())
    return "sha256:" + digest.hexdigest()


def _summarize_episodes(episode_root: Path) -> tuple[list[Path], list[dict[str, Any]], dict[str, int], int]:
    episode_dirs = sorted(path for path in episode_root.iterdir() if path.is_dir() and (path / "metadata.json").exists())
    if not episode_dirs:
        raise SystemExit(f"no episode directories found under {episode_root}")

    reports: list[dict[str, Any]] = []
    phase_counts: Counter[str] = Counter()
    total_frames = 0
    for episode_dir in episode_dirs:
        metadata = _read_json(episode_dir / "metadata.json")
        summary = _read_json(episode_dir / "summary.json")
        phase_id = str(metadata.get("phase_id", ""))
        phase_counts[phase_id] += 1
        total_frames += int(summary.get("valid_frames", 0))

        if metadata.get("joint_order") != list(PIPER_JOINT_NAMES):
            raise SystemExit(f"{episode_dir}: joint_order does not match PiPER schema")
        if int(summary.get("valid_frames", 0)) <= 0:
            raise SystemExit(f"{episode_dir}: no valid frames")
        if int(summary.get("invalid_frames", 0)) != 0:
            raise SystemExit(f"{episode_dir}: invalid frames remain")
        if int(summary.get("dropped_command_samples", 0)) != 0:
            raise SystemExit(f"{episode_dir}: dropped command samples remain")

        first_row = next(_iter_jsonl(episode_dir / "frames.jsonl"))
        if len(first_row.get("state", [])) != 7 or len(first_row.get("action", [])) != 7:
            raise SystemExit(f"{episode_dir}: first row is not 7D state/action")

        reports.append(
            {
                "episode_id": metadata.get("episode_id", episode_dir.name),
                "phase_id": phase_id,
                "instruction": metadata.get("instruction"),
                "phase_prompt": metadata.get("phase_prompt"),
                "valid_frames": int(summary.get("valid_frames", 0)),
                "action_source": metadata.get("action_source"),
                "path": str(episode_dir),
            }
        )

    missing = [phase for phase in EXPECTED_PHASE_ORDER if phase_counts[phase] == 0]
    if missing:
        raise SystemExit(f"missing expected phases: {missing}")

    reports.sort(key=lambda report: EXPECTED_PHASE_ORDER.index(str(report["phase_id"])))
    return episode_dirs, reports, dict(sorted(phase_counts.items())), total_frames


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Create dev-only PiPER checkpoint metadata from recorded phase demos. "
            "The result is intentionally not eligible for physical execution."
        )
    )
    parser.add_argument(
        "--episode-root",
        default="piper-on-bunker/data/local/openpi_episodes",
        help="Directory containing raw recorded episode directories.",
    )
    parser.add_argument(
        "--output-dir",
        default="piper-on-bunker/data/local/openpi_smoke_checkpoint",
        help="Directory where piper_checkpoint_metadata.json will be written.",
    )
    parser.add_argument("--control-frequency-hz", type=float, default=20.0)
    parser.add_argument("--action-horizon", type=int, default=10)
    args = parser.parse_args()

    episode_root = Path(args.episode_root)
    output_dir = Path(args.output_dir)
    episode_dirs, reports, phase_counts, total_frames = _summarize_episodes(episode_root)
    dataset_hash = _hash_episode_files(episode_dirs)
    statistics_hash = "sha256:" + hashlib.sha256((dataset_hash + ":smoke-normalization-placeholder").encode()).hexdigest()
    report_hash = "sha256:" + hashlib.sha256((dataset_hash + ":not-offline-validated").encode()).hexdigest()

    metadata = {
        "schema_version": "piper_openpi_checkpoint_metadata.v1",
        "checkpoint": "dev://recorded-openpi-smoke/one-four-phase-demo",
        "base_checkpoint": OPENPI_BASE_CHECKPOINT,
        "openpi_commit": OPENPI_COMMIT,
        "piper_compatible": False,
        "physical_execution_allowed": False,
        "compatibility_reason": (
            "Recorded demonstration smoke artifact only. This is not a trained OpenPI pi0.5 checkpoint, "
            "has no computed PiPER normalization asset, and has not passed offline policy validation."
        ),
        "action_semantics": PIPER_ACTION_SEMANTICS,
        "joint_names": list(PIPER_JOINT_NAMES),
        "action_dim": 7,
        "action_horizon": int(args.action_horizon),
        "control_frequency_hz": float(args.control_frequency_hz),
        "units": {"arm": "rad", "gripper": "total_jaw_opening_m"},
        "normalization_metadata": {
            "asset_id": "piper_recorded_demo_smoke_only",
            "statistics_hash": statistics_hash,
            "source": "placeholder hash over recorded demos; not a trained-policy normalization asset",
        },
        "dataset_provenance": {
            "path": str(episode_root),
            "episode_count": len(reports),
            "frame_count": total_frames,
            "phase_counts": phase_counts,
            "episodes": reports,
            "dataset_hash": dataset_hash,
        },
        "camera_schema": {
            "observation/exterior_image": {"shape": [224, 224, 3], "dtype": "uint8"},
            "observation/wrist_image": {
                "shape": [224, 224, 3],
                "dtype": "uint8",
                "note": "recorded with --no-wrist for smoke dataset; runtime may substitute a zero wrist image",
            },
        },
        "offline_validation": {
            "passed": False,
            "report_hash": report_hash,
            "reason": "no trained PiPER policy has been evaluated on held-out episodes",
        },
        "gripper_hardware_verification": {
            "passed": False,
            "report_hash": "sha256:" + hashlib.sha256((dataset_hash + ":gripper-not-proven").encode()).hexdigest(),
            "reason": "standalone learned gripper execution is not verified for this smoke artifact",
        },
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "piper_checkpoint_metadata.json"
    output_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "metadata_path": str(output_path),
                "episode_count": len(reports),
                "total_valid_frames": total_frames,
                "phase_counts": phase_counts,
                "piper_compatible": False,
                "physical_execution_allowed": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
