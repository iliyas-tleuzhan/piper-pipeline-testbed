#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path


PIPER_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PIPER_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))


EXPECTED_PHASE_ORDER = ["approach", "grasp", "transport", "release"]
EXPECTED_JOINT_ORDER = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]


def _load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _iter_jsonl(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            line = line.strip()
            if line:
                yield line_number, json.loads(line)


def _inspect_episode(path: Path, *, max_rows: int | None) -> tuple[dict, list[str], list[dict]]:
    errors: list[str] = []
    metadata_path = path / "metadata.json"
    summary_path = path / "summary.json"
    frames_path = path / "frames.jsonl"
    images_path = path / "images"

    if not metadata_path.exists():
        errors.append("missing metadata.json")
        metadata = {}
    else:
        metadata = _load_json(metadata_path)

    if not summary_path.exists():
        errors.append("missing summary.json")
        summary = {}
    else:
        summary = _load_json(summary_path)

    if not frames_path.exists():
        errors.append("missing frames.jsonl")
        frame_rows: list[dict] = []
    else:
        frame_rows = []
        last_timestamp = -float("inf")
        for line_number, row in _iter_jsonl(frames_path):
            if max_rows is not None and len(frame_rows) >= max_rows:
                break
            frame_rows.append(row)
            if row.get("valid") is not True:
                errors.append(f"frame {line_number}: invalid row: {row.get('invalid_reason')}")
            state = row.get("state")
            action = row.get("action")
            if not isinstance(state, list) or len(state) != 7:
                errors.append(f"frame {line_number}: state is not 7D")
            if not isinstance(action, list) or len(action) != 7:
                errors.append(f"frame {line_number}: action is not 7D")
            timestamp = float(row.get("timestamp_s", -float("inf")))
            if timestamp <= last_timestamp:
                errors.append(f"frame {line_number}: timestamps are not strictly monotonic")
            last_timestamp = timestamp
            for image_key in ("exterior_image", "wrist_image"):
                image_rel = row.get(image_key)
                if not isinstance(image_rel, str) or not (path / image_rel).exists():
                    errors.append(f"frame {line_number}: missing {image_key} file")

    joint_order = metadata.get("joint_order")
    if joint_order != EXPECTED_JOINT_ORDER:
        errors.append(f"joint_order mismatch: {joint_order!r}")

    if int(summary.get("valid_frames", 0)) <= 0:
        errors.append("summary reports no valid frames")
    if int(summary.get("invalid_frames", 0)) != 0:
        errors.append(f"summary reports invalid_frames={summary.get('invalid_frames')}")
    if int(summary.get("dropped_command_samples", 0)) != 0:
        errors.append(f"summary reports dropped_command_samples={summary.get('dropped_command_samples')}")
    if frames_path.exists() and max_rows is None:
        counted = sum(1 for _ in _iter_jsonl(frames_path))
        if counted != int(summary.get("frames_recorded", counted)):
            errors.append(f"frames.jsonl count {counted} differs from summary frames_recorded={summary.get('frames_recorded')}")

    report = {
        "episode_dir": str(path),
        "episode_id": metadata.get("episode_id", path.name),
        "phase_id": metadata.get("phase_id"),
        "valid_frames": summary.get("valid_frames", 0),
        "invalid_frames": summary.get("invalid_frames", 0),
        "dropped_command_samples": summary.get("dropped_command_samples", 0),
        "image_dir_exists": images_path.exists(),
        "ok": not errors,
    }
    return report, errors, frame_rows


def _write_lerobot_like_dataset(output_dir: Path, episode_reports: list[dict], rows_by_episode: dict[str, list[dict]]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    dataset_rows = []
    for episode_index, report in enumerate(episode_reports):
        episode_id = report["episode_id"]
        for row in rows_by_episode[episode_id]:
            if not row.get("valid", False):
                continue
            dataset_rows.append(
                {
                    "episode_index": episode_index,
                    "episode_id": episode_id,
                    "frame_index": row["frame_index"],
                    "task": row["phase_prompt"],
                    "instruction": row["instruction"],
                    "phase_id": row["phase_id"],
                    "state": row["state"],
                    "actions": row["action"],
                    "timestamp_s": row["timestamp_s"],
                    "exterior_image": row["exterior_image"],
                    "wrist_image": row["wrist_image"],
                }
            )

    features = {
        "observation/exterior_image": {"dtype": "image", "shape": [224, 224, 3]},
        "observation/wrist_image": {"dtype": "image", "shape": [224, 224, 3]},
        "observation/state": {"dtype": "float32", "shape": [7], "names": EXPECTED_JOINT_ORDER},
        "actions": {"dtype": "float32", "shape": [7], "names": EXPECTED_JOINT_ORDER},
        "task": {"dtype": "string"},
        "phase_id": {"dtype": "string"},
    }
    (output_dir / "features.json").write_text(json.dumps(features, indent=2, sort_keys=True), encoding="utf-8")
    (output_dir / "episodes.json").write_text(json.dumps(episode_reports, indent=2, sort_keys=True), encoding="utf-8")
    (output_dir / "data.json").write_text(json.dumps(dataset_rows, indent=2, sort_keys=True), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect raw PiPER OpenPI episode recordings.")
    parser.add_argument("episode_root", help="Episode directory or directory containing episode directories.")
    parser.add_argument("--write-lerobot-like", help="Write a lightweight JSON dataset artifact for pipeline smoke tests.")
    parser.add_argument("--max-rows", type=int, default=0, help="Inspect only the first N frame rows per episode. 0 checks all.")
    args = parser.parse_args()

    root = Path(args.episode_root)
    if not root.exists():
        raise SystemExit(f"episode path does not exist: {root}")
    single_episode = (root / "metadata.json").exists()
    if single_episode:
        episode_dirs = [root]
    else:
        episode_dirs = sorted([path for path in root.iterdir() if path.is_dir()])

    reports = []
    rows_by_episode = {}
    all_errors: dict[str, list[str]] = {}
    max_rows = args.max_rows if args.max_rows > 0 else None
    for episode_dir in episode_dirs:
        report, errors, rows = _inspect_episode(episode_dir, max_rows=max_rows)
        reports.append(report)
        rows_by_episode[report["episode_id"]] = rows
        if errors:
            all_errors[report["episode_id"]] = errors

    phase_counts = Counter(report["phase_id"] for report in reports)
    ordered_phases = [report["phase_id"] for report in sorted(reports, key=lambda item: item["episode_dir"])]
    missing_phases = [phase for phase in EXPECTED_PHASE_ORDER if phase_counts[phase] == 0]
    if missing_phases and not single_episode:
        all_errors["_phase_set"] = [f"missing phases: {missing_phases}"]
    if all(phase_counts[phase] == 1 for phase in EXPECTED_PHASE_ORDER):
        sorted_by_expected = sorted(reports, key=lambda item: EXPECTED_PHASE_ORDER.index(item["phase_id"]))
        reports = sorted_by_expected

    if args.write_lerobot_like:
        _write_lerobot_like_dataset(Path(args.write_lerobot_like), reports, rows_by_episode)

    result = {
        "episode_count": len(reports),
        "phase_counts": dict(sorted(phase_counts.items())),
        "ordered_phases_by_directory": ordered_phases,
        "total_valid_frames": sum(int(report["valid_frames"]) for report in reports),
        "reports": reports,
        "errors": all_errors,
        "ok": not all_errors,
        "lerobot_like_output": args.write_lerobot_like,
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if not all_errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
