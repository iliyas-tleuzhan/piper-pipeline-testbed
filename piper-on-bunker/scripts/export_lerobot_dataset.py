from __future__ import annotations

import argparse
import json

from piper_on_bunker.data.lerobot_export import export_raw_dataset_to_lerobot, validate_lerobot_dataset


def main() -> None:
    parser = argparse.ArgumentParser(description="Export a raw PiPER X-VLA dataset into the official LeRobot format.")
    parser.add_argument("--raw-root", required=True)
    parser.add_argument("--export-root", required=True)
    parser.add_argument("--repo-id", default="local/piper_xvla_target_v0")
    parser.add_argument("--fps", type=int, default=5)
    parser.add_argument("--validate", action="store_true")
    args = parser.parse_args()
    result = export_raw_dataset_to_lerobot(
        raw_root=args.raw_root,
        export_root=args.export_root,
        repo_id=args.repo_id,
        fps=args.fps,
        use_videos=False,
    )
    if args.validate:
        result["validation"] = validate_lerobot_dataset(args.export_root)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
