from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from _bootstrap import add_repo_src_to_syspath
else:
    from ._bootstrap import add_repo_src_to_syspath

add_repo_src_to_syspath()

from piper_on_bunker.data.dataset_validator import summarize_dataset, validate_episode_directory


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate a local PiPER X-VLA demonstration dataset.")
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--episode")
    parser.add_argument(
        "--allow-nonphysical",
        action="store_true",
        help="Allow synthetic or dry-run fixtures that were not physically executed and verified.",
    )
    args = parser.parse_args()
    require_physical_execution = not args.allow_nonphysical
    if args.episode:
        result = validate_episode_directory(args.episode, require_physical_execution=require_physical_execution)
        print(json.dumps({"ok": result.ok, "issues": result.issues, "frame_count": result.frame_count}, indent=2))
        return
    summary = summarize_dataset(args.dataset_root, require_physical_execution=require_physical_execution)
    print(
        json.dumps(
            {
                "total_episodes": summary.total_episodes,
                "valid_episodes": summary.valid_episodes,
                "invalid_episodes": summary.invalid_episodes,
                "total_frames": summary.total_frames,
                "issues": summary.issues,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
