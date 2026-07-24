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


def main() -> None:
    parser = argparse.ArgumentParser(description="Read-only summary of a recorded PiPER demonstration dataset.")
    parser.add_argument("--dataset-root", required=True)
    args = parser.parse_args()
    root = Path(args.dataset_root)
    episodes = []
    for episode_file in sorted(root.glob("episode_*/episode.json")):
        payload = json.loads(episode_file.read_text(encoding="utf-8"))
        episodes.append(
            {
                "episode_index": payload["episode_index"],
                "task_instruction": payload["task_instruction"],
                "success": payload["success"],
                "aborted": payload["aborted"],
                "frame_count": payload["frame_count"],
            }
        )
    print(json.dumps({"dataset_root": str(root), "episodes": episodes}, indent=2))


if __name__ == "__main__":
    main()
