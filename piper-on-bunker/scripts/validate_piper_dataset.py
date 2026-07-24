from __future__ import annotations

import argparse
import json

from piper_on_bunker.data.dataset_validator import summarize_dataset, validate_episode_directory


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate a local PiPER X-VLA demonstration dataset.")
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--episode")
    args = parser.parse_args()
    if args.episode:
        result = validate_episode_directory(args.episode)
        print(json.dumps({"ok": result.ok, "issues": result.issues, "frame_count": result.frame_count}, indent=2))
        return
    summary = summarize_dataset(args.dataset_root)
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
