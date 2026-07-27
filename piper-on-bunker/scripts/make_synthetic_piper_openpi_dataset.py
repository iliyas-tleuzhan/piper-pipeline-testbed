#!/usr/bin/env python3
from __future__ import annotations

import argparse

from piper_on_bunker.data.piper_openpi_dataset import make_synthetic_episode
from piper_on_bunker.data.piper_openpi_dataset import write_synthetic_lerobot_like_dataset


def main() -> int:
    parser = argparse.ArgumentParser(description="Create a synthetic PiPER OpenPI dataset fixture.")
    parser.add_argument("--output", default="piper-on-bunker/logs/openpi_synthetic_dataset")
    args = parser.parse_args()
    path = write_synthetic_lerobot_like_dataset(args.output, make_synthetic_episode())
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

