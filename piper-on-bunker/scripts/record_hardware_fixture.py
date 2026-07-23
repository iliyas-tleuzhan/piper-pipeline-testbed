import argparse
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="Record a PiPER hardware fixture later on the PiPER laptop.")
    parser.add_argument("--output", default="piper-on-bunker/fixtures/hardware/session.json")
    args = parser.parse_args()
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    payload = {"note": "placeholder schema; run on ROS laptop to fill joint states, end pose, camera info, frames, transforms, API responses, timestamps"}
    Path(args.output).write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
