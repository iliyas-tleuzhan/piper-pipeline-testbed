from __future__ import annotations

import argparse
import json

from piper_on_bunker.models import StatusCode
from piper_on_bunker.factory import build_supervisor_from_path


def run_with_optional_manual_verification(supervisor, prompt: bool):
    result = supervisor.run_button_mission()
    if result.status_code == StatusCode.WAITING_FOR_VERIFICATION and prompt:
        answer = input("Did the button press succeed? [y/N] ").strip().lower()
        if answer in {"y", "yes"}:
            supervisor.acknowledge_manual_verification()
            result = supervisor.verify_task()
            if result.success:
                result = supervisor.return_to_navigation_view()
        else:
            supervisor.reject_manual_verification()
            result = supervisor.verify_task()
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="PiPER-on-Bunker pipeline CLI")
    parser.add_argument("command", nargs="?", default="status", choices=["status", "mock-demo", "replay-demo", "read-only", "dry-demo", "live-dry-run", "hardware-demo", "stop"])
    parser.add_argument("--config", default="piper-on-bunker/config/development_mock.yaml")
    args = parser.parse_args()
    supervisor = build_supervisor_from_path(args.config)
    if args.command == "status" or args.command == "read-only":
        result = supervisor.get_robot_state()
    elif args.command in {"mock-demo", "replay-demo", "dry-demo", "live-dry-run"}:
        result = run_with_optional_manual_verification(supervisor, prompt=False)
    elif args.command == "hardware-demo":
        result = run_with_optional_manual_verification(supervisor, prompt=True)
    elif args.command == "stop":
        result = supervisor.stop_motion()
    else:
        raise ValueError(args.command)
    print(json.dumps(result.__dict__, default=lambda obj: getattr(obj, "__dict__", str(obj)), indent=2))


if __name__ == "__main__":
    main()
