from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parent
SRC_ROOT = PACKAGE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.policies.lap_policy import (
    MODEL_NAME,
    SHADOW_BANNER,
    LapWebsocketClient,
    action_to_target_pose,
    build_lap_request,
    capture_live_snapshot,
    default_log_path,
    load_lap_config,
    preview_or_execute,
    write_result,
)


def resolve_config_path(raw: str) -> Path:
    candidate = Path(raw)
    if candidate.is_absolute():
        return candidate
    repo_candidate = REPO_ROOT / candidate
    if repo_candidate.exists():
        return repo_candidate
    package_candidate = PACKAGE_ROOT / candidate
    if package_candidate.exists():
        return package_candidate
    return repo_candidate


def main() -> int:
    parser = argparse.ArgumentParser(description="Run one LAP-3B PiPER action in shadow or execute mode.")
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--config", default="config/lap_piper.yaml")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--max-actions", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--output-json", type=Path)
    args = parser.parse_args()

    config = load_lap_config(resolve_config_path(args.config))
    snapshot = capture_live_snapshot(config, timeout_s=args.timeout)
    request = build_lap_request(snapshot, args.instruction)
    remote = LapWebsocketClient(config.host, config.port, timeout_s=args.timeout).infer(request)
    converted = action_to_target_pose(snapshot, remote["response"], config, max_actions=args.max_actions)
    execution = preview_or_execute(
        converted["pose"],
        execute=args.execute,
        speed=config.max_speed_scaling,
        acceleration=config.max_acceleration_scaling,
    )
    result = {
        "model": MODEL_NAME,
        "instruction": args.instruction,
        "execution_allowed": bool(args.execute),
        "shadow_banner": None if args.execute else SHADOW_BANNER,
        "server_metadata": remote["metadata"],
        "raw_model_output": remote["response"],
        "converted_action": {
            key: value for key, value in converted.items() if key != "pose"
        },
        "moveit_result": execution,
    }
    output = args.output_json or default_log_path()
    write_result(output, result)
    if not args.execute:
        print(SHADOW_BANNER)
    print(json.dumps(result, indent=2, sort_keys=True, default=str))
    print(f"Saved LAP action result: {output}")
    return 0 if execution["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
