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
    get_motion_profile,
    get_moveit_current_tcp_pose,
    horizon_to_trajectory_plan,
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
    parser.add_argument("--max-actions", type=int, default=16, help="Number of LAP horizon rows to consume for one continuous motion.")
    parser.add_argument("--motion-profile", choices=("safe", "normal", "fast"))
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--output-json", type=Path)
    args = parser.parse_args()

    config = load_lap_config(resolve_config_path(args.config))
    motion_profile = get_motion_profile(config, args.motion_profile)
    snapshot = capture_live_snapshot(config, timeout_s=args.timeout)
    moveit_tcp_pose = get_moveit_current_tcp_pose()
    request = build_lap_request(snapshot, args.instruction)
    remote = LapWebsocketClient(config.host, config.port, timeout_s=args.timeout).infer(request)
    converted = action_to_target_pose(snapshot, moveit_tcp_pose, remote["response"], config, max_actions=args.max_actions)
    trajectory_plan = horizon_to_trajectory_plan(snapshot, moveit_tcp_pose, remote["response"], config, max_actions=args.max_actions)
    execution = preview_or_execute(
        trajectory_plan,
        execute=args.execute,
        motion_profile=motion_profile,
        config=config,
    )
    result = {
        "model": MODEL_NAME,
        "instruction": args.instruction,
        "execution_allowed": bool(args.execute),
        "shadow_banner": None if args.execute else SHADOW_BANNER,
        "motion_profile": motion_profile.name,
        "server_metadata": remote["metadata"],
        "raw_model_output": remote["response"],
        "converted_action": {
            key: value for key, value in converted.items() if key != "pose"
        },
        "combined_trajectory_request": {
            "action_semantics": trajectory_plan.action_semantics,
            "lap_horizon_length": trajectory_plan.lap_horizon_length,
            "selected_horizon_length": trajectory_plan.selected_horizon_length,
            "total_requested_tcp_displacement_m": trajectory_plan.total_requested_tcp_displacement_m,
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
