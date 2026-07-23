from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "piper-on-bunker" / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.policies.smolvla_shadow_policy import (  # noqa: E402
    DEFAULT_ENDPOINT,
    DEFAULT_TIMEOUT_S,
    SHADOW_BANNER,
    SmolVLAShadowPolicyClient,
    build_action_preview_request,
    default_log_path,
    encode_image_file,
    read_live_color_image,
    read_live_joint_state,
    read_state_json,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="SmolVLA shadow-only PiPER compatibility client")
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--image", action="append", help="PNG/JPEG path. May be repeated one or two times.")
    parser.add_argument("--state-json", type=Path)
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
    parser.add_argument("--output-json", type=Path)
    args = parser.parse_args()

    print(SHADOW_BANNER)
    if args.image:
        images = [encode_image_file(path) for path in args.image]
        image_metadata = [{"source": str(path)} for path in args.image]
    else:
        image, metadata = read_live_color_image(args.timeout)
        images = [image]
        image_metadata = [metadata]

    joint_state = read_state_json(args.state_json) if args.state_json else read_live_joint_state(args.timeout)
    payload = build_action_preview_request(
        images=images,
        task_description=args.instruction,
        joint_state=joint_state,
        timestamps={"joint_state_stamp_s": joint_state.stamp_s},
        source_metadata={"images": image_metadata},
    )
    client = SmolVLAShadowPolicyClient(args.endpoint, args.timeout)
    result = client.preview_action(payload)
    result["shadow_banner"] = SHADOW_BANNER
    result["joint_state"] = joint_state.to_log_dict()
    result["execution_allowed"] = False
    output_path = args.output_json or default_log_path(REPO_ROOT)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    print(f"Saved shadow result: {output_path}")
    return 0 if result.get("request_success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
