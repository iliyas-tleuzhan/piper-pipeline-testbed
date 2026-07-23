from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "piper-on-bunker" / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.policies.openvla_shadow_policy import (
    DEFAULT_ENDPOINT,
    DEFAULT_IMAGE_TOPIC,
    DEFAULT_TIMEOUT_S,
    SHADOW_BANNER,
    OpenVLARequestRecord,
    OpenVLAShadowPolicyClient,
    default_log_path,
    encode_image_file,
    load_saved_state_metadata,
    read_live_color_frame,
    read_live_state,
    save_shadow_result,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="OpenVLA shadow-only action preview client")
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--image", type=Path)
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--state-json", type=Path)
    parser.add_argument("--no-state-metadata", action="store_true")
    args = parser.parse_args()

    print(SHADOW_BANNER)

    if args.image:
        image_data_uri = encode_image_file(args.image)
        image_stamp = None
        camera_metadata = {
            "source": str(args.image),
            "image_model_input": True,
        }
    else:
        image_data_uri, image_stamp, camera_metadata = read_live_color_frame(args.image_topic, args.timeout)

    joint_state = None
    end_pose = None
    if args.state_json:
        joint_state, end_pose = load_saved_state_metadata(args.state_json)
    elif not args.no_state_metadata:
        try:
            joint_state, end_pose = read_live_state(args.timeout)
        except Exception as exc:
            print(f"warning: failed to read PiPER state metadata: {exc}", file=sys.stderr)

    record = OpenVLARequestRecord(
        instruction=args.instruction,
        image_data_uri=image_data_uri,
        source_timestamp_s=image_stamp,
        camera_metadata=camera_metadata,
        piper_state_metadata=joint_state,
        end_pose_metadata=end_pose,
    )
    client = OpenVLAShadowPolicyClient(args.endpoint, args.timeout)
    result = client.preview_action(record)
    result["shadow_banner"] = SHADOW_BANNER
    result["execution_allowed"] = False
    output_path = args.output_json or default_log_path(REPO_ROOT)
    save_shadow_result(output_path, record, result)
    print(json.dumps(result, indent=2, sort_keys=True))
    print(f"Saved shadow result: {output_path}")
    return 0 if result.get("request_success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
