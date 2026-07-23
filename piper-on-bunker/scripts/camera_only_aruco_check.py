from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.models import Pose, Target
from piper_on_bunker.perception.external_realsense import ExternalFixedCamera


def _clean(value: Any) -> Any:
    if isinstance(value, Pose):
        return value.__dict__
    if isinstance(value, Target):
        return {
            "label": value.label,
            "confidence": value.confidence,
            "pixel": value.pixel,
            "depth_m": value.depth_m,
            "camera_pose": _clean(value.camera_pose),
            "base_pose": _clean(value.base_pose),
        }
    if isinstance(value, dict):
        return {k: _clean(v) for k, v in value.items() if not k.startswith("_")}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only table_camera ArUco detection check")
    parser.add_argument("--camera-name", default="table_camera")
    parser.add_argument("--color-topic", default="/table_camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/table_camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--camera-info-topic", default="/table_camera/color/camera_info")
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--marker-id", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--max-color-depth-delta-s", type=float, default=0.08)
    parser.add_argument("--label", default="marked_button")
    args = parser.parse_args()

    camera = ExternalFixedCamera(
        camera_name=args.camera_name,
        color_topic=args.color_topic,
        depth_topic=args.depth_topic,
        camera_info_topic=args.camera_info_topic,
        timeout_s=args.timeout,
        max_color_depth_delta_s=args.max_color_depth_delta_s,
        marker_id=args.marker_id,
        aruco_dictionary=args.dictionary,
    )
    observation = camera.capture_observation()
    target = camera.detect_target(observation, args.label)
    payload = {
        "success": target is not None,
        "status_code": "OK" if target is not None else "TARGET_NOT_FOUND",
        "camera_name": observation.camera_name,
        "camera_frame": observation.frame_id,
        "observation": _clean(observation.metadata),
        "marker_id": args.marker_id,
        "dictionary": args.dictionary,
        "target": _clean(target),
        "motion_command_sent": False,
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if target is not None else 2


if __name__ == "__main__":
    raise SystemExit(main())
