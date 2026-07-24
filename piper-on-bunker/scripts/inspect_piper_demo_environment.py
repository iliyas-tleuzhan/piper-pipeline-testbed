from __future__ import annotations

import argparse
import json

from piper_on_bunker.data.piper_demo_recorder import RosSnapshotProvider, load_demo_config


def main() -> None:
    parser = argparse.ArgumentParser(description="Read-only PiPER/X-VLA demonstration environment inspection.")
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    config = load_demo_config(args.config)
    snapshots = RosSnapshotProvider(expected_joint_names=config.expected_joint_names)
    snapshot = snapshots.read_snapshot(
        state_timeout_s=float(config.safety.get("state_read_timeout_s", 2.0)),
        image_timeout_s=float(config.safety.get("camera_read_timeout_s", 2.0)),
        max_state_age_s=float(config.safety.get("max_state_age_s", 1.0)),
        max_image_age_s=float(config.safety.get("max_camera_age_s", 1.0)),
    )
    payload = {
        "physical_motion_enabled": config.physical_motion_enabled,
        "expected_joint_names": config.expected_joint_names,
        "joint_state": snapshot.state.to_dict(),
        "image": {
            "topic": snapshot.image_topic,
            "frame_id": snapshot.image_frame_id,
            "timestamp_s": snapshot.image_timestamp_s,
            "age_s": snapshot.image_age_s,
            "shape": list(snapshot.image_rgb.shape),
        },
        "end_pose": snapshot.end_pose.to_dict() if snapshot.end_pose else None,
    }
    print(json.dumps(payload, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
