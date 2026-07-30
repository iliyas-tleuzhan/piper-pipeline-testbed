#!/usr/bin/env python3
"""Write non-executable PiPER-X smoke checkpoint metadata for protocol tests."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "piper-on-bunker" / "src"
sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.policies.robot_checkpoint_metadata import ROBOT_CHECKPOINT_SCHEMA_V2
from piper_on_bunker.profiles.piper_x_aruco import load_piper_x_profile


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--profile", default="piper-on-bunker/config/openpi_piper_x_touch_aruco.yaml")
    parser.add_argument("--dataset-id", default="dev://piper-x-aruco-smoke")
    args = parser.parse_args()

    profile = load_piper_x_profile(args.profile)
    metadata = {
        "schema_version": ROBOT_CHECKPOINT_SCHEMA_V2,
        "checkpoint": "dev://piper-x-aruco/no-trained-checkpoint",
        "robot_profile_id": profile.robot_profile_id,
        "robot_model": profile.robot_model,
        "task_id": profile.task_id,
        "action_semantics": profile.action_semantics,
        "action_dim": 7,
        "joint_order": list(profile.joint_order),
        "state_units": dict(profile.raw["state_units"]),
        "action_units": dict(profile.raw["action_units"]),
        "control_frequency_hz": profile.control_frequency_hz,
        "camera_schema": dict(profile.raw["camera"]["schema"]),
        "camera_mount_id": profile.camera_mount_id,
        "image_preprocessing_id": profile.preprocessing_id,
        "gripper_mode": profile.gripper_mode,
        "dataset_id": args.dataset_id,
        "dataset_hash": None,
        "dataset_manifest_hash": None,
        "normalization_asset_id": profile.raw["normalization_asset_id"],
        "normalization": {"hash": None},
        "openpi_commit": profile.raw["openpi_commit"],
        "training_configuration": "pi05_piper_x_touch_aruco",
        "offline_validation": {"passed": False},
        "hardware_verification": {"passed": False},
        "piper_x_compatible": False,
        "physical_execution_allowed": False,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2, sort_keys=True)
        f.write("\n")
    print(json.dumps(metadata, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
