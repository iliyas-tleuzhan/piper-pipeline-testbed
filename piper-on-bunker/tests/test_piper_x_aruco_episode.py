from pathlib import Path

import numpy as np

from piper_on_bunker.data.piper_x_aruco_episode import (
    append_frame,
    convert_to_lerobot_scaffold,
    inspect_episode,
    label_episode_outcome,
    write_episode_splits,
    write_json_atomic,
)
from piper_on_bunker.profiles.piper_x_aruco import (
    PIPER_X_ACTION_SEMANTICS,
    PIPER_X_JOINT_ORDER,
    PIPER_X_PROFILE_ID,
    PIPER_X_TASK_ID,
    load_piper_x_profile,
)


PROFILE = Path("piper-on-bunker/config/openpi_piper_x_touch_aruco.yaml")


def _make_episode(root: Path, name: str = "episode") -> Path:
    profile = load_piper_x_profile(PROFILE)
    ep = root / name
    (ep / "images" / "wrist").mkdir(parents=True)
    (ep / "images" / "exterior").mkdir(parents=True)
    metadata = {
        "robot_profile_id": PIPER_X_PROFILE_ID,
        "robot_model": profile.robot_model,
        "task_id": PIPER_X_TASK_ID,
        "camera_schema": dict(profile.raw["camera"]["schema"]),
        "joint_order": list(PIPER_X_JOINT_ORDER),
        "action_semantics": PIPER_X_ACTION_SEMANTICS,
        "fixed_gripper_target": 0.0,
    }
    write_json_atomic(ep / "episode_metadata.json", metadata)
    for idx in range(3):
        np.save(ep / "images" / "wrist" / f"{idx:06d}.npy", np.zeros((224, 224, 3), dtype=np.uint8))
        np.save(ep / "images" / "exterior" / f"{idx:06d}.npy", np.zeros((224, 224, 3), dtype=np.uint8))
        append_frame(
            ep,
            {
                "frame_index": idx,
                "sample_time_s": float(idx) * 0.05,
                "state": [0.0] * 7,
                "action": [0.01 * idx] * 6 + [0.0],
                "prompt": "Touch the center of the ArUco marker and retract.",
                "wrist_image_path": f"images/wrist/{idx:06d}.npy",
                "exterior_image_path": f"images/exterior/{idx:06d}.npy",
                "max_source_skew_s": 0.01,
                "aruco_visible": True,
                "detected_marker_ids": [6],
            },
        )
    write_json_atomic(ep / "summary.json", {"frames_recorded": 3, "episode_id": name})
    label_episode_outcome(ep, status="success", contact_confirmed=True, marker_touched=True, retraction_completed=True)
    return ep


def test_episode_inspection_and_outcome(tmp_path):
    profile = load_piper_x_profile(PROFILE)
    ep = _make_episode(tmp_path)
    result = inspect_episode(ep, expected_profile=profile.raw)
    assert result.passed is True
    assert result.report["outcome"]["status"] == "success"


def test_episode_split_is_by_episode(tmp_path):
    _make_episode(tmp_path, "ep1")
    _make_episode(tmp_path, "ep2")
    _make_episode(tmp_path, "ep3")
    splits = write_episode_splits(tmp_path, tmp_path / "splits.json", seed=123, train_fraction=0.34, val_fraction=0.33)
    all_eps = splits["train"] + splits["validation"] + splits["test"]
    assert len(all_eps) == 3
    assert len(set(all_eps)) == 3


def test_conversion_scaffold_preserves_schema(tmp_path):
    profile = load_piper_x_profile(PROFILE)
    _make_episode(tmp_path / "raw", "ep1")
    manifest = convert_to_lerobot_scaffold(tmp_path / "raw", tmp_path / "converted", profile=profile.raw)
    assert manifest["row_count"] == 3
    assert (tmp_path / "converted" / "features.json").exists()
    assert manifest["normalization_asset_id"] == "piper_x_touch_aruco_v1"
