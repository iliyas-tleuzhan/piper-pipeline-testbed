import json
import subprocess
import sys
from pathlib import Path


def test_episode_inspector_validates_and_writes_smoke_dataset(tmp_path):
    root = tmp_path / "episodes"
    episode = root / "piper_approach_fixture"
    images = episode / "images"
    images.mkdir(parents=True)
    (images / "exterior_000000.jpg").write_bytes(b"fake")
    (images / "wrist_000000.jpg").write_bytes(b"fake")
    metadata = {
        "episode_id": "piper_approach_fixture",
        "phase_id": "approach",
        "joint_order": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"],
    }
    summary = {
        "frames_recorded": 1,
        "valid_frames": 1,
        "invalid_frames": 0,
        "dropped_command_samples": 0,
    }
    frame = {
        "frame_index": 0,
        "valid": True,
        "instruction": "Move the red cup onto the paper.",
        "phase_id": "approach",
        "phase_prompt": "Approach the red cup and finish in a grasp-ready pose.",
        "timestamp_s": 1.0,
        "state": [0.0] * 7,
        "action": [0.1] * 7,
        "exterior_image": "images/exterior_000000.jpg",
        "wrist_image": "images/wrist_000000.jpg",
    }
    (episode / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
    (episode / "summary.json").write_text(json.dumps(summary), encoding="utf-8")
    (episode / "frames.jsonl").write_text(json.dumps(frame) + "\n", encoding="utf-8")

    output = tmp_path / "dataset"
    script = Path(__file__).resolve().parents[1] / "scripts" / "inspect_openpi_piper_episode.py"
    result = subprocess.run(
        [sys.executable, str(script), str(episode), "--write-lerobot-like", str(output)],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    report = json.loads(result.stdout)
    assert report["ok"] is True
    assert report["total_valid_frames"] == 1
    assert (output / "features.json").exists()
    assert (output / "data.json").exists()
