from __future__ import annotations

import json
import shutil
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
from PIL import Image

from .action_schema import (
    GRIPPER_UNIT_SOURCE,
    GRIPPER_UNITS,
    PIPER_ACTION_NAMES,
    PIPER_JOINT_NAMES,
    PiperEpisodeRecord,
    PiperFrameRecord,
    SCHEMA_VERSION,
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass
class EpisodeSession:
    writer: "EpisodeWriter"
    episode_index: int
    task_instruction: str
    episode_dir: Path
    frame_dir: Path
    notes: List[str] = field(default_factory=list)
    frames: List[PiperFrameRecord] = field(default_factory=list)

    def append_frame(self, frame: PiperFrameRecord, image_rgb: np.ndarray) -> None:
        if image_rgb.ndim != 3 or image_rgb.shape[2] != 3:
            raise ValueError("image_rgb must be HWC RGB")
        frame_name = f"frame_{frame.frame_index:06d}.jpg"
        image_path = self.frame_dir / frame_name
        self.frame_dir.mkdir(parents=True, exist_ok=True)
        Image.fromarray(image_rgb.astype("uint8")).save(image_path, format="JPEG", quality=95)
        frame.camera_path = str(image_path.relative_to(self.episode_dir))
        self.frames.append(frame)

    def add_note(self, text: str) -> None:
        self.notes.append(text.strip())

    def save(self, success: bool, aborted: bool = False, software: Optional[Dict[str, Any]] = None) -> Path:
        record = PiperEpisodeRecord(
            schema_version=SCHEMA_VERSION,
            episode_index=self.episode_index,
            task_instruction=self.task_instruction,
            success=success,
            aborted=aborted,
            frame_count=len(self.frames),
            camera_name="table_camera",
            joint_names=list(PIPER_JOINT_NAMES),
            action_names=list(PIPER_ACTION_NAMES),
            joint_units="radians",
            gripper_units=GRIPPER_UNITS,
            gripper_unit_source=GRIPPER_UNIT_SOURCE,
            action_semantics="absolute_next_joint_and_gripper_target_sent_by_manual_recorder",
            recording_method="manual_joint_step_teleop_via_moveit_service",
            source_topics={
                "camera": "/table_camera/color/image_raw",
                "joint_state": "/joint_states_single",
                "end_pose": "/end_pose",
                "bridge_command_echo": "/piper_joint_commands",
            },
            notes="\n".join(self.notes) if self.notes else None,
            created_at=_utc_now(),
            software=software or {},
            frames=list(self.frames),
        )
        payload = record.to_dict()
        (self.episode_dir / "episode.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
        self.writer._update_manifest(record)
        return self.episode_dir

    def discard(self) -> None:
        shutil.rmtree(self.episode_dir, ignore_errors=True)


class EpisodeWriter:
    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.manifest_path = self.root / "manifest.json"
        if not self.manifest_path.exists():
            self.manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": SCHEMA_VERSION,
                        "dataset_name": self.root.name,
                        "episodes": [],
                    },
                    indent=2,
                ),
                encoding="utf-8",
            )

    def next_episode_index(self) -> int:
        manifest = json.loads(self.manifest_path.read_text(encoding="utf-8"))
        episodes = manifest.get("episodes", [])
        return 0 if not episodes else int(max(item["episode_index"] for item in episodes) + 1)

    def start_episode(self, task_instruction: str) -> EpisodeSession:
        episode_index = self.next_episode_index()
        episode_dir = self.root / f"episode_{episode_index:04d}"
        episode_dir.mkdir(parents=True, exist_ok=False)
        frame_dir = episode_dir / "frames"
        return EpisodeSession(self, episode_index, task_instruction, episode_dir, frame_dir)

    def _update_manifest(self, episode: PiperEpisodeRecord) -> None:
        manifest = json.loads(self.manifest_path.read_text(encoding="utf-8"))
        episodes = [item for item in manifest.get("episodes", []) if item["episode_index"] != episode.episode_index]
        episodes.append(
            {
                "episode_index": episode.episode_index,
                "task_instruction": episode.task_instruction,
                "success": episode.success,
                "aborted": episode.aborted,
                "frame_count": episode.frame_count,
                "path": f"episode_{episode.episode_index:04d}/episode.json",
            }
        )
        episodes.sort(key=lambda item: int(item["episode_index"]))
        manifest["episodes"] = episodes
        self.manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
