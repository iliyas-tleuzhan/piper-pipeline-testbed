import json

import numpy as np

from piper_on_bunker.data.action_schema import EndPoseMetadata, PiperCommandSample, PiperFrameRecord, PiperStateSample
from piper_on_bunker.data.episode_writer import EpisodeWriter


def make_frame(frame_index: int) -> PiperFrameRecord:
    return PiperFrameRecord(
        frame_index=frame_index,
        task_instruction="Move the gripper toward the marked target.",
        camera_path="",
        camera_topic="/table_camera/color/image_raw",
        camera_frame_id="table_camera_color_optical_frame",
        image_timestamp_s=1.0,
        image_age_s=0.01,
        state=PiperStateSample(
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            joint_positions_rad=[0.0, 0.1, -0.2, 0.3, -0.4, 0.5],
            gripper_value=0.01,
            timestamp_s=1.0,
            age_s=0.01,
        ),
        command=PiperCommandSample(
            command_joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            target_joint_positions_rad=[0.01, 0.11, -0.19, 0.31, -0.39, 0.51],
            target_gripper_value=0.01,
            timestamp_s=1.1,
            source="test",
            max_velocity=0.05,
            max_acceleration=0.05,
            moveit_service="/joint_moveit_ctrl_piper",
        ),
        end_pose=EndPoseMetadata(
            position_m=[0.1, 0.0, 0.2],
            quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
            timestamp_s=1.0,
            frame_id="piper_base",
        ),
        receive_timestamp_s=1.2,
    )


def test_episode_writer_saves_episode(tmp_path):
    writer = EpisodeWriter(tmp_path / "dataset")
    session = writer.start_episode("Move the gripper toward the marked target.")
    image = np.zeros((64, 96, 3), dtype=np.uint8)
    session.append_frame(make_frame(0), image)
    episode_dir = session.save(success=True, software={"mode": "test"})
    payload = json.loads((episode_dir / "episode.json").read_text(encoding="utf-8"))
    assert payload["frame_count"] == 1
    assert payload["success"] is True
    assert (episode_dir / payload["frames"][0]["camera_path"]).exists()


def test_episode_writer_discard(tmp_path):
    writer = EpisodeWriter(tmp_path / "dataset")
    session = writer.start_episode("Move the gripper toward the marked target.")
    session.discard()
    assert not session.episode_dir.exists()
