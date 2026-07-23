import json

import pytest

from piper_on_bunker.mission_logging import MissionLogger
from piper_on_bunker.models import Observation, Pose, Target, StatusCode
from piper_on_bunker.hardware.mock_arm import MockArm
from piper_on_bunker.mission_supervisor import MissionSupervisor
from piper_on_bunker.safety import validate_press_distance, validate_workspace
from piper_on_bunker.transforms import StaticTransform, TransformResolver


def test_workspace_rejects_out_of_bounds():
    safety = {"workspace_bounds_m": {"x": [0.1, 0.5], "y": [-0.2, 0.2], "z": [0.02, 0.3]}}
    with pytest.raises(ValueError):
        validate_workspace(Pose(0.8, 0.0, 0.1), safety)


def test_excessive_press_distance_rejected():
    with pytest.raises(ValueError):
        validate_press_distance(0.05, {"max_press_distance_m": 0.015})


def test_static_transform_identity_translation():
    tf = StaticTransform("camera", "piper_base", [0.1, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    out = tf.transform_pose(Pose(0.2, 0.0, 0.1, frame_id="camera"))
    assert out.frame_id == "piper_base"
    assert out.x == pytest.approx(0.3)
    assert out.z == pytest.approx(0.1)


def test_static_transform_90_degree_rotation():
    tf = StaticTransform("camera", "piper_base", [0.0, 0.0, 0.0], [0.0, 0.0, 0.70710678, 0.70710678])
    out = tf.transform_pose(Pose(1.0, 0.0, 0.0, frame_id="camera"))
    assert out.x == pytest.approx(0.0, abs=1e-6)
    assert out.y == pytest.approx(1.0, abs=1e-6)


def test_wrong_source_frame_rejected():
    tf = StaticTransform("camera", "piper_base", [0, 0, 0], [0, 0, 0, 1])
    with pytest.raises(ValueError):
        tf.transform_pose(Pose(0, 0, 0, frame_id="wrong"))


def test_invalid_quaternion_rejected():
    with pytest.raises(ValueError):
        StaticTransform("camera", "piper_base", [0, 0, 0], [0, 0, 0, 0])


def test_missing_transform_rejected():
    with pytest.raises(ValueError):
        TransformResolver({}).transform_pose(Pose(0, 0, 0, frame_id="camera"), "piper_base")


def test_stale_transform_rejected():
    resolver = TransformResolver({"cam_to_base": {"source_frame": "camera", "target_frame": "piper_base", "translation_m": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1], "recorded_at": "2020-01-01T00:00:00+00:00", "max_age_s": 1}})
    with pytest.raises(ValueError):
        resolver.transform_pose(Pose(0, 0, 0, frame_id="camera"), "piper_base")


class CameraOnlyTarget:
    def capture_observation(self):
        return Observation("test", "camera", "now", metadata={"color_age_s": 0.0})

    def detect_target(self, observation, label):
        return Target(label=label, confidence=1.0, pixel=(10, 10), depth_m=0.2, camera_pose=Pose(0.1, 0.0, 0.1, frame_id="camera"))


def test_successful_camera_to_piper_target_conversion():
    resolver = TransformResolver({"cam_to_base": {"source_frame": "camera", "target_frame": "piper_base", "translation_m": [0.2, 0.0, 0.0], "quaternion_xyzw": [0, 0, 0, 1]}})
    supervisor = MissionSupervisor(MockArm(), CameraOnlyTarget(), transform_resolver=resolver, planning_frame="piper_base")
    assert supervisor.detect_target().success
    result = supervisor.estimate_target_pose()
    assert result.success
    assert supervisor.last_target.base_pose.frame_id == "piper_base"
    assert supervisor.last_target.base_pose.x == pytest.approx(0.3)


def test_transformed_target_outside_workspace_rejected():
    resolver = TransformResolver({"cam_to_base": {"source_frame": "camera", "target_frame": "piper_base", "translation_m": [2.0, 0.0, 0.0], "quaternion_xyzw": [0, 0, 0, 1]}})
    supervisor = MissionSupervisor(
        MockArm(),
        CameraOnlyTarget(),
        transform_resolver=resolver,
        planning_frame="piper_base",
        safety={"workspace_bounds_m": {"x": [0.1, 0.5], "y": [-0.2, 0.2], "z": [0.02, 0.3]}},
    )
    assert supervisor.detect_target().success
    assert supervisor.estimate_target_pose().success
    result = supervisor.validate_target()
    assert not result.success
    assert result.status_code == StatusCode.SAFETY_VIOLATION


def test_jsonl_mission_logger(tmp_path):
    path = tmp_path / "mission.jsonl"
    logger = MissionLogger(str(path))
    logger.append({"mission_id": "m1", "event": "started"})
    logger.append({"mission_id": "m1", "event": "done"})
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert [row["event"] for row in rows] == ["started", "done"]
