import json

import pytest

from piper_on_bunker.mission_logging import MissionLogger
from piper_on_bunker.models import Pose
from piper_on_bunker.safety import validate_press_distance, validate_workspace
from piper_on_bunker.transforms import StaticTransform


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


def test_jsonl_mission_logger(tmp_path):
    path = tmp_path / "mission.jsonl"
    logger = MissionLogger(str(path))
    logger.append({"mission_id": "m1", "event": "started"})
    logger.append({"mission_id": "m1", "event": "done"})
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert [row["event"] for row in rows] == ["started", "done"]
