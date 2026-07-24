from __future__ import annotations

import numpy as np

from piper_on_bunker.policies.lap_policy import (
    LapRuntimeConfig,
    LapStateSnapshot,
    MoveItCurrentTcpPose,
    action_to_target_pose,
    build_lap_request,
    clamp_translation,
    euler_to_rot6d,
    normalize_gripper_width,
    preview_or_execute,
    quaternion_to_rpy_rad,
    resize_with_pad_rgb,
)


def _config() -> LapRuntimeConfig:
    return LapRuntimeConfig(
        host="127.0.0.1",
        port=8016,
        color_image_topic="/color",
        joint_state_topic="/joint",
        end_pose_topic="/pose",
        axis_map=[0, 1, 2],
        translation_scale=1.0,
        max_translation_per_action_m=0.02,
        preserve_orientation=True,
        max_speed_scaling=0.05,
        max_acceleration_scaling=0.05,
        workspace_bounds_m={"x": [0.05, 0.60], "y": [-0.35, 0.35], "z": [0.02, 0.55]},
    )


def _snapshot() -> LapStateSnapshot:
    return LapStateSnapshot(
        image_rgb=np.zeros((10, 20, 3), dtype=np.uint8),
        image_stamp_s=1.0,
        telemetry_end_pose_position_m=[0.055984, 0.000667, 0.214169],
        telemetry_end_pose_quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
        telemetry_end_pose_rpy_rad=[0.0, 0.0, 0.0],
        joint_positions_rad=[0.0] * 6,
        gripper_m=0.03,
        joint_stamp_s=1.0,
        end_pose_stamp_s=1.0,
    )


def _moveit_pose() -> MoveItCurrentTcpPose:
    return MoveItCurrentTcpPose(
        planning_frame="dummy_link",
        end_effector_link="gripper_tcp",
        position_m=[0.19124318537468837, 0.0022620599968891843, 0.22627771846438982],
        quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
        rpy_rad=[0.0, 0.0, 0.0],
    )


def test_quaternion_to_rpy_identity():
    assert quaternion_to_rpy_rad(0.0, 0.0, 0.0, 1.0) == [0.0, 0.0, 0.0]


def test_euler_to_rot6d_size():
    rot6d = euler_to_rot6d([0.0, 0.0, 0.0])
    assert len(rot6d) == 6


def test_normalize_gripper_width():
    assert normalize_gripper_width(0.03) == 0.5


def test_resize_with_pad_rgb():
    image = np.zeros((10, 20, 3), dtype=np.uint8)
    resized = resize_with_pad_rgb(image, size=224)
    assert resized.shape == (224, 224, 3)


def test_build_lap_request_shape():
    request = build_lap_request(_snapshot(), "Move toward the red cup.")
    obs = request["observation"]
    assert np.asarray(obs["base_0_rgb"]).shape == (224, 224, 3)
    assert len(obs["cartesian_position"]) == 9
    assert len(obs["joint_position"]) == 6
    assert len(obs["gripper_position"]) == 1
    assert len(obs["state"]) == 10


def test_clamp_translation():
    delta = clamp_translation(np.array([0.04, 0.0, 0.0], dtype=float), 0.02)
    assert np.isclose(np.linalg.norm(delta), 0.02)


def test_action_to_target_pose_clamps_workspace():
    converted = action_to_target_pose(
        _snapshot(),
        _moveit_pose(),
        {"actions": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]},
        _config(),
        max_actions=1,
    )
    target = converted["proposed_tcp_target"]["position"]
    assert target[0] <= 0.60


def test_action_to_target_pose_uses_moveit_tcp_origin_not_end_pose():
    converted = action_to_target_pose(
        _snapshot(),
        _moveit_pose(),
        {"actions": [0.0028122098797273565, 0.0019796282983695623, 0.002152735114336668, 9.0, 8.0, 7.0, 0.0]},
        _config(),
        max_actions=1,
    )
    assert converted["telemetry_end_pose"]["position"] == [0.055984, 0.000667, 0.214169]
    assert converted["moveit_current_tcp_pose"]["position"] == [0.19124318537468837, 0.0022620599968891843, 0.22627771846438982]
    assert converted["proposed_tcp_target"]["position"] == [0.19405539525441573, 0.004241688295258747, 0.2284304535787265]


def test_action_to_target_pose_preserves_moveit_quaternion():
    pose = MoveItCurrentTcpPose(
        planning_frame="dummy_link",
        end_effector_link="gripper_tcp",
        position_m=[0.19124318537468837, 0.0022620599968891843, 0.22627771846438982],
        quaternion_xyzw=[0.1, 0.2, 0.3, 0.9],
        rpy_rad=[0.0, 0.0, 0.0],
    )
    converted = action_to_target_pose(
        _snapshot(),
        pose,
        {"actions": [0.001, 0.002, 0.003, 5.0, 6.0, 7.0, 0.0]},
        _config(),
        max_actions=1,
    )
    assert converted["proposed_tcp_target"]["quaternion_xyzw"] == [0.1, 0.2, 0.3, 0.9]


def test_lap_rotation_is_ignored_in_translation_only_mode():
    translated = action_to_target_pose(
        _snapshot(),
        _moveit_pose(),
        {"actions": [0.001, 0.002, 0.003, 100.0, 200.0, 300.0, 0.0]},
        _config(),
        max_actions=1,
    )
    without_rotation = action_to_target_pose(
        _snapshot(),
        _moveit_pose(),
        {"actions": [0.001, 0.002, 0.003, 0.0, 0.0, 0.0, 0.0]},
        _config(),
        max_actions=1,
    )
    assert translated["proposed_tcp_target"] == without_rotation["proposed_tcp_target"]


def test_nonfinite_lap_actions_are_rejected():
    try:
        action_to_target_pose(
            _snapshot(),
            _moveit_pose(),
            {"actions": [float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            _config(),
            max_actions=1,
        )
    except ValueError as exc:
        assert "finite" in str(exc)
    else:
        raise AssertionError("expected ValueError for non-finite LAP translation")


def test_preview_or_execute_shadow_never_calls_service(monkeypatch):
    class FakeArm:
        def __init__(self, physical_motion_enabled, safety):
            self.physical_motion_enabled = physical_motion_enabled
            self.safety = safety

        def move_to_pose(self, pose):
            return type(
                "Result",
                (),
                {
                    "success": True,
                    "status_code": type("Code", (), {"value": "OK"})(),
                    "message": "dry run",
                    "outputs": {
                        "dry_run": not self.physical_motion_enabled,
                        "pose": pose.__dict__,
                        "moveit_request_preview": {"will_call_service": self.physical_motion_enabled},
                    },
                },
            )()

    monkeypatch.setattr("piper_on_bunker.policies.lap_policy.PiperRosArm", FakeArm)
    result = preview_or_execute(
        action_to_target_pose(
            _snapshot(),
            _moveit_pose(),
            {"actions": [0.0028122098797273565, 0.0019796282983695623, 0.002152735114336668, 0.0, 0.0, 0.0, 0.0]},
            _config(),
        )["pose"],
        execute=False,
        speed=0.05,
        acceleration=0.05,
    )
    assert result["success"] is True
    assert result["outputs"]["moveit_request_preview"]["will_call_service"] is False


def test_preview_or_execute_uses_corrected_tcp_target(monkeypatch):
    seen = {}

    class FakeArm:
        def __init__(self, physical_motion_enabled, safety):
            self.physical_motion_enabled = physical_motion_enabled
            self.safety = safety

        def move_to_pose(self, pose):
            seen["pose"] = pose
            return type(
                "Result",
                (),
                {
                    "success": True,
                    "status_code": type("Code", (), {"value": "OK"})(),
                    "message": "execute preview",
                    "outputs": {"pose": pose.__dict__},
                },
            )()

    monkeypatch.setattr("piper_on_bunker.policies.lap_policy.PiperRosArm", FakeArm)
    converted = action_to_target_pose(
        _snapshot(),
        _moveit_pose(),
        {"actions": [0.0028122098797273565, 0.0019796282983695623, 0.002152735114336668, 0.0, 0.0, 0.0, 0.0]},
        _config(),
    )
    preview_or_execute(converted["pose"], execute=True, speed=0.05, acceleration=0.05)
    assert seen["pose"].x == 0.19405539525441573
    assert seen["pose"].y == 0.004241688295258747
    assert seen["pose"].z == 0.2284304535787265
    assert seen["pose"].frame_id == "dummy_link"
