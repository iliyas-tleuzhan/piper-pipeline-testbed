from __future__ import annotations

import numpy as np

from piper_on_bunker.policies.lap_policy import (
    LapRuntimeConfig,
    LapStateSnapshot,
    MotionProfile,
    MoveItCurrentTcpPose,
    action_to_target_pose,
    build_lap_request,
    clamp_translation,
    euler_to_rot6d,
    get_motion_profile,
    horizon_to_trajectory_plan,
    lap_action_semantics,
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
        max_total_translation_m=0.20,
        max_adjacent_waypoint_translation_m=0.03,
        preserve_orientation=True,
        motion_profiles={
            "safe": MotionProfile("safe", 0.20, 0.15),
            "normal": MotionProfile("normal", 0.50, 0.35),
            "fast": MotionProfile("fast", 0.80, 0.60),
        },
        default_motion_profile="fast",
        workspace_bounds_m={"x": [0.05, 0.60], "y": [-0.35, 0.35], "z": [0.02, 0.55]},
        cartesian_eef_step_m=0.005,
        cartesian_jump_threshold=1.0,
        min_cartesian_path_fraction=0.90,
        max_total_tcp_displacement_m=0.20,
        max_total_joint_delta_rad=0.75,
        max_adjacent_joint_delta_rad=0.30,
        position_tolerance_m=0.01,
        planning_time_s=10.0,
        max_state_age_s=1.0,
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
    try:
        horizon_to_trajectory_plan(
            _snapshot(),
            _moveit_pose(),
            {"actions": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]},
            _config(),
            max_actions=1,
        )
    except ValueError as exc:
        assert "max_total_translation_m" in str(exc)
    else:
        raise AssertionError("expected translation bound rejection")


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
    assert converted["action_semantics"] == lap_action_semantics()


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


def test_horizon_accepts_large_total_motion_with_small_adjacent_spacing():
    plan = horizon_to_trajectory_plan(
        _snapshot(),
        _moveit_pose(),
        {
                "actions": [
                    [0.02, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.04, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.07, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.10, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.13, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                ]
            },
            _config(),
            max_actions=16,
        )
    assert plan.selected_horizon_length == 5
    assert plan.horizon_truncated is False
    assert np.isclose(plan.total_requested_tcp_displacement_m, 0.13)
    assert np.isclose(plan.maximum_adjacent_waypoint_translation_m, 0.03)


def test_horizon_rejects_adjacent_waypoint_jump():
    try:
        horizon_to_trajectory_plan(
            _snapshot(),
            _moveit_pose(),
            {"actions": [[0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]},
            _config(),
            max_actions=16,
        )
    except ValueError as exc:
        assert "adjacent displacement" in str(exc)
        assert "1" in str(exc)
    else:
        raise AssertionError("expected adjacent waypoint rejection")


def test_horizon_truncates_to_safe_prefix_on_total_limit():
    plan = horizon_to_trajectory_plan(
        _snapshot(),
        _moveit_pose(),
        {
                "actions": [
                    [0.02, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.04, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.07, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.10, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.13, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.16, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.22, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                ]
            },
            _config(),
            max_actions=16,
        )
    assert plan.selected_horizon_length == 6
    assert plan.horizon_truncated is True
    assert plan.rejected_waypoint_index == 6
    assert plan.rejected_waypoint_reason == "max_total_translation_m"
    assert np.isclose(plan.total_requested_tcp_displacement_m, 0.22)


def test_horizon_rejects_when_total_limit_kills_useful_prefix():
    try:
        horizon_to_trajectory_plan(
            _snapshot(),
            _moveit_pose(),
            {"actions": [[0.01, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0], [0.21, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0]]},
            _config(),
            max_actions=16,
        )
    except ValueError as exc:
        assert "useful safe prefix" in str(exc)
    else:
        raise AssertionError("expected safe-prefix rejection")


class _FakeDuration:
    def __init__(self, sec):
        self._sec = float(sec)

    def to_sec(self):
        return self._sec


class _FakeTrajectoryPoint:
    def __init__(self, positions, time_from_start):
        self.positions = list(positions)
        self.time_from_start = _FakeDuration(time_from_start)


class _FakeTrajectory:
    def __init__(self, points):
        self.joint_trajectory = type("JT", (), {"points": points})()


def _install_fake_moveit(monkeypatch, *, cartesian_fraction=1.0):
    executed = {}

    class _GeomPoint:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _GeomQuat:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class _GeomPose:
        def __init__(self):
            self.position = _GeomPoint()
            self.orientation = _GeomQuat()

    class FakeGroup:
        def __init__(self, name):
            self.name = name
            self.pose_target = None

        def set_start_state_to_current_state(self):
            return None

        def set_planning_time(self, value):
            self.planning_time = value

        def set_num_planning_attempts(self, value):
            self.planning_attempts = value

        def set_pose_reference_frame(self, value):
            self.frame = value

        def set_end_effector_link(self, value):
            self.link = value

        def set_max_velocity_scaling_factor(self, value):
            self.velocity = value

        def set_max_acceleration_scaling_factor(self, value):
            self.acceleration = value

        def get_current_state(self):
            return object()

        def get_active_joints(self):
            return ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        def get_current_joint_values(self):
            return [0.0] * 6

        def set_path_constraints(self, constraints):
            executed["path_constraints"] = constraints

        def clear_path_constraints(self):
            executed["path_constraints_cleared"] = True

        def compute_cartesian_path(self, waypoints, eef_step, jump_threshold):
            executed["waypoint_count"] = len(waypoints)
            executed["jump_threshold"] = jump_threshold
            return (
                _FakeTrajectory(
                    [
                        _FakeTrajectoryPoint([0.0] * 6, 0.0),
                        _FakeTrajectoryPoint([0.1] * 6, 0.5),
                        _FakeTrajectoryPoint([0.2] * 6, 1.0),
                    ]
                ),
                cartesian_fraction,
            )

        def retime_trajectory(self, state, trajectory, velocity_scaling_factor, acceleration_scaling_factor):
            executed["retime_velocity"] = velocity_scaling_factor
            executed["retime_acceleration"] = acceleration_scaling_factor
            return trajectory

        def set_pose_target(self, pose, link):
            self.pose_target = (pose, link)

        def plan(self):
            executed["planned_with_pose_target"] = self.pose_target is not None
            return True, _FakeTrajectory([_FakeTrajectoryPoint([0.0] * 6, 0.0), _FakeTrajectoryPoint([0.1] * 6, 0.8)]), 0.2, type("Err", (), {"val": 1})()

        def execute(self, trajectory, wait=True):
            executed["executed"] = True
            return True

        def stop(self):
            executed["stopped"] = True

        def clear_pose_targets(self):
            executed["cleared"] = True

        def get_current_pose(self, link):
            pose = type("PoseStamped", (), {})()
            pose.pose = type("Pose", (), {})()
            pose.pose.position = type("Point", (), {"x": 0.19124318537468837, "y": 0.0022620599968891843, "z": 0.22627771846438982})()
            pose.pose.orientation = type("Quat", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})()
            return pose

    monkeypatch.setitem(__import__("sys").modules, "moveit_commander", type("MoveIt", (), {"roscpp_initialize": staticmethod(lambda argv: None), "MoveGroupCommander": FakeGroup})())
    monkeypatch.setitem(__import__("sys").modules, "rospy", type("Rospy", (), {"get_node_uri": staticmethod(lambda: "node"), "init_node": staticmethod(lambda *a, **k: None)})())
    monkeypatch.setitem(__import__("sys").modules, "geometry_msgs.msg", type("Geom", (), {"Pose": _GeomPose})())
    return executed


def _install_fake_geometry(monkeypatch):
    class _GeomPoint:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _GeomQuat:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class _GeomPose:
        def __init__(self):
            self.position = _GeomPoint()
            self.orientation = _GeomQuat()

    monkeypatch.setitem(__import__("sys").modules, "geometry_msgs.msg", type("Geom", (), {"Pose": _GeomPose})())


def _install_fake_moveit_msgs(monkeypatch):
    class _Constraints:
        def __init__(self):
            self.name = ""
            self.joint_constraints = []

    class _JointConstraint:
        def __init__(self):
            self.joint_name = ""
            self.position = 0.0
            self.tolerance_above = 0.0
            self.tolerance_below = 0.0
            self.weight = 0.0

    monkeypatch.setitem(__import__("sys").modules, "moveit_msgs.msg", type("MoveItMsgs", (), {"Constraints": _Constraints, "JointConstraint": _JointConstraint})())


def test_preview_or_execute_shadow_never_calls_service(monkeypatch):
    executed = _install_fake_moveit(monkeypatch)
    _install_fake_moveit_msgs(monkeypatch)
    plan = horizon_to_trajectory_plan(
        _snapshot(),
        _moveit_pose(),
        {"actions": [[0.002, 0.001, 0.001, 0.0, 0.0, 0.0, 0.0], [0.004, 0.002, 0.002, 0.0, 0.0, 0.0, 0.0]]},
        _config(),
        max_actions=16,
    )
    result = preview_or_execute(plan, execute=False, motion_profile=get_motion_profile(_config(), "fast"), config=_config())
    assert result["success"] is True
    assert result["outputs"]["moveit_request_preview"]["will_call_service"] is False
    assert result["outputs"]["selected_horizon_length"] == 2
    assert executed["waypoint_count"] == 2
    assert executed["jump_threshold"] == 1.0
    assert executed["path_constraints"].name == "lap_local_joint_branch"
    assert executed["path_constraints_cleared"] is True
    assert result["outputs"]["planning_mode"] == "cartesian_path"
    assert result["outputs"]["candidate_evaluations"][0]["safe"] is True
    assert result["outputs"]["fallback_joint_branch_constraint"]["tolerance_rad"] == 0.75


def test_preview_or_execute_uses_corrected_tcp_target(monkeypatch):
    plan = horizon_to_trajectory_plan(
        _snapshot(),
        _moveit_pose(),
        {"actions": [[0.0028122098797273565, 0.0019796282983695623, 0.002152735114336668, 0.0, 0.0, 0.0, 0.0]]},
        _config(),
        max_actions=1,
    )
    first_target = plan.absolute_tcp_targets[0]
    assert first_target.x == 0.19405539525441573
    assert first_target.y == 0.004241688295258747
    assert first_target.z == 0.2284304535787265
    assert first_target.frame_id == "dummy_link"


def test_horizon_rows_are_future_deltas_from_current_state():
    plan = horizon_to_trajectory_plan(
        _snapshot(),
        _moveit_pose(),
        {"actions": [[0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]},
        _config(),
        max_actions=16,
    )
    assert plan.absolute_tcp_targets[0].x == _moveit_pose().position_m[0] + 0.01
    assert plan.absolute_tcp_targets[1].x == _moveit_pose().position_m[0] + 0.02


def test_motion_profiles_loaded():
    config = _config()
    assert get_motion_profile(config, "safe").velocity_scaling == 0.20
    assert get_motion_profile(config, "normal").acceleration_scaling == 0.35
    assert get_motion_profile(config, "fast").velocity_scaling == 0.80


def test_motion_profiles_are_bounded_when_loaded(tmp_path):
    config_path = tmp_path / "lap.yaml"
    config_path.write_text(
        """
server:
  host: 127.0.0.1
  port: 8016
topics:
  color_image: /table_camera/color/image_raw
  joint_state: /joint_states_single
  end_pose: /end_pose
motion:
  max_total_translation_m: 0.20
  max_adjacent_waypoint_translation_m: 0.03
  profiles:
    fast:
      velocity_scaling: 1.5
      acceleration_scaling: 2.0
""",
        encoding="utf-8",
    )
    from piper_on_bunker.policies.lap_policy import load_lap_config

    loaded = load_lap_config(config_path)
    fast = get_motion_profile(loaded, "fast")
    assert fast.velocity_scaling == 1.0
    assert fast.acceleration_scaling == 1.0


def test_preview_or_execute_rejects_joint_flip(monkeypatch):
    executed = _install_fake_moveit(monkeypatch)

    class FlipGroup:
        def __init__(self, name):
            self.name = name

        def set_start_state_to_current_state(self): pass
        def set_planning_time(self, value): pass
        def set_num_planning_attempts(self, value): pass
        def set_pose_reference_frame(self, value): pass
        def set_end_effector_link(self, value): pass
        def set_max_velocity_scaling_factor(self, value): pass
        def set_max_acceleration_scaling_factor(self, value): pass
        def get_current_state(self): return object()
        def set_pose_target(self, pose, link): pass
        def compute_cartesian_path(self, waypoints, eef_step, jump_threshold):
            return _FakeTrajectory([_FakeTrajectoryPoint([0.0] * 6, 0.0), _FakeTrajectoryPoint([1.06, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0)]), 1.0
        def retime_trajectory(self, state, trajectory, velocity_scaling_factor, acceleration_scaling_factor): return trajectory
        def plan(self):
            return False, _FakeTrajectory([]), 0.2, type("Err", (), {"val": -6})()

    monkeypatch.setitem(__import__("sys").modules, "moveit_commander", type("MoveIt", (), {"roscpp_initialize": staticmethod(lambda argv: None), "MoveGroupCommander": FlipGroup})())
    monkeypatch.setitem(__import__("sys").modules, "rospy", type("Rospy", (), {"get_node_uri": staticmethod(lambda: "node"), "init_node": staticmethod(lambda *a, **k: None)})())
    _install_fake_geometry(monkeypatch)
    _install_fake_moveit_msgs(monkeypatch)

    plan = horizon_to_trajectory_plan(
        _snapshot(),
        _moveit_pose(),
        {"actions": [[0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]},
        _config(),
        max_actions=1,
    )
    result = preview_or_execute(plan, execute=False, motion_profile=get_motion_profile(_config(), "fast"), config=_config())
    assert result["success"] is False
    assert result["status_code"] == "PLANNING_FAILURE"
    assert result["outputs"]["planning_mode"] == "final_pose_fallback"
    assert result["outputs"]["candidate_evaluations"][0]["safe"] is False
    assert "max joint delta" in result["outputs"]["candidate_evaluations"][0]["rejection_reason"]


def test_preview_or_execute_falls_back_to_one_final_plan(monkeypatch):
    executed = _install_fake_moveit(monkeypatch, cartesian_fraction=0.40)
    _install_fake_moveit_msgs(monkeypatch)
    plan = horizon_to_trajectory_plan(
        _snapshot(),
        _moveit_pose(),
        {"actions": [[0.003, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.006, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]},
        _config(),
        max_actions=16,
    )
    result = preview_or_execute(plan, execute=False, motion_profile=get_motion_profile(_config(), "fast"), config=_config())
    assert result["success"] is True
    assert result["outputs"]["planning_mode"] == "final_pose_fallback"
    assert result["outputs"]["selected_horizon_length"] == 2
    assert executed["planned_with_pose_target"] is True
    assert result["outputs"]["candidate_evaluations"][0]["safe"] is False
    assert "below min_cartesian_path_fraction" in result["outputs"]["candidate_evaluations"][0]["rejection_reason"]


def test_preview_or_execute_reports_truncated_safe_prefix(monkeypatch):
    executed = _install_fake_moveit(monkeypatch)
    plan = horizon_to_trajectory_plan(
        _snapshot(),
        _moveit_pose(),
        {
                "actions": [
                    [0.02, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.04, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.07, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.10, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.13, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.16, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                    [0.22, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0],
                ]
            },
            _config(),
            max_actions=16,
        )
    result = preview_or_execute(plan, execute=False, motion_profile=get_motion_profile(_config(), "fast"), config=_config())
    assert result["success"] is True
    assert result["outputs"]["horizon_truncated"] is True
    assert result["outputs"]["rejected_waypoint_index"] == 6
    assert result["outputs"]["selected_horizon_length"] == 6
    assert executed["waypoint_count"] == 6


def test_preview_or_execute_uses_safe_cartesian_candidate_when_fallback_is_unsafe(monkeypatch):
    class CartesianPreferredGroup:
        def __init__(self, name):
            self.name = name
            self.pose_target = None

        def set_start_state_to_current_state(self): pass
        def set_planning_time(self, value): pass
        def set_num_planning_attempts(self, value): pass
        def set_pose_reference_frame(self, value): pass
        def set_end_effector_link(self, value): pass
        def set_max_velocity_scaling_factor(self, value): pass
        def set_max_acceleration_scaling_factor(self, value): pass
        def get_current_state(self): return object()
        def compute_cartesian_path(self, waypoints, eef_step, jump_threshold):
            return _FakeTrajectory(
                [
                    _FakeTrajectoryPoint([0.0] * 6, 0.0),
                    _FakeTrajectoryPoint([0.1] * 6, 0.5),
                    _FakeTrajectoryPoint([0.2] * 6, 1.0),
                ]
            ), 0.9166666666666666
        def retime_trajectory(self, state, trajectory, velocity_scaling_factor, acceleration_scaling_factor): return trajectory
        def set_pose_target(self, pose, link): self.pose_target = (pose, link)
        def plan(self):
            return True, _FakeTrajectory(
                [
                    _FakeTrajectoryPoint([0.0] * 6, 0.0),
                    _FakeTrajectoryPoint([1.4, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0),
                ]
            ), 0.2, type("Err", (), {"val": 1})()
        def get_current_pose(self, link):
            pose = type("PoseStamped", (), {})()
            pose.pose = type("Pose", (), {})()
            pose.pose.position = type("Point", (), {"x": 0.19124318537468837, "y": 0.0022620599968891843, "z": 0.22627771846438982})()
            pose.pose.orientation = type("Quat", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})()
            return pose

    monkeypatch.setitem(__import__("sys").modules, "moveit_commander", type("MoveIt", (), {"roscpp_initialize": staticmethod(lambda argv: None), "MoveGroupCommander": CartesianPreferredGroup})())
    monkeypatch.setitem(__import__("sys").modules, "rospy", type("Rospy", (), {"get_node_uri": staticmethod(lambda: "node"), "init_node": staticmethod(lambda *a, **k: None)})())
    _install_fake_geometry(monkeypatch)
    _install_fake_moveit_msgs(monkeypatch)

    plan = horizon_to_trajectory_plan(
        _snapshot(),
        _moveit_pose(),
        {"actions": [[0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]},
        _config(),
        max_actions=16,
    )
    result = preview_or_execute(plan, execute=False, motion_profile=get_motion_profile(_config(), "fast"), config=_config())
    assert result["success"] is True
    assert result["outputs"]["planning_mode"] == "cartesian_path"
    assert result["outputs"]["candidate_evaluations"][0]["safe"] is True
    assert result["outputs"]["candidate_evaluations"][1]["safe"] is False
    assert "max joint delta" in result["outputs"]["candidate_evaluations"][1]["rejection_reason"]


def test_preview_or_execute_uses_safe_fallback_prefix_when_full_prefix_is_unsafe(monkeypatch):
    class PrefixFallbackGroup:
        def __init__(self, name):
            self.name = name
            self.pose_target = None
            self.requested_x = None

        def set_start_state_to_current_state(self): pass
        def set_planning_time(self, value): pass
        def set_num_planning_attempts(self, value): pass
        def set_pose_reference_frame(self, value): pass
        def set_end_effector_link(self, value): pass
        def set_max_velocity_scaling_factor(self, value): pass
        def set_max_acceleration_scaling_factor(self, value): pass
        def get_current_state(self): return object()
        def get_active_joints(self): return ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        def get_current_joint_values(self): return [0.0] * 6
        def set_path_constraints(self, constraints): self.constraints = constraints
        def clear_path_constraints(self): self.constraints_cleared = True
        def compute_cartesian_path(self, waypoints, eef_step, jump_threshold):
            return _FakeTrajectory([]), 0.2
        def retime_trajectory(self, state, trajectory, velocity_scaling_factor, acceleration_scaling_factor): return trajectory
        def set_pose_target(self, pose, link):
            self.pose_target = (pose, link)
            self.requested_x = pose.position.x
        def plan(self):
            if self.requested_x > 0.203:
                return True, _FakeTrajectory(
                    [
                        _FakeTrajectoryPoint([0.0] * 6, 0.0),
                        _FakeTrajectoryPoint([1.2, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0),
                    ]
                ), 0.2, type("Err", (), {"val": 1})()
            return True, _FakeTrajectory(
                [
                    _FakeTrajectoryPoint([0.0] * 6, 0.0),
                    _FakeTrajectoryPoint([0.2, 0.1, 0.0, 0.0, 0.0, 0.0], 1.0),
                ]
            ), 0.2, type("Err", (), {"val": 1})()
        def get_current_pose(self, link):
            pose = type("PoseStamped", (), {})()
            pose.pose = type("Pose", (), {})()
            pose.pose.position = type("Point", (), {"x": 0.19124318537468837, "y": 0.0022620599968891843, "z": 0.22627771846438982})()
            pose.pose.orientation = type("Quat", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})()
            return pose

    monkeypatch.setitem(__import__("sys").modules, "moveit_commander", type("MoveIt", (), {"roscpp_initialize": staticmethod(lambda argv: None), "MoveGroupCommander": PrefixFallbackGroup})())
    monkeypatch.setitem(__import__("sys").modules, "rospy", type("Rospy", (), {"get_node_uri": staticmethod(lambda: "node"), "init_node": staticmethod(lambda *a, **k: None)})())
    _install_fake_geometry(monkeypatch)
    _install_fake_moveit_msgs(monkeypatch)

    plan = horizon_to_trajectory_plan(
        _snapshot(),
        _moveit_pose(),
        {"actions": [[0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.015, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]},
        _config(),
        max_actions=16,
    )
    result = preview_or_execute(plan, execute=False, motion_profile=get_motion_profile(_config(), "fast"), config=_config())
    assert result["success"] is True
    assert result["outputs"]["planning_mode"] == "final_pose_fallback"
    assert result["outputs"]["selected_horizon_length"] == 2
    assert result["outputs"]["planning_prefix_truncated"] is True
    assert result["outputs"]["candidate_evaluations"][0]["safe"] is False
    assert result["outputs"]["candidate_evaluations"][-1]["safe"] is True


def test_preview_or_execute_execute_requires_enable_preflight(monkeypatch):
    import piper_on_bunker.policies.lap_policy as lap_policy

    _install_fake_moveit(monkeypatch, cartesian_fraction=1.0)
    _install_fake_moveit_msgs(monkeypatch)
    monkeypatch.setattr(lap_policy, "_enable_piper_driver", lambda rospy: {"attempted": True, "success": False, "error": "enable failed"})

    plan = horizon_to_trajectory_plan(
        _snapshot(),
        _moveit_pose(),
        {"actions": [[0.002, 0.001, 0.001, 0.0, 0.0, 0.0, 0.0]]},
        _config(),
        max_actions=1,
    )
    result = preview_or_execute(plan, execute=True, motion_profile=get_motion_profile(_config(), "fast"), config=_config())
    assert result["success"] is False
    assert result["status_code"] == "ENABLE_FAILURE"
    assert result["outputs"]["enable_preflight"]["success"] is False


def test_preview_or_execute_execute_fails_when_moveit_execute_fails(monkeypatch):
    import piper_on_bunker.policies.lap_policy as lap_policy

    class ExecuteFailGroup:
        def __init__(self, name):
            self.name = name
            self.pose_target = None

        def set_start_state_to_current_state(self): pass
        def set_planning_time(self, value): pass
        def set_num_planning_attempts(self, value): pass
        def set_pose_reference_frame(self, value): pass
        def set_end_effector_link(self, value): pass
        def set_max_velocity_scaling_factor(self, value): pass
        def set_max_acceleration_scaling_factor(self, value): pass
        def get_current_state(self): return object()
        def get_active_joints(self): return ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        def get_current_joint_values(self): return [0.0] * 6
        def set_path_constraints(self, constraints): pass
        def clear_path_constraints(self): pass
        def compute_cartesian_path(self, waypoints, eef_step, jump_threshold):
            return _FakeTrajectory([_FakeTrajectoryPoint([0.0] * 6, 0.0), _FakeTrajectoryPoint([0.1] * 6, 0.5)]), 1.0
        def retime_trajectory(self, state, trajectory, velocity_scaling_factor, acceleration_scaling_factor): return trajectory
        def set_pose_target(self, pose, link): self.pose_target = (pose, link)
        def plan(self):
            return True, _FakeTrajectory([_FakeTrajectoryPoint([0.0] * 6, 0.0), _FakeTrajectoryPoint([0.1] * 6, 0.8)]), 0.2, type("Err", (), {"val": 1})()
        def execute(self, trajectory, wait=True): return False
        def stop(self): pass
        def clear_pose_targets(self): pass
        def get_current_pose(self, link):
            pose = type("PoseStamped", (), {})()
            pose.pose = type("Pose", (), {})()
            pose.pose.position = type("Point", (), {"x": 0.19124318537468837, "y": 0.0022620599968891843, "z": 0.22627771846438982})()
            pose.pose.orientation = type("Quat", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})()
            return pose

    monkeypatch.setitem(__import__("sys").modules, "moveit_commander", type("MoveIt", (), {"roscpp_initialize": staticmethod(lambda argv: None), "MoveGroupCommander": ExecuteFailGroup})())
    monkeypatch.setitem(__import__("sys").modules, "rospy", type("Rospy", (), {"get_node_uri": staticmethod(lambda: "node"), "init_node": staticmethod(lambda *a, **k: None)})())
    _install_fake_geometry(monkeypatch)
    _install_fake_moveit_msgs(monkeypatch)
    monkeypatch.setattr(lap_policy, "_enable_piper_driver", lambda rospy: {"attempted": True, "success": True})

    plan = horizon_to_trajectory_plan(
        _snapshot(),
        _moveit_pose(),
        {"actions": [[0.002, 0.001, 0.001, 0.0, 0.0, 0.0, 0.0]]},
        _config(),
        max_actions=1,
    )
    result = preview_or_execute(plan, execute=True, motion_profile=get_motion_profile(_config(), "fast"), config=_config())
    assert result["success"] is False
    assert result["status_code"] == "EXECUTION_FAILURE"
    assert result["outputs"]["moveit_execute_result"]["returned_success"] is False
