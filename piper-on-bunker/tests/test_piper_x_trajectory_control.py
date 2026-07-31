import math
import importlib.util
from pathlib import Path

import pytest

from piper_on_bunker.hardware.piper_x_trajectory_control import PIPER_X_TRAJECTORY_JOINTS
from piper_on_bunker.hardware.piper_x_trajectory_control import PiperXTrajectoryPoint
from piper_on_bunker.hardware.piper_x_trajectory_control import endpoint_within_tolerance
from piper_on_bunker.hardware.piper_x_trajectory_control import maximum_endpoint_error
from piper_on_bunker.hardware.piper_x_trajectory_control import resample_trajectory
from piper_on_bunker.hardware.piper_x_trajectory_control import joints_rad_to_raw_mdeg
from piper_on_bunker.hardware.piper_x_trajectory_control import rad_to_raw_mdeg
from piper_on_bunker.hardware.piper_x_trajectory_control import raw_mdeg_to_rad
from piper_on_bunker.hardware.piper_x_trajectory_control import validate_trajectory_joint_names
from piper_on_bunker.hardware.piper_x_trajectory_control import validate_trajectory_points


def test_rad_to_raw_mdeg_conversion_round_trips():
    value = math.radians(12.345)
    raw = rad_to_raw_mdeg(value)
    assert raw == 12345
    assert raw_mdeg_to_rad(raw) == pytest.approx(value)


def test_six_joint_raw_conversion_requires_exact_count():
    assert joints_rad_to_raw_mdeg([0.0, math.radians(1.0), 0.0, 0.0, 0.0, 0.0]) == [0, 1000, 0, 0, 0, 0]
    with pytest.raises(ValueError, match="expected six"):
        joints_rad_to_raw_mdeg([0.0] * 5)


def test_trajectory_joint_names_must_match_piper_x_order():
    validate_trajectory_joint_names(list(PIPER_X_TRAJECTORY_JOINTS))
    with pytest.raises(ValueError, match="expected joints"):
        validate_trajectory_joint_names(["joint1", "joint2", "joint3", "joint4", "joint6", "joint5"])


def test_trajectory_points_reject_missing_nonfinite_and_nonmonotonic():
    validate_trajectory_points(
        [
            PiperXTrajectoryPoint([0.0] * 6, 0.0),
            PiperXTrajectoryPoint([0.01] * 6, 1.0),
        ]
    )
    with pytest.raises(ValueError, match="no points"):
        validate_trajectory_points([])
    with pytest.raises(ValueError, match="six positions"):
        validate_trajectory_points([PiperXTrajectoryPoint([0.0] * 5, 0.0)])
    with pytest.raises(ValueError, match="non-finite"):
        validate_trajectory_points([PiperXTrajectoryPoint([float("nan")] + [0.0] * 5, 0.0)])
    with pytest.raises(ValueError, match="strictly increasing"):
        validate_trajectory_points(
            [
                PiperXTrajectoryPoint([0.0] * 6, 1.0),
                PiperXTrajectoryPoint([0.01] * 6, 0.5),
            ]
        )
    with pytest.raises(ValueError, match="strictly increasing"):
        validate_trajectory_points(
            [
                PiperXTrajectoryPoint([0.0] * 6, 0.0),
                PiperXTrajectoryPoint([0.01] * 6, 0.0),
            ]
        )


def test_sparse_four_second_trajectory_resamples_to_approximately_201_commands():
    commands = resample_trajectory(
        [
            PiperXTrajectoryPoint([0.0] * 6, 0.0),
            PiperXTrajectoryPoint([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], 4.0),
        ],
        command_rate_hz=50.0,
    )
    assert len(commands) == 201
    assert commands[0].time_from_start_s == pytest.approx(0.0)
    assert commands[-1].time_from_start_s == pytest.approx(4.0)
    assert max(b.time_from_start_s - a.time_from_start_s for a, b in zip(commands, commands[1:])) == pytest.approx(0.02)
    assert commands[-1].positions_rad == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_resampling_interpolates_all_six_joints_smoothly():
    commands = resample_trajectory(
        [
            PiperXTrajectoryPoint([0.0] * 6, 0.0),
            PiperXTrajectoryPoint([0.5, -0.5, 1.0, -1.0, 0.25, -0.25], 1.0),
        ],
        command_rate_hz=10.0,
    )
    middle = commands[5]
    assert middle.time_from_start_s == pytest.approx(0.5)
    assert middle.positions_rad == pytest.approx([0.25, -0.25, 0.5, -0.5, 0.125, -0.125])


def test_resampling_blends_current_feedback_to_first_point():
    commands = resample_trajectory(
        [
            PiperXTrajectoryPoint([1.0] * 6, 0.0),
            PiperXTrajectoryPoint([2.0] * 6, 1.0),
        ],
        command_rate_hz=50.0,
        current_positions_rad=[0.0] * 6,
        first_point_blend_s=0.2,
    )
    assert commands[0].positions_rad == [0.0] * 6
    assert commands[-1].positions_rad == [2.0] * 6
    assert commands[-1].time_from_start_s == pytest.approx(1.0)
    assert max(b.time_from_start_s - a.time_from_start_s for a, b in zip(commands, commands[1:])) <= 0.0200001


def test_endpoint_tolerance_completion_logic():
    actual = [0.0, 0.01, -0.02, 0.025, 0.0, 0.0]
    target = [0.0] * 6
    assert maximum_endpoint_error(actual, target) == pytest.approx(0.025)
    assert endpoint_within_tolerance(actual, target, 0.03)
    assert not endpoint_within_tolerance(actual, target, 0.02)


def test_controller_hold_current_feedback_uses_measured_positions():
    path = Path("piper-on-bunker/scripts/piper_x_moveit_sdk_trajectory_controller.py")
    import sys
    sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location("piper_x_moveit_sdk_trajectory_controller", path)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)

    class FakeRospy:
        def logwarn(self, *args):
            raise AssertionError(f"unexpected warning: {args}")

    class FakeArm:
        def __init__(self):
            self.writes = []

        def write_joints_raw(self, values):
            self.writes.append(list(values))

    controller = module.PiperXMoveItSdkTrajectoryController.__new__(module.PiperXMoveItSdkTrajectoryController)
    controller.rospy = FakeRospy()
    controller.args = type("Args", (), {"feedback_topic": "/joint_states"})()
    controller.latest_positions = {
        "joint1": 0.0,
        "joint2": math.radians(1.0),
        "joint3": math.radians(-2.0),
        "joint4": math.radians(3.0),
        "joint5": math.radians(-4.0),
        "joint6": math.radians(5.0),
    }
    arm = FakeArm()
    controller._hold_current_feedback(arm)
    assert arm.writes == [[0, 1000, -2000, 3000, -4000, 5000]]
