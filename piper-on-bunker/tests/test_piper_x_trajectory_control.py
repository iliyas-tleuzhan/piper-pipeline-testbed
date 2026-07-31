import math

import pytest

from piper_on_bunker.hardware.piper_x_trajectory_control import PIPER_X_TRAJECTORY_JOINTS
from piper_on_bunker.hardware.piper_x_trajectory_control import PiperXTrajectoryPoint
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
    with pytest.raises(ValueError, match="monotonic"):
        validate_trajectory_points(
            [
                PiperXTrajectoryPoint([0.0] * 6, 1.0),
                PiperXTrajectoryPoint([0.01] * 6, 0.5),
            ]
        )
