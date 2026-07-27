import numpy as np
import pytest

from piper_on_bunker.control.phase_chunk_buffer import JointSafetyLimits
from piper_on_bunker.control.phase_chunk_buffer import blend_chunks
from piper_on_bunker.control.phase_chunk_buffer import interpolate_chunk
from piper_on_bunker.control.phase_chunk_buffer import validate_action_chunk


def test_rejects_one_sample_noop_chunk():
    with pytest.raises(ValueError, match="at least two"):
        validate_action_chunk(np.zeros((1, 7)), np.zeros(7), frequency_hz=20.0, limits=JointSafetyLimits())


def test_rejects_nonfinite_action():
    actions = np.zeros((3, 7))
    actions[1, 0] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        validate_action_chunk(actions, np.zeros(7), frequency_hz=20.0, limits=JointSafetyLimits())


def test_rejects_initial_jump():
    actions = np.zeros((3, 7))
    actions[:, 0] = 0.5
    with pytest.raises(ValueError, match="initial joint jump"):
        validate_action_chunk(actions, np.zeros(7), frequency_hz=20.0, limits=JointSafetyLimits())


def test_interpolates_to_hardware_rate():
    actions = np.zeros((3, 7))
    actions[:, 0] = [0.0, 0.01, 0.02]
    stream = interpolate_chunk(actions, source_frequency_hz=20.0, target_frequency_hz=50.0)
    assert stream.shape[1] == 7
    assert stream.shape[0] > actions.shape[0]


def test_blend_does_not_change_endpoint():
    prev = np.zeros((3, 7))
    nxt = np.ones((4, 7)) * 0.02
    blended = blend_chunks(prev, nxt, blend_samples=2)
    assert blended.shape == nxt.shape
    assert np.allclose(blended[-1], nxt[-1])

