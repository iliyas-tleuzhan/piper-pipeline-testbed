from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class JointSafetyLimits:
    joint_min_rad: tuple[float, float, float, float, float, float] = (-2.618, 0.0, -2.967, -1.745, -1.22, -2.094)
    joint_max_rad: tuple[float, float, float, float, float, float] = (2.618, 3.14, 0.0, 1.745, 1.22, 2.094)
    gripper_min_m: float = 0.0
    gripper_max_m: float = 0.06
    max_initial_jump_rad: float = 0.25
    max_adjacent_joint_delta_rad: float = 0.20
    max_joint_velocity_rad_s: float = 1.0
    max_joint_acceleration_rad_s2: float = 4.0
    max_joint_jerk_rad_s3: float = 30.0
    max_gripper_step_m: float = 0.02


def validate_action_chunk(
    actions,
    current_state,
    *,
    frequency_hz: float,
    limits: JointSafetyLimits,
) -> np.ndarray:
    chunk = np.asarray(actions, dtype=np.float64)
    current = np.asarray(current_state, dtype=np.float64)
    if chunk.ndim != 2 or chunk.shape[1] != 7:
        raise ValueError(f"action chunk must have shape [horizon, 7], got {chunk.shape}")
    if current.shape != (7,):
        raise ValueError(f"current state must have shape (7,), got {current.shape}")
    if chunk.shape[0] < 2:
        raise ValueError("action chunk must contain at least two samples; one-sample no-op plans are refused")
    if frequency_hz <= 0:
        raise ValueError("frequency_hz must be positive")
    if not np.all(np.isfinite(chunk)) or not np.all(np.isfinite(current)):
        raise ValueError("action chunk and current state must be finite")

    joint_min = np.asarray(limits.joint_min_rad, dtype=np.float64)
    joint_max = np.asarray(limits.joint_max_rad, dtype=np.float64)
    arm = chunk[:, :6]
    gripper = chunk[:, 6]
    if np.any(arm < joint_min) or np.any(arm > joint_max):
        raise ValueError("action chunk violates authoritative PiPER joint limits")
    if np.any(gripper < limits.gripper_min_m) or np.any(gripper > limits.gripper_max_m):
        raise ValueError("action chunk violates authoritative PiPER gripper limits")

    initial_jump = np.max(np.abs(chunk[0, :6] - current[:6]))
    if initial_jump > limits.max_initial_jump_rad:
        raise ValueError(
            f"initial joint jump {initial_jump:.6f} exceeds max_initial_jump_rad={limits.max_initial_jump_rad:.6f}"
        )
    adjacent = np.diff(chunk, axis=0)
    max_adjacent = np.max(np.abs(adjacent[:, :6]))
    if max_adjacent > limits.max_adjacent_joint_delta_rad:
        raise ValueError(
            f"adjacent joint delta {max_adjacent:.6f} exceeds {limits.max_adjacent_joint_delta_rad:.6f}"
        )
    max_gripper_step = np.max(np.abs(adjacent[:, 6]))
    if max_gripper_step > limits.max_gripper_step_m:
        raise ValueError(f"gripper step {max_gripper_step:.6f} exceeds {limits.max_gripper_step_m:.6f}")

    dt = 1.0 / float(frequency_hz)
    velocity = adjacent[:, :6] / dt
    max_velocity = np.max(np.abs(velocity))
    if max_velocity > limits.max_joint_velocity_rad_s:
        raise ValueError(f"implied joint velocity {max_velocity:.6f} exceeds {limits.max_joint_velocity_rad_s:.6f}")
    if len(velocity) >= 2:
        acceleration = np.diff(velocity, axis=0) / dt
        max_acceleration = np.max(np.abs(acceleration))
        if max_acceleration > limits.max_joint_acceleration_rad_s2:
            raise ValueError(
                f"implied joint acceleration {max_acceleration:.6f} exceeds "
                f"{limits.max_joint_acceleration_rad_s2:.6f}"
            )
        if len(acceleration) >= 2:
            jerk = np.diff(acceleration, axis=0) / dt
            max_jerk = np.max(np.abs(jerk))
            if max_jerk > limits.max_joint_jerk_rad_s3:
                raise ValueError(f"implied joint jerk {max_jerk:.6f} exceeds {limits.max_joint_jerk_rad_s3:.6f}")
    return chunk


def interpolate_chunk(actions, *, source_frequency_hz: float, target_frequency_hz: float) -> np.ndarray:
    chunk = np.asarray(actions, dtype=np.float64)
    if source_frequency_hz <= 0 or target_frequency_hz <= 0:
        raise ValueError("frequencies must be positive")
    if chunk.ndim != 2 or chunk.shape[0] < 2:
        raise ValueError("cannot interpolate a chunk with fewer than two samples")
    if abs(source_frequency_hz - target_frequency_hz) < 1e-9:
        return chunk.copy()
    duration = (chunk.shape[0] - 1) / float(source_frequency_hz)
    source_t = np.linspace(0.0, duration, chunk.shape[0])
    target_count = int(round(duration * target_frequency_hz)) + 1
    target_t = np.linspace(0.0, duration, target_count)
    out = np.empty((target_count, chunk.shape[1]), dtype=np.float64)
    for dim in range(chunk.shape[1]):
        out[:, dim] = np.interp(target_t, source_t, chunk[:, dim])
    return out


def blend_chunks(previous_tail, next_head, *, blend_samples: int) -> np.ndarray:
    prev = np.asarray(previous_tail, dtype=np.float64)
    nxt = np.asarray(next_head, dtype=np.float64)
    if blend_samples <= 0:
        return nxt.copy()
    if prev.shape[1:] != nxt.shape[1:] or prev.shape[1] != 7:
        raise ValueError("chunks must both have action dimension 7")
    samples = min(blend_samples, len(prev), len(nxt))
    blended = nxt.copy()
    weights = np.linspace(0.0, 1.0, samples + 2, dtype=np.float64)[1:-1]
    blended[:samples] = (1.0 - weights[:, None]) * prev[-samples:] + weights[:, None] * nxt[:samples]
    return blended

