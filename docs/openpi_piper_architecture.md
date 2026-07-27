# PiPER-on-Bunker OpenPI Manipulation Architecture

Default manipulation now follows:

```text
User task
  -> OpenClaw semantic plan
  -> phase orchestrator
  -> PiPER-fine-tuned pi0.5 policy server
  -> direct joint phase executor
  -> /piper_joint_commands
  -> PiPER driver / CAN can0
  -> PiPER arm and gripper

Feedback:
  cameras + /joint_states_single + gripper feedback
  -> phase observation
  -> pi0.5 inference and verification
  -> OpenClaw phase status

OpenClaw also controls:
  Bunker navigation/docking -> base lock -> manipulation phases
```

The default manipulation path does not use LAP-3B, MoveIt, OMPL, IK,
Cartesian TCP conversion, `MoveGroupCommander`, `FollowJointTrajectory`, or the
`piper_trajectory_bridge`. Those remain archived for explicit legacy
experiments only.

## Roles

OpenClaw is the semantic task planner. It breaks a human instruction into
explicit phases such as `APPROACH_OBJECT`, `GRASP_OBJECT`,
`TRANSPORT_OBJECT`, and `RELEASE_OBJECT`. It checks preconditions and phase
completion evidence, and it selects recovery or abort behavior.

pi0.5 is the sole manipulation motion planner. It returns absolute PiPER joint
action chunks. Public checkpoints such as `pi05_base` and `pi05_droid` are not
PiPER-compatible and may only be used for install, tensor-shape, protocol, and
shadow-mode tests.

The direct executor validates, interpolates, schedules, blends, streams, reads
feedback, and stops safely. It does not calculate IK, choose alternate joint
targets, shorten a phase because another path is easier, or invent a geometric
trajectory.

## Schemas

Observation:

```json
{
  "observation/exterior_image": "uint8 RGB image",
  "observation/wrist_image": "uint8 RGB image",
  "observation/state": [
    "joint1_current_rad",
    "joint2_current_rad",
    "joint3_current_rad",
    "joint4_current_rad",
    "joint5_current_rad",
    "joint6_current_rad",
    "gripper_total_jaw_opening_m"
  ],
  "prompt": "phase-specific instruction"
}
```

Action:

```json
{
  "actions": [["joint1_rad", "joint2_rad", "joint3_rad", "joint4_rad", "joint5_rad", "joint6_rad", "gripper_total_jaw_opening_m"]],
  "action_semantics": "absolute_piper_joint_targets",
  "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
}
```

Phase plans use schema `openclaw.manipulation_plan.v1`.

## OpenPI Pin

- Upstream: `https://github.com/Physical-Intelligence/openpi.git`
- Commit: `15a9616a00943ada6c20a0f158e3adb39df2ccac`
- Base checkpoint for PiPER fine-tuning: `gs://openpi-assets/checkpoints/pi05_base`
- Public base checkpoint physical status: `piper_compatible: false`

At this OpenPI commit, JAX supports pi0.5 training/inference and the official
low-memory LoRA-style variants. PyTorch support exists for pi0/pi0.5 inference
and fine-tuning, but OpenPI documents PyTorch LoRA and FSDP as unsupported.

## Dataset And Training

Record command actions, not inferred future states:

- external RGB image
- wrist RGB image
- seven-dimensional state
- commanded seven-dimensional action
- overall instruction
- phase prompt and phase id
- timestamps for camera, state, and command
- episode metadata and success/failure labels

Synthetic conversion check:

```bash
cd ~/piper-pipeline-testbed
PYTHONPATH=piper-on-bunker/src \
  python3 piper-on-bunker/scripts/make_synthetic_piper_openpi_dataset.py
```

Official OpenPI commands after adding the PiPER config to an OpenPI checkout:

```bash
cd ~/ABot-Claw-piper/openpi
git checkout 15a9616a00943ada6c20a0f158e3adb39df2ccac
GIT_LFS_SKIP_SMUDGE=1 uv sync
uv run scripts/compute_norm_stats.py --config-name pi05_piper_single_arm
XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 \
  uv run scripts/train.py pi05_piper_single_arm --exp-name piper_cup_phase_v1 --overwrite
uv run scripts/serve_policy.py policy:checkpoint \
  --policy.config=pi05_piper_single_arm \
  --policy.dir=checkpoints/pi05_piper_single_arm/piper_cup_phase_v1/20000 \
  --port=8017
```

A checkpoint may be marked `piper_compatible: true` only after PiPER-specific
normalization statistics, schema metadata, offline validation, and checkpoint
metadata are complete.

## Shadow And Physical Gates

Shadow task:

```bash
cd ~/piper-pipeline-testbed
PYTHONPATH=piper-on-bunker/src \
  python3 piper-on-bunker/scripts/run_openpi_piper_task.py \
    --instruction "Move the red cup onto the paper."
```

There is intentionally no guarded physical command yet. It must only be added
after a PiPER-specific checkpoint exists, offline replay passes, ROS shadow
passes, standalone gripper control is verified, and command authority is
exclusive.

