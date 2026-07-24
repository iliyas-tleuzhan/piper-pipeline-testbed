# X-VLA PiPER pipeline

Status on July 24, 2026:

- Implemented: PiPER-specific raw demonstration recorder, dataset validator, LeRobot export path, and exact manual collection commands.
- Synthetically tested: raw episode writing, dataset validation, and fake-LeRobot export contract.
- Waiting for human demonstrations: yes.
- Training completed: no.
- Live shadow tested: no.
- Planning-only tested: no.
- Manual physical handoff generated: not yet.

## Narrow first task

Initial task text:

`Move the gripper toward the marked target.`

Initial embodiment:

- one external RealSense RGB camera
- one PiPER state vector `[joint1..joint6, gripper]`
- one commanded target vector `[target_joint1..target_joint6, target_gripper]`
- one language instruction

## Why the recorder uses a manual teleop layer

The audited PiPER stack already exposes:

- live state on `/joint_states_single`
- live end pose on `/end_pose`
- accepted arm-joint targets on `/piper_joint_commands`

But `/piper_joint_commands` does not include the gripper target. For the first PiPER-specific dataset, logging state deltas as if they were commands would be semantically wrong. The recorder therefore uses a small manual joint/gripper command interface on top of the existing MoveIt service layer and records the exact target vector it sends for every demonstration step.

This is a data-collection interface, not a learned-policy executor.

## Current gripper-unit position

Verified from the local ABot-Claw PiPER source:

- public gripper control is documented as meters opening width
- source range is approximately `0.0` closed to `0.06` open

Live collection should still confirm the observed ROS value behavior before training conclusions are drawn.

## Raw dataset layout

Ignored local storage:

`piper-on-bunker/data/local/piper_xvla_target_v0/`

Per episode:

- `episode_XXXX/episode.json`
- `episode_XXXX/frames/frame_*.jpg`

The raw episode stores:

- pre-command RGB frame
- pre-command PiPER state
- exact commanded joint/gripper target
- end-pose metadata
- source topics
- timestamps and freshness ages
- success or abort outcome

## Synthetic validation

The repo now includes:

- `record_piper_demo.py`
- `validate_piper_dataset.py`
- `export_lerobot_dataset.py`
- `replay_piper_dataset_readonly.py`

The included tests cover:

- manual command parsing
- raw episode writing
- dataset validation
- LeRobot export contract with a fake dataset backend

Full official LeRobot export and X-VLA training remain blocked on real demonstrations and a dedicated LeRobot runtime environment.

## First human collection workflow

Read-only environment check:

```bash
./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/inspect_piper_demo_environment.py \
    --config piper-on-bunker/config/piper_laptop_hardware.yaml
```

Manual recording session:

```bash
./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/record_piper_demo.py \
    --config piper-on-bunker/config/piper_laptop_hardware.yaml \
    --dataset-root piper-on-bunker/data/local/piper_xvla_target_v0 \
    --task "Move the gripper toward the marked target."
```

Why this wrapper exists:

- the running `abot-piper-noetic` container currently mounts `~/ABot-Claw` only
- `docker exec` into that container would write the demo dataset into an ephemeral
  container filesystem copy of `~/piper-pipeline-testbed`
- the wrapper launches a short-lived ROS-enabled sidecar from the same image with
  the testbed bind-mounted, so `piper-on-bunker/data/local/piper_xvla_target_v0`
  persists directly on the host

Inside the recorder:

- `start`
- bounded commands such as `j2+`, `j2-`, `j3+`, `j3-`, `g+`, `g-`
- `save` for a successful episode
- `abort` to discard

## Next gate

After 5 to 10 real successful episodes exist:

1. validate raw episodes
2. export to official LeRobot format
3. inspect ranges and synchronization
4. move to remote X-VLA training scaffolding

No physical movement was executed by Codex during this implementation pass.
