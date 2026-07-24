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

Why the sidecar wrapper must keep stdin open:

- the recorder is interactive and uses `input()`
- `docker run --rm` without `-i` closes stdin immediately, which causes an `EOFError`
- `tools/run_in_noetic_container.sh` now always uses Docker `-i` and adds `-t` only when launched from a real terminal
- that keeps interactive collection usable while preserving noninteractive helpers such as `--help`, validation, replay, and export

Configuration split:

- `piper-on-bunker/config/piper_laptop_hardware.yaml`
  - read-only inspection and dry-run-safe hardware config
  - committed `physical_motion_enabled: false`
- `piper-on-bunker/config/piper_laptop_demo_collection.yaml`
  - dedicated live manual demonstration-collection config
  - declares physical intent, but still requires an ignored local activation file before any motion
- `piper-on-bunker/config/piper_laptop_demo_collection.local.yaml`
  - ignored local activation checkpoint
  - create it from `piper_laptop_demo_collection.local.example.yaml`
  - set `local_activation.physical_motion_enabled: true` only immediately before a supervised collection session

Read-only environment check:

```bash
./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/inspect_piper_demo_environment.py \
    --config piper-on-bunker/config/piper_laptop_hardware.yaml
```

Read-only recorder startup check:

```bash
./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/record_piper_demo.py \
    --config piper-on-bunker/config/piper_laptop_hardware.yaml \
    --dataset-root piper-on-bunker/data/local/piper_xvla_target_v0 \
    --task "Move the gripper toward the marked target."
```

That read-only command may be used to verify `status` and `quit`. It must not be used to collect real demonstrations.

Live human collection command:

```bash
./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/record_piper_demo.py \
    --config piper-on-bunker/config/piper_laptop_demo_collection.yaml \
    --dataset-root piper-on-bunker/data/local/piper_xvla_target_v0 \
    --task "Move the gripper toward the marked target."
```

Before the live command is allowed to enter the collection loop, it now requires:

- an interactive terminal
- the ignored local activation file for the live config
- live camera and joint-state freshness checks
- operator confirmation that:
  - the Bunker is immobilized
  - the workspace is clear
  - the emergency stop is reachable
  - the camera and joint state are live

The recorder prints a prominent startup warning showing:

- physical motion is enabled
- MoveIt service being used
- joint step size
- gripper step size
- maximum speed scaling
- maximum acceleration scaling
- dataset output path

Recorded frames are tagged as either:

- `dry_run_preview`
- `synthetic_fixture`
- `physical_execution_verified`

Only `physical_execution_verified` frames are valid for real training.

Persistent host dataset location:

`~/piper-pipeline-testbed/piper-on-bunker/data/local/piper_xvla_target_v0/`

The live recorder now verifies before saving a frame that:

- joint state was fresh before the command
- camera image was fresh before the command
- the six-joint target was finite and inside verified PiPER joint limits
- the gripper target was finite and inside the verified 0.0 to 0.06 m live range
- the MoveIt service returned success
- the resulting live arm state reached the commanded joint target within tolerance and timeout
- gripper verification passed where live gripper feedback was available

If any of those checks fail, the frame is not appended.

Abort and discard rules:

- `abort` discards the active episode
- `quit` exits without sending a command
- if stdin closes unexpectedly, the recorder exits without sending a command
- failed command attempts do not append a frame

To verify that a saved frame was physically executed rather than previewed, inspect `episode.json` and confirm:

- `command.execution_allowed: true`
- `command.execution_mode: "physical_execution_verified"`
- `command.physically_executed: true`
- `command.physical_execution_verified: true`
- `command.target_reached_verified: true`
- `command.service_response_success: true`

Validation for a real training dataset:

```bash
./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/validate_piper_dataset.py \
    --dataset-root piper-on-bunker/data/local/piper_xvla_target_v0
```

Synthetic or dry-run fixture validation only:

```bash
./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/validate_piper_dataset.py \
    --dataset-root piper-on-bunker/data/local/piper_xvla_target_v0 \
    --allow-nonphysical
```

Replay:

```bash
./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/replay_piper_dataset_readonly.py \
    --dataset-root piper-on-bunker/data/local/piper_xvla_target_v0
```

LeRobot export:

```bash
./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/export_lerobot_dataset.py \
    --raw-root piper-on-bunker/data/local/piper_xvla_target_v0 \
    --export-root piper-on-bunker/data/local/piper_xvla_target_v0_lerobot \
    --validate
```

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
