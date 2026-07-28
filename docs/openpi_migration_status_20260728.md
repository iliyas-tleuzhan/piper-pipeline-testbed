# OpenPI PiPER Migration Status - 2026-07-28

This note records the current state of the PiPER-on-Bunker manipulation migration after replacing the default LAP-3B + MoveIt execution path with the OpenClaw + OpenPI/π0.5 direct joint-control architecture.

It is intentionally practical: what exists, what was changed, what worked, what failed, and what still has to happen before real learned VLA motion is possible.

## Executive Summary

The default manipulation direction is now:

```text
OpenClaw semantic task phases
-> PiPER OpenPI/π0.5 policy interface
-> direct absolute PiPER joint action chunks
-> /piper_joint_commands
-> PiPER ROS driver
-> CAN
-> physical PiPER arm
```

The old default path was:

```text
camera + instruction
-> LAP-3B Cartesian action horizon
-> MoveIt Cartesian/IK planning
-> FollowJointTrajectory
-> piper_trajectory_bridge
-> /piper_joint_commands
-> PiPER ROS driver
-> CAN
-> physical PiPER arm
```

The new default removes LAP-3B and MoveIt from the default manipulation path. The legacy code and logs remain in the repository for archived experiments, but the default stack now describes and starts the non-MoveIt OpenPI direct joint architecture.

Important current limitation: there is not yet a trained PiPER-compatible π0.5 checkpoint. The system can record data, validate metadata, run live observation shadow tests, and replay recorded demonstrations through the direct joint path. It cannot honestly run learned physical VLA motion until a PiPER-specific checkpoint has been trained and validated.

## Why The Migration Happened

The LAP-3B + MoveIt path became too complex and brittle for the intended manipulation architecture:

- LAP-3B produced Cartesian deltas, not direct PiPER joint actions.
- MoveIt then had to convert those deltas into joint motion using Cartesian planning, IK, OMPL, and trajectory execution.
- The system repeatedly hit planning failures, IK branch jumps, controller availability issues, and cases where RViz looked plausible but the physical arm did not move.
- Debugging involved many layers: LAP output, target conversion, MoveIt planning, FollowJointTrajectory, bridge execution, ROS controllers, PiPER driver, CAN.

The intended VLA architecture is different:

- OpenClaw plans semantic phases such as approach, grasp, transport, release.
- π0.5 chooses the manipulation motion directly as PiPER absolute joint action chunks.
- A thin runtime validates, times, and streams those chunks.
- No IK, Cartesian target conversion, MoveIt, OMPL, or trajectory fallback should choose a different motion after the VLA.

## Current Repository State

Main repository:

```text
~/piper-pipeline-testbed
branch: main
latest relevant commit: 6d9fbfc Enable PiPER before demo replay execution
```

Recent OpenPI migration commits:

```text
6d9fbfc Enable PiPER before demo replay execution
54602c9 Add recorded PiPER demo replay runner
66e959c Add OpenPI smoke checkpoint metadata tooling
8f2ee25 Fix OpenPI runner preflight CLIs
e5febab Add OpenPI episode inspection smoke tool
e439240 Fail fast when episode recorder observations are missing
2a456d5 Add OpenPI PiPER teleop episode recorder
b8b6aa5 Clarify OpenPI physical metadata refusal
b29d0f2 Add guarded OpenPI physical eligibility checks
7ad0b5c Add live OpenPI PiPER shadow runner
482245c Add OpenPI PiPER direct joint runtime
```

Legacy archival marker:

```text
archive-lap-moveit-default-20260727
```

Known unrelated dirty work at the time of writing:

```text
tools/start_current_windows.sh
```

That file is untracked and unrelated to this OpenPI migration note.

## Main Components Added

### Semantic Phase Planning

OpenClaw now plans manipulation as restricted semantic phases instead of generic motion commands.

Current core phases for the cup task:

```text
approach  -> APPROACH_OBJECT
grasp     -> GRASP_OBJECT
transport -> TRANSPORT_OBJECT
release   -> RELEASE_OBJECT
```

Example task:

```text
Move the red cup onto the paper.
```

Expected phase prompts:

```text
Approach the red cup and finish in a grasp-ready pose.
Align with and securely grasp the red cup.
Lift the red cup and move it above the paper.
Place the red cup on the paper, release it, and retract.
```

Relevant code:

```text
piper-on-bunker/src/piper_on_bunker/control/phase_orchestrator.py
piper-on-bunker/tests/test_openpi_semantic_plan.py
```

### OpenPI PiPER Policy Interface

The OpenPI policy interface defines the PiPER observation/action contract.

Observation schema:

```json
{
  "observation/exterior_image": "uint8 RGB image",
  "observation/wrist_image": "uint8 RGB image or zero image when --no-wrist",
  "observation/state": [
    "joint1_current_rad",
    "joint2_current_rad",
    "joint3_current_rad",
    "joint4_current_rad",
    "joint5_current_rad",
    "joint6_current_rad",
    "gripper_current_m"
  ],
  "prompt": "phase-specific instruction"
}
```

Action schema:

```json
[
  "joint1_absolute_target_rad",
  "joint2_absolute_target_rad",
  "joint3_absolute_target_rad",
  "joint4_absolute_target_rad",
  "joint5_absolute_target_rad",
  "joint6_absolute_target_rad",
  "gripper_absolute_target_m"
]
```

Action chunks have shape:

```text
[action_horizon, 7]
```

Current constants:

```text
OpenPI commit: 15a9616a00943ada6c20a0f158e3adb39df2ccac
Base checkpoint: gs://openpi-assets/checkpoints/pi05_base
Action semantics: absolute_piper_joint_targets
Joint order: joint1, joint2, joint3, joint4, joint5, joint6, gripper
```

Relevant code:

```text
piper-on-bunker/src/piper_on_bunker/policies/openpi_piper_policy.py
piper-on-bunker/config/openpi_piper.yaml
piper-on-bunker/openpi_extension/
```

### Direct Joint Phase Executor

The executor receives model-selected absolute joint chunks and streams them to `/piper_joint_commands`.

It does not:

- calculate IK
- create Cartesian goals
- call MoveIt
- call FollowJointTrajectory
- use OMPL
- choose a different robot path

It does:

- check shape and joint order
- check finite values
- check joint/gripper limits
- check stale observations and stale policy responses
- interpolate to hardware publishing frequency
- use a command-authority lock
- publish `sensor_msgs/JointState` commands to `/piper_joint_commands`

Relevant code:

```text
piper-on-bunker/src/piper_on_bunker/control/piper_joint_phase_executor.py
piper-on-bunker/src/piper_on_bunker/control/phase_chunk_buffer.py
piper-on-bunker/tests/test_openpi_phase_executor.py
```

### Live Shadow Runner

The live shadow runner verifies that the current ROS stack can capture:

- PiPER joint state
- exterior camera image
- wrist camera image or zero wrist substitute
- semantic phase plan
- shadow direct action validation

Command:

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/run_openpi_piper_live_shadow.py \
    --instruction "Move the red cup onto the paper." \
    --phase-id approach \
    --no-wrist
```

Relevant code:

```text
piper-on-bunker/scripts/run_openpi_piper_live_shadow.py
piper-on-bunker/src/piper_on_bunker/hardware/live_openpi_observation.py
```

### Guarded Physical Runner

The guarded runner is the future physical execution entry point for real PiPER-compatible OpenPI checkpoints.

It refuses physical execution unless:

- `--execute` is passed
- checkpoint metadata exists
- `piper_compatible: true`
- OpenPI commit matches
- action schema matches
- normalization metadata exists
- dataset provenance exists
- offline validation passed
- gripper verification passed
- live observation is fresh

Command for preflight only:

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/run_openpi_piper_guarded_physical.py \
    --instruction "Move the red cup onto the paper." \
    --phase-id approach \
    --no-wrist \
    --checkpoint-metadata piper-on-bunker/data/local/openpi_smoke_checkpoint/piper_checkpoint_metadata.json
```

Relevant code:

```text
piper-on-bunker/scripts/run_openpi_piper_guarded_physical.py
piper-on-bunker/scripts/validate_openpi_piper_checkpoint.py
piper-on-bunker/src/piper_on_bunker/policies/checkpoint_metadata.py
piper-on-bunker/tests/test_openpi_checkpoint_metadata.py
```

### Episode Recorder

A PiPER OpenPI recorder was added to collect demonstrations from teleoperation.

It records:

- exterior RGB image
- wrist RGB image or substitute
- current 6 arm joints
- current gripper feedback
- commanded 6 arm targets
- commanded gripper target
- instruction
- phase ID
- phase prompt
- timestamps
- command source
- operator notes
- validity/skew status

Typical command:

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/record_openpi_piper_episode.py \
    --instruction "Move the red cup onto the paper." \
    --phase-id approach \
    --action-source can \
    --can-interface can0 \
    --no-wrist \
    --gripper-raw-closed 0 \
    --gripper-raw-open 70000 \
    --operator-notes "wired teleop approach demo"
```

Relevant code:

```text
piper-on-bunker/scripts/record_openpi_piper_episode.py
piper-on-bunker/scripts/inspect_openpi_piper_episode.py
piper-on-bunker/tests/test_openpi_episode_inspector.py
```

### Recorded Four-Phase Demo

One full task was recorded as four phase-specific episodes:

```text
piper_approach_1785209728_086eb530   512 valid frames
piper_grasp_1785209438_694d09e6      885 valid frames
piper_transport_1785209596_e4b01db2  537 valid frames
piper_release_1785209651_af959f98    848 valid frames
```

Total:

```text
2782 valid frames
0 invalid frames
0 dropped command samples
```

Stored under:

```text
piper-on-bunker/data/local/openpi_episodes/
```

Generated smoke dataset:

```text
piper-on-bunker/data/local/openpi_smoke_dataset/
```

Inspect/convert command:

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/inspect_openpi_piper_episode.py \
    piper-on-bunker/data/local/openpi_episodes \
    --write-lerobot-like piper-on-bunker/data/local/openpi_smoke_dataset
```

### Smoke Checkpoint Metadata

Smoke metadata was generated from the four demos so the pipeline can test checkpoint metadata handling.

Path:

```text
piper-on-bunker/data/local/openpi_smoke_checkpoint/piper_checkpoint_metadata.json
```

Important: this is not a trained checkpoint.

It intentionally says:

```json
{
  "checkpoint": "dev://recorded-openpi-smoke/one-four-phase-demo",
  "piper_compatible": false,
  "physical_execution_allowed": false
}
```

Validator output correctly rejects it for hardware:

```text
piper_compatible must be true
offline_validation.passed must be true
gripper hardware verification must pass
```

Command to regenerate:

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/make_openpi_piper_smoke_checkpoint_metadata.py \
    --episode-root piper-on-bunker/data/local/openpi_episodes \
    --output-dir piper-on-bunker/data/local/openpi_smoke_checkpoint
```

### Recorded Demo Replay Runner

Because no trained PiPER π0.5 checkpoint exists yet, a recorded demo replay runner was added to prove the non-MoveIt direct joint path.

It is explicitly not π0.5 inference.

It replays recorded joint actions through:

```text
recorded action rows
-> run_openpi_piper_demo_replay.py
-> /piper_joint_commands
-> piper_ctrl_single_node.py
-> CAN
-> PiPER arm
```

Command:

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/run_openpi_piper_demo_replay.py \
    --phase-id approach \
    --no-wrist \
    --max-samples 0 \
    --execute
```

The runner now calls `/enable_srv` automatically before publishing. This was added because the PiPER ROS driver ignores `/piper_joint_commands` unless its internal enable flag is true.

Relevant code:

```text
piper-on-bunker/scripts/run_openpi_piper_demo_replay.py
```

## Current Hardware/Runtime Stack

Local container:

```text
abot-piper-noetic
```

Remote GPU server:

```text
iliyas@192.168.1.104
```

Known service ports:

```text
8012 Spatial Memory
8013 YOLO
8014 VLAC
8015 GraspAnything / depth fallback
8016 LAP legacy
8017 intended OpenPI policy service
```

Important ROS topics/services:

```text
/joint_states_single    live PiPER joints and gripper feedback
/piper_joint_commands   direct arm joint command topic
/enable_srv             PiPER driver enable service
/gripper_srv            PiPER gripper service
/arm_status             PiPER driver status
/table_camera/color/image_raw
/table_camera/color/image_rect_color
/table_camera/color/camera_info
```

Expected lower-stack startup:

```bash
cd ~/ABot-Claw
./start_abotclaw_all.sh --lower-only --restart --no-attach
```

Expected full-stack startup:

```bash
cd ~/ABot-Claw
./start_abotclaw_full_stack.sh --restart
```

Expected status:

```bash
cd ~/ABot-Claw
./start_abotclaw_full_stack.sh --status
```

The current default startup says:

```text
Default manipulation path: OpenClaw semantic phases -> OpenPI pi0.5 direct joint chunks -> /piper_joint_commands.
Legacy MoveIt trajectory services are not started. Use --legacy-moveit only for archived experiments.
```

## Camera Calibration State

The D555/table camera calibration was brought up for the non-MoveIt runtime.

Important TF:

```text
base_link -> table_camera_color_optical_frame
```

Saved hand-eye TF sample reported:

```text
translation: [-0.022472715844708044, 0.09063024791490812, 0.1431389227317595]
rotation xyzw: [0.32977342452790814, -0.22294097429222082, 0.08080811456157992, 0.913792568955217]
```

Validated transform output:

```text
Translation: [-0.022, 0.091, 0.143]
Rotation quaternion: [0.330, -0.223, 0.081, 0.914]
RPY degree: [39.677, -27.435, 0.042]
```

The full-stack status later showed:

```text
robot_state_publisher: ok
base_link -> gripper_base TF: ok
RealSense RGB: ok
RealSense aligned depth: ok
camera_info: ok
image_proc rectification: ok
saved hand-eye TF publisher: running
saved camera TF: valid
```

## What Worked

### OpenClaw Phase Planning

The task:

```text
Move the red cup onto the paper.
```

is converted into:

```text
approach -> grasp -> transport -> release
```

with explicit preconditions and completion checks.

### Live Observation Capture

The preflight runner captured:

```text
fresh /joint_states_single
fresh /table_camera/color/image_raw
zero wrist image when --no-wrist is used
```

Example ages were around:

```text
state_age_s: ~0.005-0.010
exterior_image_age_s: ~0.08-0.11
```

### Dataset Recording

The recorder successfully captured four phase demos with valid frames and no command drops.

### Metadata Gating

The system correctly rejects demo-only metadata for physical VLA execution.

This is intentional. Public/base/demo artifacts must not be marked PiPER-compatible.

### Direct Joint Path Proof

The recorded replay runner successfully published direct joint commands through `/piper_joint_commands`.

In one observed run, after the lower stack was restarted and `/enable_srv` returned true, the live joint state matched the final recorded approach target:

```text
current joints:
[0.348409, 1.195455, -0.591317, 0.012978, 0.164200, 0.061804]

final recorded approach action:
[0.348, 1.195, -0.593, 0.013, 0.165, 0.062]

max delta: ~0.0014 rad
```

That proved the non-MoveIt command path can move the PiPER when the CAN driver is healthy.

## What Failed / Current Blocker

The main current blocker is not OpenPI code. It is the USB-CAN / PiPER driver state.

Observed symptoms:

```text
/enable_srv returns enable_response: False
piper_ctrl_single_node.py logs SEND_MESSAGE_FAILED (100017)
can0 TX errors and dropped packets increase
can0 becomes DOWN / STOPPED
ip link set can0 up returns RTNETLINK answers: Protocol error
```

Observed bad CAN state:

```text
can0: state DOWN
can state STOPPED
TX errors: 1324
TX dropped: 1324
```

This means ROS may be alive and `/piper_joint_commands` may have a subscriber, but the PiPER SDK cannot actually transmit CAN command frames.

When this happens, restarting ROS alone is not enough.

## CAN Recovery Procedure

First stop the PiPER driver:

```bash
docker exec abot-piper-noetic bash -lc 'pkill -f piper_ctrl_single_node.py || true'
```

Try resetting the USB-CAN adapter. The adapter was observed as:

```text
Bus 003 Device 009: ID 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter
USB path: 3-6
```

Reset:

```bash
echo -n '3-6' | sudo tee /sys/bus/usb/drivers/usb/unbind
sleep 2
echo -n '3-6' | sudo tee /sys/bus/usb/drivers/usb/bind
sleep 2
```

Then:

```bash
ip -details link show type can
```

Bring CAN up:

```bash
sudo ip link set can0 down || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
ip -statistics -details link show can0
```

Expected:

```text
state UP
can state ERROR-ACTIVE
```

If this still reports `RTNETLINK answers: Protocol error`, physically unplug and replug the USB-CAN adapter, then repeat the `ip link set can0 ... up` commands.

Then restart the lower stack:

```bash
cd ~/ABot-Claw
./start_abotclaw_all.sh --lower-only --restart --no-attach
```

Verify:

```bash
docker exec -it abot-piper-noetic bash -lc '
source /opt/ros/noetic/setup.bash
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash
rosservice call /enable_srv "enable_request: true"
rostopic echo -n 1 /arm_status
rostopic info /piper_joint_commands
rostopic echo -n 1 /joint_states_single
'
```

Good signs:

```text
enable_response: True
err_code: 0
teach_status: 0
/piper_joint_commands has subscriber /piper_ctrl_single_node...
```

## Why Replay Sometimes Looked Like It Did Not Move

There were three separate issues:

1. The PiPER ROS driver ignores joint commands unless `/enable_srv` has succeeded.

   The driver code gates command forwarding:

   ```python
   if self.GetEnableFlag():
       self.piper.JointCtrl(...)
   ```

   The replay runner was patched to call `/enable_srv` automatically before publishing.

2. The PiPER SDK CAN socket can go stale or fail.

   In that state, ROS messages are published, but the driver logs:

   ```text
   JointCtrl_J12 send failed: SEND_MESSAGE_FAILED (100017)
   JointCtrl_J34 send failed: SEND_MESSAGE_FAILED (100017)
   JointCtrl_J56 send failed: SEND_MESSAGE_FAILED (100017)
   ```

3. Sometimes the arm is already near the target.

   After one test, live feedback was already at the final approach target, so replaying approach again would produce little visible motion until reset to the approach start.

## Commands For Current Development

### Status

```bash
cd ~/ABot-Claw
./start_abotclaw_full_stack.sh --status
```

### Lower Stack Restart

```bash
cd ~/ABot-Claw
./start_abotclaw_all.sh --lower-only --restart --no-attach
```

### Live Shadow

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/run_openpi_piper_live_shadow.py \
    --instruction "Move the red cup onto the paper." \
    --phase-id approach \
    --no-wrist
```

### Guarded Preflight With Smoke Metadata

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/run_openpi_piper_guarded_physical.py \
    --instruction "Move the red cup onto the paper." \
    --phase-id approach \
    --no-wrist \
    --checkpoint-metadata piper-on-bunker/data/local/openpi_smoke_checkpoint/piper_checkpoint_metadata.json
```

Expected:

```text
eligible_for_physical_execution: false
physical_motion_performed: false
```

### Recorded Demo Replay

Approach:

```bash
cd ~/piper-pipeline-testbed

./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/run_openpi_piper_demo_replay.py \
    --phase-id approach \
    --no-wrist \
    --max-samples 0 \
    --execute
```

Grasp:

```bash
./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/run_openpi_piper_demo_replay.py \
    --phase-id grasp \
    --no-wrist \
    --max-samples 0 \
    --execute
```

Transport:

```bash
./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/run_openpi_piper_demo_replay.py \
    --phase-id transport \
    --no-wrist \
    --max-samples 0 \
    --execute
```

Release:

```bash
./tools/run_in_noetic_container.sh \
  python3 piper-on-bunker/scripts/run_openpi_piper_demo_replay.py \
    --phase-id release \
    --no-wrist \
    --max-samples 0 \
    --execute
```

These commands replay recorded data. They are not learned OpenPI policy execution.

## Tests

Focused OpenPI test set:

```bash
cd ~/piper-pipeline-testbed

PYTHONPATH=piper-on-bunker/src python3 -m pytest \
  piper-on-bunker/tests/test_openpi_checkpoint_metadata.py \
  piper-on-bunker/tests/test_openpi_runner_cli.py \
  piper-on-bunker/tests/test_openpi_piper_policy.py \
  piper-on-bunker/tests/test_openpi_phase_executor.py \
  piper-on-bunker/tests/test_openpi_semantic_plan.py
```

Last observed result after the OpenPI runner changes:

```text
17 passed
```

## What Still Needs To Be Done

### 1. Stabilize CAN / PiPER Driver Operations

Before any serious policy work, the CAN adapter needs a reliable recovery procedure.

Recommended improvements:

- Add a script that checks `can0` state before replay/execution.
- Fail early if `can0` is not `UP / ERROR-ACTIVE`.
- Detect increasing TX errors/drops before command streaming.
- Document when physical USB replug is required.
- Avoid starting PiPER driver before `can0` is confirmed healthy.

### 2. Convert The Recorded Dataset Properly

The current smoke dataset is enough for plumbing tests, but not enough for training.

Needed:

- Convert raw episodes to the current official OpenPI-compatible LeRobot format.
- Preserve exact feature names.
- Preserve phase prompts.
- Preserve joint/action units.
- Include camera metadata.
- Compute hashes and dataset provenance.

### 3. Compute PiPER Normalization Statistics

Do not reuse DROID, LIBERO, ALOHA, LAP, or public checkpoint statistics.

Needed:

- PiPER-specific state/action normalization.
- Exact joint order.
- Exact gripper semantics.
- Same transforms for dataset, training, inference, and runtime.

### 4. Fine-Tune π0.5

Use the official OpenPI repository at the pinned commit.

Starting point:

```text
gs://openpi-assets/checkpoints/pi05_base
```

But it remains:

```text
piper_compatible: false
```

until fine-tuned and validated for this PiPER embodiment.

### 5. Offline Validation

Before real VLA execution, evaluate the trained checkpoint on held-out PiPER episodes.

Required checks:

- predicted joint target error
- gripper prediction error
- maximum initial jump
- maximum adjacent action jump
- implied velocity
- implied acceleration
- jerk
- joint limit violations
- phase consistency
- missing wrist camera behavior
- latency sensitivity

Only after passing should checkpoint metadata contain:

```json
{
  "piper_compatible": true
}
```

### 6. Deploy Policy Service On Port 8017

Target:

```text
iliyas@192.168.1.104
port 8017
```

Service should expose:

- health
- readiness
- metadata
- checkpoint metadata
- inference

The local guarded runner should then call that service for actual action chunks.

### 7. Physical VLA Motion

Physical VLA motion is allowed only after:

- PiPER-specific checkpoint exists
- metadata validates
- offline validation passes
- gripper path is proven
- `can0` is healthy
- `/enable_srv` succeeds
- command authority is exclusive
- shadow mode passes

## Current Honest Status

Done:

- Default architecture moved away from LAP/MoveIt.
- OpenClaw semantic phases exist.
- PiPER OpenPI observation/action contract exists.
- Direct joint executor exists.
- Live shadow/preflight exists.
- Dataset recorder exists.
- Four phase demos recorded.
- Smoke metadata exists.
- Recorded demo replay exists.
- The direct `/piper_joint_commands` path was shown to be capable of reaching a recorded target when CAN is healthy.

Not done:

- No trained PiPER-compatible π0.5 checkpoint yet.
- No OpenPI policy server on port 8017 serving a PiPER checkpoint yet.
- No physical learned VLA execution yet.
- CAN adapter is currently the major operational instability.

The next real engineering milestone is:

```text
raw four-phase demos
-> official OpenPI dataset conversion
-> PiPER normalization stats
-> π0.5 fine-tuning
-> offline validation
-> metadata piper_compatible=true
-> shadow service test
-> guarded physical VLA execution
```

