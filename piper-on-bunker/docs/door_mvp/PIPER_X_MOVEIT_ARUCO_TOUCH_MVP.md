# PiPER-X MoveIt Fixed ArUco Touch MVP

This is a constrained deterministic baseline. It does not use OpenPI, VLA inference, dynamic marker targeting, Bunker navigation, Cartesian press generation, or the rejected D435i hand-eye calibration.

## Task

Touch and retract from one fixed marker:

- ArUco dictionary: `DICT_ARUCO_ORIGINAL`
- Marker ID: `6`
- Black-square side length: `0.100 m`
- Marker mounting: vertical rigid board
- Base: fixed
- Board: fixed
- Marker location: fixed

The marker is only a visibility and identity gate for this MVP. The system does not transform `aruco_marker_frame` into `base_link` and does not use marker pose to generate the target.

## Architecture

User command -> ABot-Claw restricted action -> deterministic marker-touch state machine -> MoveIt manipulation backend -> PiPER-X.

Full-touch state machine:

`IDLE -> CHECK_MARKER -> MOVE_HOME -> MOVE_PRE_TOUCH -> MOVE_TOUCH -> HOLD -> MOVE_RETRACT -> MOVE_HOME -> COMPLETE`

Pre-touch test state machine:

`IDLE -> CHECK_MARKER -> MOVE_HOME -> MOVE_PRE_TOUCH -> MOVE_RETRACT -> MOVE_HOME -> COMPLETE`

Failure states include:

- `marker_missing`
- `wrong_marker`
- `stale_camera`
- `stale_joint_state`
- `MoveIt planning failure`
- `execution failure`
- `operator abort`
- `timeout`
- `workspace violation`
- `joint-limit violation`

## Current Configuration

Config:

`piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml`

Current conservative defaults:

- Planning group: `arm`
- End-effector link: `gripper_base`
- Active motion strategy: `taught_joint_sequence`
- Required taught poses: `home`, `pre_touch`, `touch`, `retract`
- Hold duration: `1.0 s`
- Velocity scaling: `0.05`
- Acceleration scaling: `0.05`
- Physical execution enabled in committed config: `false`

Cartesian press settings remain only under `future_cartesian_mode.enabled: false`. They are not active in v1 because PiPER-X FK/URDF is not verified.

The PiPER-X model remains unverified. The candidate URDF is:

`/home/dase-hw101/Iliyas/piper-vr-teleop/third_party/agx_arm_urdf/piper_x/urdf/piper_x_with_gripper_description.xacro`

Expected SHA256:

`0ee3f52f9acf3060a7f3e439c6e5e46b6a53d4fa509dd495045548be6b7f90cd`

Do not treat this candidate as verified yet.

The runtime stages the PiPER-X URDF asset package from:

`/home/dase-hw101/Iliyas/piper-vr-teleop/third_party/agx_arm_urdf/piper_x`

into the container under:

`/tmp/piper_x_moveit_ros/agx_arm_description/agx_arm_urdf/piper_x`

The launched MoveIt model must report:

- `robot_description_name: piper_x`
- `/piper_x_moveit/model_source: agx_arm_description piper_x_with_gripper_description.xacro`
- `/piper_x_moveit/model_verified: false`

If RViz shows the normal PiPER arm, close that RViz instance and reopen it with the PiPER-X helper below. Old RViz launch files from `piper_with_gripper_moveit` use the normal-PiPER visual setup and are not the correct viewer for this MVP.

## Existing MoveIt Support Audit

Existing PiPER MoveIt material in this repository and the ABot-Claw robot layer points to:

- MoveIt service names: `/joint_moveit_ctrl_arm`, `/joint_moveit_ctrl_endpose`, `/joint_moveit_ctrl_gripper`, `/joint_moveit_ctrl_piper`
- Service type: `moveit_ctrl/JointMoveitCtrl`
- Historical planning group: `arm`
- Historical end-effector link for TCP verification: `gripper_tcp`
- Historical active arm joints: `joint1` through `joint6`
- Historical planning frame in earlier normal-PiPER tests: `dummy_link`

This MVP records those findings but does not claim the model is correct for PiPER-X. The current PiPER-X FK mismatch may also invalidate ordinary MoveIt Cartesian behavior until the correct robot model is verified.

The active PiPER-X MoveIt runtime does not use the historical `gripper_tcp` endpoint for target generation. For the taught-joint sequence, `gripper_base` is the configured informational endpoint, and all motion targets are joint configurations.

## Rejected Calibration Boundary

Rejected calibration:

`handeye_failure_diagnostics/rejected_piper_x_d435i_handeye_20260730.yaml`

It showed about `0.365 m` false fixed-marker motion during validation. Do not publish it for execution. Do not use `base_link -> wrist_camera_color_optical_frame` from that calibration for physical targeting.

## Stage 1 - Offline/Mock

Run tests:

```bash
cd ~/piper-pipeline-testbed
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=piper-on-bunker/src \
  python3 -m pytest -q piper-on-bunker/tests/test_moveit_aruco_touch_mvp.py
```

Run a mock mission:

```bash
cd ~/piper-pipeline-testbed
python3 piper-on-bunker/scripts/run_moveit_aruco_touch.py \
  --config piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml \
  --mock \
  --mock-taught-poses \
  --planning-only \
  --sequence full_touch
```

Inspect the audit log:

`piper-on-bunker/logs/moveit_aruco_touch/mock_mission.jsonl`

## Stage 2 - Live Read-Only

Start ROS and MoveIt without physical execution:

```bash
cd ~/piper-pipeline-testbed
./tools/start_piper_x_moveit_aruco_touch_runtime.sh
```

Check readiness:

```bash
cd ~/piper-pipeline-testbed
./tools/check_piper_x_moveit_aruco_touch_readiness.sh
```

Open RViz with the PiPER-X model path:

```bash
cd ~/piper-pipeline-testbed
./tools/open_piper_x_moveit_aruco_touch_rviz.sh
```

Verify:

- MoveIt model appears in RViz as PiPER-X, not the normal PiPER arm.
- Readiness reports `robot_description_name: piper_x`.
- Each physical joint matches RViz when moved manually through the separate proven teleoperation setup.
- The current normal-PiPER URDF is not assumed correct for PiPER-X.
- ArUco ID 6 is visible.
- The marker detector reports `DICT_ARUCO_ORIGINAL`, ID `6`, size `0.100 m`.

Inspect current/saved pose metadata without moving:

```bash
cd ~/piper-pipeline-testbed
./tools/inspect_piper_x_moveit_pose.sh \
  --config piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml
```

## Stage 3 - Teach Poses

Use the separate proven PiPER-X teleoperation system to manually position the arm. This repository does not move the robot during pose teaching.

Required taught poses:

- `home`
- `pre_touch`
- `touch`
- `retract`
- optional `safe_recovery`

Teach all active motion as stopped joint poses. Do not use camera pose or FK to calculate any target.

Procedure:

1. Teach `home`: move through existing proven teleoperation, stop completely, inspect current state, save `home`.
2. Teach `pre_touch`: place the tool several centimetres before marker, stop, save `pre_touch`.
3. Teach `touch`: manually and slowly position the tool at very light marker contact using the proven teleoperation system, do not push deeply, stop, save `touch`.
4. Teach `retract`: manually move safely away from marker, stop, save `retract`.
5. Return to `home` manually.

Save a stopped pose after operator inspection:

```bash
cd ~/piper-pipeline-testbed
./tools/save_piper_x_moveit_taught_pose.sh \
  --config piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml \
  --pose-name home \
  --ack SAVE_STOPPED_POSE
```

Repeat for `pre_touch`, `touch`, and `retract`.

## Stage 4 - Planning-Only

After taught poses exist, run live check-only first:

```bash
cd ~/piper-pipeline-testbed
python3 piper-on-bunker/scripts/run_moveit_aruco_touch.py \
  --config piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml \
  --live \
  --check-only
```

Then run planning-only for the safe first sequence:

```bash
cd ~/piper-pipeline-testbed
python3 piper-on-bunker/scripts/run_moveit_aruco_touch.py \
  --config piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml \
  --live \
  --planning-only \
  --sequence pre_touch_test
```

Planning-only must report:

- active robot model;
- planning group;
- end-effector link;
- marker status;
- taught pose source;
- trajectory points;
- estimated duration;
- maximum joint delta;
- maximum adjacent joint delta;
- first/final trajectory positions;
- target error;
- segment continuity error;
- effective derived velocity and acceleration;
- velocity and acceleration scaling;
- why execution is blocked.

Inspect every planned segment in RViz before any physical test:

```bash
cd ~/piper-pipeline-testbed
python3 piper-on-bunker/scripts/run_moveit_aruco_touch.py \
  --config piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml \
  --live \
  --planning-only \
  --sequence pre_touch_test \
  --publish-plans-to-rviz
```

Generate both planning reports:

```bash
cd ~/piper-pipeline-testbed
./tools/check_piper_x_moveit_plans.sh
```

The controller now plans sequentially:

`actual current state -> home -> pre_touch -> touch -> retract -> home`

Every segment must start at the previous segment's final joint state. Any discontinuity above `continuity_tolerance_rad` fails planning.

Current diagnostic timing gate:

- `max_segment_duration_s: 30.0`
- `max_mission_duration_s: 90.0`
- `min_effective_joint_velocity_rad_s: 0.001`
- `max_adjacent_joint_delta_rad: 0.10`

A plan that assigns roughly `100 s` to a segment is treated as diagnostic-only and blocked. With the current MoveIt limits, the effective joint speed is `0.5 rad/s * 0.05 = 0.025 rad/s`; a taught segment that moves one joint about `2.5 rad` will therefore take about `100 s`. Do not shorten trajectory timestamps manually. Either review/reteach poses, then rerun planning, or explicitly revise the conservative velocity configuration after operator review.

## Stage 5 - Guarded Physical Test

Do not run physical motion until:

- PiPER-X model/URDF is verified or a taught-joint fallback is explicitly chosen;
- marker ID 6 is fresh and visible;
- taught `home`, `pre_touch`, `touch`, and `retract` poses are reviewed;
- collision scene is checked;
- workspace is clear;
- a soft/blunt tool is attached if contact is possible;
- emergency stop is ready.

First physical test must omit the touch segment:

```bash
cd ~/piper-pipeline-testbed
python3 piper-on-bunker/scripts/run_moveit_aruco_touch.py \
  --config piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml \
  --live \
  --execute \
  --sequence pre_touch_test \
  --confirm PRE_TOUCH_TEST
```

That sequence is:

`home -> pre_touch -> retract -> home`

Only after that succeeds, enable the full touch segment at minimum speed:

```bash
--execute --sequence full_touch --confirm FIXED_ARUCO_TOUCH
```

The committed config keeps physical execution disabled by default, so these commands are intentionally not ready to move the robot yet.

## Restricted ABot-Claw API

Only these semantic actions should be exposed:

- `check_aruco_target`
- `plan_aruco_touch`
- `execute_aruco_touch`
- `retract_from_aruco`
- `move_arm_home`
- `get_aruco_touch_status`

Do not expose arbitrary joint targets or arbitrary Cartesian poses through the ABot-Claw mission API.

## Future Dynamic Targeting

Later versions may add:

- valid hand-eye calibration;
- marker pose transformed into `base_link`;
- dynamically generated MoveIt target;
- visual correction;
- VLA/OpenPI backend.

None of those belong in this fixed-position MVP.
