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

`IDLE -> CHECK_MARKER -> MOVE_STAGING -> MOVE_PRE_TOUCH -> MOVE_TOUCH -> HOLD -> MOVE_RETRACT -> MOVE_STAGING -> COMPLETE`

Pre-touch test state machine:

`IDLE -> CHECK_MARKER -> MOVE_STAGING -> MOVE_PRE_TOUCH -> MOVE_RETRACT -> MOVE_STAGING -> COMPLETE`

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
- Required taught poses for the first contact MVP: `staging`, `pre_touch`, `touch`, `retract`
- Optional later transit pose: `home`
- Hold duration: `1.0 s`
- Motion profiles:
  - `transit`: velocity/acceleration scaling `0.10`
  - `approach`: velocity/acceleration scaling `0.05`
  - `touch`: velocity/acceleration scaling `0.02`
  - `retract`: velocity/acceleration scaling `0.05`
- Physical execution enabled in committed config: `false`
- Authoritative PiPER-X feedback topic: `/piper_x/joint_states`
- MoveIt joint-state topic: `/joint_states`, relayed only from `/piper_x/joint_states`

The normal PiPER ROS driver topic `/joint_states_single` is rejected for this PiPER-X runtime. A live failure showed raw CAN traffic on IDs including `2A2..2A8`, `251..256`, and `261..266`, while `/piper_ctrl_single_node` published fresh all-zero `/joint_states_single` and `/end_pose`. Fresh timestamps with all-zero values are not valid evidence of a stationary PiPER-X arm when the source cannot prove it decoded real PiPER-X feedback.

Current feedback implementation:

- Adapter: passive SocketCAN RX-only
- Source decoder repository: `/home/dase-hw101/Iliyas/piper-lora-teleop-bridge`
- Source decoder commit: `521c9c5fdfd9ee63bd96c0f9342fca6b2398092e`
- CAN IDs: `0x2A5`, `0x2A6`, `0x2A7`
- Raw format: two signed big-endian int32 values per frame
- Units: raw `0.001 degrees`, converted to radians
- Joint order: `0x2A5 -> joint1,joint2`, `0x2A6 -> joint3,joint4`, `0x2A7 -> joint5,joint6`

This bridge initially publishes only `/piper_x/joint_states` and `/piper_x/feedback_status`. It must not be relayed to `/joint_states` until the manual joint-by-joint verification below passes and a verification artifact is recorded.

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
  --sequence fixed_touch
```

Inspect the audit log:

`piper-on-bunker/logs/moveit_aruco_touch/mock_mission.jsonl`

## Stage 2 - Live Read-Only

Start ROS and MoveIt without physical execution:

```bash
cd ~/piper-pipeline-testbed
./tools/install_piper_x_feedback_dependency.sh
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
- Readiness reports `/piper_x/joint_states: ok` and `passive_socketcan_feedback_valid: True`.
- `/joint_states_single` is not the MoveIt state authority.
- Until manual verification is complete, `/joint_states` should not be relayed from the new bridge.
- If `/joint_states` is still published by `/piper_x_arm_joint_state_relay`, stale normal-PiPER state is still present and planning is blocked.
- Each physical joint matches RViz when moved manually through the separate proven teleoperation setup.
- The current normal-PiPER URDF is not assumed correct for PiPER-X.
- ArUco ID 6 is visible.
- The marker detector reports `DICT_ARUCO_ORIGINAL`, ID `6`, size `0.100 m`.

Manual joint-by-joint feedback verification:

1. Start the read-only runtime.
2. Print the live bridge output:

   ```bash
   docker exec -i abot-piper-noetic bash -lc '
     source /opt/ros/noetic/setup.bash
     export ROS_MASTER_URI=http://localhost:11311
     export ROS_HOSTNAME=localhost
     rostopic echo -n 1 /piper_x/joint_states
     rostopic echo -n 1 /piper_x/feedback_status
   '
   ```

3. Using the separate proven PiPER-X teleoperation system, move only `joint1` slightly and stop.
4. Verify only `joint1` changes in `/piper_x/joint_states`, with the expected sign and approximate magnitude.
5. Repeat for `joint2` through `joint6`.
6. Compare the bridge values against the working teleoperation program.
7. Save the before/after samples and hash that verification artifact before recapturing taught poses.

Do not automate this movement from this repository.

Inspect current/saved pose metadata without moving:

```bash
cd ~/piper-pipeline-testbed
./tools/inspect_piper_x_moveit_pose.sh \
  --config piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml
```

## Stage 3 - Teach Poses

Use the separate proven PiPER-X teleoperation system to manually position the arm. This repository does not move the robot during pose teaching.

Required taught poses:

- `staging`
- `pre_touch`
- `touch`
- `retract`
- optional `home`
- optional `safe_recovery`

Teach all active motion as stopped joint poses. Do not use camera pose or FK to calculate any target.

Procedure:

1. Teach `staging`: move through existing proven teleoperation to a collision-free pose near the marker but safely clear of it, stop completely, inspect current state, save `staging`.
2. Teach `pre_touch`: place the tool several centimetres before marker, stop, save `pre_touch`.
3. Teach `touch`: manually and slowly position the tool at very light marker contact using the proven teleoperation system, do not push deeply, stop, save `touch`.
4. Teach `retract`: manually move safely away from marker, stop, save `retract`.
5. Keep `home` as an optional later transit pose, not part of the first contact test.

Save a stopped pose after operator inspection:

```bash
cd ~/piper-pipeline-testbed
./tools/save_piper_x_moveit_taught_pose.sh \
  --config piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml \
  --pose-name staging \
  --ack SAVE_STOPPED_POSE
```

Repeat for `pre_touch`, `touch`, and `retract`.

Any pose captured before the `/piper_x/joint_states` bridge was verified is invalidated and must be recaptured. Future taught poses must include:

- `feedback_source_id: piper_x_passive_socketcan_feedback_v1`
- `feedback_adapter_type: passive_socketcan`
- `dependency_commit: 521c9c5fdfd9ee63bd96c0f9342fca6b2398092e`
- `source_can_ids: 0x2A5, 0x2A6, 0x2A7`
- `joint_mapping_version: piper_x_lora_feedback_2a5_2a6_2a7_raw001deg_to_rad_v1`
- source topic `/piper_x/joint_states`

The existing local manifest is not deleted, but the planner refuses old poses that lack this source identity.

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
  --sequence staging_test
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
  --sequence staging_test \
  --publish-plans-to-rviz
```

Generate both planning reports:

```bash
cd ~/piper-pipeline-testbed
./tools/check_piper_x_moveit_plans.sh
```

The controller now plans sequentially:

`actual current state -> staging -> pre_touch -> touch -> retract -> staging`

The first guarded test omits touch:

`actual current state -> staging -> pre_touch -> retract -> staging`

The old home transit path is available only as:

`--sequence home_transit_diagnostic`

It remains a planning-only diagnostic until separately verified.

Every segment must start at the previous segment's final joint state. Any discontinuity above `continuity_tolerance_rad` fails planning.

Current diagnostic timing gate:

- `max_segment_duration_s: 30.0`
- `max_mission_duration_s: 90.0`
- `min_effective_joint_velocity_rad_s: 0.001`
- `max_adjacent_joint_delta_rad: 0.10`

A plan that assigns roughly `100 s` to a non-diagnostic staging/contact segment is treated as diagnostic-only and blocked. With the current MoveIt limits, the effective approach speed is `0.5 rad/s * 0.05 = 0.025 rad/s`; a taught segment that moves one joint about `2.5 rad` will therefore take about `100 s`. Do not shorten trajectory timestamps manually. Teach `staging` near the marker so the first test avoids the folded-home to marker transition.

## Stage 5 - Guarded Physical Test

Do not run physical motion until:

- PiPER-X model/URDF is verified or a taught-joint fallback is explicitly chosen;
- marker ID 6 is fresh and visible;
- taught `staging`, `pre_touch`, `touch`, and `retract` poses are reviewed;
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
  --sequence staging_test \
  --confirm STAGING_TEST
```

That sequence is:

`staging -> pre_touch -> retract -> staging`

Only after that succeeds, enable the full touch segment at minimum speed:

```bash
--execute --sequence fixed_touch --confirm FIXED_ARUCO_TOUCH
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
