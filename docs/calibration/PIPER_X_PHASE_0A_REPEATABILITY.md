# PiPER-X Phase 0A Repeatability Diagnostics

Phase 0A is the non-destructive diagnostic gate before joint-zero changes, FK
edits, TCP measurement, or another eye-on-hand calibration.

It answers whether the PiPER-X state source, controller settling, and physical
endpoint repeatability are stable enough to proceed. It does not prove absolute
accuracy.

## Current Runtime

Known commands for the current PiPER-X wrist D435i MoveIt runtime:

```bash
cd /home/dase-hw101/piper-pipeline-testbed
./tools/start_piper_x_moveit_aruco_touch_runtime.sh
```

Physical stop path:

```bash
cd /home/dase-hw101/piper-pipeline-testbed
./tools/stop_piper_x_moveit_motion.sh
```

Planning-only MoveIt touch command:

```bash
python3 piper-on-bunker/scripts/run_moveit_aruco_touch.py \
  --config piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.local.yaml \
  --live \
  --planning-only \
  --sequence fixed_touch
```

Guarded physical taught-pose command, when enabled locally:

```bash
python3 piper-on-bunker/scripts/run_moveit_aruco_touch.py \
  --config piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.local.yaml \
  --live \
  --execute \
  --sequence staging_test \
  --confirm STAGING_TEST
```

Current PiPER-X execution backend:

```text
piper-on-bunker/scripts/piper_x_moveit_sdk_trajectory_controller.py
```

It exposes a `FollowJointTrajectory` action and uses the pyAgxArm PiPER-X path:

```text
ArmModel.PIPER_X
PiperFW.V189
set_follower_mode()
set_speed_percent(...)
set_motion_mode("js")
enable(255)
move_js([...six radians...])
```

Current MoveIt and state details:

```text
planning_group: arm
planning_frame: world
end_effector_link: gripper_base
authoritative_feedback_topic: /piper_x/joint_states
MoveIt joint-state relay: /piper_x/joint_states -> /joint_states
joint_order: joint1, joint2, joint3, joint4, joint5, joint6
joint_units: radians
trajectory_command_rate: 50 Hz
runtime_speed_percent: 30 by default
endpoint_tolerance: 0.03 rad
settle_timeout: 3.0 s
taught_pose_manifest: piper-on-bunker/data/local/moveit_aruco_touch/taught_poses.yaml
```

Do not use `/joint_states_single` as a PiPER-X authority. It previously
published fresh all-zero values while raw PiPER-X CAN feedback was nonzero.

## Trusted And Untrusted

Trusted enough for Phase 0A:

- passive SocketCAN feedback on `/piper_x/joint_states`;
- CAN IDs `0x2A5`, `0x2A6`, `0x2A7`;
- joint names `joint1` through `joint6`;
- joint units in radians;
- MoveIt planning with the current PiPER-X model for diagnostic planning;
- pyAgxArm PiPER-X `move_js` backend for future guarded physical cycles.

Not trusted:

- `/joint_states_single`;
- normal-PiPER `JointCtrl` state/control path;
- rejected hand-eye calibration
  `handeye_failure_diagnostics/rejected_piper_x_d435i_handeye_20260730.yaml`;
- precise camera-to-base targeting;
- PiPER-X FK/URDF as metrology;
- gripper tip estimate `[0, 0, 0.138] m` as a measured TCP;
- complete collision geometry;
- calibrated 3D door-button targeting.

## What Phase 0A Can Prove

Phase 0A separates these questions:

- software feedback repeatability: do the reported joints remain stable while
  stopped?
- controller and endpoint settling: does feedback reach the commanded taught
  pose repeatedly?
- physical endpoint repeatability: does the actual tool reference return to the
  same measured physical location?

Phase 0A cannot prove:

- joint zero is correct;
- FK or URDF is correct;
- the wrist camera hand-eye transform is correct;
- the TCP/contact-tip transform is measured;
- absolute physical target accuracy.

## Commands

Readiness:

```bash
cd /home/dase-hw101/piper-pipeline-testbed
./tools/check_piper_x_phase_0a_readiness.sh \
  --config piper-on-bunker/config/piper_x_phase_0a_repeatability.yaml
```

Observation-only, no motion:

```bash
cd /home/dase-hw101/piper-pipeline-testbed
./tools/run_piper_x_phase_0a_repeatability.sh \
  --config piper-on-bunker/config/piper_x_phase_0a_repeatability.yaml \
  --mode observe \
  --duration-s 10
```

Planning-only repeatability:

```bash
cd /home/dase-hw101/piper-pipeline-testbed
./tools/run_piper_x_phase_0a_repeatability.sh \
  --config piper-on-bunker/config/piper_x_phase_0a_repeatability.yaml \
  --mode repeat_pose \
  --pose staging \
  --cycles 3
```

Future physical repeatability command:

```bash
cd /home/dase-hw101/piper-pipeline-testbed
./tools/run_piper_x_phase_0a_repeatability.sh \
  --config piper-on-bunker/config/piper_x_phase_0a_repeatability.local.yaml \
  --mode repeat_pose \
  --pose staging \
  --cycles 3 \
  --execute \
  --confirm RUN_PHASE_0A_REPEATABILITY
```

The committed config keeps physical execution disabled. A local ignored config
must be reviewed before any physical run.

## Initial Arm Position

Start from a safe stationary non-contact staging pose:

- already in the approved taught-pose set;
- clear of the wall, marker, Bunker, inactive arm, LiDAR, and operator;
- close enough to the test pose that the movement is bounded;
- no tool or camera cable under tension;
- not manually forced while the controller is active.

The diagnostic plans from the actual measured current state. It never assumes
the arm starts at zero.

## Approved Poses

Phase 0A refuses arbitrary joint arrays. It only accepts taught poses listed in
the config allowlist:

```yaml
repeatability:
  allowed_pose_names:
    - staging
```

Add another pose only after the operator confirms it is non-contact, reachable,
not near limits, not near a singularity based on planning diagnostics, and
physically measurable from a fixed reference.

## Mechanical Checklist

Every artifact directory contains `operator_checklist.md`. It must remain
incomplete until an operator checks:

- Bunker stationary;
- PiPER-X base bolts tight;
- arm mounting plate rigid;
- no Bunker-to-arm base movement;
- wrist camera bracket rigid;
- D435i screws tight;
- USB cable strain relieved;
- gripper/tool rigid;
- no visible joint damage;
- no unusual noise or obvious backlash;
- inactive arm outside workspace;
- workspace clear;
- stop command ready;
- operator standing clear;
- physical measurement reference fixed.

Software must not fabricate this checklist as completed.

## Physical Measurement

The software cannot prove physical endpoint repeatability without an external
measurement source. The artifact includes `physical_measurements.csv` with:

```text
cycle_index, method, reference_frame, reference_description,
x_mm, y_mm, z_mm, estimated_measurement_uncertainty_mm, notes
```

Acceptable measurement methods include manual XYZ from a fixed Bunker reference,
pointer-to-grid, fixed calibration board coordinates, ruler, or dial indicator.

When no physical measurement is supplied, the classification may pass only as
`JOINT_REPEATABLE_PHYSICAL_REPEATABILITY_UNKNOWN`, not as physical repeatability
acceptable.

## Artifact Structure

Artifacts are written under:

```text
piper-on-bunker/data/local/calibration/phase_0a/<diagnostic_id>/
  manifest.yaml
  cycles.jsonl
  joint_summary.json
  physical_measurements.csv
  operator_checklist.md
  final_report.md
```

`data/local` remains ignored and should not be committed.

Schema version:

```text
piper_x.repeatability.v1
```

## Metrics

Joint feedback repeatability:

- per-joint mean: sum(values) / N;
- per-joint standard deviation: population standard deviation;
- per-joint range: max(values) - min(values);
- max absolute deviation from mean;
- target signed error: measured - target;
- target max absolute error;
- target RMS error: `sqrt(mean(error^2))`.

Endpoint settling:

- fraction reaching tolerance;
- mean and maximum settle time;
- timeout count;
- controller abort count;
- stale feedback count.

Physical endpoint repeatability:

- per-axis mean, standard deviation, and range;
- centroid: mean XYZ;
- distance from centroid for each sample;
- RMS distance from centroid;
- maximum distance from centroid;
- maximum pairwise distance;
- measurement uncertainty summary.

Repeatability is dispersion across repeated returns. Target accuracy is distance
from the intended physical location. Phase 0A is primarily repeatability.

## Classification

Possible classifications:

- `INSUFFICIENT_DATA`;
- `FEEDBACK_UNSTABLE`;
- `CONTROLLER_OR_SETTLING_INCONSISTENT`;
- `MECHANICAL_REPEATABILITY_SUSPECT`;
- `JOINT_REPEATABLE_PHYSICAL_REPEATABILITY_UNKNOWN`;
- `JOINT_AND_PHYSICAL_REPEATABILITY_ACCEPTABLE`;
- `MODEL_OR_CALIBRATION_INVESTIGATION_REQUIRED`;
- `TEST_ABORTED`.

The diagnostic is deliberately conservative:

- stale or incomplete joint feedback -> `FEEDBACK_UNSTABLE`;
- controller abort or endpoint timeout -> `CONTROLLER_OR_SETTLING_INCONSISTENT`;
- joints repeat but physical measurements spread too much ->
  `MECHANICAL_REPEATABILITY_SUSPECT`;
- joints repeat and no physical measurement exists ->
  `JOINT_REPEATABLE_PHYSICAL_REPEATABILITY_UNKNOWN`;
- joints and physical endpoint repeat -> `JOINT_AND_PHYSICAL_REPEATABILITY_ACCEPTABLE`.

Phase 0A pass only means the system is repeatable enough to proceed to Phase 0B
joint-zero verification and Phase 0C FK validation. It does not mark the arm
calibrated.

## Stop And Failure Handling

Stop command:

```bash
cd /home/dase-hw101/piper-pipeline-testbed
./tools/stop_piper_x_moveit_motion.sh
```

On Ctrl+C, timeout, preemption, controller abort, feedback loss, malformed
trajectory, or unexpected exception, the diagnostic stops further cycles,
invokes the stop/hold path when available, saves partial artifacts, and reports
whether return-to-staging was completed, failed, or not attempted.

Do not blindly command staging after a serious controller or feedback failure.
The safest response may be hold and manual recovery.

## Criteria To Proceed

Proceed to Phase 0B/0C only when:

- `/piper_x/joint_states` is fresh and stable;
- at least the configured minimum cycles completed;
- endpoint tolerance is reached consistently;
- no stale feedback or controller aborts occurred;
- physical endpoint repeatability is acceptable, or it is explicitly unknown
  and a measurement method has been scheduled;
- mechanical checklist has no blocking issues.

Stop and repair mechanics/controller first when:

- feedback is stale/incomplete;
- repeated commands do not settle;
- final joint states vary beyond thresholds;
- physical endpoint measurements spread beyond thresholds;
- mounting, camera, tool, or cable flex is observed.

## No Premature Calibration Changes

Phase 0A must not:

- call persistent zero-setting APIs;
- write motor limits;
- alter pyAgxArm joint profiles;
- edit the URDF based on visual observation;
- activate a new hand-eye transform;
- write `tcp_measured: true`;
- write `fk_validated: true`;
- write `handeye_verified: true`;
- write `calibrated_touch_validated: true`.
