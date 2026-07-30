# PiPER-X VLA And Hand-Eye Pause Handoff

Date: 2026-07-30

Status: paused, not abandoned.

Latest completed technical commit before this pause handoff:

`07d7331ef1ac570681f178952a873dd912b7bb76` - `Fix PiPER-X FK model before hand-eye recalibration`

The next immediate project phase is the ABot-Claw + MoveIt door-opening MVP. Do not continue PiPER-X D435i hand-eye calibration or OpenPI/VLA execution until the FK verification gate below passes.

## Current Architecture Context

The paused VLA direction is:

User task -> OpenClaw semantic phases -> OpenPI pi0.5 direct PiPER-X joint chunks -> direct joint executor -> PiPER-X driver/CAN.

For the PiPER-X ArUco touch data-collection path, the intended embodiment is:

- Robot: single fixed-base AgileX PiPER-X arm
- Camera: wrist-mounted Intel RealSense D435i
- Policy observation: wrist RGB image, deterministic zero exterior image, 7D state, language prompt
- Policy action: 7D absolute joint targets `[joint1, joint2, joint3, joint4, joint5, joint6, gripper]`
- Gripper mode for the first checkpoint: fixed hold

This work remains data-collection/training-pipeline readiness only. No PiPER-X-compatible checkpoint has been trained or validated.

## Completed Work

- Added PiPER-X ArUco touch OpenPI collection scaffolding.
- Added a separate PiPER-X robot profile instead of repurposing the normal PiPER profile.
- Added wrist-only observation handling with zero exterior image placeholder.
- Added passive episode recording, inspection, labeling, split and conversion scaffolding.
- Added fixed-gripper-hold metadata and validation.
- Added guarded shadow and physical refusal paths for unvalidated PiPER-X checkpoints.
- Added D435i wrist hand-eye calibration launch helpers using `abot-piper-noetic`.
- Added a custom read-only ArUco detector for calibration, including `DICT_ARUCO_ORIGINAL`, ID 6, 0.100 m marker size, debug image output, and solvePnP fallback for OpenCV builds without `estimatePoseSingleMarkers`.
- Added raw and rectified image-geometry modes. Rectified mode uses CameraInfo `P` left 3x3 with zero distortion. Raw mode uses `K` and `D`.
- Added read-only FK capture and offline FK mismatch analysis tools.
- Added fail-closed PiPER-X model/URDF gates before hand-eye collection.
- Added publish protection so rejected hand-eye calibration cannot be published by default.

## Rejected Calibration

The saved D435i hand-eye calibration was explicitly rejected and must remain rejected:

`handeye_failure_diagnostics/rejected_piper_x_d435i_handeye_20260730.yaml`

Relevant details recorded there:

- Source saved YAML: `/root/.ros/easy_handeye/piper_x_d435i_wrist_eye_on_hand.yaml`
- Transform: `gripper_base -> wrist_camera_color_optical_frame`
- Samples: 23
- Algorithm: OpenCV Tsai-Lenz via easy_handeye
- Validation result: failed
- Maximum observed fixed-marker drift: approximately `0.365 m`
- `fk_verified: false`
- `handeye_collection_allowed: false`
- `recalibration_allowed: false`

Do not overwrite this file. Do not publish this transform for execution.

## FK Mismatch Evidence

The captured diagnostics support these conclusions:

- `/joint_states_single` and `/joint_states` match exactly for joints 1 through 6.
- The joint relay is numerically copying feedback correctly.
- `/end_pose` and `base_link -> gripper_base` disagree by about 6-13 cm depending on pose.
- The orientation discrepancy also changes with configuration.
- The mismatch is not explained by one constant TCP transform.
- `aruco_marker_frame` disappears when the marker is not detected, so stale marker TF was not the source of the validation failure.
- The primary blocker is PiPER-X FK/URDF/frame semantics.

Observed false fixed-marker motion after the rejected calibration included approximately:

- Baseline: `[-0.239, -0.047, 0.932]`
- Other poses: `[-0.458, -0.099, 0.949]`
- Other poses: `[-0.205, -0.003, 0.885]`
- Other poses: `[-0.067, 0.090, 0.810]`
- Other poses: `[0.057, 0.020, 0.730]`

Maximum observed drift was approximately `0.365 m`.

## PiPER-X URDF Candidate

Candidate path:

`/home/dase-hw101/Iliyas/piper-vr-teleop/third_party/agx_arm_urdf/piper_x/urdf/piper_x_description.urdf`

Expected SHA256:

`34126caac7d5b37bc2409f337ac246afbe0bb8cd47fc9f16df5038f19bd21e3a`

This URDF is a candidate only. It is not verified for the current physical arm.

## Current Safety Gates

The calibration launcher must fail closed unless all required PiPER-X model inputs are explicitly provided and verified:

- `PHYSICAL_MODEL_ID`
- `FIRMWARE_VERSION`
- `ROBOT_URDF_PATH`
- `ROBOT_URDF_SHA256`
- `PIPER_X_FK_VERIFIED=true`

The sampling/backend readiness must remain blocked unless:

- physical model is identified;
- firmware is known;
- selected URDF path/hash are known;
- joint mapping signs, offsets and units are verified;
- controller/SDK FK agrees with ROS TF across multiple stopped poses;
- endpoint frame semantics are known;
- image geometry mode matches the camera topic;
- marker dictionary, ID and size are correct;
- marker pose is live;
- wrist bracket rigidity is verified.

Warning: do not set `PIPER_X_FK_VERIFIED=true` manually and do not bypass the verification gate.

## Unresolved Blockers

- Exact PiPER-X physical model identification remains unresolved.
- Exact firmware string remains unresolved.
- `/end_pose` endpoint semantics are not proven to be `gripper_base`.
- Controller/SDK FK support did not yet provide a trusted comparison path.
- Correct PiPER-X URDF, joint axes, signs, offsets and frame semantics are not verified.
- The normal PiPER URDF is not accepted as correct for this PiPER-X setup.
- Hand-eye recalibration remains blocked.
- OpenPI/VLA physical execution remains blocked.

## Resume Diagnostics Later

Use read-only diagnostics only until FK is verified:

```bash
cd ~/piper-pipeline-testbed

POSE_LABEL=pose_1 ./tools/check_piper_x_fk_consistency.sh

PYTHONPATH=piper-on-bunker/src \
  python3 piper-on-bunker/scripts/analyze_piper_x_fk_diagnostics.py \
    piper-on-bunker/data/local/piper_x_fk_diagnostics/*/*.json
```

To inspect the calibration runtime state later:

```bash
cd ~/piper-pipeline-testbed
./tools/check_piper_x_d435i_handeye_readiness.sh
./tools/check_piper_x_d435i_aruco_image.sh
```

Do not restart sampling or compute a new calibration until the FK gate passes.

## Next Immediate Project

The next project phase is:

ABot-Claw + MoveIt door-opening MVP

The goal is to build a deterministic door/button manipulation baseline first, integrate it into an ABot-Claw mission/state machine, add Bunker navigation after the arm-only sequence works, and return to this PiPER-X hand-eye/VLA path later.
