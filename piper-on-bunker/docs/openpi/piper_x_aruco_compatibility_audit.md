# PiPER-X ArUco OpenPI Compatibility Audit

Date: 2026-07-30

Scope: prepare data collection and training-pipeline readiness for one fixed-base AgileX PiPER-X arm with one wrist-mounted RGB camera, touching the center of a visible ArUco marker on a vertical surface, holding briefly, and retracting.

## Confirmed PiPER-X Facts

- The target robot profile is distinct from the existing normal PiPER profile.
- The policy observation for the first checkpoint is wrist-camera only:
  - `observation/wrist_image`: real wrist camera
  - `observation/exterior_image`: deterministic zero placeholder
  - `observation/state`: 7D `[joint1..joint6, gripper]`
  - `prompt`: phase/task language
- The policy action is 7D absolute targets:
  `[joint1, joint2, joint3, joint4, joint5, joint6, gripper]`.
- Gripper mode for this experiment is `fixed_hold`; gripper opening/closing is not trained.
- ArUco detection is diagnostic/evaluation metadata only. Marker center or pose is not added to the policy observation.
- Current OpenPI pin for future work is `15a9616a00943ada6c20a0f158e3adb39df2ccac`.
- Future base checkpoint is `gs://openpi-assets/checkpoints/pi05_base`; it was not downloaded.
- Official OpenPI documentation expects custom data conversion to LeRobot, custom training configs/transforms, normalization-stat computation, then training and policy serving after a trained checkpoint exists.

## Inherited Normal-PiPER Assumptions

These are present in older local work but are not automatically valid for PiPER-X:

- Normal PiPER ROS topics such as `/joint_states_single` and `/piper_joint_commands`.
- Normal PiPER CAN frame decoding and gripper units.
- Normal PiPER joint limits and firmware limit profiles.
- Previous external-camera OpenPI observation schema.
- Previous PiPER smoke checkpoint metadata using `piper_compatible`.

The new PiPER-X profile keeps these assumptions separate and fail-closed.

## Existing Local Support Found

- `/home/dase-hw101/Iliyas/piper-wireless-teleop` contains normal PiPER master/slave teleop code that reads slave feedback and commands a slave through `piper_sdk`.
- `/home/dase-hw101/Iliyas/piper-lora-teleop-bridge` documents normal PiPER command/feedback CAN sources and normal PiPER raw joint units.
- `/home/dase-hw101/Iliyas/piper-vr-teleop/third_party/agx_arm_urdf/piper_x` contains PiPER-X URDF/mesh descriptions.
- Prior PiPER-X `pyAgxArm` conversion logic and limits were not found in a verified importable form in this repository during this audit.

## Unresolved Hardware Facts

Do not run physical policy execution until these are measured and written into local manifests:

- PiPER-X authoritative joint limits, velocity limits, acceleration limits, and jerk limits.
- PiPER-X zero convention and joint sign convention for the active arm.
- PiPER-X gripper/tool feedback units and fixed safe hold value.
- The exact wrist camera topic, encoding, rotation, calibration ID, and rigid mount revision.
- Whether `/joint_states_single` is authoritative for this PiPER-X setup.
- Whether `/piper_x_joint_commands`, CAN command frames, or `pyAgxArm` leader feedback provide exact converted absolute slave/follower targets.
- Marker side length in meters and target surface setup.

## Values That Must Be Measured On Hardware

- Arm serial, firmware, SDK/driver source and commit.
- Camera model, serial, calibration hash, and mount ID.
- ArUco dictionary, marker ID, physical marker side length, and vertical target geometry.
- Collection FPS actually sustained with synchronized wrist image, state, and action labels.
- Fixed gripper safe hold target and tolerance.

## Safe Read-Only Collection Interfaces

Approved only when preflight proves freshness and provenance:

- Wrist camera image topic, read-only subscription.
- Camera info topic, read-only subscription.
- Joint state topic, read-only subscription.
- A ROS command-label topic that publishes exact absolute follower targets after all conversions/clamps.

## Not Yet Approved For Physical Policy Execution

- Any PiPER-X checkpoint metadata.
- Normal PiPER `piper_compatible` metadata.
- Public `pi05_base`, `pi05_droid`, DROID, ALOHA, LIBERO, LAP, or other foreign robot statistics/checkpoints.
- SocketCAN command decoding for PiPER-X labels until verified.
- `pyAgxArm` leader feedback labels until exact slave-target conversion is verified.
- Any physical runner while PiPER-X limits remain unresolved.

## Compatibility Conclusion

The repository is ready to prepare passive PiPER-X ArUco data collection and conversion scaffolding. It is not ready for PiPER-X physical policy execution, checkpoint compatibility claims, or learned motion.
