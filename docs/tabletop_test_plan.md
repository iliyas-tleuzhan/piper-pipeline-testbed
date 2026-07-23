# Tabletop Test Plan

Mission: find the marked button, inspect it, press it, verify the result, and return to the forward navigation-view pose.

Nominal flow:

`START -> SYSTEM_CHECK -> ACQUIRE_ARM_AUTHORITY -> HOME -> NAVIGATION_VIEW -> INSPECT -> DETECT_TARGET -> ESTIMATE_TARGET -> VALIDATE_TARGET -> PRE_CONTACT -> APPROACH -> PRESS -> RETRACT -> VERIFY -> RETURN_TO_NAVIGATION_VIEW -> RELEASE_ARM_AUTHORITY -> COMPLETE`

Failure flow:

`STOP -> RETRACT_IF_SAFE -> SAFE_RECOVERY -> RELEASE_AUTHORITY -> FAILED`

Before first physical motion:

1. Verify `can0` exists and is up at the expected bitrate.
2. Verify ROS Noetic master in `abot-piper-noetic`.
3. Verify `/joint_states_single`, `/end_pose`, table camera topics, TF, and MoveIt services.
4. Verify `8891 /health` and `/state`.
5. Calibrate named poses into the ignored local config.
6. Configure workspace bounds and camera-to-PiPER transform.
7. Run unit tests, mock mission, camera-only capture, marker detection, transform validation, and complete dry mission.
8. Show the proposed first named-pose movement and exact command.
9. Ask the operator once before moving the physical arm.

Physical tests were not performed in the 2026-07-23 implementation pass.
