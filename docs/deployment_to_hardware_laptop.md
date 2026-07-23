# Deployment To Hardware Laptop

Use this split:

- Host Ubuntu 24.04: repo management and non-ROS unit tests.
- `abot-piper-noetic`: physical PiPER runtime with ROS Noetic and Python 3.8.
- ABot-Claw: read-only reference unless a separate minimal compatibility change is explicitly approved.

Before motion:

1. Start the ABot-Claw stack manually.
2. Verify `can0`, ROS master, `/joint_states_single`, `/end_pose`, table camera topics, TF, MoveIt services, and `8891`.
3. Copy `piper-on-bunker/config/piper_laptop_hardware.local.example.yaml` to `piper-on-bunker/config/piper_laptop_hardware.local.yaml`.
4. Calibrate named poses with `piper-on-bunker/scripts/calibrate_named_pose.py`.
5. Configure camera-to-PiPER transform and workspace bounds.
6. Keep `local_activation.physical_motion_enabled: false` until dry-run and operator approval.

The committed hardware config keeps `physical_motion_enabled: false`.
