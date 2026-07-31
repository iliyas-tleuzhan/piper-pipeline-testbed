# PiPER-X ArUco Servo Current State

Date: 2026-07-31

Branch: `abotclaw-moveit-door-mvp`

Current checkpoint commit:

`5d62a5f56cddf6b44e36c0320dec53c85ab30452`

## Current Command

```bash
cd ~/piper-pipeline-testbed

python3 piper-on-bunker/scripts/run_piper_x_visual_servo_aruco_touch.py \
  --config piper-on-bunker/config/piper_x_visual_servo_aruco_touch.local.yaml \
  --live \
  --mode continuous_simple_up_forward \
  --execute \
  --confirm CONTINUOUS_SIMPLE_UP_FORWARD
```

## Intended Behavior

The current mode is deliberately simple:

1. Use the D435i wrist camera to detect `DICT_ARUCO_ORIGINAL` marker ID `6`.
2. Use ArUco only while aligning vertically in the wrist image.
3. Move only up/down in world `Z` until the marker vertical pixel error is within tolerance.
4. After vertical alignment, stop using the marker.
5. During forward motion, use only aligned depth at the image center.
6. Move forward along configured world `+X` while keeping height fixed.
7. Lock `joint1` during the forward phase so MoveIt cannot solve forward motion by yawing the base joint.
8. Keep advancing/replanning through partial MoveIt Cartesian plans.
9. Stop only when center depth is `<= 0.01 m`, ROS shuts down, MoveIt returns zero usable path, or the operator stops it.

## Local Config

The physical execution config is intentionally ignored by Git:

`piper-on-bunker/config/piper_x_visual_servo_aruco_touch.local.yaml`

A copy was checkpointed locally under:

`piper-on-bunker/data/local/project_checkpoints/20260731T104135Z_piper_x_aruco_servo/`

Important local values:

- `simple_forward_axis_world: [1.0, 0.0, 0.0]`
- `simple_up_step_m: 0.02`
- `simple_forward_step_m: 0.02`
- `continuous_forward_stop_depth_m: 0.01`
- `forward_lock_joint1_tolerance_rad: 0.01`

Flip `simple_forward_axis_world` to `[-1.0, 0.0, 0.0]` if the physical forward direction is wrong.

## Caveat

The saved hand-eye transform in the local config comes from the 2026-07-30 calibration that was previously rejected due to fixed-marker drift. The current servo path should not trust that transform for targeting. It uses simple world-frame nudges and depth monitoring instead.
