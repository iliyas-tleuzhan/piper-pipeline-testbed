# PiPER-X Wrist-Depth ArUco Touch

This is the guarded setup for a visual/depth touch path:

1. The D435i wrist camera detects `DICT_ARUCO_ORIGINAL` marker ID `6`.
2. Aligned depth estimates the marker-center distance.
3. The configured `gripper_base -> wrist_camera_color_optical_frame` transform maps the marker point into the gripper frame.
4. The configured gripper contact-tip offset is subtracted.
5. The tool reports the limited gripper-frame alignment/forward step.

The committed config is blocked for physical execution by default. Do not use the rejected 2026-07-30 hand-eye calibration as a verified execution transform.

Read-only check:

```bash
cd ~/piper-pipeline-testbed
./tools/check_piper_x_visual_servo_aruco_touch.sh
```

Config:

```bash
piper-on-bunker/config/piper_x_visual_servo_aruco_touch.yaml
```

Before physical visual servoing can be enabled in a local ignored config, fill in:

- measured gripper contact-tip offset in `gripper_base`;
- validated eye-in-hand transform `gripper_base -> wrist_camera_color_optical_frame`;
- marker visible and centered within tolerance;
- live aligned depth on `/wrist_camera/aligned_depth_to_color/image_raw`.

The current tool does not command motion. It reports blockers and the computed step so the setup can be checked safely.
