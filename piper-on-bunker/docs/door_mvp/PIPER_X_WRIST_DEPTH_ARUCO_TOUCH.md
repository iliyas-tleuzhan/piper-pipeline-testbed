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

Execution mode, after using an ignored local config with measured values:

```bash
cd ~/piper-pipeline-testbed
python3 piper-on-bunker/scripts/run_piper_x_visual_servo_aruco_touch.py \
  --config piper-on-bunker/config/piper_x_visual_servo_aruco_touch.local.yaml \
  --live \
  --mode align_then_depth_touch \
  --execute \
  --confirm ALIGN_DEPTH_TOUCH
```

Execution behavior:

1. Estimate marker center and depth.
2. If the marker is not centered, execute only the lateral alignment component.
3. Repeat until the marker is within `image_center_tolerance_px`.
4. Stop alignment and take a fresh depth estimate.
5. Execute only the bounded forward component toward the marker.

The command refuses execution when the marker is missing, depth is stale, the
gripper tip offset is unmeasured, the eye-in-hand transform is unverified, or
physical execution is disabled in the selected config.
