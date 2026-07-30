# PiPER-X D435i Hand-Eye Failure Analysis

This note records the read-only diagnosis of the failed PiPER-X wrist-mounted D435i hand-eye calibration. The failed calibration remains rejected. Do not recalibrate until the robot kinematic model and endpoint semantics are verified across multiple stopped poses.

## Safety Status

- No arm enable command was sent.
- No robot motion command was published.
- No OpenPI execution or trajectory replay was run.
- The existing failed calibration files were not deleted or accepted.

## Captured Evidence

Diagnostics were preserved in `handeye_failure_diagnostics/` and sorted by file timestamp during analysis rather than relying only on names:

- `pose_1.txt`
- `pose_2.txt`
- `pose_3.txt`
- `fk_mismatch_analysis.json`
- `live_fk_diagnostic_20260730T102601Z.json`

The relay evidence is good: `/joint_states_single` and `/joint_states` copy `joint1` through `joint6` exactly in the captured poses. That only verifies numeric relay behavior. It does not verify URDF joint axes, signs, offsets, or PiPER-X model compatibility.

## Active URDF

The running `robot_state_publisher` is using:

`/root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/src/piper_ros/src/piper_description/urdf/piper_description.urdf`

Observed file hash:

`9355d8a35d3f36691266163c06618b23bb7305c92b918c8eedeac31cb3652033`

Runtime `/robot_description` hash:

`c9ce0b71e1f5684d5c95f8b9895495bfac3c9e48e6484f63959cc9a44502941a`

Other installed URDF variants found in the same description package include `piper_description_old.urdf` and `piper_description_v100.urdf`. No explicitly named PiPER-X-specific URDF was found in the installed `piper_description` package during this audit.

A PiPER-X-specific URDF candidate exists outside the active ROS description package:

`/home/dase-hw101/Iliyas/piper-vr-teleop/third_party/agx_arm_urdf/piper_x/urdf/piper_x_description.urdf`

Observed hash:

`34126caac7d5b37bc2409f337ac246afbe0bb8cd47fc9f16df5038f19bd21e3a`

This is evidence that a PiPER-X model exists on the machine, but it is not automatically verified for this physical arm, firmware, ROS frame contract, or `easy_handeye` endpoint. The calibration launcher now refuses to start `robot_state_publisher` unless an explicit URDF path/hash and an FK verification result are provided.

Official `piper_ros` guidance says firmware older than `S-V1.6-3` should use `piper_description_old.urdf`; firmware `S-V1.6-3` or newer should use `piper_description.urdf`. The firmware could not be read successfully in this runtime, so the correct URDF remains unresolved.

## Firmware Status

The installed `piper_sdk` signatures were inspected before use. The read-only API methods exist:

- `SearchPiperFirmwareVersion()`
- `GetPiperFirmwareVersion()`
- `EnableFkCal()`
- `GetFK(mode="feedback"|"control")`

A separate read-only SDK query returned firmware value `-1199`, and SDK FK values remained zero. This does not identify the firmware. It likely means the separate SDK instance could not obtain firmware/FK data from the live CAN/driver setup. Firmware version is therefore unresolved.

Installed revisions observed during the audit:

- `piper_ros`: `d539c15dd39371db625ee5b41e21c2bc1cd95c75` on branch `noetic`, with local modifications in the checkout.
- host `piper_sdk`: `c05c5454b1cf61c05ad26385e0c0a3aa6d3c7bad`.
- PiPER-X teleop repository: `9eec6e26d927a495efaaa0e7e5af2895310caefe`, with unrelated untracked/generated files.

## `/end_pose` Meaning

`/end_pose` is published by:

`/root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/src/piper_ros/src/piper/scripts/piper_ctrl_single_node.py`

The source uses `self.piper.GetArmEndPoseMsgs().end_pose`, converts:

- `X_axis`, `Y_axis`, `Z_axis` from integer micrometers to meters by dividing by `1000000`
- `RX_axis`, `RY_axis`, `RZ_axis` from millidegrees to degrees by dividing by `1000`, then to radians
- RPY to quaternion using `tf.transformations.quaternion_from_euler`

The publisher sets `header.stamp` but does not set `header.frame_id`, so `/end_pose.header.frame_id` is empty. The code does not document whether the controller endpoint is `link6`, `gripper_base`, `gripper_tcp`, or a firmware TCP with an offset. The captured data proves `/end_pose` must not be treated as `base_link -> gripper_base` without further verification.

## FK Comparison

Offline comparison used the captured `/end_pose` controller pose and `base_link -> gripper_base` from URDF TF.

| Pose | Position Error | Angular Error |
| --- | ---: | ---: |
| `pose_1.txt` | 0.059 m | 127.827 deg |
| `pose_2.txt` | 0.109 m | 106.712 deg |
| `pose_3.txt` | 0.133 m | 174.463 deg |

The mismatch changes with configuration. A single constant `gripper_base -> controller_endpoint` TCP transform does not explain the disagreement:

- maximum translation residual: 0.124793 m
- maximum angular residual: 132.044431 deg
- verification thresholds: 0.020 m and 5.0 deg

Result: FK is not verified.

## Marker Evidence

When the marker was visible, `base_link -> aruco_marker_frame` moved even though the physical marker was fixed:

- pose 1: `[0.017, 0.032, 0.756]`
- pose 3: `[-0.202, -0.074, 0.882]`

The apparent fixed-marker displacement in the original three-pose diagnostic was about 0.274 m. Later validation of the saved 23-sample calibration observed up to about 0.365 m of false marker motion:

```text
baseline: [-0.239, -0.047, 0.932]
samples:  [-0.458, -0.099, 0.949]
          [-0.205, -0.003, 0.885]
          [-0.067,  0.090, 0.810]
          [ 0.057,  0.020, 0.730]
```

In the pose where the marker was not visible, `aruco_marker_frame` correctly disappeared, so stale marker TF is not the explanation.

## Conclusion

The failed calibration should stay rejected. The immediate blocker is not ArUco detection; it is the disagreement between controller/SDK endpoint semantics and the URDF TF chain.

Do not recalibrate until all of these are true:

1. The exact physical PiPER-X model is identified.
2. Firmware version is known.
3. The correct URDF is selected for that firmware and physical model.
4. Joint axes, signs, offsets, and zero conventions are verified.
5. `/end_pose` endpoint semantics and any controller TCP offset are known.
6. Controller/SDK FK and `robot_state_publisher` TF agree across multiple stopped poses.

## Read-Only Commands

Analyze preserved diagnostics:

```bash
cd ~/piper-pipeline-testbed
PYTHONPATH=piper-on-bunker/src \
  python3 piper-on-bunker/scripts/analyze_piper_x_fk_diagnostics.py \
    handeye_failure_diagnostics/pose_*.txt \
    --output-json handeye_failure_diagnostics/fk_mismatch_analysis.json
```

Capture another stopped-pose diagnostic without enabling or moving the arm:

```bash
cd ~/piper-pipeline-testbed
PYTHONPATH=piper-on-bunker/src \
  python3 piper-on-bunker/scripts/capture_piper_x_fk_diagnostic.py \
    --output handeye_failure_diagnostics/live_fk_diagnostic_$(date -u +%Y%m%dT%H%M%SZ).json
```
