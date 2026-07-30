# PiPER-X Wrist D435i Hand-Eye Calibration Setup

This note records the reproducible calibration setup for an Intel RealSense D435i rigidly mounted on a single AgileX PiPER-X wrist.

No robot motion is commanded by these tools. The PiPER driver is launched with `auto_enable:=false`; the arm is not enabled, and no trajectory/OpenPI replay process is started.

## Calibration Target

Calibration mode:

```text
eye_on_hand: true
solve transform: gripper_base -> wrist_camera_color_optical_frame
robot_base_frame: base_link
robot_effector_frame: gripper_base
tracking_base_frame: wrist_camera_color_optical_frame
tracking_marker_frame: aruco_marker_frame
```

The camera is mounted on the PiPER-X wrist. The ArUco marker is stationary on a vertical wall or rigid vertical board.

## Marker Source Of Truth

Use the measured black-square side length:

```text
dictionary: DICT_ARUCO_ORIGINAL
marker_id: 6
marker_size_m: 0.100
```

The old `0.040 m` value is wrong for this marker and must not be used for this calibration.

Do not confuse this physical calibration marker with the OpenPI PiPER-X ArUco touch task profile. The current OpenPI task profile still declares `DICT_4X4_50` for future dataset/task work. The marker physically in view for this hand-eye calibration is detected as `DICT_ARUCO_ORIGINAL`, ID `6`.

The PiPER-X task profile also records this value at:

```text
piper-on-bunker/config/openpi_piper_x_touch_aruco.yaml
marker.side_length_m: 0.100
```

## Runtime Container

Use the existing named container:

```bash
abot-piper-noetic
```

Do not use `./tools/run_in_noetic_container.sh` for this calibration session. That helper starts a throwaway container and is not the running calibration container.

Correct source order inside `abot-piper-noetic`:

```bash
source /opt/ros/noetic/setup.bash
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash
source /root/easy_handeye_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
```

## Controlled Restart

Restart the background calibration stack with:

```bash
cd ~/piper-pipeline-testbed
./tools/start_piper_x_d435i_handeye_calibration.sh
```

This recreates the detached tmux session:

```text
piper_x_wrist_calib
```

Windows:

```text
0: roscore
1: piper_driver_readonly
2: joint_relay
3: robot_state_pub
4: d435i_wrist
5: image_rectify
6: aruco
7: handeye_backend
```

The `aruco` pane runs the repo OpenCV detector:

```text
piper-on-bunker/scripts/piper_x_aruco_pose_node.py
```

It explicitly uses `DICT_ARUCO_ORIGINAL`, marker ID `6`, and marker size `0.100 m` by default. It publishes the same contract expected by `easy_handeye`:

```text
/aruco_simple/pose
/aruco_simple/debug_image
wrist_camera_color_optical_frame -> aruco_marker_frame
```

It publishes no robot commands and does not replay stale transforms after marker loss. The raw image topics do not contain overlays; use `/aruco_simple/debug_image` for the annotated view.

## Readiness Check

After restart, run:

```bash
cd ~/piper-pipeline-testbed
./tools/check_piper_x_d435i_handeye_readiness.sh
```

The check verifies:

- `auto_enable` is `false`;
- wrist raw image is live;
- `CameraInfo` is live and nonzero;
- rectified image is live;
- `/joint_states_single` is live;
- `/joint_states` is live;
- `base_link -> gripper_base` TF is live;
- `/aruco_simple/pose` is live;
- `wrist_camera_color_optical_frame -> aruco_marker_frame` TF is live;
- marker dictionary is `DICT_ARUCO_ORIGINAL`;
- marker ID is `6`;
- marker size source of truth is `0.100 m`;
- easy_handeye backend is running;
- no obvious motion stack nodes were started by this calibration setup.

`/aruco_simple/pose` and `aruco_marker_frame` will remain unavailable until marker ID `6` is visible in the wrist camera image.

If the image topics are live but `/aruco_simple/pose` is not ready, check one rectified image directly:

```bash
cd ~/piper-pipeline-testbed
./tools/check_piper_x_d435i_aruco_image.sh
```

This reports the detected ArUco IDs, rejected candidates, and whether `/aruco_simple/debug_image` is live for `DICT_ARUCO_ORIGINAL`. If `detected_marker_ids` is empty, the marker is not visible enough, is blurred, too small, too oblique, partly cropped, badly lit, or was printed from a different dictionary.

## Debug Image

Raw and rectified camera topics show only camera pixels:

```text
/wrist_camera/color/image_raw
/wrist_camera/color/image_rect_color
```

The annotated feed is:

```text
/aruco_simple/debug_image
```

Open it with:

```bash
cd ~/piper-pipeline-testbed
./tools/open_piper_x_d435i_aruco_debug_view.sh
```

The debug image preserves the input image timestamp and frame ID. It draws detected marker borders, highlights configured marker ID `6`, writes dictionary/ID/size/status text, and draws XYZ axes when pose is available.

Before saving hand-eye calibration, set RViz `Fixed Frame` to:

```text
wrist_camera_color_optical_frame
```

After publishing a saved hand-eye calibration, you can use:

```text
base_link
```

## Sampling GUI

Open the GUI only when ready to take samples:

```bash
cd ~/piper-pipeline-testbed
./tools/open_piper_x_d435i_handeye_gui.sh
```

The GUI uses:

```text
ROS_NAMESPACE=/piper_x_d435i_wrist_eye_on_hand
```

The tmux session intentionally does not start RViz or the sampling GUI.

## Compute Crash From Wrong OpenCV

If the sampling GUI closes or the backend logs this error after pressing `Compute`:

```text
module 'cv2' has no attribute 'calibrateHandEye'
```

the active Python process imported the `/usr/local` OpenCV package instead of the ROS/system OpenCV package. The launcher forces the easy_handeye backend to use:

```text
PYTHONPATH=/usr/lib/python3/dist-packages
```

because the system OpenCV `4.2.0` includes both ArUco and `cv2.calibrateHandEye`. The `/usr/local` OpenCV `5.0.0` package has ArUco in this container but does not expose `calibrateHandEye`.

## Sample Procedure

Use the already-proven PiPER-X teleoperation system to manually reposition the arm between samples. Keep this calibration stack read-only.

Recommended process:

1. Confirm the marker is rigid, flat, vertical, and stationary.
2. Confirm the D435i wrist bracket is tight and cannot shift.
3. Confirm marker ID `6` is visible in the wrist image.
4. Run the readiness script.
5. Open the sampling GUI.
6. Collect 15-25 varied poses.
7. Use varied wrist orientations, distances, and image locations.
8. Avoid samples where the marker is blurred, heavily oblique, or partly out of frame.
9. Wait for the arm and image to settle before pressing `Take Sample`.
10. Compute calibration in the GUI.
11. Save only if the result is stable and physically plausible.

Do not infer calibration quality from one pose. After saving, validate that the fixed marker remains stable in `base_link` across several manually moved arm poses.

## Saved Calibration

`easy_handeye` stores calibration under the `piper_x_d435i_wrist` namespace in the container ROS home, typically below:

```text
/root/.ros/easy_handeye/
```

The exact file name is generated by `easy_handeye` from the calibration namespace. Keep the saved YAML with the experiment manifest.

## Publish Saved Transform

After a calibration has been saved, publish it with:

```bash
cd ~/piper-pipeline-testbed
./tools/publish_piper_x_d435i_handeye.sh
```

This runs:

```text
roslaunch easy_handeye publish.launch eye_on_hand:=true namespace_prefix:=piper_x_d435i_wrist
```

## Validate Saved Transform

Read-only validation:

```bash
cd ~/piper-pipeline-testbed
./tools/validate_piper_x_d435i_handeye.sh
```

This checks:

- `gripper_base -> wrist_camera_color_optical_frame` exists;
- `base_link -> aruco_marker_frame` is observable;
- repeated marker observations can be logged;
- no physical motion is commanded.

The operator should manually move the arm through multiple poses using the proven teleoperation system, then re-run validation. The marker should remain stable in `base_link`. Do not invent or report an accuracy number unless it is measured.

## Manifest Fields

Record these fields with the dataset/calibration manifest:

- PiPER-X serial/config identifier;
- D435i serial `243322074578`;
- camera mount revision;
- physical calibration marker dictionary `DICT_ARUCO_ORIGINAL`;
- OpenPI task profile marker dictionary `DICT_4X4_50`;
- marker ID `6`;
- measured marker size `0.100 m`;
- target surface orientation `vertical`;
- calibration namespace `piper_x_d435i_wrist`;
- saved easy_handeye YAML path;
- date/time;
- operator notes;
- whether the bracket or camera moved after calibration.

Recalibrate whenever the camera, bracket, wrist mount, marker, or arm tool geometry changes.

## Safety Boundaries

- These tools do not enable the arm.
- These tools do not publish robot motion commands.
- These tools do not run OpenPI physical execution.
- These tools do not run recorded trajectory replay.
- These tools do not launch MoveIt execution.
- Manual repositioning is done separately by the operator through the proven PiPER-X teleoperation system.
