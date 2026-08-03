# PiPER-X D435i Hand-Eye Calibration Progress

This document checkpoints the current PiPER-X wrist-mounted Intel RealSense D435i hand-eye calibration work so another session can continue without relying on chat history.

No tool in this calibration stack should enable the PiPER-X, publish robot motion commands, run OpenPI physical execution, replay trajectories, or launch MoveIt execution.

## Current Architecture

The calibration is eye-on-hand:

```text
fixed PiPER-X base
-> base_link
-> gripper_base
-> wrist-mounted D435i
-> wrist_camera_color_optical_frame
-> stationary ArUco marker on a vertical surface
```

The target transform to solve is:

```text
gripper_base -> wrist_camera_color_optical_frame
```

`easy_handeye` receives:

```text
robot motion chain: base_link -> gripper_base
tracking chain: wrist_camera_color_optical_frame -> aruco_marker_frame
```

The operator manually repositions the arm through the separate proven PiPER-X teleoperation system. The calibration stack itself is read-only.

## Current Validation Status

The saved PiPER-X D435i hand-eye calibration is rejected. The fixed marker appeared to move by about `0.365 m` in `base_link` across different stopped arm poses:

```text
baseline: [-0.239, -0.047, 0.932]
samples:  [-0.458, -0.099, 0.949]
          [-0.205, -0.003, 0.885]
          [-0.067,  0.090, 0.810]
          [ 0.057,  0.020, 0.730]
```

Additional FK diagnostics showed `/joint_states_single` and `/joint_states` copy joints 1-6 exactly, but `/end_pose` and `base_link -> gripper_base` disagree by roughly 6-13 cm and changing orientation. A single constant TCP transform does not explain the mismatch.

Do not recalibrate until Phase 0A repeatability diagnostics, joint-zero
verification, FK validation, and TCP measurement have been addressed. The first
new gate is documented in:

```text
docs/calibration/PIPER_X_PHASE_0A_REPEATABILITY.md
```

Phase 0A does not validate FK or hand-eye; it determines whether feedback,
controller settling, and physical endpoint repeatability are credible enough to
continue to joint-zero and FK work.

## Runtime

Container:

```text
abot-piper-noetic
```

Detached tmux session:

```text
piper_x_wrist_calib
```

Expected windows:

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

Correct source order inside the container:

```bash
source /opt/ros/noetic/setup.bash
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash
source /root/easy_handeye_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
```

## Frames And Topics

PiPER-X:

```text
/joint_states_single
/joint_states
base_link -> gripper_base
```

`/end_pose` is controller feedback from `GetArmEndPoseMsgs().end_pose`. The ROS publisher converts controller position units from micrometers to meters and controller RPY from millidegrees to radians/quaternion, but leaves `header.frame_id` empty. It is not proven to mean `base_link -> gripper_base`, `link6`, `gripper_tcp`, or any other specific endpoint frame.

D435i wrist camera:

```text
serial: 243322074578
/wrist_camera/color/image_raw
/wrist_camera/color/camera_info
/wrist_camera/color/image_rect_color
wrist_camera_color_optical_frame
```

ArUco detector output:

```text
node: /piper_x_aruco_pose_node
pose topic: /aruco_simple/pose
debug image: /aruco_simple/debug_image
TF: wrist_camera_color_optical_frame -> aruco_marker_frame
```

easy_handeye:

```text
namespace_prefix: piper_x_d435i_wrist
namespace: /piper_x_d435i_wrist_eye_on_hand
eye_on_hand: true
freehand_robot_movement: true
robot_base_frame: base_link
robot_effector_frame: gripper_base
tracking_base_frame: wrist_camera_color_optical_frame
tracking_marker_frame: aruco_marker_frame
start_rviz: false
start_sampling_gui: false
```

## Marker Contract

The physical marker currently used for hand-eye calibration is:

```text
dictionary: DICT_ARUCO_ORIGINAL
marker_id: 6
measured black-square side length: 0.100 m
surface: vertical
```

Do not confuse this with the OpenPI ArUco-touch task profile. The OpenPI task profile may still expect `DICT_4X4_50` ID `6` for future dataset work. That contract must not be silently changed when calibrating with the current physical `DICT_ARUCO_ORIGINAL` marker.

## Detector Implementation

The calibration stack uses the repo custom OpenCV detector rather than depending on the installed `aruco_ros` dictionary behavior:

```text
piper-on-bunker/scripts/piper_x_aruco_pose_node.py
piper-on-bunker/src/piper_on_bunker/perception/piper_x_aruco_pose.py
```

The current OpenCV build lacks `cv2.aruco.estimatePoseSingleMarkers`, so the detector falls back to `cv2.solvePnP`, preferably with `SOLVEPNP_IPPE_SQUARE` when available. It publishes no pose and no TF when marker ID `6` is not visible, preventing stale marker transforms.

The raw image topics do not contain overlays. Use `/aruco_simple/debug_image` for annotated marker borders, highlighted marker ID `6`, status text, and axes when pose is available.

Image geometry mode is now explicit. The default calibration mode is rectified-consistent:

```text
image_geometry_mode: rectified
image_topic: /wrist_camera/color/image_rect_color
pose intrinsics: CameraInfo.P left 3x3
distortion: zero
```

Raw mode uses `image_raw`, `CameraInfo.K`, and `CameraInfo.D`. The detector fails closed when the topic and geometry mode are inconsistent.

## Commands

Restart the read-only calibration stack:

```bash
cd ~/piper-pipeline-testbed
PHYSICAL_MODEL_ID=agilex_piper_x \
FIRMWARE_VERSION=<verified-firmware-string> \
ROBOT_URDF_PATH=<verified-piper-x-urdf-path> \
ROBOT_URDF_SHA256=<verified-urdf-sha256> \
PIPER_X_FK_VERIFIED=true \
MARKER_DICTIONARY=DICT_ARUCO_ORIGINAL \
./tools/start_piper_x_d435i_handeye_calibration.sh
```

The launcher intentionally fails without those FK/model values.

Capture one stopped-pose FK diagnostic:

```bash
cd ~/piper-pipeline-testbed
POSE_LABEL=pose_1 ./tools/check_piper_x_fk_consistency.sh
```

Analyze several captured poses:

```bash
cd ~/piper-pipeline-testbed
PYTHONPATH=piper-on-bunker/src \
  python3 piper-on-bunker/scripts/analyze_piper_x_fk_diagnostics.py \
    piper-on-bunker/data/local/piper_x_fk_diagnostics/*/*.json
```

Check readiness:

```bash
cd ~/piper-pipeline-testbed
./tools/check_piper_x_d435i_handeye_readiness.sh
```

Check ArUco image detection and debug topic:

```bash
cd ~/piper-pipeline-testbed
./tools/check_piper_x_d435i_aruco_image.sh
```

Open the annotated debug view:

```bash
cd ~/piper-pipeline-testbed
./tools/open_piper_x_d435i_aruco_debug_view.sh
```

Open RViz:

```bash
xhost +SI:localuser:root
docker exec -it \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  abot-piper-noetic bash -lc '
    source /opt/ros/noetic/setup.bash
    source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash
    source /root/easy_handeye_ws/devel/setup.bash
    export ROS_MASTER_URI=http://localhost:11311
    export ROS_HOSTNAME=localhost
    rviz
  '
```

Before a saved hand-eye transform is published, set RViz `Fixed Frame` to:

```text
wrist_camera_color_optical_frame
```

After publishing a saved hand-eye transform, `base_link` can be used.

Open the sampling GUI:

```bash
cd ~/piper-pipeline-testbed
./tools/open_piper_x_d435i_handeye_gui.sh
```

Publish the saved transform after calibration has actually been computed and saved:

```bash
cd ~/piper-pipeline-testbed
./tools/publish_piper_x_d435i_handeye.sh
```

The current saved transform is rejected, so this helper refuses by default. It may only be republished for read-only diagnostic reproduction:

```bash
cd ~/piper-pipeline-testbed
ALLOW_REJECTED_HANDEYE_PUBLISH_FOR_DIAGNOSTICS=true \
  ./tools/publish_piper_x_d435i_handeye.sh
```

Validate the saved transform read-only:

```bash
cd ~/piper-pipeline-testbed
./tools/validate_piper_x_d435i_handeye.sh
```

Snapshot the current runtime state:

```bash
cd ~/piper-pipeline-testbed
./tools/snapshot_piper_x_d435i_handeye_state.sh
```

## Completed

- Reproducible tmux launcher for the read-only PiPER-X D435i hand-eye stack.
- PiPER driver launched read-only with `auto_enable:=false`.
- D435i wrist image, CameraInfo, and rectification paths.
- Robot state relay and `base_link -> gripper_base` TF path.
- Custom OpenCV ArUco detector with runtime dictionary selection.
- Physical calibration marker contract corrected to `DICT_ARUCO_ORIGINAL`, ID `6`, size `0.100 m`.
- `solvePnP` pose fallback for the installed OpenCV build.
- `/aruco_simple/pose`, `/aruco_simple/debug_image`, and marker TF publishing when the marker is visible.
- Readiness, ArUco image, debug-view, GUI, saved-transform publishing, validation, and snapshot helper scripts.
- Documentation warning that the calibration marker family and OpenPI task marker family are currently distinct.
- Rejected calibration metadata preserved in `handeye_failure_diagnostics/rejected_piper_x_d435i_handeye_20260730.yaml`.
- Hard FK/model gate added before `robot_state_publisher` can be started by the calibration launcher.
- Rectified image geometry fixed to use `CameraInfo.P` with zero distortion.

## Not Completed Yet

- A 23-sample PiPER-X hand-eye YAML exists in `/root/.ros/easy_handeye/piper_x_d435i_wrist_eye_on_hand.yaml`, but it is rejected.
- No computed calibration has been validated.
- No `gripper_base -> wrist_camera_color_optical_frame` saved transform should be used until FK is verified and a new calibration passes fixed-marker validation.
- No OpenPI checkpoint has been trained or marked physically compatible from this calibration work.

## Current Safety Status

- Calibration stack is read-only.
- `auto_enable` must remain `false`.
- No OpenPI execution or trajectory replay is part of calibration.
- Manual repositioning happens outside this repo through the separately proven PiPER-X teleoperation system.
- Readiness checks report obvious motion-stack nodes as a blocker if they appear.

## Next Steps

1. Identify the exact physical PiPER-X model.
2. Obtain the exact firmware string.
3. Select an explicit PiPER-X URDF and record its SHA256.
4. Verify joint mapping one joint at a time using the separate proven teleoperation setup.
5. Capture several stopped-pose FK diagnostics.
6. Confirm controller/SDK FK and the selected ROS TF endpoint agree across poses.
7. Only after `fk_verified: true`, restart the calibration stack.
8. Start with `DICT_ARUCO_ORIGINAL`.
9. Confirm readiness reports `handeye_collection_allowed: true`.
10. Keep the marker and PiPER-X base fixed.
11. Open `rqt_easy_handeye`.
12. Reposition the arm using the separate proven PiPER-X teleoperation system.
13. Take three diagnostic samples.
14. Collect 15-25 varied samples.
15. Compute and save calibration.
16. Publish saved transform.
17. Validate `base_link -> aruco_marker_frame` stability across new poses.
18. Copy the saved YAML out of the container.
19. Update hardware/manifests only after validation.
