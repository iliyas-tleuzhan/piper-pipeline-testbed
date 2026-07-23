# Live Hardware Audit

Audit date: 2026-07-23. Updated after CAN, ROS, and RealSense follow-up.

## Environment

- Testbed path: `/home/dase-hw101/piper-pipeline-testbed`.
- ABot-Claw reference path: `/home/dase-hw101/ABot-Claw`.
- Host OS: Ubuntu 24.04.4 LTS, kernel `6.17.0-1028-oem`.
- Host Python: `3.12.3`.
- Host ROS environment: ROS 2 Jazzy variables were present, so host Python is not the intended PiPER runtime.
- Intended runtime: Docker container `abot-piper-noetic`.
- Container mount: `/home/dase-hw101/ABot-Claw:/root/ABot-Claw`.
- Container Python: `3.8.10`.
- Container ROS: Noetic.

The container was started for read-only Python/ROS inspection only. No ROS stack or robot motion was started.

## Live Hardware Visibility

- `can0`: present, `UP`, `LOWER_UP`, `ERROR-ACTIVE`, bitrate `1000000`, zero bus errors.
- Passive `candump -L can0`: continuous PiPER CAN traffic observed, including IDs `2A1..2A8` and `251..256`.
- USB CAN adapter: OpenMoko/Geschwister Schneider CAN adapter via `gs_usb`.
- RealSense D555: enumerated as Intel RealSense D555 serial `352222303634`, firmware `7.56.37776.6014`.
- ROS lower stack: started with existing ABot-Claw infrastructure script; no mission or motion command was run.
- `8891 /health`: success after starting the restricted action server; `robot_initialized=false`.
- `8891 /state`: success; live joints matched `/joint_states_single`.
- `/joint_states_single`: live `sensor_msgs/JointState`, names `joint1..joint6, gripper`.
- `/end_pose`: live `geometry_msgs/PoseStamped`.
- Table camera topics: live `sensor_msgs/Image` color/depth and `sensor_msgs/CameraInfo`.
- MoveIt services: live, type `moveit_ctrl/JointMoveitCtrl`.
- MoveIt planning frame: `dummy_link`.
- End-effector link: `gripper_tcp`.
- Active MoveIt arm joints: `joint1..joint6`.
- TF: `dummy_link -> base_link` is identity. No live TF from `table_camera_color_optical_frame` to the arm planning tree was available.
- Live dry-run result: real state and camera observation succeeded, no-motion MoveIt request previews were generated, but ArUco target detection returned `TARGET_NOT_FOUND`.

## ABot-Claw 8891 API Found In Source

Source: `/home/dase-hw101/ABot-Claw/robot_layer/arm_piper/agent_server/piper_language_action_server.py`.

- `GET /health`
- `GET /state`
- `POST /move_up` with `{joint_step, speed, accel}`
- `POST /move_down` with `{joint_step, speed, accel}`
- `POST /open_gripper`
- `POST /close_gripper`
- `POST /set_gripper` with `{position, speed, accel}`

No named-pose, arbitrary target-pose, stop, cancel, disable, or physical estop endpoint was found in the 8891 source.

## MoveIt Service Contract Found In Source

Service type: `moveit_ctrl/JointMoveitCtrl`.

```text
float64[6] joint_states
float64 gripper
float64[7] joint_endpose
float64 max_velocity
float64 max_acceleration
---
int64 error_code
bool status
```

Expected service names:

- `/joint_moveit_ctrl_arm`
- `/joint_moveit_ctrl_endpose`
- `/joint_moveit_ctrl_gripper`
- `/joint_moveit_ctrl_piper`

These were found in source and verified live after starting the lower stack.

## Live Camera Result

CameraInfo:

- Frame: `table_camera_color_optical_frame`
- Resolution: `1280x720`
- Intrinsics: `fx=638.4649`, `fy=638.4649`, `ppx=628.1332`, `ppy=364.4467`
- Depth encoding observed through the Python publisher: `16UC1`

A temporary frame was inspected at `/tmp/table_camera_latest.jpg`. It showed the tabletop and PiPER arm, but no detectable ArUco marker. A dictionary sweep over the frame found no ArUco detections.

## Startup Checklist

Read-only/live dry-run sequence:

1. Confirm `can0`: `ip -details -statistics link show can0`.
2. Confirm passive CAN traffic: `timeout 5 candump -L can0`.
3. Confirm RealSense USB: `rs-enumerate-devices -s`.
4. Start lower stack only: `PIPER_TMUX_ATTACH=0 ~/ABot-Claw/start_abotclaw_all.sh --lower-only --no-attach`.
5. Verify topics: `rostopic list`.
6. Verify services: `rosservice list`.
7. Verify 8891 read-only: `curl http://localhost:8891/health` and `curl http://localhost:8891/state`.
8. Verify camera-only capture.
9. Place the configured ArUco marker visibly in the camera view.
10. Run full live dry-run.

## Startup Scripts Found

- `/home/dase-hw101/ABot-Claw/start_abotclaw_all.sh`
- `/home/dase-hw101/ABot-Claw/start_abotclaw_full_stack.sh`
- `/home/dase-hw101/ABot-Claw/robot_layer/arm_piper/agent_server/start_piper_language_stack_tmux.sh`
- `/home/dase-hw101/ABot-Claw/robot_layer/arm_piper/agent_server/start_realsense_d555_py.sh`

## Implementation Levels

- Implemented and unit-tested: strict 8891 adapter, gated ROS MoveIt adapter, RealSense ROS subscriber, ArUco detector, static transform utility, local named-pose calibration, safety checks, JSONL logging, restricted agent API, dual-arm mock.
- Verified read-only on real laptop: host/container runtime, ABot-Claw source contracts, container mount, `can0`, passive CAN traffic, RealSense USB, ROS topics/services, MoveIt planning frame, 8891 health/state, camera frame capture.
- Physically executed successfully: none.
