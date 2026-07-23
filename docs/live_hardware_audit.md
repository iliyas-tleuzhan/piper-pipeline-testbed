# Live Hardware Audit

Audit date: 2026-07-23.

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

- `can0`: not present during audit, so state and bitrate were not verified.
- Running Docker containers at first check: none.
- `8891 /health`: connection refused.
- `8891 /state`: connection refused.
- ROS master in container: not running.
- `/joint_states_single`, `/end_pose`, table camera topics, TF tree: not live-verified.
- USB: no Intel RealSense device was enumerated by `lsusb`; the integrated Bison camera was visible.

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

These were found in source but not verified live because ROS master was not running.

## Startup Scripts Found

- `/home/dase-hw101/ABot-Claw/start_abotclaw_all.sh`
- `/home/dase-hw101/ABot-Claw/start_abotclaw_full_stack.sh`
- `/home/dase-hw101/ABot-Claw/robot_layer/arm_piper/agent_server/start_piper_language_stack_tmux.sh`
- `/home/dase-hw101/ABot-Claw/robot_layer/arm_piper/agent_server/start_realsense_d555_py.sh`

## Implementation Levels

- Implemented and unit-tested: strict 8891 adapter, gated ROS MoveIt adapter, RealSense ROS subscriber, ArUco detector, static transform utility, local named-pose calibration, safety checks, JSONL logging, restricted agent API, dual-arm mock.
- Verified read-only on real laptop: host/container runtime, ABot-Claw source contracts, container mount, absence of `can0`, absence of running 8891, absence of enumerated RealSense.
- Physically executed successfully: none.
