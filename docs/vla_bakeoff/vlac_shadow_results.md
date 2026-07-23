# VLAC Shadow Results

Date: July 23, 2026

Remote RTX 5090 service host is reachable at `192.168.1.104`.

Service status:

- Critic service `8014`: healthy, existing container `abot-vlac`, left running.
- Policy-preview service `8016`: healthy, separate container `abot-vlac-policy`, shadow-only.
- GPU selection: physical GPU `0` for policy preview; existing critic remains on the other GPU.
- Policy health reports `execution_allowed: false`.

Saved-image test:

Used one bundled VLAC example image with a converted example end-effector state. All responses preserved the raw model output and returned `execution_allowed: false`.

| Instruction | Raw output | Parsed raw translation | Parse reliability | Latency |
| --- | --- | --- | --- | --- |
| Move the arm to the left. | `{x: 91.6mm, y: 7.3mm, z: 9.4mm, roll: 4.4 degrees, pitch: -5.6 degrees, yaw: -8.1 degrees, open: 0.0}` | `[0.0916, 0.0073, 0.0094] m` | exact grammar match | 997 ms |
| Move the arm to the right. | `{x: 89.8mm, y: 9.4mm, z: 8.8mm, roll: -2.1 degrees, pitch: -6.7 degrees, yaw: -6.9 degrees, open: 0.0}` | `[0.0898, 0.0094, 0.0088] m` | exact grammar match | 345 ms |
| Move the arm upward. | `{x: 92.4mm, y: 8.0mm, z: 5.2mm, roll: 4.4 degrees, pitch: -6.2 degrees, yaw: -7.5 degrees, open: 0.0}` | `[0.0924, 0.0080, 0.0052] m` | exact grammar match | 336 ms |
| Move the arm downward. | `{x: 90.2mm, y: 10.0mm, z: 8.7mm, roll: 3.8 degrees, pitch: -5.6 degrees, yaw: -6.5 degrees, open: 0.0}` | `[0.0902, 0.0100, 0.0087] m` | exact grammar match | 335 ms |
| Move the arm forward. | `{x: 95.8mm, y: 6.9mm, z: 8.0mm, roll: 3.9 degrees, pitch: -5.8 degrees, yaw: -7.4 degrees, open: 0.0}` | `[0.0958, 0.0069, 0.0080] m` | exact grammar match | 334 ms |
| Move the arm backward. | `{x: 90.5mm, y: 9.1mm, z: 9.5mm, roll: 3.6 degrees, pitch: -5.4 degrees, yaw: -8.3 degrees, open: 0.0}` | `[0.0905, 0.0091, 0.0095] m` | exact grammar match | 335 ms |
| Open the gripper. | `{x: 94.1mm, y: 7.4mm, z: 10.6mm, roll: -2.8 degrees, pitch: -6.4 degrees, yaw: -1.6 degrees, open: 0.0}` | `[0.0941, 0.0074, 0.0106] m` | exact grammar match | 332 ms |
| Close the gripper. | `{x: 85.2mm, y: 12.8mm, z: 10.1mm, roll: 3.2 degrees, pitch: -3.5 degrees, yaw: 1.1 degrees, open: 0.0}` | `[0.0852, 0.0128, 0.0101] m` | exact grammar match | 334 ms |
| Move toward the marked button. | `{x: 85.7mm, y: 9.7mm, z: 8.2mm, roll: 4.1 degrees, pitch: -5.3 degrees, yaw: -7.6 degrees, open: 0.0}` | `[0.0857, 0.0097, 0.0082] m` | exact grammar match | 338 ms |
| Press the marked button. | `{x: 104.6mm, y: 10.4mm, z: 12.4mm, roll: 4.2 degrees, pitch: -3.5 degrees, yaw: -4.4 degrees, open: 0.0}` | `[0.1046, 0.0104, 0.0124] m` | exact grammar match | 333 ms |
| Retract from the button. | `{x: 95.6mm, y: 8.7mm, z: 8.0mm, roll: -3.4 degrees, pitch: -5.6 degrees, yaw: -6.7 degrees, open: 0.0}` | `[0.0956, 0.0087, 0.0080] m` | exact grammar match | 333 ms |

Saved-image directional reasonableness: failed. The model produced broadly similar positive `x/y/z` values for contradictory left/right, up/down, forward/backward, and approach/press/retract prompts. Zero-shot PiPER execution is rejected.

Live RealSense/PiPER state test:

The shadow client ran inside the ROS Noetic container, read `/table_camera/color/image_raw`, `/end_pose`, and `/joint_states_single`, and sent one request per instruction to `http://192.168.1.104:8016/action-preview`.

Live state used:

- `x_m`: `0.055984`
- `y_m`: `0.000667`
- `z_m`: `0.214169`
- `roll_rad`: `-0.23647466035271253`
- `pitch_rad`: `1.4782764231466787`
- `yaw_rad`: `-0.2255314459427081`
- `gripper_m`: `0.0`

| Instruction | Raw output | Parsed raw translation | Parse reliability | Execution |
| --- | --- | --- | --- | --- |
| Move toward the marked button. | `{x: 0.1mm, y: 0.0mm, z: 0.0mm, roll: 0.0 degrees, pitch: 0.0 degrees, yaw: 0.0 degrees, open: 0.7}` | `[0.0001, 0.0, 0.0] m` | exact grammar match | disabled |
| Press the marked button. | `{x: 0.0mm, y: 0.0mm, z: 0.0mm, roll: 0.0 degrees, pitch: 0.0 degrees, yaw: 0.0 degrees, open: 0.5}` | `[0.0, 0.0, 0.0] m` | exact grammar match | disabled |
| Retract from the button. | `{x: 0.0mm, y: 0.0mm, z: 0.0mm, roll: 0.0 degrees, pitch: 0.0 degrees, yaw: 0.0 degrees, open: 0.4}` | `[0.0, 0.0, 0.0] m` | exact grammar match | disabled |

Live-image directional reasonableness: failed. The action semantics remain unknown and the live outputs did not demonstrate useful zero-shot behavior for approach, press, or retract.

No PiPER command interface was called.
