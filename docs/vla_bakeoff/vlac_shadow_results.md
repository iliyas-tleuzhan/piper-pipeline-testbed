# VLAC Shadow Results

Date: July 23, 2026

No model inference results are recorded yet. The remote RTX 5090 service host was unreachable:

- `ssh iliyas@master`: hostname resolution failed.
- `ssh iliyas@192.168.1.104`: connection timed out.
- `ssh -p 6001 iliyas@156.226.181.157`: connection reset.

Because the policy service on port 8016 could not be created or reached, saved-image and live-image VLAC inference were not run.

Local shadow-client validation completed:

- The client can read live ROS state and RealSense when run inside the ROS Noetic container.
- The client refuses non-finite state and invalid images.
- Unavailable service responses remain shadow-only and include `execution_allowed: false`.

Live shadow-client probe against an intentionally refused endpoint:

- Instruction: `Move toward the marked button`
- Endpoint: `http://127.0.0.1:1`
- Result: service unavailable, as expected.
- Execution: not allowed.
- `execution_allowed`: `false`
- Live state read from `/end_pose`:
  - `x_m`: `0.055984`
  - `y_m`: `0.000667`
  - `z_m`: `0.214169`
  - `roll_rad`: `-0.23647466035271253`
  - `pitch_rad`: `1.4782764231466787`
  - `yaw_rad`: `-0.2255314459427081`
  - `gripper_m`: `0.0`

Directional reasonableness: not assessed. Human review is still required after real VLAC outputs are available.
