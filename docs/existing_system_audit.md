# Existing System Audit

The authenticated GitHub account is `iliyas-tleuzhan`. `gh repo list --limit 100` showed `iliyas-tleuzhan/ABot-Claw-piper`; `gh repo view` reported visibility `PUBLIC`, default branch `main`, pushed `2026-07-17T01:39:58Z`. This differs from the prompt's “private” wording and is recorded as found, not assumed.

The reference repo was cloned read-only for inspection at `C:/Users/user/.codex/reference/ABot-Claw-piper`. The local `C:/Users/user/ABot-Claw` checkout was also inspected.

Verified current interfaces:

- Current safe language action API: `http://localhost:8891`, `piper_language_action_server_v1`.
- ROS state topics: `/joint_states_single`, `/end_pose`.
- MoveIt service layer names: `joint_moveit_ctrl_arm`, `joint_moveit_ctrl_endpose`, `joint_moveit_ctrl_gripper`, `joint_moveit_ctrl_piper`.
- External RealSense D555 tabletop topics: `/table_camera/color/image_raw`, `/table_camera/aligned_depth_to_color/image_raw`, `/table_camera/color/camera_info`.
- PiPER laptop context from runbooks: host `HKU-CPS`, repo `~/ABot-Claw`, container `abot-piper-noetic`, container path `/root/ABot-Claw`, tmux `piper_language_stack`, Ubuntu 20.04/ROS Noetic, CAN `can0` at `1000000`.

Deprecated or legacy:

- `http://localhost:8890` is the old minimal action server and reference checks explicitly warn not to use it.
- `http://localhost:8888` is the older full ABot/Piper agent server with `/code/execute`; this testbed does not use it for unrestricted execution.

Remote GPU services are documented as configurable only and disabled by default.
