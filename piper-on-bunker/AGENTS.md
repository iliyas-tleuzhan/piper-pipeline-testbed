# AGENTS.md

This file applies to `piper-on-bunker/` and inherits the repository root `AGENTS.md`.

## Local Scope

Keep this package focused on:

- mission supervisor and state machines
- PiPER arm/camera adapter interfaces
- safety validators and authority management
- shadow-policy clients
- structured mission logging
- mock/replay/live-read-only tooling

## Adapter Rules

Prefer lazy ROS imports so unit tests can run outside ROS.

Do not silently switch between backends such as `piper_ros`, `abotclaw_api`, mock, replay, or shadow-policy clients. The selected backend must be explicit in config and logs.

Dry-run and read-only modes must never call motion services, `8891` movement endpoints, gripper movement endpoints, or any direct robot command path.

Hardware mode must fail closed when activation files, calibrated poses, fresh state, fresh camera data, valid transforms, workspace bounds, or base-lock prerequisites are missing.

## Data and Logging

Use structured JSON/JSONL logs with monotonic freshness checks and explicit error codes.

Never serialize raw ROS messages or NumPy arrays directly into mission logs without converting them to stable JSON-safe structures.

Persist source timestamps, frame names, command previews, safety decisions, and whether an action was simulated, previewed, or physically executed.

## State and Perception

Map joint states by configured joint name. Reject missing or duplicate joints.

For image/depth work, preserve encoding, synchronization assumptions, and transform provenance. A valid 2D detection is not a valid 3D target without verified depth and deprojection.

## Physical VLA Test Handoff

If a shadow-policy or planning workflow reaches the point where one bounded hardware check is justified, stop at a manual handoff. Codex may generate one exact command plus the expected movement, required initial pose, tested model action, completion signal, stop command, and log path, but the user must enter that command manually.

Do not add repeated confirmation flags, do not execute the command during the audit, and do not allow model output to choose speed, acceleration, limits, or duration. If frames, transforms, units, gripper semantics, or command mapping remain unresolved, provide the exact calibration or validation step instead of executable motion.

## Local Test Pattern

When available, use the established test entrypoint:

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest piper-on-bunker/tests -q`

Also run compile checks for modified Python source and scripts.
