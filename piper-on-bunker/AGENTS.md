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

## Local Test Pattern

When available, use the established test entrypoint:

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest piper-on-bunker/tests -q`

Also run compile checks for modified Python source and scripts.
