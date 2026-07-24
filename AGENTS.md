# AGENTS.md

This file applies to the entire `piper-pipeline-testbed` repository.

## Purpose

This repository owns the PiPER-side tabletop and future mobile-manipulation pipeline work:

- mission supervision and state machines
- camera/perception clients
- VLA shadow clients
- safety validation
- authority/resource management
- mock and replay testing
- hardware-facing PiPER adapters only when explicitly enabled
- future integration contracts for the Bunker controller

The final system is multi-process. Do not collapse it into one implicit controller. Keep responsibility split between the mission supervisor, deterministic robot skills, hardware adapters, and external services.

## Physical Safety

Default to no physical motion.

Never send PiPER, Bunker, MoveIt, CAN, `piper_sdk`, joint, gripper, Cartesian, or `8891` movement commands merely because a model, dry run, or test produced an action.

Physical movement requires all of the following:

1. The user explicitly requests physical movement in the current task.
2. The exact hardware and command path are identified.
3. Current robot state is fresh.
4. Required camera/state transforms are valid.
5. Workspace and joint limits are checked.
6. The base is stopped and locked for manipulation.
7. A stop or recovery mechanism is available.
8. The movement starts with a small, low-speed, bounded test.
9. The user is told exactly what will move before execution.

Do not treat “run the demo,” “finish the pipeline,” or “test the model” as permission to move hardware.

Keep committed hardware configuration disabled by default. Never silently change `physical_motion_enabled` or equivalent settings.

## Repository Boundaries

This repository owns:

- PiPER-side orchestration
- state machines
- perception and transform clients
- safety checks and dry-run semantics
- JSONL mission logging
- restricted agent/tool contracts
- mock, replay, and read-only live checks

It does not own the Bunker controller implementation. Treat Bunker navigation/manipulation coordination as an external contract. Do not invent or silently implement a Bunker API. Require explicit base stopped/locked state before manipulation.

## Shadow Model Rules

All unvalidated VLA/model integrations must remain shadow-only.

Every shadow response must enforce `execution_allowed: false`. Force that locally even if a remote service returns `true`.

Shadow clients and helpers must not import or call:

- MoveIt motion services
- PiPER command APIs
- `piper_sdk` movement methods
- CAN transmission
- ROS command publishers
- `8891` movement endpoints
- Bunker movement interfaces
- generic physical robot executors

Keep these outcomes separate:

- request success
- compatibility result
- inference success
- parser reliability
- output-shape validity
- model confidence
- behavioral success
- execution permission

Do not use ambiguous generic `success: true` fields without defining what succeeded. Parser reliability is not model confidence.

## Compatibility Before Implementation

Before implementing a new VLA checkpoint or client, perform a static compatibility audit from the official source and checkpoint configuration.

Verify:

1. required camera count and image keys
2. preprocessing and normalization
3. required robot-state dimension and semantics
4. output action dimension and semantics
5. relative vs absolute actions
6. joint-space vs Cartesian outputs
7. rotation representation and units
8. gripper representation and units
9. dataset statistics and metadata requirements
10. action chunk length
11. GPU/VRAM and disk requirements
12. zero-shot support vs fine-tuning requirement
13. whether omitted inputs or dimension adaptation are legitimate

Do not force compatibility by padding/truncating dimensions, inventing camera keys, borrowing unrelated normalization statistics, or treating any arbitrary 7D output as a PiPER action.

## Embodiment and State Rules

Current proposed PiPER joint-space representation:

- state: `[joint1, joint2, joint3, joint4, joint5, joint6, gripper]`
- action: `[target_joint1, target_joint2, target_joint3, target_joint4, target_joint5, target_joint6, target_gripper]`

Assume arm joints are radians unless the verified interface says otherwise. Treat gripper observation/action units as unverified until confirmed from the PiPER driver and live messages.

Map ROS joints by name, not array position. Reject missing joints, duplicate joints, stale state, stale images, non-finite values, unexpected names, malformed images, and invalid transform provenance.

Preserve source timestamps, receive timestamps, topic/frame names, units, joint ordering, transform provenance, and calibration provenance.

## Navigation and Manipulation Coordination

Use explicit operational modes such as:

- `FORWARD_TRANSIT`
- `REVERSE_TRANSIT`
- `TABLE_SEARCH`
- `SIDE_DOCKING`
- `ACTIVE_OBSERVATION`
- `MANIPULATION`
- `CARRYING`
- `RECOVERY`

During navigation, bounded arm observation is allowed only as an explicit camera-stand role. Manipulation is not.

During manipulation, the base must be stopped and locked, the selected arm must have clear authority, the other arm must have an explicit role, and recovery must not depend on a learned policy.

Use deterministic conventional commands for observation poses, stow poses, safe retraction, and recovery.

## Testing Order

Use this progression:

1. static source/config audit
2. unit tests
3. mock test
4. replay test
5. saved-image/state shadow test
6. live read-only state and camera test
7. live no-motion action preview
8. offline behavioral comparison
9. planning-only or simulation validation
10. small physical movement only after explicit user authorization

Do not skip from model output to physical execution.

Contradictory-prompt tests should cover left/right, up/down, forward/backward, open/close, approach/retract, and press/retract. For stochastic models, run repeated trials and report variance.

## Physical VLA Test Handoff

Use this durable workflow for any model-driven physical test:

1. Codex performs compatibility, mock, replay, saved-input, live read-only, shadow-inference, and planning-only tests itself.
2. Codex does not execute physical motion during the audit.
3. When technically justified, the final report provides one exact physical-test command for the user to enter manually.
4. That report must also provide the expected movement, required initial pose, input/model action being tested, expected completion signal, exact stop command, and resulting log path.
5. Avoid repeated confirmation arguments. The user's manual command entry is the authorization boundary.
6. The command must perform one bounded action or short action chunk, never an autonomous indefinite loop.
7. Retain verified joint/workspace limits, fresh-state validation, finite-value checks, base-stopped requirement, communication-loss stopping, and result logging.
8. Do not let model output choose speed, acceleration, limits, or duration.
9. If frame semantics, units, normalization, transforms, gripper semantics, or command mapping remain unknown, provide the exact calibration or validation step instead of inventing executable motion.
10. Clearly report whether Codex executed no movement, whether a manual command was generated, whether the user later ran it, and whether physical behavior was observed.

## Git and Filesystem Safety

Always inspect `git status` before editing. Preserve unrelated modified and untracked files.

Do not use `git reset --hard`, `git clean`, broad `git restore`, force push, recursive deletion of broad directories, or broad cache deletion.

Use forward commits to `main` unless the user explicitly requests another workflow.

Before committing:

1. review the diff
2. run relevant tests and compile checks
3. confirm no unrelated files are staged
4. confirm no secrets, tokens, logs, model weights, or captured camera frames are staged
5. confirm ignored local hardware-activation files remain untracked
6. report the exact tests actually run

## Documentation and Reporting

Keep detailed model results in the existing docs, not here.

Known conclusions belong in `docs/vla_bakeoff/` and related architecture/audit docs:

- VLAC-2B was runnable in shadow mode but rejected as a zero-shot PiPER policy.
- SmolVLA’s audited base checkpoint was statically incompatible with the proposed PiPER schema and was removed.
- OpenVLA-7B was runnable in shadow mode but rejected as a zero-shot PiPER policy.
- Future VLA work should prioritize PiPER-specific demonstration collection and embodiment adaptation/fine-tuning.
- X-VLA is a candidate for PiPER-specific fine-tuning, not an assumed zero-shot controller.

When reporting work, clearly distinguish: implemented, mocked, replayed, observed read-only on live hardware, physically executed, blocked, and inferred but unverified.
