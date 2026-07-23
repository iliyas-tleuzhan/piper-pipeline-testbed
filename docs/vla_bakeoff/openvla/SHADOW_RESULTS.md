# OpenVLA Shadow Results

Date: 2026-07-23

Image source used for the bakeoff:

- One RealSense RGB frame captured from the live `table_camera` stream.
- The frame shows the PiPER workspace and a marked target card.
- It does not clearly show a physical button, so button-specific semantic claims remain limited.

Saved-image bakeoff:

- 11 instructions
- 5 repetitions per instruction
- deterministic inference
- 7D output shape valid for every response
- all repetitions for the same prompt were identical

Observed behavior:

- Contradictory prompts changed the 7D output numerically.
- `Open the gripper.` and `Close the gripper.` did not produce a useful gripper-channel distinction.
- No result was connected to any executor.

Live-image follow-up:

- A second fresh RealSense frame and matching live PiPER state were captured after the D555 publisher was restored.
- `Move toward`, `Press`, `Retract`, `Open`, and `Close` all returned valid 7D outputs.
- The gripper semantics remained unverified and did not justify execution.

Classification:

- Outcome B: technically runnable but behaviorally failed as a zero-shot PiPER execution policy.

Raw JSON results were saved outside Git during the run.
