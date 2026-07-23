# VLA Bakeoff

VLAC is being tested only in shadow mode. The model may propose text or delta actions, but no output is connected to MoveIt, ABot-Claw movement endpoints, ROS publishers, or PiPER command topics.
The parser may recognize the expected VLAC text grammar, but that is not a calibrated model confidence score and not evidence that the action is correct.

Current split:

- Port 8014 remains the existing VLAC critic service.
- Port 8016 is reserved for a separate VLAC action-preview service.
- Port 8018 is reserved for a separate SmolVLA compatibility and action-preview service.
- `piper-pipeline-testbed` only contains a shadow client and model-independent action schema.

Zero-shot action correctness is not assumed. The action frame, units, parser grammar, and directional behavior must be verified before any execution design is considered.
The current VLAC-2B results fail the left/right, up/down, forward/backward, open/close, and approach/press/retract behavioral checks, so zero-shot PiPER execution is rejected. The model remains available only for shadow research.
The current SmolVLA pass is stricter still: the released `lerobot/smolvla_base` checkpoint is treated as incompatible with the PiPER 7D joint-plus-gripper schema unless embodiment-specific dataset statistics and camera/state/action features are established honestly.
