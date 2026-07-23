# VLA Bakeoff

VLAC is being tested only in shadow mode. The model may propose text or delta actions, but no output is connected to MoveIt, ABot-Claw movement endpoints, ROS publishers, or PiPER command topics.

Current split:

- Port 8014 remains the existing VLAC critic service.
- Port 8016 is reserved for a separate VLAC action-preview service.
- `piper-pipeline-testbed` only contains a shadow client and model-independent action schema.

Zero-shot action correctness is not assumed. The action frame, units, parser grammar, and directional behavior must be verified before any execution design is considered.
