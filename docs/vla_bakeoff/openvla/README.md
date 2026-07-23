# OpenVLA Shadow Bakeoff

This directory tracks the OpenVLA-7B PiPER audit.

- Service port: `8018`
- Service mode: shadow-only
- Model: `openvla/openvla-7b`
- No OpenVLA output is executable.
- PiPER state is logged as metadata only and is not fed into the official checkpoint.

OpenVLA can be schema-compatible for shadow inference while remaining physically incompatible with PiPER execution because the BridgeData action frame, rotation convention, and gripper semantics are not PiPER-verified.
The July 23, 2026 bakeoff showed deterministic 7D outputs and some language-conditioned changes, but not enough evidence to justify zero-shot PiPER execution.
