# PiPER Pipeline Testbed

This repository is an offline-first testbed for a restricted tabletop PiPER manipulation pipeline.

The first supported mission is tabletop-only: find a marked button, inspect it, press/touch it, verify the result, and return to a forward navigation-view pose.

Current implementation levels:

- Implemented and unit-tested: mock/dry-run mission flow, strict 8891 client, gated ROS MoveIt adapter, RealSense ROS camera adapter, ArUco detection, static transforms, local named-pose calibration, JSONL logging, safety checks, restricted agent API, and dual-arm mock.
- Verified read-only on the hardware laptop: ABot-Claw source contracts, `abot-piper-noetic` Python/ROS versions, container mount, and current absence of `can0`, running `8891`, and enumerated RealSense.
- Physically executed successfully: none.

Committed hardware configuration disables physical motion. Motion requires an ignored local activation file, calibrated named poses, safety bounds, live state, and operator approval.

## Quick Start

```bash
make install
make audit
make test
make mock-demo
make replay-demo
```

Hardware commands are for the PiPER-connected laptop later:

```bash
make piper-read-only
make camera-check
make dry-demo
make calibrate
make hardware-demo
make stop
```

The control shape is:

```text
Restricted mission supervisor
  -> deterministic robot skills
  -> MoveIt / PiPER adapter
  -> PiPER hardware
```

VLA/VLAC integrations are optional backends for constrained manipulation episodes. They are disabled by default and are not whole-robot controllers.
