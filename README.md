# PiPER Pipeline Testbed

This repository is an offline-first testbed for a restricted PiPER-on-Bunker manipulation pipeline.

The first supported mission is tabletop-only: find a marked button, inspect it, press/touch it, verify the result, and return to a forward navigation-view pose. This laptop does not have PiPER, CAN, ROS, RealSense, Jetson, Bunker, or ABot-Claw hardware access, so all local tests use mock or replay adapters.

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
