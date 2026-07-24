# Architecture

The pipeline is intentionally restricted:

```text
mission supervisor / restricted agent
  -> deterministic robot skills
  -> MoveIt and PiPER adapter
  -> PiPER hardware
```

The arm serves as movable camera stand, active perception device, and manipulator. The state machine supports `IDLE`, `STOWED`, `NAVIGATION_VIEW`, `ACTIVE_SCAN`, `TASK_INSPECTION`, `PRE_MANIPULATION`, `MANIPULATION`, `VERIFYING`, `RETRACTING`, `FAULT`, and `ESTOP`.

VLA/VLAC is optional and disabled by default. It can propose or score a bounded manipulation episode, but it is not the whole-robot controller.

Repository operating guidance lives in `../AGENTS.md` and `../piper-on-bunker/AGENTS.md`. Keep detailed experiment outcomes in the docs, and keep durable agent behavior rules in those AGENTS files.
