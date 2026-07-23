# Next VLA Models

The standard shadow action schema is intentionally model-independent so later candidates can be compared without wiring any model to robot execution:

- SmolVLA
- X-VLA
- GR00T
- OpenVLA
- pi0.5

Each model must first run in shadow mode with:

- raw output preservation
- parser confidence
- explicit action-frame reporting
- explicit unit reporting
- `execution_allowed: false`

No candidate should be connected to PiPER movement until units, frame, and safety behavior are independently verified.
