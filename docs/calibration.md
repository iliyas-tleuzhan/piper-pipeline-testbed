# Calibration

No physical named-pose joint values are invented in hardware config. Real values must be recorded on the PiPER laptop or validated against the existing MoveIt configuration.

For PiPER-X wrist D435i work, run Phase 0A repeatability diagnostics before
joint-zero changes, FK edits, TCP measurement, or another hand-eye calibration:

```text
docs/calibration/PIPER_X_PHASE_0A_REPEATABILITY.md
```

Phase 0A only checks feedback, controller settling, and physical endpoint
repeatability. It does not mark joint zero, FK, TCP, or hand-eye calibration as
valid.
