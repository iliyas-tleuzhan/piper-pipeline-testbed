# VLA Bakeoff

This directory keeps the durable zero-shot evaluation record for rejected or non-executable VLA experiments.

- Port `8014` remains the existing VLAC critic service.
- Port `8016` remains available for the separate VLAC action-preview service.
- Zero-shot action correctness is not assumed.
- Shadow parsing or schema compatibility is not evidence that a model is safe to execute on PiPER.

Historical conclusions recorded here:

- `VLAC-2B` was technically runnable in shadow mode but rejected as a zero-shot PiPER policy.
- `OpenVLA-7B` was audited and rejected as a zero-shot PiPER policy.
- `SmolVLA` was statically incompatible with the proposed PiPER embodiment and was removed.
- `X-VLA` is no longer the active path for this repository.
