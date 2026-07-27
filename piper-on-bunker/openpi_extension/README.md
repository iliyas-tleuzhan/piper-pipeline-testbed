# PiPER OpenPI Extension

These files are intentionally small extension points for the official OpenPI
repository pinned at `15a9616a00943ada6c20a0f158e3adb39df2ccac`.

They are not a vendored OpenPI copy. Apply them to an official OpenPI checkout
when creating the PiPER training/service environment on the RTX 5090 server.

Recommended starting point:

- model: pi0.5 flow-matching model
- base checkpoint: `gs://openpi-assets/checkpoints/pi05_base`
- fine-tuning mode: official low-memory JAX LoRA-style variant
- action dim: 7
- action horizon: 10
- state dim: 7
- image keys: `observation/exterior_image`, `observation/wrist_image`
- action semantics: absolute PiPER joint targets

Do not mark checkpoints PiPER-compatible until the dataset, normalization
statistics, checkpoint metadata, and offline safety validation pass.

