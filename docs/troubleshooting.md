# Troubleshooting

- If hardware imports fail locally, use `development_mock.yaml` or `tabletop_replay.yaml`.
- If `8891` is unavailable on the PiPER laptop, start `start_piper_language_stack_tmux.sh`.
- If unit tests on the host import ROS Jazzy pytest plugins, run with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.
- If hardware mode reports disabled, check both committed config and ignored `piper_laptop_hardware.local.yaml`.
- If target detection has only a pixel and no 3D pose, verify aligned depth and camera intrinsics.
- If `stop` reports `NOT_IMPLEMENTED`, no verified physical stop API was found.
- If `8890` is active, treat it as deprecated.
- If target transforms are invalid, stop before motion and recalibrate camera-to-base.
