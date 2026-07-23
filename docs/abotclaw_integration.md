# ABot-Claw Integration

ABot-Claw should call the restricted skill/API surface in this repository. It should not call `8888 /code/execute` for this pipeline and should not publish raw ROS commands.

Current safe server `8891` supports bounded language actions; this repository maps higher-level deterministic skills to that interface in dry-run/hardware modes.
