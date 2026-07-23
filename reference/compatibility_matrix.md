# Compatibility Matrix

| Mode | ROS | CAN | RealSense | PiPER | Remote GPU | Physical motion |
| --- | --- | --- | --- | --- | --- | --- |
| development_mock | no | no | no | no | no | no |
| tabletop_replay | no | no | no | no | no | no |
| piper_laptop_dry_run | optional state API | no | optional | no | disabled | no |
| piper_laptop_hardware | yes | yes | yes | yes | optional | explicit enable only |
| future_dual_piper_bunker | no | no | mock | mock | no | no |
