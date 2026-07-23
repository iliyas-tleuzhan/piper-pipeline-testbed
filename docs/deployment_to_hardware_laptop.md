# Deployment To Hardware Laptop

Use this split:

- Host Ubuntu 24.04: repo management and non-ROS unit tests.
- `abot-piper-noetic`: physical PiPER runtime with ROS Noetic and Python 3.8.
- ABot-Claw: read-only reference unless a separate minimal compatibility change is explicitly approved.

Before motion:

1. Start the ABot-Claw stack manually.
2. Verify `can0`, ROS master, `/joint_states_single`, `/end_pose`, table camera topics, TF, MoveIt services, and `8891`.
3. Copy `piper-on-bunker/config/piper_laptop_hardware.local.example.yaml` to `piper-on-bunker/config/piper_laptop_hardware.local.yaml`.
4. Calibrate named poses with `piper-on-bunker/scripts/calibrate_named_pose.py`.
5. Configure camera-to-PiPER transform and workspace bounds.
6. Keep `local_activation.physical_motion_enabled: false` until dry-run and operator approval.

The committed hardware config keeps `physical_motion_enabled: false`.

Current discovered hardware values:

- Complete mission backend: `arm_adapter: piper_ros`.
- Restricted 8891 backend: health/state and bounded nudge/gripper experiments only.
- MoveIt planning frame: `dummy_link`.
- Camera frame: `table_camera_color_optical_frame`.
- Active arm joints: `joint1..joint6`.

Full live dry-run command:

```bash
cd ~/piper-pipeline-testbed
docker exec -it abot-piper-noetic bash -lc '
  cd /tmp/piper-pipeline-testbed-live &&
  source /opt/ros/noetic/setup.bash &&
  source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash &&
  export ROS_MASTER_URI=http://localhost:11311 ROS_HOSTNAME=localhost &&
  export PYTHONPATH=$PWD/piper-on-bunker/src:$PYTHONPATH &&
  python3 -m piper_on_bunker.cli live-dry-run --config piper-on-bunker/config/piper_laptop_dry_run.yaml
'
```
