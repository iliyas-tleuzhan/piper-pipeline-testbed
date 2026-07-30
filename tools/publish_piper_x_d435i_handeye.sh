#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${CONTAINER:-abot-piper-noetic}"
ALLOW_REJECTED_HANDEYE_PUBLISH_FOR_DIAGNOSTICS="${ALLOW_REJECTED_HANDEYE_PUBLISH_FOR_DIAGNOSTICS:-false}"

if [ "$ALLOW_REJECTED_HANDEYE_PUBLISH_FOR_DIAGNOSTICS" != "true" ]; then
  cat >&2 <<'EOF'
ERROR: refusing to publish the saved PiPER-X D435i hand-eye transform.

The current saved calibration is explicitly rejected because fixed-marker
validation drifted by about 0.365 m. Publish it only for read-only diagnostic
reproduction with:

  ALLOW_REJECTED_HANDEYE_PUBLISH_FOR_DIAGNOSTICS=true ./tools/publish_piper_x_d435i_handeye.sh

Do not use it for new calibration acceptance or physical policy execution.
EOF
  exit 2
fi

docker exec -i "$CONTAINER" bash -lc '
source /opt/ros/noetic/setup.bash
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash
source /root/easy_handeye_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
exec roslaunch easy_handeye publish.launch eye_on_hand:=true namespace_prefix:=piper_x_d435i_wrist
'
