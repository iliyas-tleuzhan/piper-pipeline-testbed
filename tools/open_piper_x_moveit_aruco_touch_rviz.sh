#!/usr/bin/env bash
set -euo pipefail

CONTAINER=${CONTAINER:-abot-piper-noetic}
AGX_ARM_URDF_HOST=${AGX_ARM_URDF_HOST:-/home/dase-hw101/Iliyas/piper-vr-teleop/third_party/agx_arm_urdf}
STAGED_ROS_PKGS=${STAGED_ROS_PKGS:-/tmp/piper_x_moveit_ros}

cd "$(dirname "$0")/.."

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "Container $CONTAINER is not running; start the PiPER-X MoveIt runtime first." >&2
  exit 2
fi

if [[ ! -d "$AGX_ARM_URDF_HOST/piper_x" ]]; then
  echo "PiPER-X URDF asset tree not found: $AGX_ARM_URDF_HOST/piper_x" >&2
  exit 2
fi

xhost +SI:localuser:root >/dev/null
docker exec -i "$CONTAINER" bash -lc "mkdir -p '$STAGED_ROS_PKGS/agx_arm_description/agx_arm_urdf'"
tar -C "$AGX_ARM_URDF_HOST" -cf - piper_x | docker exec -i "$CONTAINER" bash -lc "tar -C '$STAGED_ROS_PKGS/agx_arm_description/agx_arm_urdf' -xf -"
docker exec -i "$CONTAINER" bash -lc "cat > '$STAGED_ROS_PKGS/agx_arm_description/package.xml' <<'XML'
<?xml version=\"1.0\"?>
<package format=\"2\">
  <name>agx_arm_description</name>
  <version>0.0.0</version>
  <description>Runtime-staged PiPER-X URDF assets for piper-pipeline-testbed.</description>
  <maintainer email=\"noreply@example.com\">piper-pipeline-testbed</maintainer>
  <license>Proprietary</license>
</package>
XML"

docker exec -it \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  "$CONTAINER" bash -lc "
    source /opt/ros/noetic/setup.bash
    source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash 2>/dev/null || true
    export ROS_PACKAGE_PATH=/root/piper-pipeline-testbed/piper-on-bunker/ros:$STAGED_ROS_PKGS:\${ROS_PACKAGE_PATH:-}
    export ROS_MASTER_URI=http://localhost:11311
    export ROS_HOSTNAME=localhost
    echo robot_description_name: \$(python3 - <<'PY'
import subprocess
import xml.etree.ElementTree as ET
try:
    xml = subprocess.check_output(['rosparam', 'get', '/robot_description'], text=True)
    print(ET.fromstring(xml).attrib.get('name', 'unknown'))
except Exception as exc:
    print('unavailable:', exc)
PY
)
    roslaunch piper_x_moveit_config rviz.launch
  "
