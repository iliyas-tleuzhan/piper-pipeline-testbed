#!/usr/bin/env bash
set -euo pipefail

CONTAINER=${CONTAINER:-abot-piper-noetic}

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "Container $CONTAINER is not running." >&2
  exit 2
fi

docker exec -i "$CONTAINER" bash -lc '
source /opt/ros/noetic/setup.bash
source /root/ABot-Claw/robot_layer/arm_piper/agent_server/robot_driver_ros/devel/setup.bash 2>/dev/null || true
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
python3 - <<PY
import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction

rospy.init_node("stop_piper_x_moveit_motion", anonymous=True, disable_signals=True)
stopped = {}
try:
    import moveit_commander
    moveit_commander.roscpp_initialize([])
    group = moveit_commander.MoveGroupCommander("arm")
    group.stop()
    stopped["move_group_stop"] = True
except Exception as exc:
    stopped["move_group_stop"] = f"unavailable: {exc!r}"

client = actionlib.SimpleActionClient("/arm_controllers/follow_joint_trajectory", FollowJointTrajectoryAction)
if client.wait_for_server(rospy.Duration(1.0)):
    client.cancel_all_goals()
    stopped["follow_joint_trajectory_cancel"] = True
else:
    stopped["follow_joint_trajectory_cancel"] = "action server unavailable"
print(stopped)
PY
'
