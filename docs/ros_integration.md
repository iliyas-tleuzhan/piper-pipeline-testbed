# ROS Integration

Required later on PiPER laptop:

- `/joint_states_single`
- `/end_pose`
- `joint_moveit_ctrl_arm`
- `joint_moveit_ctrl_endpose`
- `joint_moveit_ctrl_gripper`
- `joint_moveit_ctrl_piper`
- `can0` at `1000000`

Hardware imports are optional in this repository and fail with a clear message on non-ROS machines.

## LAP-3B Cartesian Execution Note

For the current PiPER proof of concept, LAP horizon rows are treated according to the
official LAP real-robot postprocessing: each returned row is a future Cartesian delta
from the current TCP pose, not an increment from the previous LAP row.

The local adapter therefore:

- reads MoveIt's live `gripper_tcp` pose in planning frame `dummy_link`
- converts the selected LAP horizon rows into absolute TCP waypoints from that one current pose
- preserves the current TCP quaternion for the translation-only proof of concept
- plans one continuous MoveIt trajectory for the accepted horizon instead of issuing one
  MoveIt request per tiny LAP row

`/end_pose` remains telemetry only unless its represented link is explicitly transformed
into the same TCP frame.
