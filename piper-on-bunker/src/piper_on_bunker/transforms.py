from piper_on_bunker.models import Pose


def require_base_pose(pose: Pose | None) -> Pose:
    if pose is None:
        raise ValueError("target has no valid base-frame transform")
    return pose
