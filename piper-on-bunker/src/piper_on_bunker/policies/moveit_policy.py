from piper_on_bunker.models import Pose


class MoveItPolicy:
    def pre_contact_pose(self, target: Pose) -> Pose:
        return Pose(x=target.x, y=target.y, z=target.z + 0.08, qx=target.qx, qy=target.qy, qz=target.qz, qw=target.qw)
