from piper_on_bunker.perception.external_realsense import ExternalFixedCamera


class WristCamera(ExternalFixedCamera):
    def __init__(self) -> None:
        super().__init__(camera_name="wrist_camera")
