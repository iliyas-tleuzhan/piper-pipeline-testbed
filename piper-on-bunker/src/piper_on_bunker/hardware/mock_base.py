class MockBase:
    def __init__(self, direction: str = "forward") -> None:
        self.locked = False
        self.direction = direction

    def lock_base(self) -> bool:
        self.locked = True
        return True

    def unlock_base(self) -> bool:
        self.locked = False
        return True

    def is_base_stopped(self) -> bool:
        return self.locked

    def request_navigation_pause(self) -> bool:
        return self.lock_base()

    def navigation_ready(self) -> bool:
        return not self.locked

    def get_navigation_direction(self) -> str:
        return self.direction
