class VlacPolicy:
    def __init__(self, enabled: bool = False) -> None:
        self.enabled = enabled

    def require_enabled(self) -> None:
        if not self.enabled:
            raise RuntimeError("VLAC policy is optional and disabled by default")
