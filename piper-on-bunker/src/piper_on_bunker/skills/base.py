from piper_on_bunker.models import SkillResult


class Skill:
    name = "base"

    def run(self, *args, **kwargs) -> SkillResult:
        raise NotImplementedError
