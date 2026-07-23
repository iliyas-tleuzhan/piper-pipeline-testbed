from piper_on_bunker.agent.command_mapper import ALLOWED_AGENT_TOOLS


def list_allowed_tools() -> list[str]:
    return sorted(ALLOWED_AGENT_TOOLS)
