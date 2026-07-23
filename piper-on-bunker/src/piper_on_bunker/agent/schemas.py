from pydantic import BaseModel, Field


class AgentCommand(BaseModel):
    command: str = Field(..., min_length=1)
    target: str = "marked_button"
