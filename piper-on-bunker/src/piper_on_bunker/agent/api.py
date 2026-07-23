from __future__ import annotations

import argparse

import uvicorn
from fastapi import FastAPI

from piper_on_bunker.agent.command_mapper import CommandMapper
from piper_on_bunker.agent.schemas import AgentCommand
from piper_on_bunker.agent.tools import list_allowed_tools
from piper_on_bunker.factory import build_supervisor_from_path


def create_app(config_path: str = "piper-on-bunker/config/development_mock.yaml") -> FastAPI:
    supervisor = build_supervisor_from_path(config_path)
    mapper = CommandMapper(supervisor)
    app = FastAPI(title="Restricted PiPER Pipeline Agent API")

    @app.get("/health")
    def health() -> dict:
        return {"success": True, "server": "piper_pipeline_restricted_agent", "allowed_tools": list_allowed_tools()}

    @app.get("/status")
    def status() -> dict:
        return supervisor.get_robot_state().__dict__

    @app.post("/command")
    def command(req: AgentCommand) -> dict:
        return mapper.map_command(req.command)().__dict__

    return app


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="piper-on-bunker/config/development_mock.yaml")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8892)
    args = parser.parse_args()
    uvicorn.run(create_app(args.config), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
