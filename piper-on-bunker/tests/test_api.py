from fastapi.testclient import TestClient

from piper_on_bunker.agent.api import create_app


def test_agent_api_health_and_command():
    client = TestClient(create_app("piper-on-bunker/config/development_mock.yaml"))
    assert client.get("/health").json()["success"]
    assert client.post("/command", json={"command": "status"}).json()["success"]
