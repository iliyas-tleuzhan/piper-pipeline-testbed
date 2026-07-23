import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from threading import Thread

from piper_on_bunker.hardware.abotclaw_api_arm import ABotClawApiArm
from piper_on_bunker.models import Pose, StatusCode


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, payload):
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(payload).encode())

    def do_GET(self):
        if self.path == "/health":
            self._send(200, {"success": True, "server": "piper_language_action_server_v1"})
        elif self.path == "/state":
            self._send(200, {"success": True, "server": "piper_language_action_server_v1", "joint_positions": [0, 0, 0, 0, 0, 0]})
        else:
            self._send(404, {"success": False, "error": "not found"})

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        payload = json.loads(self.rfile.read(length).decode() or "{}")
        if self.path not in {"/move_down", "/move_up", "/open_gripper", "/close_gripper", "/set_gripper"}:
            self._send(404, {"success": False, "error": "not found"})
            return
        if self.path in {"/move_down", "/move_up"} and set(payload) != {"joint_step", "speed", "accel"}:
            self._send(422, {"success": False, "error": "bad schema", "payload": payload})
            return
        self._send(200, {"success": True, "server": "piper_language_action_server_v1", "payload": payload})

    def log_message(self, format, *args):
        return


def with_server():
    server = HTTPServer(("127.0.0.1", 0), Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def test_abotclaw_adapter_strict_contract():
    server = with_server()
    try:
        arm = ABotClawApiArm(f"http://127.0.0.1:{server.server_port}", dry_run=False)
        assert arm.health().success
        assert arm.read_state().success
        press = arm.press(Pose(0, 0, 0), 0.01)
        assert press.success
        assert press.outputs["payload"] == {"joint_step": 0.02, "speed": 0.03, "accel": 0.03}
        assert "target_pose_used_by_pipeline" in press.outputs
        assert arm.retract().success
    finally:
        server.shutdown()


def test_abotclaw_adapter_refuses_unsupported_motion():
    live_mode_arm = ABotClawApiArm("http://127.0.0.1:1", dry_run=False)
    assert live_mode_arm.move_to_named_pose("tabletop_home").status_code == StatusCode.NOT_IMPLEMENTED
    assert live_mode_arm.move_to_pose(Pose(0, 0, 0)).status_code == StatusCode.NOT_IMPLEMENTED


def test_abotclaw_adapter_dry_run_simulates_without_hardware():
    arm = ABotClawApiArm("http://127.0.0.1:1", dry_run=True)
    assert arm.move_to_named_pose("tabletop_home").success
    assert arm.move_to_pose(Pose(0, 0, 0)).success
    assert arm.press(Pose(0, 0, 0), 0.01).success
    assert arm.stop().status_code == StatusCode.NOT_IMPLEMENTED


def test_read_only_state_failure_is_not_success():
    from piper_on_bunker.mission_supervisor import MissionSupervisor
    from piper_on_bunker.perception.mock_camera import MockCamera

    supervisor = MissionSupervisor(ABotClawApiArm("http://127.0.0.1:1", dry_run=True), MockCamera())
    result = supervisor.get_robot_state()
    assert not result.success
    assert result.status_code == StatusCode.STALE_STATE
