import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from threading import Thread

from piper_on_bunker.hardware.abotclaw_api_arm import ABotClawApiArm
from piper_on_bunker.models import Pose


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps({"joint_positions": [0, 0, 0, 0, 0, 0]}).encode())

    def do_POST(self):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps({"success": True}).encode())

    def log_message(self, format, *args):
        return


def test_abotclaw_adapter_mock_http_server():
    server = HTTPServer(("127.0.0.1", 0), Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        arm = ABotClawApiArm(f"http://127.0.0.1:{server.server_port}", dry_run=False)
        assert "joint_positions" in arm.get_state()
        assert arm.press(Pose(0, 0, 0), 0.01).success
    finally:
        server.shutdown()
