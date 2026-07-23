import base64
from http.server import BaseHTTPRequestHandler, HTTPServer
from threading import Thread
from pathlib import Path

import pytest

from piper_on_bunker.policies.openvla_shadow_policy import (
    OpenVLARequestRecord,
    OpenVLAShadowPolicyClient,
    _map_joint_state,
    default_log_path,
    encode_image_bytes,
    save_shadow_result,
)


PNG_1PX = (
    b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01"
    b"\x08\x02\x00\x00\x00\x90wS\xde\x00\x00\x00\x0cIDATx\x9cc```\x00\x00\x00\x04\x00\x01"
    b"\xf6\x178U\x00\x00\x00\x00IEND\xaeB`\x82"
)
PNG_8PX = (
    b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x08\x00\x00\x00\x08"
    b"\x08\x02\x00\x00\x00Km)\xdc\x00\x00\x00\x13IDATx\x9cc`\xa0\x1c`\x0c\xa4"
    b"\x86Q\rCH\x03\x00@\xd8\x00\x09\xd0\xd3r\x89\x00\x00\x00\x00IEND\xaeB`\x82"
)


class Handler(BaseHTTPRequestHandler):
    compatibility = {
        "request_success": True,
        "shadow_inference_allowed": True,
        "physical_execution_compatible": False,
        "execution_allowed": False,
    }
    action = {
        "request_success": True,
        "compatibility_verified": True,
        "inference_attempted": True,
        "inference_success": True,
        "bridge_unnormalized_action": [0.0] * 7,
        "execution_allowed": True,
    }

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length)
        if self.path == "/compatibility-check":
            payload = self.compatibility
        elif self.path == "/action-preview":
            assert b"execution_allowed" in body
            assert b"piper_state_metadata" in body
            payload = self.action
        else:
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(__import__("json").dumps(payload).encode())

    def log_message(self, *args):
        return


def serve():
    server = HTTPServer(("127.0.0.1", 0), Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def test_saved_image_mode_bytes():
    encoded = encode_image_bytes(PNG_8PX)
    assert encoded.startswith("data:image/png;base64,")


def test_malformed_image_rejected():
    with pytest.raises(ValueError):
        encode_image_bytes(b"not-an-image")


def test_joint_name_mapping():
    state = _map_joint_state(
        ["gripper", "joint3", "joint1", "joint6", "joint4", "joint2", "joint5"],
        [0.7, 0.3, 0.1, 0.6, 0.4, 0.2, 0.5],
    )
    assert state.joint_names == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    assert state.joint_positions_rad == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert state.gripper_raw == pytest.approx(0.7)


def test_missing_joint_rejected():
    with pytest.raises(ValueError):
        _map_joint_state(["joint1"], [0.1])


def test_duplicate_joint_rejected():
    with pytest.raises(ValueError):
        _map_joint_state(
            ["joint1", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"],
            [0.0] * 8,
        )


def test_service_unavailable():
    record = OpenVLARequestRecord("Move left", encode_image_bytes(PNG_8PX), None, {"source": "test"})
    result = OpenVLAShadowPolicyClient("http://127.0.0.1:1", timeout_s=0.1).preview_action(record)
    assert result["request_success"] is False
    assert result["execution_allowed"] is False


def test_execution_allowed_forced_false():
    server = serve()
    try:
        record = OpenVLARequestRecord("Move left", encode_image_bytes(PNG_8PX), None, {"source": "test"})
        result = OpenVLAShadowPolicyClient(f"http://127.0.0.1:{server.server_port}", timeout_s=2).preview_action(record)
        assert result["request_success"] is True
        assert result["execution_allowed"] is False
    finally:
        server.shutdown()


def test_compatibility_refusal():
    server = serve()
    Handler.compatibility = {
        "request_success": True,
        "shadow_inference_allowed": False,
        "physical_execution_compatible": False,
        "execution_allowed": False,
    }
    try:
        record = OpenVLARequestRecord("Move left", encode_image_bytes(PNG_8PX), None, {"source": "test"})
        result = OpenVLAShadowPolicyClient(f"http://127.0.0.1:{server.server_port}", timeout_s=2).preview_action(record)
        assert result["request_success"] is False
        assert result["inference_attempted"] is False
        assert result["execution_allowed"] is False
    finally:
        Handler.compatibility = {
            "request_success": True,
            "shadow_inference_allowed": True,
            "physical_execution_compatible": False,
            "execution_allowed": False,
        }
        server.shutdown()


def test_malformed_response_rejected():
    server = serve()
    Handler.action = {"request_success": True, "bridge_unnormalized_action": [0.0] * 6, "execution_allowed": False}
    try:
        record = OpenVLARequestRecord("Move left", encode_image_bytes(PNG_8PX), None, {"source": "test"})
        result = OpenVLAShadowPolicyClient(f"http://127.0.0.1:{server.server_port}", timeout_s=2).preview_action(record)
        assert result["request_success"] is False
        assert result["inference_attempted"] is True
        assert result["execution_allowed"] is False
    finally:
        Handler.action = {
            "request_success": True,
            "compatibility_verified": True,
            "inference_attempted": True,
            "inference_success": True,
            "bridge_unnormalized_action": [0.0] * 7,
            "execution_allowed": True,
        }
        server.shutdown()


def test_logging(tmp_path: Path):
    record = OpenVLARequestRecord("Move left", encode_image_bytes(PNG_8PX), None, {"source": "test"})
    out = tmp_path / "result.json"
    save_shadow_result(out, record, {"request_success": True, "execution_allowed": True})
    data = out.read_text(encoding="utf-8")
    assert "NO ROBOT ACTION WILL BE EXECUTED" in data
    assert '"execution_allowed": false' in data


def test_default_log_path(tmp_path: Path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    path = default_log_path(tmp_path)
    assert "openvla" in str(path)
    assert path.name.endswith("_openvla_shadow.json")
