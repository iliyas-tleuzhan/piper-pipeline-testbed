import base64
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest

from piper_on_bunker.policies.smolvla_shadow_policy import (
    JointStateRecord,
    SmolVLAShadowPolicyClient,
    build_action_preview_request,
    build_piper_schema_payload,
    default_log_path,
    encode_image_bytes,
    read_state_json,
)


PNG_1PX = (
    "data:image/png;base64,"
    + base64.b64encode(
        b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01"
        b"\x08\x02\x00\x00\x00\x90wS\xde\x00\x00\x00\x0cIDATx\x9cc```\x00\x00\x00\x04\x00\x01"
        b"\xf6\x178U\x00\x00\x00\x00IEND\xaeB`\x82"
    ).decode("ascii")
)


def sample_state() -> JointStateRecord:
    return JointStateRecord(
        arm_joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        arm_positions_rad=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        gripper_value=0.0,
        stamp_s=100.0,
        age_s=0.1,
    )


class Handler(BaseHTTPRequestHandler):
    compatibility = {"request_success": True, "compatible": False, "execution_allowed": False}
    preview = {"request_success": True, "compatible": True, "inference_attempted": False, "execution_allowed": True}

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        body = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
        if self.path == "/compatibility-check":
            assert body["schema"]["action"]["execution_allowed"] is False
            payload = self.compatibility
            status = 200
        elif self.path == "/action-preview":
            payload = self.preview
            status = 200
        else:
            payload = {"request_success": False, "execution_allowed": False}
            status = 404
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(payload).encode("utf-8"))

    def log_message(self, *args):
        return


def serve(handler=Handler):
    server = HTTPServer(("127.0.0.1", 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def test_build_schema_payload_is_shadow_only():
    payload = build_piper_schema_payload()
    assert payload["schema"]["action"]["execution_allowed"] is False
    assert payload["schema"]["observation"]["state_dimension"] == 7


def test_build_action_preview_request():
    payload = build_action_preview_request([PNG_1PX], "Move toward the button", sample_state())
    assert payload["joint_state"]["joint1"] == pytest.approx(0.1)
    assert payload["execution_allowed"] is False


def test_read_state_json_supports_joint_names(tmp_path: Path):
    path = tmp_path / "state.json"
    path.write_text(
        json.dumps(
            {
                "joint_names": ["joint3", "joint2", "gripper", "joint1", "joint4", "joint6", "joint5"],
                "joint_positions": [0.3, 0.2, 0.9, 0.1, 0.4, 0.6, 0.5],
                "stamp_s": 123.0,
            }
        ),
        encoding="utf-8",
    )
    state = read_state_json(path)
    assert state.arm_positions_rad == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert state.gripper_value == pytest.approx(0.9)


def test_compatibility_rejection_short_circuits_action_preview():
    server = serve()
    try:
        client = SmolVLAShadowPolicyClient(f"http://127.0.0.1:{server.server_port}", timeout_s=2)
        result = client.preview_action(build_action_preview_request([PNG_1PX], "Move left", sample_state()))
        assert result["request_success"] is True
        assert result["inference_attempted"] is False
        assert result["execution_allowed"] is False
    finally:
        server.shutdown()


def test_execution_allowed_is_forced_false_even_if_server_lies():
    class SuccessHandler(Handler):
        compatibility = {"request_success": True, "compatible": True, "execution_allowed": True}
        preview = {"request_success": True, "compatible": True, "inference_attempted": True, "execution_allowed": True}

    server = serve(SuccessHandler)
    try:
        client = SmolVLAShadowPolicyClient(f"http://127.0.0.1:{server.server_port}", timeout_s=2)
        result = client.preview_action(build_action_preview_request([PNG_1PX], "Move left", sample_state()))
        assert result["execution_allowed"] is False
    finally:
        server.shutdown()


def test_service_unavailable():
    client = SmolVLAShadowPolicyClient("http://127.0.0.1:1", timeout_s=0.1)
    result = client.preview_action(build_action_preview_request([PNG_1PX], "Move left", sample_state()))
    assert result["request_success"] is False
    assert result["execution_allowed"] is False


def test_image_encoder_rejects_non_image_bytes():
    with pytest.raises(ValueError):
        encode_image_bytes(b"not-an-image")


def test_default_log_path(tmp_path: Path):
    path = default_log_path(tmp_path)
    assert "smolvla_shadow" in path.name
    assert path.parent.name == "smolvla"


def test_shadow_module_does_not_import_motion_adapters():
    import piper_on_bunker.policies.smolvla_shadow_policy as module

    names = set(module.__dict__)
    assert "PiperRosArm" not in names
    assert "ABotClawApiArm" not in names
