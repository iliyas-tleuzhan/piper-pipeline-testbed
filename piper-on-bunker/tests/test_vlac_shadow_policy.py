import base64
import math
from http.server import BaseHTTPRequestHandler, HTTPServer
from threading import Thread

import pytest

from piper_on_bunker.policies.standard_action import StandardAction
from piper_on_bunker.policies.vlac_shadow_policy import (
    EndEffectorStateSI,
    VlacShadowPolicyClient,
    build_action_preview_request,
    parse_vlac_actions,
    quaternion_to_rpy_rad,
    si_to_model_state,
    validate_images,
)


PNG_1PX = base64.b64encode(
    b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01"
    b"\x08\x02\x00\x00\x00\x90wS\xde\x00\x00\x00\x0cIDATx\x9cc```\x00\x00\x00\x04\x00\x01"
    b"\xf6\x178U\x00\x00\x00\x00IEND\xaeB`\x82"
).decode("ascii")


def state():
    return EndEffectorStateSI(0.1, -0.2, 0.3, 0.01, -0.02, 0.03, 0.004)


def test_si_to_model_state_si_json():
    assert si_to_model_state(state(), "si_json")["x_m"] == pytest.approx(0.1)


def test_si_to_model_state_legacy_units():
    converted = si_to_model_state(state(), "legacy_xyz_0p001mm_rpy_0p001deg")
    assert converted[:3] == pytest.approx([100000.0, -200000.0, 300000.0])
    assert converted[3] == pytest.approx(math.degrees(0.01) * 1000.0)


def test_quaternion_to_rpy_yaw():
    yaw = math.pi / 2.0
    roll, pitch, got_yaw = quaternion_to_rpy_rad(0, 0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)
    assert got_yaw == pytest.approx(yaw)


def test_non_finite_state_rejected():
    with pytest.raises(ValueError):
        EndEffectorStateSI(float("nan"), 0, 0, 0, 0, 0).validate()


def test_image_validation_counts_and_invalid_data():
    valid = "data:image/png;base64," + PNG_1PX
    assert validate_images([valid, valid, valid])
    with pytest.raises(ValueError):
        validate_images([])
    with pytest.raises(ValueError):
        validate_images([valid] * 4)
    with pytest.raises(ValueError):
        validate_images(["not-base64"])


def test_missing_instruction_rejected():
    with pytest.raises(ValueError):
        build_action_preview_request(["data:image/png;base64," + PNG_1PX], " ", state())


def test_parse_json_action_success():
    actions = parse_vlac_actions('{"dx_m": 0.01, "dy_m": -0.02, "dz_m": 0.0, "droll_rad": 0, "dpitch_rad": 0, "dyaw_rad": 0, "gripper": 1, "action_frame": "eef"}')
    assert len(actions) == 1
    assert actions[0].translation_delta_m == [0.01, -0.02, 0.0]
    assert actions[0].action_frame == "eef"
    assert actions[0].execution_allowed is False


def test_parse_numeric_action_medium_confidence():
    action = parse_vlac_actions("action: [0.1, 0.2, 0.3, 0, 0, 0, -1]")[0]
    assert action.confidence == "medium"
    assert action.gripper_command == -1


def test_parse_vlac_prompt_units_to_standard_si():
    action = parse_vlac_actions(
        "{x: 0.1mm, y: 2.0mm, z: -3.0mm, roll: 1.0 degrees, pitch: -2.0 degrees, yaw: 3.0 degrees, open: 0.8}"
    )[0]
    assert action.confidence == "high"
    assert action.translation_delta_m == pytest.approx([0.0001, 0.002, -0.003])
    assert action.rotation_delta_rad == pytest.approx([math.radians(1.0), math.radians(-2.0), math.radians(3.0)])
    assert action.gripper_command == pytest.approx(0.8)
    assert action.execution_allowed is False


def test_parse_failure():
    assert parse_vlac_actions("move left please") == []


def test_unknown_units_and_action_frame():
    with pytest.raises(ValueError):
        si_to_model_state(state(), "unknown")
    action = parse_vlac_actions('{"dx_m": 0.01, "action_frame": "tool_magic"}')[0]
    assert action.action_frame == "unknown"


class Handler(BaseHTTPRequestHandler):
    response = {"success": True, "raw_model_output": '{"dx_m": 0.01, "dy_m": 0, "dz_m": 0, "droll_rad": 0, "dpitch_rad": 0, "dyaw_rad": 0}'}

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length)
        assert b"execution_allowed" in body
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(__import__("json").dumps(self.response).encode())

    def log_message(self, *args):
        return


def serve():
    server = HTTPServer(("127.0.0.1", 0), Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def test_client_mandatory_execution_false_and_schema():
    server = serve()
    try:
        payload = build_action_preview_request(["data:image/png;base64," + PNG_1PX], "Move left", state())
        result = VlacShadowPolicyClient(f"http://127.0.0.1:{server.server_port}", timeout_s=2).preview_action(payload)
        assert result["execution_allowed"] is False
        assert result["standard_actions"][0]["execution_allowed"] is False
    finally:
        server.shutdown()


def test_unavailable_service_returns_shadow_failure():
    payload = build_action_preview_request(["data:image/png;base64," + PNG_1PX], "Move left", state())
    result = VlacShadowPolicyClient("http://127.0.0.1:1", timeout_s=0.1).preview_action(payload)
    assert result["success"] is False
    assert result["execution_allowed"] is False
    assert result["parse_confidence"] == "failed"


def test_standard_action_serializes_execution_false():
    data = StandardAction([0, 0, 0], [0, 0, 0], None, "vlac-2b", "raw", execution_allowed=True).to_dict()
    assert data["execution_allowed"] is False


def test_shadow_module_does_not_import_motion_adapters():
    import piper_on_bunker.policies.vlac_shadow_policy as module

    names = set(module.__dict__)
    assert "PiperRosArm" not in names
    assert "ABotClawApiArm" not in names
