# VLAC Shadow Contract

Remote audit status: completed against `iliyas@192.168.1.104` on July 23, 2026.

Action-preview endpoint:

- `GET /health`
- `GET /model-info`
- `POST /action-preview`
- Port: `8016`
- Mode: `shadow_only`
- Mandatory response field: `execution_allowed: false`
- Existing critic remains separate on port `8014` with `POST /critic`.

Laptop request format uses SI units:

```json
{
  "images": ["data:image/jpeg;base64,..."],
  "task_description": "Move toward the marked button",
  "end_effector_state": {
    "x_m": 0.0,
    "y_m": 0.0,
    "z_m": 0.0,
    "roll_rad": 0.0,
    "pitch_rad": 0.0,
    "yaw_rad": 0.0,
    "gripper_m": 0.0
  },
  "input_state_model_units": {},
  "state_format": "si_json",
  "history": [],
  "execution_allowed": false
}
```

Discovered VLAC policy formatting:

- Source: `/data/home/iliyas/ABot-Claw-piper/service_layer/VLAC/evo_vlac/examples/vla_example.py`
- Policy object: `GAC_model(tag="Policy")`
- Input images: one to three images.
- Input state: seven values `[x, y, z, roll, pitch, yaw, gripper]`.
- Example pre-format units: XYZ in `0.001 mm`, RPY in `0.001 degrees`; gripper convention remains unverified.
- `GAC_model.format_state(..., gripper_format=False)` divides the first six values by `1000`, producing prompt state units in `mm` and `degrees`; it also divides the gripper value by `1000`.
- Prompt action grammar: `{x: ...mm, y: ...mm, z: ...mm, roll: ... degrees, pitch: ... degrees, yaw: ... degrees, open: ...}`.
- The shadow client preserves parsed numeric values as neutral raw translation and rotation values only. It does not claim they are relative PiPER deltas.
- Action semantics remain `unknown_songling_convention`.
- The testbed client sends SI units to 8016 and preserves the original SI values. The 8016 service performs the explicit SI-to-legacy conversion for the model prompt.
- The gripper input convention is unverified. The shadow client records the live PiPER gripper value, but the remote 8016 service may substitute a documented placeholder when calling `format_state`.
- Synchronous policy inference is not safely interruptible in the current 8016 service. The client can time out its own HTTP request, but the server does not currently preempt a running model call.

Standard model-independent action:

```json
{
  "raw_translation_m": [null, null, null],
  "raw_rotation_rad": [null, null, null],
  "gripper_command_raw": null,
  "model_name": "vlac-2b",
  "raw_action": "...",
  "action_semantics": "unknown_songling_convention",
  "parse_reliability": "failed",
  "model_confidence": null,
  "unverified_adapter_assumption": null,
  "execution_allowed": false
}
```
