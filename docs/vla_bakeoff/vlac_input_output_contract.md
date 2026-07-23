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
- Action output is treated as a delta end-effector action from the VLAC/Songling convention. The frame is not verified for PiPER and remains `unknown`.
- The testbed client sends SI units to 8016 and preserves the original SI values. The 8016 service performs the explicit SI-to-legacy conversion for the model prompt.

Standard model-independent action:

```json
{
  "translation_delta_m": [null, null, null],
  "rotation_delta_rad": [null, null, null],
  "gripper_command": null,
  "model_name": "vlac-2b",
  "raw_action": "...",
  "action_frame": "unknown",
  "confidence": "failed",
  "execution_allowed": false
}
```
