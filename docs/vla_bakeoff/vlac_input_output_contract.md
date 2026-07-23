# VLAC Shadow Contract

Remote audit status: blocked on SSH connectivity to `iliyas@master` and `iliyas@192.168.1.104` on July 23, 2026.

Expected action-preview endpoint:

- `GET /health`
- `GET /model-info`
- `POST /action-preview`
- Port: `8016`
- Mode: `shadow_only`
- Mandatory response field: `execution_allowed: false`

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

The legacy example note says the policy may expect XYZ in `0.001 mm` and RPY in `0.001 degrees`. That is not yet verified against `get_action_prompt`, `format_state`, `results_format`, or the policy system prompt because the remote repository is unreachable. The client supports this conversion only under the explicit format name `legacy_xyz_0p001mm_rpy_0p001deg`.

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
