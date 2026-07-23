# OpenVLA Action Contract

The laptop client expects the remote shadow service to return:

- `bridge_unnormalized_action`: exactly 7 channels
- `action_channels`: `["x", "y", "z", "roll", "pitch", "yaw", "gripper"]`
- `raw_normalized_action` when available
- `execution_allowed: false`

The client rejects malformed action lengths and forces `execution_allowed` to false even if the remote side misbehaves.
