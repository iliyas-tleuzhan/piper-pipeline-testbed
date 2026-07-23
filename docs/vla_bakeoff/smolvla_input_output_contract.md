# SmolVLA Input/Output Contract

Remote shadow service:

- `GET http://192.168.1.104:8018/health`
- `GET http://192.168.1.104:8018/model-info`
- `POST http://192.168.1.104:8018/compatibility-check`
- `POST http://192.168.1.104:8018/action-preview`

Safety rules:

- `execution_allowed` is always `false`.
- The client calls `/compatibility-check` before `/action-preview`.
- If compatibility fails, the client refuses inference and logs the refusal.
- No MoveIt, 8891, CAN, or PiPER motion adapter is imported or called.

Current PiPER schema under audit:

- one external RealSense RGB image
- optional wrist RGB image
- 7D state `[joint1..joint6, gripper]`
- language instruction
- proposed 7D joint-target action

Current result:

- the released `lerobot/smolvla_base` checkpoint is incompatible with this PiPER schema
- the shadow client records the incompatibility result instead of forcing inference
