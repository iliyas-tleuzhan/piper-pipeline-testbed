# SmolVLA Shadow Results

Current status: incompatibility audit only.

- Outcome: `compatible: false`
- Inference attempted: `false`
- Execution allowed: `false`

Known blockers from the official checkpoint contract:

- checkpoint `observation.state` dimension is 6, while PiPER proposes 7
- checkpoint `action` dimension is 6, while PiPER proposes 7
- checkpoint expects three camera inputs, while PiPER currently provides one required external camera and one optional wrist camera
- checkpoint normalization statistics are tied to the pretrained schema

No action output is converted into a robot command.
