# Agent Tool Contract

Allowed commands:

- `prepare_navigation_view`
- `inspect_workspace`
- `scan_region`
- `find_target`
- `press_target`
- `retract`
- `return_to_navigation_view`
- `run_button_mission`
- `get_pipeline_status`
- `stop`

The `target` request field is passed to detection and mission execution.

Denied by design:

- arbitrary Python
- shell execution
- ROS publishing
- raw joint arrays through the API
- `/code/execute`

`stop` is mission/controller cancellation. It is not a verified physical emergency stop unless the live stack provides and verifies a physical stop or disable API.
