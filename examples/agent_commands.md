# Restricted Agent Commands

Allowed: `prepare_navigation_view`, `inspect_workspace`, `scan_region`, `find_target`, `press_target`, `retract`, `return_to_navigation_view`, `run_button_mission`, `get_pipeline_status`, `stop`.

Denied: arbitrary ROS publishing, Python execution, shell execution, raw motor commands, unvalidated joint arrays, disabling safety checks.
