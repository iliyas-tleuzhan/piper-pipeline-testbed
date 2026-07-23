SAFE_NAMED_POSES = {
    "tabletop_home",
    "simulated_front_nav_view",
    "simulated_rear_nav_view",
    "scan_left",
    "scan_center",
    "scan_right",
    "inspect_workspace",
    "pre_contact",
    "retracted",
    "stowed",
    "safe_recovery",
}


def validate_named_pose(name: str) -> None:
    if name not in SAFE_NAMED_POSES:
        raise ValueError(f"Unknown named pose: {name}")
