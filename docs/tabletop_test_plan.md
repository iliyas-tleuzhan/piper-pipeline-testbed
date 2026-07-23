# Tabletop Test Plan

Mission: find the marked button, inspect it, press it, verify the result, and return to the forward navigation-view pose.

Nominal flow:

`START -> SYSTEM_CHECK -> ACQUIRE_ARM_AUTHORITY -> HOME -> NAVIGATION_VIEW -> INSPECT -> DETECT_TARGET -> ESTIMATE_TARGET -> VALIDATE_TARGET -> PRE_CONTACT -> APPROACH -> PRESS -> RETRACT -> VERIFY -> RETURN_TO_NAVIGATION_VIEW -> RELEASE_ARM_AUTHORITY -> COMPLETE`

Failure flow:

`STOP -> RETRACT_IF_SAFE -> SAFE_RECOVERY -> RELEASE_AUTHORITY -> FAILED`
