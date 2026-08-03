from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any


SCHEMA_VERSION = "piper_x.repeatability.v1"
PHASE_0A_CONFIRMATION_TOKEN = "RUN_PHASE_0A_REPEATABILITY"
PHASE_0A_BASE_STOPPED_TOKEN = "BUNKER_STOPPED"

CLASS_INSUFFICIENT_DATA = "INSUFFICIENT_DATA"
CLASS_FEEDBACK_UNSTABLE = "FEEDBACK_UNSTABLE"
CLASS_CONTROLLER_OR_SETTLING_INCONSISTENT = "CONTROLLER_OR_SETTLING_INCONSISTENT"
CLASS_MECHANICAL_REPEATABILITY_SUSPECT = "MECHANICAL_REPEATABILITY_SUSPECT"
CLASS_JOINT_REPEATABLE_PHYSICAL_REPEATABILITY_UNKNOWN = "JOINT_REPEATABLE_PHYSICAL_REPEATABILITY_UNKNOWN"
CLASS_JOINT_AND_PHYSICAL_REPEATABILITY_ACCEPTABLE = "JOINT_AND_PHYSICAL_REPEATABILITY_ACCEPTABLE"
CLASS_MODEL_OR_CALIBRATION_INVESTIGATION_REQUIRED = "MODEL_OR_CALIBRATION_INVESTIGATION_REQUIRED"
CLASS_TEST_ABORTED = "TEST_ABORTED"

PHYSICAL_STATUS_ACCEPTABLE = "ACCEPTABLE"
PHYSICAL_STATUS_SUSPECT = "SUSPECT"
PHYSICAL_STATUS_UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class Phase0AThresholds:
    minimum_completed_cycles: int = 3
    maximum_joint_endpoint_error_rad: float = 0.02
    maximum_joint_repeatability_std_rad: float = 0.005
    maximum_joint_repeatability_range_rad: float = 0.01
    maximum_settle_time_s: float = 5.0
    maximum_stale_feedback_events: int = 0
    physical_repeatability_target_mm: float = 10.0
    maximum_pairwise_physical_spread_mm: float = 20.0
    maximum_measurement_uncertainty_mm: float = 5.0


@dataclass(frozen=True)
class Phase0AConfig:
    diagnostic_id_prefix: str = "piper_x_phase_0a"
    output_root: str = "piper-on-bunker/data/local/calibration/phase_0a"
    taught_pose_manifest: str = "piper-on-bunker/data/local/moveit_aruco_touch/taught_poses.yaml"
    moveit_touch_config: str = "piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml"
    measurement_pose_name: str = "staging"
    allowed_departure_pose_names: tuple[str, ...] = ("repeatability_departure",)
    default_departure_pose_name: str = "repeatability_departure"
    joint_state_topic: str = "/piper_x/joint_states"
    expected_feedback_source_id: str = "piper_x_passive_socketcan_feedback_v1"
    expected_joint_mapping_version: str = "piper_x_lora_feedback_2a5_2a6_2a7_raw001deg_to_rad_v1"
    expected_dependency_commit: str = "521c9c5fdfd9ee63bd96c0f9342fca6b2398092e"
    can_interface: str = "can0"
    arm_id: str = "unknown"
    arm_serial: str = "unknown"
    firmware: str = "unknown"
    physical_mounting_id: str = "unknown"
    side: str = "unknown"
    camera_serial: str = "unknown"
    camera_mount_id: str = "unknown"
    tool_id: str = "unknown"
    tool_description: str = "unknown"
    planning_group: str = "arm"
    planning_frame: str = "world"
    end_effector_link: str = "gripper_base"
    urdf_path: str = "unknown"
    urdf_sha256: str = "unknown"
    command_rate_hz: float = 50.0
    speed_percent: int = 30
    endpoint_tolerance_rad: float = 0.03
    settle_timeout_s: float = 3.0
    max_joint_state_age_s: float = 0.5
    max_abs_velocity_rad_s: float = 0.01
    observation_sample_rate_hz: float = 20.0
    observation_duration_s: float = 10.0
    require_base_stopped_ack: bool = True
    physical_execution_enabled_by_default: bool = False
    lock_path: str = "/tmp/piper_x_phase_0a_repeatability.lock"
    checklist_path: str = ""
    stop_command: tuple[str, ...] = ("tools/stop_piper_x_moveit_motion.sh",)
    rejected_handeye_path: str = "handeye_failure_diagnostics/rejected_piper_x_d435i_handeye_20260730.yaml"
    thresholds: Phase0AThresholds = field(default_factory=Phase0AThresholds)

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "Phase0AConfig":
        repeatability = data.get("repeatability") or {}
        runtime = data.get("runtime") or {}
        identity = data.get("identity") or {}
        robot_model = data.get("robot_model") or {}
        execution = data.get("execution") or {}
        thresholds = repeatability.get("thresholds") or {}
        threshold_obj = Phase0AThresholds(
            minimum_completed_cycles=int(thresholds.get("minimum_completed_cycles", 3)),
            maximum_joint_endpoint_error_rad=float(thresholds.get("maximum_joint_endpoint_error_rad", 0.02)),
            maximum_joint_repeatability_std_rad=float(thresholds.get("maximum_joint_repeatability_std_rad", 0.005)),
            maximum_joint_repeatability_range_rad=float(thresholds.get("maximum_joint_repeatability_range_rad", 0.01)),
            maximum_settle_time_s=float(thresholds.get("maximum_settle_time_s", 5.0)),
            maximum_stale_feedback_events=int(thresholds.get("maximum_stale_feedback_events", 0)),
            physical_repeatability_target_mm=float(thresholds.get("physical_repeatability_target_mm", 10.0)),
            maximum_pairwise_physical_spread_mm=float(thresholds.get("maximum_pairwise_physical_spread_mm", 20.0)),
            maximum_measurement_uncertainty_mm=float(thresholds.get("maximum_measurement_uncertainty_mm", 5.0)),
        )
        stop_command = execution.get("stop_command", ["tools/stop_piper_x_moveit_motion.sh"])
        return cls(
            diagnostic_id_prefix=str(repeatability.get("diagnostic_id_prefix", "piper_x_phase_0a")),
            output_root=str(repeatability.get("output_root", "piper-on-bunker/data/local/calibration/phase_0a")),
            taught_pose_manifest=str(repeatability.get("taught_pose_manifest", "piper-on-bunker/data/local/moveit_aruco_touch/taught_poses.yaml")),
            moveit_touch_config=str(repeatability.get("moveit_touch_config", "piper-on-bunker/config/piper_x_moveit_touch_aruco_fixed.yaml")),
            measurement_pose_name=str(repeatability.get("measurement_pose_name", "staging")),
            allowed_departure_pose_names=tuple(str(v) for v in repeatability.get("allowed_departure_pose_names", ["repeatability_departure"])),
            default_departure_pose_name=str(repeatability.get("default_departure_pose_name", "repeatability_departure")),
            joint_state_topic=str(runtime.get("joint_state_topic", "/piper_x/joint_states")),
            expected_feedback_source_id=str(runtime.get("expected_feedback_source_id", "piper_x_passive_socketcan_feedback_v1")),
            expected_joint_mapping_version=str(runtime.get("expected_joint_mapping_version", "piper_x_lora_feedback_2a5_2a6_2a7_raw001deg_to_rad_v1")),
            expected_dependency_commit=str(runtime.get("expected_dependency_commit", "521c9c5fdfd9ee63bd96c0f9342fca6b2398092e")),
            can_interface=str(runtime.get("can_interface", "can0")),
            arm_id=str(identity.get("arm_id", "unknown")),
            arm_serial=str(identity.get("arm_serial", "unknown")),
            firmware=str(identity.get("firmware", "unknown")),
            physical_mounting_id=str(identity.get("physical_mounting_id", "unknown")),
            side=str(identity.get("side", "unknown")),
            camera_serial=str(identity.get("camera_serial", "unknown")),
            camera_mount_id=str(identity.get("camera_mount_id", "unknown")),
            tool_id=str(identity.get("tool_id", "unknown")),
            tool_description=str(identity.get("tool_description", "unknown")),
            planning_group=str(robot_model.get("planning_group", "arm")),
            planning_frame=str(robot_model.get("planning_frame", "world")),
            end_effector_link=str(robot_model.get("end_effector_link", "gripper_base")),
            urdf_path=str(robot_model.get("urdf_path", "unknown")),
            urdf_sha256=str(robot_model.get("urdf_sha256", "unknown")),
            command_rate_hz=float(execution.get("command_rate_hz", 50.0)),
            speed_percent=int(execution.get("speed_percent", 30)),
            endpoint_tolerance_rad=float(execution.get("endpoint_tolerance_rad", 0.03)),
            settle_timeout_s=float(execution.get("settle_timeout_s", 3.0)),
            max_joint_state_age_s=float(runtime.get("max_joint_state_age_s", 0.5)),
            max_abs_velocity_rad_s=float(runtime.get("max_abs_velocity_rad_s", 0.01)),
            observation_sample_rate_hz=float(repeatability.get("observation_sample_rate_hz", 20.0)),
            observation_duration_s=float(repeatability.get("observation_duration_s", 10.0)),
            require_base_stopped_ack=bool(repeatability.get("require_base_stopped_ack", True)),
            physical_execution_enabled_by_default=bool(execution.get("physical_execution_enabled_by_default", False)),
            lock_path=str(repeatability.get("lock_path", "/tmp/piper_x_phase_0a_repeatability.lock")),
            checklist_path=str(repeatability.get("checklist_path", "")),
            stop_command=tuple(str(v) for v in stop_command),
            rejected_handeye_path=str(repeatability.get("rejected_handeye_path", "handeye_failure_diagnostics/rejected_piper_x_d435i_handeye_20260730.yaml")),
            thresholds=threshold_obj,
        )

    def identity_dict(self) -> dict[str, str]:
        return {
            "arm_id": self.arm_id,
            "physical_mounting_id": self.physical_mounting_id,
            "camera_serial": self.camera_serial,
            "camera_mount_id": self.camera_mount_id,
            "tool_id": self.tool_id,
        }


def finite_vector(values: list[float] | tuple[float, ...], *, length: int | None = None) -> list[float]:
    out = [float(v) for v in values]
    if length is not None and len(out) != length:
        raise ValueError(f"expected {length} values, got {len(out)}")
    if not all(math.isfinite(v) for v in out):
        raise ValueError("vector contains non-finite values")
    return out
