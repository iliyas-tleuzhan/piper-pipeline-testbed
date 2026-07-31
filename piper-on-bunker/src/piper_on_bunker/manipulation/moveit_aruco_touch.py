from __future__ import annotations

import json
import math
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

import yaml

from piper_on_bunker.mission_logging import MissionLogger
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_DECODER_COMMIT
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_FEEDBACK_SOURCE_ID
from piper_on_bunker.hardware.piper_x_feedback import PIPER_X_JOINT_MAPPING_VERSION


EXPECTED_MARKER_DICTIONARY = "DICT_ARUCO_ORIGINAL"
EXPECTED_MARKER_ID = 6
EXPECTED_MARKER_SIZE_M = 0.100
EXPECTED_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
REJECTED_HANDEYE_PATH = "handeye_failure_diagnostics/rejected_piper_x_d435i_handeye_20260730.yaml"


class TouchFailure(str):
    MARKER_MISSING = "marker_missing"
    WRONG_MARKER = "wrong_marker"
    STALE_CAMERA = "stale_camera"
    STALE_JOINT_STATE = "stale_joint_state"
    MOVEIT_PLANNING_FAILURE = "MoveIt planning failure"
    EXECUTION_FAILURE = "execution failure"
    OPERATOR_ABORT = "operator abort"
    TIMEOUT = "timeout"
    WORKSPACE_VIOLATION = "workspace violation"
    JOINT_LIMIT_VIOLATION = "joint-limit violation"
    MISSING_TAUGHT_POSE = "missing taught pose"
    SCHEMA_MISMATCH = "joint schema mismatch"
    EXECUTION_BLOCKED = "execution blocked"


@dataclass(frozen=True)
class MarkerStatus:
    visible: bool
    marker_id: int | None
    dictionary: str | None
    marker_size_m: float | None
    pose_age_s: float
    image_age_s: float
    stable_duration_s: float = 0.0
    detected_ids: list[int] = field(default_factory=list)


@dataclass(frozen=True)
class JointStateSnapshot:
    joint_names: list[str]
    positions: list[float]
    age_s: float
    velocities: list[float] = field(default_factory=list)
    stamp_s: float | None = None
    source_topic: str | None = None


@dataclass(frozen=True)
class TaughtPose:
    name: str
    joint_names: list[str]
    positions: list[float]
    source: str
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class TrajectoryMetrics:
    joint_names: list[str]
    trajectory_points: int
    first_point_positions: list[float]
    final_point_positions: list[float]
    target_positions: list[float]
    start_positions: list[float]
    target_error_by_joint: dict[str, float]
    continuity_error_by_joint: dict[str, float]
    maximum_target_error_rad: float
    maximum_continuity_error_rad: float
    maximum_joint_delta_rad: float
    maximum_adjacent_joint_delta_rad: float
    maximum_adjacent_joint_delta_joint: str | None
    maximum_adjacent_joint_delta_point_index: int | None
    total_duration_s: float
    minimum_adjacent_timestep_s: float | None
    maximum_adjacent_timestep_s: float | None
    maximum_derived_velocity_rad_s: float
    maximum_derived_velocity_joint: str | None
    maximum_derived_acceleration_rad_s2: float | None
    maximum_derived_acceleration_joint: str | None
    monotonic_timestamps: bool
    nonzero_motion: bool


@dataclass(frozen=True)
class PlanSummary:
    name: str
    success: bool
    trajectory_points: int
    path_fraction: float = 1.0
    estimated_duration_s: float = 0.0
    maximum_joint_delta_rad: float = 0.0
    maximum_adjacent_joint_delta_rad: float = 0.0
    reason: str | None = None
    execution_capable: bool = False
    metrics: dict[str, Any] | None = None
    start_pose_name: str | None = None
    target_pose_name: str | None = None
    motion_profile_name: str | None = None
    velocity_scaling: float | None = None
    acceleration_scaling: float | None = None
    duration_limit_s: float | None = None
    duration_gate_passed: bool | None = None
    per_joint_total_displacement_rad: dict[str, float] | None = None
    largest_displacement_joint: str | None = None
    largest_displacement_rad: float | None = None
    large_displacement_review_required: bool = False
    effective_joint_velocity_limits_rad_s: dict[str, float | None] | None = None


@dataclass(frozen=True)
class TouchConfig:
    profile_id: str
    task_id: str
    marker_dictionary: str
    marker_id: int
    marker_size_m: float
    camera_topic: str
    marker_pose_topic: str
    planning_group: str
    end_effector_link: str
    staging_pose_name: str
    home_pose_name: str
    pre_touch_pose_name: str
    touch_pose_name: str
    retract_pose_name: str
    optional_safe_recovery_pose_name: str | None
    taught_pose_manifest: str
    motion_strategy: str
    hold_duration_s: float
    velocity_scaling: float
    acceleration_scaling: float
    motion_profiles: dict[str, dict[str, float]]
    segment_duration_limits_s: dict[str, float]
    large_pose_delta_review_threshold_rad: float
    planning_timeout_s: float
    execution_timeout_s: float
    max_joint_jump_rad: float
    max_adjacent_joint_delta_rad: float
    max_segment_duration_s: float
    max_mission_duration_s: float
    home_transit_diagnostic_max_mission_duration_s: float
    min_effective_joint_velocity_rad_s: float
    max_target_error_rad: float
    continuity_tolerance_rad: float
    future_cartesian_mode: dict[str, Any]
    max_marker_pose_age_s: float
    max_image_age_s: float
    max_joint_state_age_s: float
    marker_stability_required_s: float
    taught_pose_limit_tolerance_rad: float
    joint_limits: dict[str, list[float]]
    workspace_limits: dict[str, list[float]]
    physical_execution_enabled_by_default: bool
    robot_model: str
    robot_urdf_candidate_path: str
    robot_urdf_candidate_sha256: str
    piper_x_model_verified: bool
    authoritative_joint_state_topic: str
    required_feedback_source_id: str
    required_joint_mapping_version: str
    required_dependency_commit: str

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "TouchConfig":
        marker = data["marker"]
        camera = data["camera"]
        moveit = data["moveit"]
        poses = data["taught_poses"]
        motion = data["motion"]
        safety = data["safety"]
        robot = data["robot_model"]
        return cls(
            profile_id=str(data["profile_id"]),
            task_id=str(data["task_id"]),
            marker_dictionary=str(marker["dictionary"]),
            marker_id=int(marker["id"]),
            marker_size_m=float(marker["size_m"]),
            camera_topic=str(camera["image_topic"]),
            marker_pose_topic=str(camera["marker_pose_topic"]),
            planning_group=str(moveit["planning_group"]),
            end_effector_link=str(moveit["end_effector_link"]),
            staging_pose_name=str(poses.get("staging", "staging")),
            home_pose_name=str(poses["home"]),
            pre_touch_pose_name=str(poses["pre_touch"]),
            touch_pose_name=str(poses["touch"]),
            retract_pose_name=str(poses["retract"]),
            optional_safe_recovery_pose_name=poses.get("safe_recovery"),
            taught_pose_manifest=str(poses["manifest"]),
            motion_strategy=str(motion["motion_strategy"]),
            hold_duration_s=float(motion["hold_duration_s"]),
            velocity_scaling=float(motion.get("velocity_scaling", motion.get("motion_profiles", {}).get("approach", {}).get("velocity_scaling", 0.05))),
            acceleration_scaling=float(motion.get("acceleration_scaling", motion.get("motion_profiles", {}).get("approach", {}).get("acceleration_scaling", 0.05))),
            motion_profiles={
                str(name): {
                    "velocity_scaling": float(payload["velocity_scaling"]),
                    "acceleration_scaling": float(payload["acceleration_scaling"]),
                }
                for name, payload in (motion.get("motion_profiles") or {
                    "approach": {"velocity_scaling": motion.get("velocity_scaling", 0.05), "acceleration_scaling": motion.get("acceleration_scaling", 0.05)}
                }).items()
            },
            segment_duration_limits_s={str(k): float(v) for k, v in (motion.get("segment_duration_limits_s") or {}).items()},
            large_pose_delta_review_threshold_rad=float(motion.get("large_pose_delta_review_threshold_rad", 1.0)),
            planning_timeout_s=float(motion["planning_timeout_s"]),
            execution_timeout_s=float(motion["execution_timeout_s"]),
            max_joint_jump_rad=float(motion["max_joint_jump_rad"]),
            max_adjacent_joint_delta_rad=float(motion.get("max_adjacent_joint_delta_rad", motion["max_joint_jump_rad"])),
            max_segment_duration_s=float(motion.get("max_segment_duration_s", 30.0)),
            max_mission_duration_s=float(motion.get("max_mission_duration_s", 120.0)),
            home_transit_diagnostic_max_mission_duration_s=float(motion.get("home_transit_diagnostic_max_mission_duration_s", 240.0)),
            min_effective_joint_velocity_rad_s=float(motion.get("min_effective_joint_velocity_rad_s", 0.001)),
            max_target_error_rad=float(motion.get("max_target_error_rad", 0.02)),
            continuity_tolerance_rad=float(motion.get("continuity_tolerance_rad", 1e-3)),
            future_cartesian_mode=dict(motion.get("future_cartesian_mode") or {}),
            max_marker_pose_age_s=float(safety["max_marker_pose_age_s"]),
            max_image_age_s=float(safety["max_image_age_s"]),
            max_joint_state_age_s=float(safety["max_joint_state_age_s"]),
            marker_stability_required_s=float(safety["marker_stability_required_s"]),
            taught_pose_limit_tolerance_rad=float(safety.get("taught_pose_limit_tolerance_rad", 0.0)),
            joint_limits={str(k): [float(v[0]), float(v[1])] for k, v in safety["joint_limits"].items()},
            workspace_limits={str(k): [float(v[0]), float(v[1])] for k, v in safety["workspace_limits"].items()},
            physical_execution_enabled_by_default=bool(safety["physical_execution_enabled_by_default"]),
            robot_model=str(robot["model"]),
            robot_urdf_candidate_path=str(robot["urdf_candidate_path"]),
            robot_urdf_candidate_sha256=str(robot["urdf_candidate_sha256"]),
            piper_x_model_verified=bool(robot.get("piper_x_model_verified", False)),
            authoritative_joint_state_topic=str(
                data.get("feedback", {}).get("authoritative_joint_state_topic", "/piper_x/joint_states")
            ),
            required_feedback_source_id=str(data.get("feedback", {}).get("required_feedback_source_id", PIPER_X_FEEDBACK_SOURCE_ID)),
            required_joint_mapping_version=str(data.get("feedback", {}).get("required_joint_mapping_version", PIPER_X_JOINT_MAPPING_VERSION)),
            required_dependency_commit=str(data.get("feedback", {}).get("required_dependency_commit", PIPER_X_DECODER_COMMIT)),
        )


def load_touch_config(path: str | Path) -> TouchConfig:
    with Path(path).open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    return TouchConfig.from_mapping(data)


def load_taught_poses(path: str | Path) -> dict[str, TaughtPose]:
    pose_path = Path(path)
    if not pose_path.exists():
        return {}
    with pose_path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    poses = {}
    for name, payload in (data.get("poses") or {}).items():
        poses[str(name)] = TaughtPose(
            name=str(name),
            joint_names=[str(v) for v in payload["joint_names"]],
            positions=[float(v) for v in payload["positions"]],
            source=str(payload.get("source", pose_path)),
            metadata=dict(payload.get("metadata") or {}),
        )
    return poses


def save_taught_pose_manifest(path: str | Path, pose: TaughtPose, metadata: dict[str, Any]) -> None:
    pose_path = Path(path)
    existing = {}
    if pose_path.exists():
        with pose_path.open("r", encoding="utf-8") as fh:
            existing = yaml.safe_load(fh) or {}
    existing.setdefault("schema_version", "piper_x_moveit_taught_poses.v1")
    existing.setdefault("metadata", {}).update(metadata)
    existing.setdefault("poses", {})
    existing["poses"][pose.name] = {
        "joint_names": list(pose.joint_names),
        "positions": [float(v) for v in pose.positions],
        "source": pose.source,
        "metadata": dict(pose.metadata),
        "saved_unix_s": time.time(),
    }
    pose_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = pose_path.with_suffix(pose_path.suffix + ".tmp")
    with tmp_path.open("w", encoding="utf-8") as fh:
        yaml.safe_dump(existing, fh, sort_keys=True)
    tmp_path.replace(pose_path)


def validate_joint_schema(joint_names: list[str]) -> None:
    if list(joint_names) != EXPECTED_JOINT_NAMES:
        raise ValueError(f"expected joint schema {EXPECTED_JOINT_NAMES}, got {joint_names}")


def extract_named_joint_state(
    *,
    names: list[str],
    positions: list[float],
    velocities: list[float] | None,
    stamp_s: float,
    now_s: float,
    expected_names: list[str] | None = None,
    max_age_s: float = 0.5,
    max_abs_velocity_rad_s: float = 0.01,
    source_topic: str = "/joint_states_single",
) -> JointStateSnapshot:
    expected = list(expected_names or EXPECTED_JOINT_NAMES)
    if len(names) != len(set(names)):
        raise ValueError("joint state contains duplicate joint names")
    by_name = {str(name): i for i, name in enumerate(names)}
    missing = [name for name in expected if name not in by_name]
    if missing:
        raise ValueError(f"joint state missing expected joints: {missing}")
    age_s = float(now_s) - float(stamp_s)
    if age_s < -0.05 or age_s > max_age_s:
        raise ValueError(f"stale joint state: age {age_s:.3f}s exceeds {max_age_s:.3f}s")
    mapped_positions = [float(positions[by_name[name]]) for name in expected]
    if not all(math.isfinite(value) for value in mapped_positions):
        raise ValueError("joint state contains non-finite position")
    mapped_velocities: list[float] = []
    if velocities and len(velocities) >= len(names):
        mapped_velocities = [float(velocities[by_name[name]]) for name in expected]
        if not all(math.isfinite(value) for value in mapped_velocities):
            raise ValueError("joint state contains non-finite velocity")
        moving = [name for name, value in zip(expected, mapped_velocities) if abs(value) > max_abs_velocity_rad_s]
        if moving:
            raise ValueError(f"robot is not stopped; moving joints above {max_abs_velocity_rad_s:.4f} rad/s: {moving}")
    return JointStateSnapshot(expected, mapped_positions, age_s, mapped_velocities, float(stamp_s), source_topic)


def validate_joint_values(joint_names: list[str], positions: list[float], limits: dict[str, list[float]], *, tolerance_rad: float = 0.0) -> None:
    validate_joint_schema(joint_names)
    if len(positions) != 6:
        raise ValueError("expected exactly six arm joint positions")
    tolerance = max(0.0, float(tolerance_rad))
    for name, value in zip(joint_names, positions):
        if not math.isfinite(float(value)):
            raise ValueError(f"non-finite joint value for {name}")
        lower, upper = limits[name]
        if float(value) < lower - tolerance or float(value) > upper + tolerance:
            if tolerance:
                raise ValueError(f"{name}={value:.6f} outside limits [{lower:.6f}, {upper:.6f}] with tolerance {tolerance:.6f}")
            raise ValueError(f"{name}={value:.6f} outside limits [{lower:.6f}, {upper:.6f}]")


def build_inverse_press(press_vector: list[float]) -> list[float]:
    return [-float(v) for v in press_vector]


class MoveItTouchBackend(Protocol):
    def describe(self) -> dict[str, Any]:
        ...

    def read_joint_state(self) -> JointStateSnapshot:
        ...

    def read_marker_status(self) -> MarkerStatus:
        ...

    def plan_joint_pose(
        self,
        name: str,
        pose: TaughtPose,
        config: TouchConfig,
        *,
        start_state: JointStateSnapshot,
        motion_profile: dict[str, float],
    ) -> PlanSummary:
        ...

    def publish_plans_to_rviz(self) -> bool:
        ...

    def execute_plan(self, plan: PlanSummary, timeout_s: float) -> bool:
        ...

    def stop(self) -> None:
        ...

    def wait_until_stopped(self) -> bool:
        ...

    def check_ready(self) -> dict[str, Any]:
        ...


class MockMoveItTouchBackend:
    def __init__(
        self,
        *,
        marker: MarkerStatus | None = None,
        joint_state: JointStateSnapshot | None = None,
        max_joint_delta_rad: float = 0.03,
        execute_ok: bool = True,
    ) -> None:
        self.marker = marker or MarkerStatus(True, 6, EXPECTED_MARKER_DICTIONARY, EXPECTED_MARKER_SIZE_M, 0.0, 0.0, 1.0, [6])
        self.joint_state = joint_state or JointStateSnapshot(list(EXPECTED_JOINT_NAMES), [0.0] * 6, 0.0)
        self.max_joint_delta_rad = float(max_joint_delta_rad)
        self.execute_ok = bool(execute_ok)
        self.executed: list[str] = []
        self.stopped = False

    def describe(self) -> dict[str, Any]:
        return {
            "backend": "mock_moveit",
            "robot_model": "agilex_piper_x_unverified",
            "planning_group": "arm",
            "end_effector_link": "gripper_base",
            "robot_urdf": "mock",
        }

    def read_joint_state(self) -> JointStateSnapshot:
        return self.joint_state

    def read_marker_status(self) -> MarkerStatus:
        return self.marker

    def check_ready(self) -> dict[str, Any]:
        return {
            "ready": True,
            "backend": "mock_moveit",
            "planning_only_supported": True,
            "effective_joint_limits": {
                name: {
                    "max_velocity_rad_s": 0.5,
                    "velocity_scaling": 0.05,
                    "effective_max_velocity_rad_s": 0.025,
                    "has_acceleration_limits": False,
                    "max_acceleration_rad_s2": None,
                    "effective_max_acceleration_rad_s2": None,
                }
                for name in EXPECTED_JOINT_NAMES
            },
        }

    def plan_joint_pose(
        self,
        name: str,
        pose: TaughtPose,
        config: TouchConfig,
        *,
        start_state: JointStateSnapshot,
        motion_profile: dict[str, float],
    ) -> PlanSummary:
        metrics = linear_trajectory_metrics(
            joint_names=list(pose.joint_names),
            start_positions=list(start_state.positions),
            target_positions=list(pose.positions),
            duration_s=2.0,
        )
        maximum_delta = max(metrics.maximum_joint_delta_rad, self.max_joint_delta_rad)
        if maximum_delta != metrics.maximum_joint_delta_rad:
            metrics = TrajectoryMetrics(
                **{**_metrics_to_dict(metrics), "maximum_joint_delta_rad": maximum_delta}
            )
        self.joint_state = JointStateSnapshot(list(pose.joint_names), list(pose.positions), 0.0)
        return PlanSummary(
            name,
            True,
            trajectory_points=metrics.trajectory_points,
            estimated_duration_s=metrics.total_duration_s,
            maximum_joint_delta_rad=metrics.maximum_joint_delta_rad,
            maximum_adjacent_joint_delta_rad=metrics.maximum_adjacent_joint_delta_rad,
            execution_capable=True,
            metrics=_metrics_to_dict(metrics),
            velocity_scaling=float(motion_profile["velocity_scaling"]),
            acceleration_scaling=float(motion_profile["acceleration_scaling"]),
        )

    def execute_plan(self, plan: PlanSummary, timeout_s: float) -> bool:
        self.executed.append(plan.name)
        return self.execute_ok

    def stop(self) -> None:
        self.stopped = True

    def wait_until_stopped(self) -> bool:
        return True

    def publish_plans_to_rviz(self) -> bool:
        return True


class RosMoveItJointSequenceBackend:
    """Lazy ROS MoveIt backend for the taught joint-sequence MVP.

    Planning uses moveit_commander when available. The legacy JointMoveitCtrl
    services are inspected but are not used for planning-only because the
    observed contract does not prove a no-execution planning mode.
    """

    JOINT_TOPIC = "/joint_states_single"
    SERVICE_NAMES = [
        "/joint_moveit_ctrl_arm",
        "/joint_moveit_ctrl_endpose",
        "/joint_moveit_ctrl_gripper",
        "/joint_moveit_ctrl_piper",
    ]

    def __init__(self, config: TouchConfig, *, joint_topic: str = "/joint_states_single") -> None:
        self.config = config
        self.joint_topic = joint_topic
        self._last_plan: Any = None
        self._planned_trajectories: list[Any] = []
        self._group: Any = None
        self._moveit_commander: Any = None
        try:
            import rospy
        except Exception as exc:
            raise RuntimeError("rospy is unavailable; live MoveIt backend cannot start") from exc
        self.rospy = rospy
        if not rospy.get_node_uri():
            rospy.init_node("piper_x_moveit_aruco_touch", anonymous=True, disable_signals=True)

    def describe(self) -> dict[str, Any]:
        ready = self.check_ready()
        return {
            "backend": "ros_moveit_joint_sequence",
            "planning_group": self.config.planning_group,
            "end_effector_link": self.config.end_effector_link,
            **ready,
        }

    def check_ready(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "ready": False,
            "moveit_commander_available": False,
            "planning_only_supported": False,
            "services": {},
            "move_group_available": False,
            "joint_topic": self.joint_topic,
            "authoritative_joint_state_topic": self.config.authoritative_joint_state_topic,
        }
        try:
            import rosservice
            for name in self.SERVICE_NAMES:
                try:
                    out["services"][name] = rosservice.get_service_type(name)
                except Exception as exc:
                    out["services"][name] = f"unavailable: {exc!r}"
        except Exception as exc:
            out["service_inspection_error"] = repr(exc)
        try:
            group = self._get_group()
            out["moveit_commander_available"] = True
            out["move_group_available"] = True
            out["planning_frame"] = group.get_planning_frame()
            out["active_joints"] = list(group.get_active_joints())
            out["current_end_effector_link"] = getattr(group, "get_end_effector_link", lambda: None)()
            out["effective_joint_limits"] = self._effective_joint_limits()
            out["planning_only_supported"] = True
            out["ready"] = True
        except Exception as exc:
            out["moveit_commander_error"] = repr(exc)
        return out

    def _effective_joint_limits(self) -> dict[str, dict[str, Any]]:
        try:
            limits = self.rospy.get_param("/robot_description_planning/joint_limits", {})
        except Exception:
            limits = {}
        out: dict[str, dict[str, Any]] = {}
        for name in EXPECTED_JOINT_NAMES:
            payload = limits.get(name, {}) if isinstance(limits, dict) else {}
            has_velocity = bool(payload.get("has_velocity_limits", False))
            max_velocity = float(payload.get("max_velocity", 0.0)) if has_velocity else None
            has_acceleration = bool(payload.get("has_acceleration_limits", False))
            max_acceleration = float(payload.get("max_acceleration", 0.0)) if has_acceleration else None
            out[name] = {
                "has_velocity_limits": has_velocity,
                "max_velocity_rad_s": max_velocity,
                "velocity_scaling": self.config.velocity_scaling,
                "effective_max_velocity_rad_s": None if max_velocity is None else max_velocity * self.config.velocity_scaling,
                "has_acceleration_limits": has_acceleration,
                "max_acceleration_rad_s2": max_acceleration,
                "acceleration_scaling": self.config.acceleration_scaling,
                "effective_max_acceleration_rad_s2": None if max_acceleration is None else max_acceleration * self.config.acceleration_scaling,
            }
        return out

    def _get_group(self, *, motion_profile: dict[str, float] | None = None) -> Any:
        if self._group is None:
            import moveit_commander

            moveit_commander.roscpp_initialize([])
            self._moveit_commander = moveit_commander
            self._group = moveit_commander.MoveGroupCommander(self.config.planning_group)
        group = self._group
        profile = motion_profile or {"velocity_scaling": self.config.velocity_scaling, "acceleration_scaling": self.config.acceleration_scaling}
        group.set_max_velocity_scaling_factor(float(profile["velocity_scaling"]))
        group.set_max_acceleration_scaling_factor(float(profile["acceleration_scaling"]))
        group.set_planning_time(self.config.planning_timeout_s)
        return group

    def _robot_state_from_snapshot(self, snapshot: JointStateSnapshot) -> Any:
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        state = RobotState()
        state.joint_state = JointState()
        state.joint_state.header.stamp = self.rospy.Time.now()
        state.joint_state.name = list(snapshot.joint_names)
        state.joint_state.position = [float(v) for v in snapshot.positions]
        return state

    def read_joint_state(self) -> JointStateSnapshot:
        from sensor_msgs.msg import JointState

        msg = self.rospy.wait_for_message(self.joint_topic, JointState, timeout=self.config.max_joint_state_age_s)
        return extract_named_joint_state(
            names=list(msg.name),
            positions=list(msg.position),
            velocities=list(msg.velocity),
            stamp_s=float(msg.header.stamp.to_sec()),
            now_s=float(self.rospy.Time.now().to_sec()),
            max_age_s=self.config.max_joint_state_age_s,
            source_topic=self.joint_topic,
        )

    def read_marker_status(self) -> MarkerStatus:
        from geometry_msgs.msg import PoseStamped

        dictionary = self.rospy.get_param("/aruco_simple/dictionary", None)
        marker_id = self.rospy.get_param("/aruco_simple/marker_id", None)
        marker_size = self.rospy.get_param("/aruco_simple/marker_size", None)
        try:
            msg = self.rospy.wait_for_message(self.config.marker_pose_topic, PoseStamped, timeout=self.config.max_marker_pose_age_s)
            pose_age = float(self.rospy.Time.now().to_sec()) - float(msg.header.stamp.to_sec())
            visible = pose_age <= self.config.max_marker_pose_age_s
            return MarkerStatus(
                visible=visible,
                marker_id=int(marker_id) if marker_id is not None else None,
                dictionary=str(dictionary) if dictionary is not None else None,
                marker_size_m=float(marker_size) if marker_size is not None else None,
                pose_age_s=pose_age,
                image_age_s=pose_age,
                stable_duration_s=self.config.marker_stability_required_s if visible else 0.0,
                detected_ids=[int(marker_id)] if visible and marker_id is not None else [],
            )
        except Exception:
            return MarkerStatus(False, int(marker_id) if marker_id is not None else None, str(dictionary) if dictionary is not None else None, float(marker_size) if marker_size is not None else None, 999.0, 999.0, 0.0, [])

    def plan_joint_pose(
        self,
        name: str,
        pose: TaughtPose,
        config: TouchConfig,
        *,
        start_state: JointStateSnapshot,
        motion_profile: dict[str, float],
    ) -> PlanSummary:
        try:
            if max(abs(float(goal) - float(start)) for goal, start in zip(pose.positions, start_state.positions)) <= 1e-4:
                metrics = _trajectory_metrics_from_arrays(
                    joint_names=list(pose.joint_names),
                    start_positions=list(start_state.positions),
                    target_positions=list(pose.positions),
                    point_positions=[list(start_state.positions)],
                    point_times_s=[0.0],
                )
                return PlanSummary(
                    name,
                    True,
                    metrics.trajectory_points,
                    estimated_duration_s=0.0,
                    maximum_joint_delta_rad=metrics.maximum_joint_delta_rad,
                    maximum_adjacent_joint_delta_rad=metrics.maximum_adjacent_joint_delta_rad,
                    execution_capable=True,
                    metrics=_metrics_to_dict(metrics),
                    reason="already at taught target",
                    velocity_scaling=float(motion_profile["velocity_scaling"]),
                    acceleration_scaling=float(motion_profile["acceleration_scaling"]),
                )
            group = self._get_group(motion_profile=motion_profile)
            if list(group.get_active_joints()) != list(pose.joint_names):
                return PlanSummary(name, False, 0, reason=f"MoveIt active joints {group.get_active_joints()} do not match taught pose {pose.joint_names}")
            group.set_start_state(self._robot_state_from_snapshot(start_state))
            group.set_joint_value_target(dict(zip(pose.joint_names, pose.positions)))
            plan_result = group.plan()
            trajectory = _normalize_moveit_plan(plan_result)
            if trajectory is None:
                return PlanSummary(name, False, 0, reason="MoveIt returned no plan")
            metrics = trajectory_metrics_from_moveit(
                trajectory=trajectory,
                expected_joint_names=pose.joint_names,
                start_positions=start_state.positions,
                target_positions=pose.positions,
            )
            self._last_plan = trajectory
            self._planned_trajectories.append(trajectory)
            return PlanSummary(
                name,
                True,
                metrics.trajectory_points,
                estimated_duration_s=metrics.total_duration_s,
                maximum_joint_delta_rad=metrics.maximum_joint_delta_rad,
                maximum_adjacent_joint_delta_rad=metrics.maximum_adjacent_joint_delta_rad,
                execution_capable=True,
                metrics=_metrics_to_dict(metrics),
                velocity_scaling=float(motion_profile["velocity_scaling"]),
                acceleration_scaling=float(motion_profile["acceleration_scaling"]),
            )
        except Exception as exc:
            return PlanSummary(name, False, 0, reason=f"MoveIt planning failed: {exc!r}")

    def execute_plan(self, plan: PlanSummary, timeout_s: float) -> bool:
        if plan.metrics and not bool(plan.metrics.get("nonzero_motion", True)):
            return True
        if self._last_plan is None:
            return False
        try:
            group = self._get_group()
            return bool(group.execute(self._last_plan, wait=True))
        except Exception:
            return False

    def stop(self) -> None:
        try:
            self._get_group().stop()
        except Exception:
            pass

    def wait_until_stopped(self) -> bool:
        state = self.read_joint_state()
        return not state.velocities or max(abs(v) for v in state.velocities) <= 0.01

    def publish_plans_to_rviz(self) -> bool:
        if not self._planned_trajectories:
            return False
        try:
            from moveit_msgs.msg import DisplayTrajectory

            display = DisplayTrajectory()
            try:
                display.model_id = str(self.rospy.get_param("/robot_description_name", "piper_x"))
            except Exception:
                display.model_id = "piper_x"
            display.trajectory_start = self._get_group().get_current_state()
            display.trajectory = list(self._planned_trajectories)
            pub = self.rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=1, latch=True)
            deadline = time.time() + 1.0
            while pub.get_num_connections() == 0 and time.time() < deadline and not self.rospy.is_shutdown():
                self.rospy.sleep(0.05)
            pub.publish(display)
            self.rospy.sleep(0.2)
            return True
        except Exception:
            return False


def _normalize_moveit_plan(plan_result: Any) -> Any:
    if isinstance(plan_result, tuple):
        if len(plan_result) >= 2 and bool(plan_result[0]):
            return plan_result[1]
        return None
    return plan_result


def _duration_to_seconds(value: Any) -> float:
    return float(value.to_sec()) if hasattr(value, "to_sec") else float(value)


def _metrics_to_dict(metrics: TrajectoryMetrics) -> dict[str, Any]:
    return {
        "joint_names": metrics.joint_names,
        "trajectory_points": metrics.trajectory_points,
        "first_point_positions": metrics.first_point_positions,
        "final_point_positions": metrics.final_point_positions,
        "target_positions": metrics.target_positions,
        "start_positions": metrics.start_positions,
        "target_error_by_joint": metrics.target_error_by_joint,
        "continuity_error_by_joint": metrics.continuity_error_by_joint,
        "maximum_target_error_rad": metrics.maximum_target_error_rad,
        "maximum_continuity_error_rad": metrics.maximum_continuity_error_rad,
        "maximum_joint_delta_rad": metrics.maximum_joint_delta_rad,
        "maximum_adjacent_joint_delta_rad": metrics.maximum_adjacent_joint_delta_rad,
        "maximum_adjacent_joint_delta_joint": metrics.maximum_adjacent_joint_delta_joint,
        "maximum_adjacent_joint_delta_point_index": metrics.maximum_adjacent_joint_delta_point_index,
        "total_duration_s": metrics.total_duration_s,
        "minimum_adjacent_timestep_s": metrics.minimum_adjacent_timestep_s,
        "maximum_adjacent_timestep_s": metrics.maximum_adjacent_timestep_s,
        "maximum_derived_velocity_rad_s": metrics.maximum_derived_velocity_rad_s,
        "maximum_derived_velocity_joint": metrics.maximum_derived_velocity_joint,
        "maximum_derived_acceleration_rad_s2": metrics.maximum_derived_acceleration_rad_s2,
        "maximum_derived_acceleration_joint": metrics.maximum_derived_acceleration_joint,
        "monotonic_timestamps": metrics.monotonic_timestamps,
        "nonzero_motion": metrics.nonzero_motion,
    }


def _trajectory_metrics_from_arrays(
    *,
    joint_names: list[str],
    start_positions: list[float],
    target_positions: list[float],
    point_positions: list[list[float]],
    point_times_s: list[float],
) -> TrajectoryMetrics:
    validate_joint_schema(joint_names)
    if len(start_positions) != 6 or len(target_positions) != 6:
        raise ValueError("expected six start and target positions")
    if len(point_positions) != len(point_times_s):
        raise ValueError("trajectory position/time length mismatch")
    if not point_positions:
        raise ValueError("trajectory has no points")
    for index, positions in enumerate(point_positions):
        if len(positions) != 6:
            raise ValueError(f"trajectory point {index} has {len(positions)} positions, expected 6")
        if not all(math.isfinite(float(value)) for value in positions):
            raise ValueError(f"trajectory point {index} contains non-finite positions")
    if not all(math.isfinite(float(value)) for value in point_times_s):
        raise ValueError("trajectory contains non-finite timestamps")

    first_positions = [float(v) for v in point_positions[0]]
    final_positions = [float(v) for v in point_positions[-1]]
    start = [float(v) for v in start_positions]
    target = [float(v) for v in target_positions]
    target_error_by_joint = {name: abs(final - goal) for name, final, goal in zip(joint_names, final_positions, target)}
    continuity_error_by_joint = {name: abs(first - initial) for name, first, initial in zip(joint_names, first_positions, start)}
    maximum_target_error = max(target_error_by_joint.values(), default=0.0)
    maximum_continuity_error = max(continuity_error_by_joint.values(), default=0.0)
    maximum_joint_delta = max((abs(value - start_value) for positions in point_positions for value, start_value in zip(positions, start)), default=0.0)

    maximum_adjacent_delta = 0.0
    maximum_adjacent_joint = None
    maximum_adjacent_index = None
    timestep_values: list[float] = []
    velocity_samples: list[list[float]] = []
    maximum_velocity = 0.0
    maximum_velocity_joint = None
    monotonic = True
    for point_index in range(1, len(point_positions)):
        dt = float(point_times_s[point_index]) - float(point_times_s[point_index - 1])
        if dt <= 0.0:
            monotonic = False
        else:
            timestep_values.append(dt)
        velocities: list[float] = []
        for joint_index, name in enumerate(joint_names):
            delta = abs(float(point_positions[point_index][joint_index]) - float(point_positions[point_index - 1][joint_index]))
            if delta > maximum_adjacent_delta:
                maximum_adjacent_delta = delta
                maximum_adjacent_joint = name
                maximum_adjacent_index = point_index
            velocity = delta / dt if dt > 0.0 else math.inf
            velocities.append(velocity)
            if velocity > maximum_velocity:
                maximum_velocity = velocity
                maximum_velocity_joint = name
        velocity_samples.append(velocities)

    maximum_acceleration: float | None = None
    maximum_acceleration_joint = None
    for index in range(1, len(velocity_samples)):
        dt = point_times_s[index + 1] - point_times_s[index] if index + 1 < len(point_times_s) else 0.0
        if dt <= 0.0:
            maximum_acceleration = math.inf
            maximum_acceleration_joint = joint_names[0]
            break
        for joint_index, name in enumerate(joint_names):
            accel = abs(velocity_samples[index][joint_index] - velocity_samples[index - 1][joint_index]) / dt
            if maximum_acceleration is None or accel > maximum_acceleration:
                maximum_acceleration = accel
                maximum_acceleration_joint = name

    nonzero_motion = max((abs(goal - initial) for goal, initial in zip(target, start)), default=0.0) > 1e-6
    return TrajectoryMetrics(
        joint_names=list(joint_names),
        trajectory_points=len(point_positions),
        first_point_positions=first_positions,
        final_point_positions=final_positions,
        target_positions=target,
        start_positions=start,
        target_error_by_joint=target_error_by_joint,
        continuity_error_by_joint=continuity_error_by_joint,
        maximum_target_error_rad=maximum_target_error,
        maximum_continuity_error_rad=maximum_continuity_error,
        maximum_joint_delta_rad=maximum_joint_delta,
        maximum_adjacent_joint_delta_rad=maximum_adjacent_delta,
        maximum_adjacent_joint_delta_joint=maximum_adjacent_joint,
        maximum_adjacent_joint_delta_point_index=maximum_adjacent_index,
        total_duration_s=float(point_times_s[-1]) if point_times_s else 0.0,
        minimum_adjacent_timestep_s=min(timestep_values) if timestep_values else None,
        maximum_adjacent_timestep_s=max(timestep_values) if timestep_values else None,
        maximum_derived_velocity_rad_s=maximum_velocity,
        maximum_derived_velocity_joint=maximum_velocity_joint,
        maximum_derived_acceleration_rad_s2=maximum_acceleration,
        maximum_derived_acceleration_joint=maximum_acceleration_joint,
        monotonic_timestamps=monotonic,
        nonzero_motion=nonzero_motion,
    )


def trajectory_metrics_from_moveit(
    *,
    trajectory: Any,
    expected_joint_names: list[str],
    start_positions: list[float],
    target_positions: list[float],
) -> TrajectoryMetrics:
    joint_trajectory = getattr(trajectory, "joint_trajectory", None)
    if joint_trajectory is None:
        raise ValueError("MoveIt plan has no joint_trajectory")
    joint_names = [str(name) for name in getattr(joint_trajectory, "joint_names", [])]
    if joint_names != list(expected_joint_names):
        raise ValueError(f"trajectory joint names {joint_names} do not match expected {expected_joint_names}")
    points = list(getattr(joint_trajectory, "points", []) or [])
    return _trajectory_metrics_from_arrays(
        joint_names=joint_names,
        start_positions=start_positions,
        target_positions=target_positions,
        point_positions=[[float(v) for v in getattr(point, "positions", [])] for point in points],
        point_times_s=[_duration_to_seconds(getattr(point, "time_from_start", 0.0)) for point in points],
    )


def linear_trajectory_metrics(
    *,
    joint_names: list[str],
    start_positions: list[float],
    target_positions: list[float],
    duration_s: float,
) -> TrajectoryMetrics:
    return _trajectory_metrics_from_arrays(
        joint_names=joint_names,
        start_positions=start_positions,
        target_positions=target_positions,
        point_positions=[list(start_positions), list(target_positions)],
        point_times_s=[0.0, float(duration_s)],
    )


@dataclass
class TouchMissionResult:
    success: bool
    state: str
    failure_reason: str | None
    planning_only: bool
    physical_motion_performed: bool
    outputs: dict[str, Any]


class MoveItArucoTouchController:
    STATES = ["IDLE", "CHECK_MARKER", "MOVE_STAGING", "MOVE_PRE_TOUCH", "MOVE_TOUCH", "HOLD", "MOVE_RETRACT", "MOVE_STAGING", "COMPLETE"]

    def __init__(
        self,
        config: TouchConfig,
        backend: MoveItTouchBackend,
        *,
        taught_poses: dict[str, TaughtPose] | None = None,
        logger: MissionLogger | None = None,
    ) -> None:
        self.config = config
        self.backend = backend
        self.taught_poses = taught_poses if taught_poses is not None else load_taught_poses(config.taught_pose_manifest)
        self.logger = logger or MissionLogger(enabled=False)
        self.mission_id = str(uuid.uuid4())
        self.transitions: list[str] = []
        self._partial_plans: list[PlanSummary] = []
        self._last_rviz_published = False
        self._backend_ready: dict[str, Any] = {}

    def run(
        self,
        *,
        planning_only: bool = True,
        execute: bool = False,
        confirm: str | None = None,
        sequence: str = "fixed_touch",
        publish_plans_to_rviz: bool = False,
    ) -> TouchMissionResult:
        self.logger.start_mission(self.mission_id)
        physical = bool(execute)
        if sequence not in {"staging_test", "fixed_touch", "home_transit_diagnostic"}:
            return self._fail("IDLE", TouchFailure.EXECUTION_BLOCKED, f"unknown sequence {sequence}", planning_only, False)
        if physical:
            if planning_only:
                return self._fail("IDLE", TouchFailure.EXECUTION_BLOCKED, "execute cannot be combined with planning_only", planning_only, False)
            expected_confirm = "STAGING_TEST" if sequence == "staging_test" else "FIXED_ARUCO_TOUCH"
            if confirm != expected_confirm:
                return self._fail("IDLE", TouchFailure.EXECUTION_BLOCKED, f"physical execution requires --confirm {expected_confirm}", planning_only, False)
            if sequence == "home_transit_diagnostic":
                return self._fail("IDLE", TouchFailure.EXECUTION_BLOCKED, "home_transit_diagnostic is planning-only until separately verified", planning_only, False)
            if not self.config.physical_execution_enabled_by_default:
                return self._fail("IDLE", TouchFailure.EXECUTION_BLOCKED, "physical execution disabled in committed config", planning_only, False)

        try:
            self._preflight(require_taught_poses=True, sequence=sequence)
            marker = self._check_marker()
            plans: list[PlanSummary] = []
            planned_state = self._read_and_validate_current_state()
            planned_state_label = "current_state"
            steps = self._sequence_steps(sequence)

            for step in steps:
                state = step["state"]
                plan_name = step["plan_name"]
                pose = self._pose(step["target_pose_name"])
                motion_profile_name = step["motion_profile_name"]
                motion_profile = self._motion_profile(motion_profile_name)
                duration_limit_s = self._duration_limit(plan_name)
                if state == "MOVE_TOUCH":
                    marker_before_touch = self._check_marker()
                    if not marker_before_touch.visible:
                        return self._fail("MOVE_TOUCH", TouchFailure.MARKER_MISSING, "marker lost before taught touch move", planning_only, physical)
                    self._transition("OPERATOR_CONFIRMATION_REQUIRED", {"sequence": sequence, "planning_only": planning_only})
                self._validate_movement_inputs(pose, planned_state)
                plan = self.backend.plan_joint_pose(plan_name, pose, self.config, start_state=planned_state, motion_profile=motion_profile)
                plan = self._annotate_plan(plan, step, planned_state, planned_state_label, pose, motion_profile, duration_limit_s)
                self._partial_plans.append(plan)
                self._validate_plan(plan, duration_limit_s=duration_limit_s)
                plans.append(plan)
                planned_state = self._state_from_plan(plan)
                planned_state_label = step["target_pose_name"]
                self._transition(state, {"plan": plan.__dict__, "pose": pose.name})
                if physical and not self.backend.execute_plan(plan, self.config.execution_timeout_s):
                    self.backend.stop()
                    return self._fail(state, TouchFailure.EXECUTION_FAILURE, "MoveIt joint plan execution failed", planning_only, True)
                if state == "MOVE_TOUCH":
                    self._transition("HOLD", {"hold_duration_s": self.config.hold_duration_s, "contact_inferred": False})

            self._validate_mission_duration(plans, sequence=sequence)
            rviz_published = False
            if publish_plans_to_rviz:
                rviz_published = bool(self.backend.publish_plans_to_rviz())
                self._last_rviz_published = rviz_published
            self._transition("COMPLETE", {"plans": [p.__dict__ for p in plans], "sequence": sequence, "rviz_published": rviz_published})
            return TouchMissionResult(
                True,
                "COMPLETE",
                None,
                planning_only=planning_only,
                physical_motion_performed=physical,
                outputs=self._report(marker, plans, execution_blocked=not physical, sequence=sequence, rviz_published=rviz_published),
            )
        except ValueError as exc:
            if publish_plans_to_rviz:
                self._last_rviz_published = bool(self.backend.publish_plans_to_rviz())
            return self._fail(self.transitions[-1] if self.transitions else "IDLE", str(exc), str(exc), planning_only, False)

    def check_only(self) -> TouchMissionResult:
        self.logger.start_mission(self.mission_id)
        try:
            self._preflight(require_taught_poses=False, sequence="check_only")
            marker = self._check_marker()
            return TouchMissionResult(
                True,
                "CHECK_MARKER",
                None,
                planning_only=True,
                physical_motion_performed=False,
                outputs=self._report(marker, [], execution_blocked=True, sequence="check_only"),
            )
        except ValueError as exc:
            return self._fail(self.transitions[-1] if self.transitions else "IDLE", str(exc), str(exc), True, False)

    def _preflight(self, *, require_taught_poses: bool = True, sequence: str = "fixed_touch") -> None:
        self._transition("IDLE", {"backend": self.backend.describe()})
        ready = self.backend.check_ready()
        self._backend_ready = ready
        if not ready.get("ready"):
            raise ValueError(f"MoveIt backend not ready: {ready}")
        if self.config.motion_strategy != "taught_joint_sequence":
            raise ValueError("active motion strategy must be taught_joint_sequence")
        if self.config.future_cartesian_mode.get("enabled"):
            raise ValueError("future Cartesian mode must remain disabled for v1")
        if self.config.marker_dictionary != EXPECTED_MARKER_DICTIONARY:
            raise ValueError(f"configured marker dictionary must be {EXPECTED_MARKER_DICTIONARY}")
        if self.config.marker_id != EXPECTED_MARKER_ID:
            raise ValueError("configured marker ID must be 6")
        if abs(self.config.marker_size_m - EXPECTED_MARKER_SIZE_M) > 1e-9:
            raise ValueError("configured marker size must be 0.100 m")
        if self.config.velocity_scaling > 0.25 or self.config.acceleration_scaling > 0.20:
            raise ValueError("velocity and acceleration scaling must remain <= 0.25 velocity and <= 0.20 acceleration")
        if require_taught_poses:
            for pose_name in self._required_pose_names(sequence):
                pose = self._pose(pose_name)
                self._validate_taught_pose_source(pose)
                validate_joint_values(pose.joint_names, pose.positions, self.config.joint_limits, tolerance_rad=self.config.taught_pose_limit_tolerance_rad)
        for name, profile in self.config.motion_profiles.items():
            if float(profile["velocity_scaling"]) > 0.25 or float(profile["acceleration_scaling"]) > 0.20:
                raise ValueError(f"motion profile {name} scaling must remain <= 0.25 velocity and <= 0.20 acceleration")
            if float(profile["velocity_scaling"]) <= 0.0 or float(profile["acceleration_scaling"]) <= 0.0:
                raise ValueError(f"motion profile {name} scaling must be positive")

    def _required_pose_names(self, sequence: str) -> list[str]:
        if sequence == "staging_test":
            return [self.config.staging_pose_name, self.config.pre_touch_pose_name, self.config.retract_pose_name]
        if sequence == "fixed_touch":
            return [self.config.staging_pose_name, self.config.pre_touch_pose_name, self.config.touch_pose_name, self.config.retract_pose_name]
        if sequence == "home_transit_diagnostic":
            return [self.config.home_pose_name, self.config.pre_touch_pose_name, self.config.retract_pose_name]
        return []

    def _sequence_steps(self, sequence: str) -> list[dict[str, str]]:
        if sequence == "staging_test":
            return [
                {"state": "MOVE_STAGING", "plan_name": "move_staging", "target_pose_name": self.config.staging_pose_name, "motion_profile_name": "transit"},
                {"state": "MOVE_PRE_TOUCH", "plan_name": "move_pre_touch", "target_pose_name": self.config.pre_touch_pose_name, "motion_profile_name": "approach"},
                {"state": "MOVE_RETRACT", "plan_name": "move_retract", "target_pose_name": self.config.retract_pose_name, "motion_profile_name": "retract"},
                {"state": "MOVE_STAGING", "plan_name": "move_staging_final", "target_pose_name": self.config.staging_pose_name, "motion_profile_name": "transit"},
            ]
        if sequence == "fixed_touch":
            return [
                {"state": "MOVE_STAGING", "plan_name": "move_staging", "target_pose_name": self.config.staging_pose_name, "motion_profile_name": "transit"},
                {"state": "MOVE_PRE_TOUCH", "plan_name": "move_pre_touch", "target_pose_name": self.config.pre_touch_pose_name, "motion_profile_name": "approach"},
                {"state": "MOVE_TOUCH", "plan_name": "move_touch", "target_pose_name": self.config.touch_pose_name, "motion_profile_name": "touch"},
                {"state": "MOVE_RETRACT", "plan_name": "move_retract", "target_pose_name": self.config.retract_pose_name, "motion_profile_name": "retract"},
                {"state": "MOVE_STAGING", "plan_name": "move_staging_final", "target_pose_name": self.config.staging_pose_name, "motion_profile_name": "transit"},
            ]
        if sequence == "home_transit_diagnostic":
            return [
                {"state": "MOVE_HOME", "plan_name": "move_home", "target_pose_name": self.config.home_pose_name, "motion_profile_name": "transit"},
                {"state": "MOVE_PRE_TOUCH", "plan_name": "move_pre_touch", "target_pose_name": self.config.pre_touch_pose_name, "motion_profile_name": "approach"},
                {"state": "MOVE_RETRACT", "plan_name": "move_retract", "target_pose_name": self.config.retract_pose_name, "motion_profile_name": "retract"},
                {"state": "MOVE_HOME", "plan_name": "move_home_final", "target_pose_name": self.config.home_pose_name, "motion_profile_name": "transit"},
            ]
        raise ValueError(f"unknown sequence {sequence}")

    def _motion_profile(self, name: str) -> dict[str, float]:
        if name not in self.config.motion_profiles:
            raise ValueError(f"missing motion profile: {name}")
        return self.config.motion_profiles[name]

    def _duration_limit(self, plan_name: str) -> float:
        return float(self.config.segment_duration_limits_s.get(plan_name, self.config.max_segment_duration_s))

    def _read_and_validate_current_state(self) -> JointStateSnapshot:
        state = self.backend.read_joint_state()
        if state.age_s > self.config.max_joint_state_age_s:
            raise ValueError(TouchFailure.STALE_JOINT_STATE)
        validate_joint_values(state.joint_names, state.positions, self.config.joint_limits, tolerance_rad=self.config.taught_pose_limit_tolerance_rad)
        if not self.backend.wait_until_stopped():
            raise ValueError("previous trajectory still active or robot is not stopped")
        return state

    def _validate_movement_inputs(self, pose: TaughtPose, start_state: JointStateSnapshot) -> None:
        validate_joint_values(start_state.joint_names, start_state.positions, self.config.joint_limits, tolerance_rad=self.config.taught_pose_limit_tolerance_rad)
        validate_joint_values(pose.joint_names, pose.positions, self.config.joint_limits, tolerance_rad=self.config.taught_pose_limit_tolerance_rad)

    def _check_marker(self) -> MarkerStatus:
        self._transition("CHECK_MARKER", {})
        marker = self.backend.read_marker_status()
        if marker.image_age_s > self.config.max_image_age_s or marker.pose_age_s > self.config.max_marker_pose_age_s:
            raise ValueError(TouchFailure.STALE_CAMERA)
        if marker.dictionary != EXPECTED_MARKER_DICTIONARY or marker.marker_size_m is None or abs(marker.marker_size_m - EXPECTED_MARKER_SIZE_M) > 1e-9:
            raise ValueError(TouchFailure.WRONG_MARKER)
        if marker.marker_id != EXPECTED_MARKER_ID or EXPECTED_MARKER_ID not in marker.detected_ids:
            raise ValueError(TouchFailure.WRONG_MARKER if marker.detected_ids else TouchFailure.MARKER_MISSING)
        if not marker.visible:
            raise ValueError(TouchFailure.MARKER_MISSING)
        if marker.stable_duration_s < self.config.marker_stability_required_s:
            raise ValueError(TouchFailure.STALE_CAMERA)
        self.logger.append({"mission_id": self.mission_id, "event": "marker_status", **marker.__dict__})
        return marker

    def _pose(self, name: str) -> TaughtPose:
        if name not in self.taught_poses:
            raise ValueError(f"{TouchFailure.MISSING_TAUGHT_POSE}: {name}")
        pose = self.taught_poses[name]
        validate_joint_schema(pose.joint_names)
        return pose

    def _validate_taught_pose_source(self, pose: TaughtPose) -> None:
        metadata = pose.metadata or {}
        expected = {
            "feedback_source_id": self.config.required_feedback_source_id,
            "joint_mapping_version": self.config.required_joint_mapping_version,
            "dependency_commit": self.config.required_dependency_commit,
        }
        missing = [name for name in expected if name not in metadata]
        if missing:
            raise ValueError(
                f"taught pose {pose.name} requires recapture from verified PiPER-X feedback; "
                f"missing metadata {missing}"
            )
        mismatches = {
            name: {"expected": value, "actual": metadata.get(name)}
            for name, value in expected.items()
            if str(metadata.get(name)) != str(value)
        }
        if mismatches:
            raise ValueError(f"taught pose {pose.name} feedback-source mismatch: {mismatches}")

    def _annotate_plan(
        self,
        plan: PlanSummary,
        step: dict[str, str],
        start_state: JointStateSnapshot,
        start_pose_name: str,
        target_pose: TaughtPose,
        motion_profile: dict[str, float],
        duration_limit_s: float,
    ) -> PlanSummary:
        per_joint = {
            name: abs(float(target) - float(start))
            for name, start, target in zip(target_pose.joint_names, start_state.positions, target_pose.positions)
        }
        largest_joint = max(per_joint, key=per_joint.get) if per_joint else None
        largest_delta = per_joint[largest_joint] if largest_joint is not None else 0.0
        duration = plan.estimated_duration_s
        if plan.metrics:
            duration = float(plan.metrics.get("total_duration_s", duration))
        return PlanSummary(
            name=plan.name,
            success=plan.success,
            trajectory_points=plan.trajectory_points,
            path_fraction=plan.path_fraction,
            estimated_duration_s=plan.estimated_duration_s,
            maximum_joint_delta_rad=plan.maximum_joint_delta_rad,
            maximum_adjacent_joint_delta_rad=plan.maximum_adjacent_joint_delta_rad,
            reason=plan.reason,
            execution_capable=plan.execution_capable,
            metrics=plan.metrics,
            start_pose_name=start_pose_name,
            target_pose_name=step["target_pose_name"],
            motion_profile_name=step["motion_profile_name"],
            velocity_scaling=float(motion_profile["velocity_scaling"]),
            acceleration_scaling=float(motion_profile["acceleration_scaling"]),
            duration_limit_s=float(duration_limit_s),
            duration_gate_passed=duration <= duration_limit_s,
            per_joint_total_displacement_rad=per_joint,
            largest_displacement_joint=largest_joint,
            largest_displacement_rad=largest_delta,
            large_displacement_review_required=largest_delta > self.config.large_pose_delta_review_threshold_rad,
            effective_joint_velocity_limits_rad_s=self._effective_velocity_limits_for_profile(motion_profile),
        )

    def _effective_velocity_limits_for_profile(self, motion_profile: dict[str, float]) -> dict[str, float | None]:
        limits = self._backend_ready.get("effective_joint_limits") if isinstance(self._backend_ready, dict) else None
        out: dict[str, float | None] = {}
        for name in EXPECTED_JOINT_NAMES:
            payload = limits.get(name, {}) if isinstance(limits, dict) else {}
            base = payload.get("max_velocity_rad_s")
            out[name] = None if base is None else float(base) * float(motion_profile["velocity_scaling"])
        return out

    def _validate_plan(self, plan: PlanSummary, *, duration_limit_s: float) -> None:
        if not plan.success:
            raise ValueError(plan.reason or TouchFailure.MOVEIT_PLANNING_FAILURE)
        if not plan.metrics:
            raise ValueError("MoveIt plan did not include trajectory metrics")
        metrics = plan.metrics
        if metrics["trajectory_points"] <= 0:
            raise ValueError("MoveIt returned empty trajectory")
        if metrics["nonzero_motion"] and metrics["trajectory_points"] <= 1:
            raise ValueError("MoveIt returned one-point trajectory for nonzero move")
        if not metrics["monotonic_timestamps"]:
            raise ValueError("trajectory timestamps are not strictly monotonic")
        if metrics["nonzero_motion"] and metrics["total_duration_s"] <= 0.0:
            raise ValueError("trajectory has zero duration for nonzero move")
        if plan.maximum_adjacent_joint_delta_rad > self.config.max_adjacent_joint_delta_rad:
            raise ValueError(
                f"adjacent joint step {plan.maximum_adjacent_joint_delta_rad:.6f} exceeds limit "
                f"{self.config.max_adjacent_joint_delta_rad:.6f}"
            )
        if metrics["maximum_continuity_error_rad"] > self.config.continuity_tolerance_rad:
            raise ValueError(
                f"trajectory start continuity error {metrics['maximum_continuity_error_rad']:.6f} exceeds "
                f"{self.config.continuity_tolerance_rad:.6f}"
            )
        if metrics["maximum_target_error_rad"] > self.config.max_target_error_rad:
            raise ValueError(
                f"trajectory endpoint target error {metrics['maximum_target_error_rad']:.6f} exceeds "
                f"{self.config.max_target_error_rad:.6f}"
            )
        if metrics["total_duration_s"] > duration_limit_s:
            raise ValueError(
                f"segment duration {metrics['total_duration_s']:.3f}s exceeds limit "
                f"{duration_limit_s:.3f}s"
            )
        if metrics["nonzero_motion"]:
            effective_velocity = metrics["maximum_joint_delta_rad"] / metrics["total_duration_s"]
            if effective_velocity < self.config.min_effective_joint_velocity_rad_s:
                raise ValueError(
                    f"effective joint velocity {effective_velocity:.6f} rad/s below diagnostic floor "
                    f"{self.config.min_effective_joint_velocity_rad_s:.6f} rad/s"
                )

    def _validate_mission_duration(self, plans: list[PlanSummary], *, sequence: str) -> None:
        duration = sum(float(plan.estimated_duration_s) for plan in plans) + self.config.hold_duration_s
        limit = self.config.home_transit_diagnostic_max_mission_duration_s if sequence == "home_transit_diagnostic" else self.config.max_mission_duration_s
        if duration > limit:
            raise ValueError(f"mission duration {duration:.3f}s exceeds limit {limit:.3f}s")

    def _state_from_plan(self, plan: PlanSummary) -> JointStateSnapshot:
        if not plan.metrics:
            raise ValueError("missing plan metrics")
        return JointStateSnapshot(
            joint_names=list(plan.metrics["joint_names"]),
            positions=[float(v) for v in plan.metrics["final_point_positions"]],
            age_s=0.0,
            velocities=[0.0] * 6,
            stamp_s=None,
            source_topic=f"planned_endpoint:{plan.name}",
        )

    def _transition(self, state: str, outputs: dict[str, Any]) -> None:
        self.transitions.append(state)
        self.logger.append({"mission_id": self.mission_id, "event": "state_transition", "state": state, **outputs})

    def _fail(self, state: str, reason: str, message: str, planning_only: bool, physical_motion_performed: bool) -> TouchMissionResult:
        self.logger.append({"mission_id": self.mission_id, "event": "failure", "state": state, "reason": reason, "message": message})
        return TouchMissionResult(
            False,
            state,
            reason,
            planning_only,
            physical_motion_performed,
            {
                "message": message,
                "transitions": self.transitions,
                "partial_plans": [plan.__dict__ for plan in self._partial_plans],
                "rviz_display_trajectory_published": self._last_rviz_published,
                "backend": self.backend.describe(),
            },
        )

    def _report(self, marker: MarkerStatus, plans: list[PlanSummary], *, execution_blocked: bool, sequence: str, rviz_published: bool = False) -> dict[str, Any]:
        backend = self.backend.describe()
        return {
            "mission_id": self.mission_id,
            "profile_id": self.config.profile_id,
            "task_id": self.config.task_id,
            "planning_group": self.config.planning_group,
            "end_effector_link": self.config.end_effector_link,
            "robot_model": self.config.robot_model,
            "robot_urdf_candidate_path": self.config.robot_urdf_candidate_path,
            "robot_urdf_candidate_sha256": self.config.robot_urdf_candidate_sha256,
            "piper_x_model_verified": self.config.piper_x_model_verified,
            "marker_status": marker.__dict__,
            "plans": [p.__dict__ for p in plans],
            "sequence": sequence,
            "motion_strategy": self.config.motion_strategy,
            "trajectory_points": sum(p.trajectory_points for p in plans),
            "estimated_motion_duration_s": sum(p.estimated_duration_s for p in plans) + self.config.hold_duration_s,
            "maximum_joint_delta_rad": max((p.maximum_joint_delta_rad for p in plans), default=0.0),
            "maximum_adjacent_joint_delta_rad": max((p.maximum_adjacent_joint_delta_rad for p in plans), default=0.0),
            "velocity_scaling": self.config.velocity_scaling,
            "acceleration_scaling": self.config.acceleration_scaling,
            "motion_profiles": self.config.motion_profiles,
            "segment_duration_limits_s": self.config.segment_duration_limits_s,
            "duration_sanity": {
                "max_segment_duration_s": self.config.max_segment_duration_s,
                "max_mission_duration_s": self.config.max_mission_duration_s,
                "home_transit_diagnostic_max_mission_duration_s": self.config.home_transit_diagnostic_max_mission_duration_s,
                "min_effective_joint_velocity_rad_s": self.config.min_effective_joint_velocity_rad_s,
                "max_target_error_rad": self.config.max_target_error_rad,
                "continuity_tolerance_rad": self.config.continuity_tolerance_rad,
                "large_pose_delta_review_threshold_rad": self.config.large_pose_delta_review_threshold_rad,
            },
            "rviz_display_trajectory_published": rviz_published,
            "execution_blocked": execution_blocked,
            "rejected_handeye_used_for_targeting": False,
            "rejected_handeye_path": REJECTED_HANDEYE_PATH,
            "backend": backend,
            "transitions": self.transitions,
        }


class RestrictedArucoTouchAPI:
    ALLOWED = {
        "check_aruco_target",
        "plan_aruco_touch",
        "execute_aruco_touch",
        "retract_from_aruco",
        "move_arm_home",
        "get_aruco_touch_status",
    }

    def __init__(self, controller: MoveItArucoTouchController) -> None:
        self.controller = controller
        self.last_result: TouchMissionResult | None = None

    def call(self, action: str, **kwargs) -> dict[str, Any]:
        if action not in self.ALLOWED:
            return {"accepted": False, "reason": "arbitrary targets are not exposed by the restricted API", "action": action}
        if action == "plan_aruco_touch":
            self.last_result = self.controller.run(planning_only=True)
            return {"accepted": True, "result": self.last_result.__dict__}
        if action == "execute_aruco_touch":
            self.last_result = self.controller.run(planning_only=False, execute=True, confirm=kwargs.get("confirm"))
            return {"accepted": True, "result": self.last_result.__dict__}
        if action == "get_aruco_touch_status":
            return {"accepted": True, "last_result": None if self.last_result is None else self.last_result.__dict__}
        return {"accepted": True, "action": action, "planning_only": True}


def result_to_json(result: TouchMissionResult) -> str:
    return json.dumps(result.__dict__, indent=2, sort_keys=True)
