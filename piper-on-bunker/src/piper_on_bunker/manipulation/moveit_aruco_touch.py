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
    planning_timeout_s: float
    execution_timeout_s: float
    max_joint_jump_rad: float
    future_cartesian_mode: dict[str, Any]
    max_marker_pose_age_s: float
    max_image_age_s: float
    max_joint_state_age_s: float
    marker_stability_required_s: float
    joint_limits: dict[str, list[float]]
    workspace_limits: dict[str, list[float]]
    physical_execution_enabled_by_default: bool
    robot_model: str
    robot_urdf_candidate_path: str
    robot_urdf_candidate_sha256: str
    piper_x_model_verified: bool

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
            home_pose_name=str(poses["home"]),
            pre_touch_pose_name=str(poses["pre_touch"]),
            touch_pose_name=str(poses["touch"]),
            retract_pose_name=str(poses["retract"]),
            optional_safe_recovery_pose_name=poses.get("safe_recovery"),
            taught_pose_manifest=str(poses["manifest"]),
            motion_strategy=str(motion["motion_strategy"]),
            hold_duration_s=float(motion["hold_duration_s"]),
            velocity_scaling=float(motion["velocity_scaling"]),
            acceleration_scaling=float(motion["acceleration_scaling"]),
            planning_timeout_s=float(motion["planning_timeout_s"]),
            execution_timeout_s=float(motion["execution_timeout_s"]),
            max_joint_jump_rad=float(motion["max_joint_jump_rad"]),
            future_cartesian_mode=dict(motion.get("future_cartesian_mode") or {}),
            max_marker_pose_age_s=float(safety["max_marker_pose_age_s"]),
            max_image_age_s=float(safety["max_image_age_s"]),
            max_joint_state_age_s=float(safety["max_joint_state_age_s"]),
            marker_stability_required_s=float(safety["marker_stability_required_s"]),
            joint_limits={str(k): [float(v[0]), float(v[1])] for k, v in safety["joint_limits"].items()},
            workspace_limits={str(k): [float(v[0]), float(v[1])] for k, v in safety["workspace_limits"].items()},
            physical_execution_enabled_by_default=bool(safety["physical_execution_enabled_by_default"]),
            robot_model=str(robot["model"]),
            robot_urdf_candidate_path=str(robot["urdf_candidate_path"]),
            robot_urdf_candidate_sha256=str(robot["urdf_candidate_sha256"]),
            piper_x_model_verified=bool(robot.get("piper_x_model_verified", False)),
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


def validate_joint_values(joint_names: list[str], positions: list[float], limits: dict[str, list[float]]) -> None:
    validate_joint_schema(joint_names)
    if len(positions) != 6:
        raise ValueError("expected exactly six arm joint positions")
    for name, value in zip(joint_names, positions):
        if not math.isfinite(float(value)):
            raise ValueError(f"non-finite joint value for {name}")
        lower, upper = limits[name]
        if float(value) < lower or float(value) > upper:
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

    def plan_joint_pose(self, name: str, pose: TaughtPose, config: TouchConfig) -> PlanSummary:
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
            "end_effector_link": "gripper_tcp",
            "robot_urdf": "mock",
        }

    def read_joint_state(self) -> JointStateSnapshot:
        return self.joint_state

    def read_marker_status(self) -> MarkerStatus:
        return self.marker

    def check_ready(self) -> dict[str, Any]:
        return {"ready": True, "backend": "mock_moveit", "planning_only_supported": True}

    def plan_joint_pose(self, name: str, pose: TaughtPose, config: TouchConfig) -> PlanSummary:
        current = self.read_joint_state()
        max_delta = max(abs(float(a) - float(b)) for a, b in zip(current.positions, pose.positions))
        return PlanSummary(name, True, trajectory_points=8, estimated_duration_s=2.0, maximum_joint_delta_rad=max(max_delta, self.max_joint_delta_rad), execution_capable=True)

    def execute_plan(self, plan: PlanSummary, timeout_s: float) -> bool:
        self.executed.append(plan.name)
        return self.execute_ok

    def stop(self) -> None:
        self.stopped = True

    def wait_until_stopped(self) -> bool:
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
            import moveit_commander

            moveit_commander.roscpp_initialize([])
            group = moveit_commander.MoveGroupCommander(self.config.planning_group)
            out["moveit_commander_available"] = True
            out["move_group_available"] = True
            out["planning_frame"] = group.get_planning_frame()
            out["active_joints"] = list(group.get_active_joints())
            out["current_end_effector_link"] = getattr(group, "get_end_effector_link", lambda: None)()
            out["planning_only_supported"] = True
            out["ready"] = True
        except Exception as exc:
            out["moveit_commander_error"] = repr(exc)
        return out

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

    def plan_joint_pose(self, name: str, pose: TaughtPose, config: TouchConfig) -> PlanSummary:
        try:
            import moveit_commander

            moveit_commander.roscpp_initialize([])
            group = moveit_commander.MoveGroupCommander(config.planning_group)
            group.set_max_velocity_scaling_factor(config.velocity_scaling)
            group.set_max_acceleration_scaling_factor(config.acceleration_scaling)
            group.set_planning_time(config.planning_timeout_s)
            if list(group.get_active_joints()) != list(pose.joint_names):
                return PlanSummary(name, False, 0, reason=f"MoveIt active joints {group.get_active_joints()} do not match taught pose {pose.joint_names}")
            group.set_joint_value_target(dict(zip(pose.joint_names, pose.positions)))
            plan_result = group.plan()
            trajectory = _normalize_moveit_plan(plan_result)
            points = getattr(getattr(trajectory, "joint_trajectory", None), "points", []) if trajectory is not None else []
            if not points:
                return PlanSummary(name, False, 0, reason="MoveIt returned no joint trajectory")
            self._last_plan = trajectory
            max_delta = _trajectory_max_joint_delta(points)
            duration = float(points[-1].time_from_start.to_sec()) if hasattr(points[-1], "time_from_start") else 0.0
            return PlanSummary(name, True, len(points), estimated_duration_s=duration, maximum_joint_delta_rad=max_delta, execution_capable=True)
        except Exception as exc:
            return PlanSummary(name, False, 0, reason=f"MoveIt planning failed: {exc!r}")

    def execute_plan(self, plan: PlanSummary, timeout_s: float) -> bool:
        if self._last_plan is None:
            return False
        try:
            import moveit_commander

            group = moveit_commander.MoveGroupCommander(self.config.planning_group)
            return bool(group.execute(self._last_plan, wait=True))
        except Exception:
            return False

    def stop(self) -> None:
        try:
            import moveit_commander

            group = moveit_commander.MoveGroupCommander(self.config.planning_group)
            group.stop()
        except Exception:
            pass

    def wait_until_stopped(self) -> bool:
        state = self.read_joint_state()
        return not state.velocities or max(abs(v) for v in state.velocities) <= 0.01


def _normalize_moveit_plan(plan_result: Any) -> Any:
    if isinstance(plan_result, tuple):
        if len(plan_result) >= 2 and bool(plan_result[0]):
            return plan_result[1]
        if len(plan_result) >= 1:
            return plan_result[0]
    return plan_result


def _trajectory_max_joint_delta(points: list[Any]) -> float:
    if not points:
        return 0.0
    max_delta = 0.0
    previous = None
    for point in points:
        positions = [float(v) for v in getattr(point, "positions", [])]
        if previous is not None and positions:
            max_delta = max(max_delta, max(abs(a - b) for a, b in zip(previous, positions)))
        previous = positions
    return max_delta


@dataclass
class TouchMissionResult:
    success: bool
    state: str
    failure_reason: str | None
    planning_only: bool
    physical_motion_performed: bool
    outputs: dict[str, Any]


class MoveItArucoTouchController:
    STATES = ["IDLE", "CHECK_MARKER", "MOVE_HOME", "MOVE_PRE_TOUCH", "MOVE_TOUCH", "HOLD", "MOVE_RETRACT", "MOVE_HOME", "COMPLETE"]

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

    def run(self, *, planning_only: bool = True, execute: bool = False, confirm: str | None = None, sequence: str = "full_touch") -> TouchMissionResult:
        self.logger.start_mission(self.mission_id)
        physical = bool(execute)
        if sequence not in {"full_touch", "pre_touch_test"}:
            return self._fail("IDLE", TouchFailure.EXECUTION_BLOCKED, f"unknown sequence {sequence}", planning_only, False)
        if physical:
            if planning_only:
                return self._fail("IDLE", TouchFailure.EXECUTION_BLOCKED, "execute cannot be combined with planning_only", planning_only, False)
            expected_confirm = "PRE_TOUCH_TEST" if sequence == "pre_touch_test" else "FIXED_ARUCO_TOUCH"
            if confirm != expected_confirm:
                return self._fail("IDLE", TouchFailure.EXECUTION_BLOCKED, f"physical execution requires --confirm {expected_confirm}", planning_only, False)
            if not self.config.physical_execution_enabled_by_default:
                return self._fail("IDLE", TouchFailure.EXECUTION_BLOCKED, "physical execution disabled in committed config", planning_only, False)

        try:
            self._preflight()
            marker = self._check_marker()
            plans: list[PlanSummary] = []
            home = self._pose(self.config.home_pose_name)
            pre_touch = self._pose(self.config.pre_touch_pose_name)
            touch = self._pose(self.config.touch_pose_name)
            retract = self._pose(self.config.retract_pose_name)
            steps = [
                ("MOVE_HOME", "move_home", home),
                ("MOVE_PRE_TOUCH", "move_pre_touch", pre_touch),
            ]
            if sequence == "full_touch":
                steps.append(("MOVE_TOUCH", "move_touch", touch))
            steps.extend(
                [
                    ("MOVE_RETRACT", "move_retract", retract),
                    ("MOVE_HOME", "move_home_final", home),
                ]
            )

            for state, plan_name, pose in steps:
                if state == "MOVE_TOUCH":
                    marker_before_touch = self._check_marker()
                    if not marker_before_touch.visible:
                        return self._fail("MOVE_TOUCH", TouchFailure.MARKER_MISSING, "marker lost before taught touch move", planning_only, physical)
                    self._transition("OPERATOR_CONFIRMATION_REQUIRED", {"sequence": sequence, "planning_only": planning_only})
                self._validate_before_movement(pose)
                plan = self.backend.plan_joint_pose(plan_name, pose, self.config)
                self._validate_plan(plan)
                plans.append(plan)
                self._transition(state, {"plan": plan.__dict__, "pose": pose.name})
                if physical and not self.backend.execute_plan(plan, self.config.execution_timeout_s):
                    self.backend.stop()
                    return self._fail(state, TouchFailure.EXECUTION_FAILURE, "MoveIt joint plan execution failed", planning_only, True)
                if state == "MOVE_TOUCH":
                    self._transition("HOLD", {"hold_duration_s": self.config.hold_duration_s, "contact_inferred": False})

            self._transition("COMPLETE", {"plans": [p.__dict__ for p in plans], "sequence": sequence})
            return TouchMissionResult(
                True,
                "COMPLETE",
                None,
                planning_only=planning_only,
                physical_motion_performed=physical,
                outputs=self._report(marker, plans, execution_blocked=not physical, sequence=sequence),
            )
        except ValueError as exc:
            return self._fail(self.transitions[-1] if self.transitions else "IDLE", str(exc), str(exc), planning_only, False)

    def check_only(self) -> TouchMissionResult:
        self.logger.start_mission(self.mission_id)
        try:
            self._preflight()
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

    def _preflight(self) -> None:
        self._transition("IDLE", {"backend": self.backend.describe()})
        ready = self.backend.check_ready()
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
        if self.config.velocity_scaling > 0.05 or self.config.acceleration_scaling > 0.05:
            raise ValueError("velocity and acceleration scaling must remain <= 0.05")
        for pose_name in [self.config.home_pose_name, self.config.pre_touch_pose_name, self.config.touch_pose_name, self.config.retract_pose_name]:
            pose = self._pose(pose_name)
            validate_joint_values(pose.joint_names, pose.positions, self.config.joint_limits)

    def _validate_before_movement(self, pose: TaughtPose) -> None:
        state = self.backend.read_joint_state()
        if state.age_s > self.config.max_joint_state_age_s:
            raise ValueError(TouchFailure.STALE_JOINT_STATE)
        validate_joint_values(state.joint_names, state.positions, self.config.joint_limits)
        validate_joint_values(pose.joint_names, pose.positions, self.config.joint_limits)
        if not self.backend.wait_until_stopped():
            raise ValueError("previous trajectory still active or robot is not stopped")

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

    def _validate_plan(self, plan: PlanSummary) -> None:
        if not plan.success:
            raise ValueError(plan.reason or TouchFailure.MOVEIT_PLANNING_FAILURE)
        if plan.maximum_joint_delta_rad > self.config.max_joint_jump_rad:
            raise ValueError(f"joint jump {plan.maximum_joint_delta_rad:.6f} exceeds limit {self.config.max_joint_jump_rad:.6f}")

    def _transition(self, state: str, outputs: dict[str, Any]) -> None:
        self.transitions.append(state)
        self.logger.append({"mission_id": self.mission_id, "event": "state_transition", "state": state, **outputs})

    def _fail(self, state: str, reason: str, message: str, planning_only: bool, physical_motion_performed: bool) -> TouchMissionResult:
        self.logger.append({"mission_id": self.mission_id, "event": "failure", "state": state, "reason": reason, "message": message})
        return TouchMissionResult(False, state, reason, planning_only, physical_motion_performed, {"message": message, "transitions": self.transitions})

    def _report(self, marker: MarkerStatus, plans: list[PlanSummary], *, execution_blocked: bool, sequence: str) -> dict[str, Any]:
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
            "velocity_scaling": self.config.velocity_scaling,
            "acceleration_scaling": self.config.acceleration_scaling,
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
