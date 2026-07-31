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
    optional_safe_recovery_pose_name: str | None
    taught_pose_manifest: str
    touch_direction_ee: list[float]
    press_distance_m: float
    hold_duration_s: float
    retract_distance_m: float
    velocity_scaling: float
    acceleration_scaling: float
    planning_timeout_s: float
    execution_timeout_s: float
    cartesian_fraction_threshold: float
    max_joint_jump_rad: float
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
            optional_safe_recovery_pose_name=poses.get("safe_recovery"),
            taught_pose_manifest=str(poses["manifest"]),
            touch_direction_ee=[float(v) for v in motion["touch_direction_ee"]],
            press_distance_m=float(motion["press_distance_m"]),
            hold_duration_s=float(motion["hold_duration_s"]),
            retract_distance_m=float(motion["retract_distance_m"]),
            velocity_scaling=float(motion["velocity_scaling"]),
            acceleration_scaling=float(motion["acceleration_scaling"]),
            planning_timeout_s=float(motion["planning_timeout_s"]),
            execution_timeout_s=float(motion["execution_timeout_s"]),
            cartesian_fraction_threshold=float(motion["cartesian_fraction_threshold"]),
            max_joint_jump_rad=float(motion["max_joint_jump_rad"]),
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

    def plan_cartesian_press(self, name: str, direction_ee: list[float], distance_m: float, config: TouchConfig) -> PlanSummary:
        ...

    def execute_plan(self, plan: PlanSummary, timeout_s: float) -> bool:
        ...

    def stop(self) -> None:
        ...


class MockMoveItTouchBackend:
    def __init__(
        self,
        *,
        marker: MarkerStatus | None = None,
        joint_state: JointStateSnapshot | None = None,
        cartesian_fraction: float = 1.0,
        max_joint_delta_rad: float = 0.03,
        execute_ok: bool = True,
    ) -> None:
        self.marker = marker or MarkerStatus(True, 6, EXPECTED_MARKER_DICTIONARY, EXPECTED_MARKER_SIZE_M, 0.0, 0.0, 1.0, [6])
        self.joint_state = joint_state or JointStateSnapshot(list(EXPECTED_JOINT_NAMES), [0.0] * 6, 0.0)
        self.cartesian_fraction = float(cartesian_fraction)
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

    def plan_joint_pose(self, name: str, pose: TaughtPose, config: TouchConfig) -> PlanSummary:
        current = self.read_joint_state()
        max_delta = max(abs(float(a) - float(b)) for a, b in zip(current.positions, pose.positions))
        return PlanSummary(name, True, trajectory_points=8, estimated_duration_s=2.0, maximum_joint_delta_rad=max_delta)

    def plan_cartesian_press(self, name: str, direction_ee: list[float], distance_m: float, config: TouchConfig) -> PlanSummary:
        points = max(2, int(math.ceil(float(distance_m) / 0.005)) + 1)
        return PlanSummary(
            name,
            True,
            trajectory_points=points,
            path_fraction=self.cartesian_fraction,
            estimated_duration_s=max(0.5, float(distance_m) / 0.01),
            maximum_joint_delta_rad=self.max_joint_delta_rad,
            maximum_adjacent_joint_delta_rad=self.max_joint_delta_rad / max(1, points - 1),
        )

    def execute_plan(self, plan: PlanSummary, timeout_s: float) -> bool:
        self.executed.append(plan.name)
        return self.execute_ok

    def stop(self) -> None:
        self.stopped = True


@dataclass
class TouchMissionResult:
    success: bool
    state: str
    failure_reason: str | None
    planning_only: bool
    physical_motion_performed: bool
    outputs: dict[str, Any]


class MoveItArucoTouchController:
    STATES = ["IDLE", "CHECK_MARKER", "MOVE_HOME", "MOVE_PRE_TOUCH", "APPROACH", "TOUCH", "HOLD", "RETRACT", "MOVE_HOME", "COMPLETE"]

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

    def run(self, *, planning_only: bool = True, execute: bool = False, confirm: str | None = None) -> TouchMissionResult:
        self.logger.start_mission(self.mission_id)
        physical = bool(execute)
        if physical:
            if planning_only:
                return self._fail("IDLE", TouchFailure.EXECUTION_BLOCKED, "execute cannot be combined with planning_only", planning_only, False)
            if confirm != "FIXED_ARUCO_TOUCH":
                return self._fail("IDLE", TouchFailure.EXECUTION_BLOCKED, "physical execution requires --confirm FIXED_ARUCO_TOUCH", planning_only, False)
            if not self.config.physical_execution_enabled_by_default:
                return self._fail("IDLE", TouchFailure.EXECUTION_BLOCKED, "physical execution disabled in committed config", planning_only, False)

        try:
            self._preflight()
            marker = self._check_marker()
            plans = []
            home = self._pose(self.config.home_pose_name)
            pre_touch = self._pose(self.config.pre_touch_pose_name)

            for state in ["MOVE_HOME", "MOVE_PRE_TOUCH"]:
                pose = home if state == "MOVE_HOME" else pre_touch
                plan = self.backend.plan_joint_pose(state.lower(), pose, self.config)
                self._validate_plan(plan, require_cartesian=False)
                plans.append(plan)
                self._transition(state, {"plan": plan.__dict__})
                if physical and not self.backend.execute_plan(plan, self.config.execution_timeout_s):
                    return self._fail(state, TouchFailure.EXECUTION_FAILURE, "MoveIt joint plan execution failed", planning_only, True)

            press = self.backend.plan_cartesian_press("approach_touch", self.config.touch_direction_ee, self.config.press_distance_m, self.config)
            self._validate_plan(press, require_cartesian=True)
            plans.append(press)
            self._transition("APPROACH", {"plan": press.__dict__})
            marker_before_press = self._check_marker()
            if not marker_before_press.visible:
                return self._fail("APPROACH", TouchFailure.MARKER_MISSING, "marker lost before press", planning_only, physical)
            self._transition("TOUCH", {"press_distance_m": self.config.press_distance_m})
            if physical and not self.backend.execute_plan(press, self.config.execution_timeout_s):
                self.backend.stop()
                return self._fail("TOUCH", TouchFailure.EXECUTION_FAILURE, "MoveIt press execution failed", planning_only, True)

            self._transition("HOLD", {"hold_duration_s": self.config.hold_duration_s})

            retract_vector = build_inverse_press([float(v) * self.config.retract_distance_m for v in self.config.touch_direction_ee])
            retract = self.backend.plan_cartesian_press("retract_inverse_press", retract_vector, self.config.retract_distance_m, self.config)
            self._validate_plan(retract, require_cartesian=True)
            plans.append(retract)
            self._transition("RETRACT", {"plan": retract.__dict__, "inverse_press_vector": retract_vector})
            if physical and not self.backend.execute_plan(retract, self.config.execution_timeout_s):
                self.backend.stop()
                return self._fail("RETRACT", TouchFailure.EXECUTION_FAILURE, "MoveIt retract execution failed", planning_only, True)

            final_home = self.backend.plan_joint_pose("move_home_final", home, self.config)
            self._validate_plan(final_home, require_cartesian=False)
            plans.append(final_home)
            self._transition("MOVE_HOME", {"plan": final_home.__dict__, "final": True})
            if physical and not self.backend.execute_plan(final_home, self.config.execution_timeout_s):
                return self._fail("MOVE_HOME", TouchFailure.EXECUTION_FAILURE, "MoveIt final home execution failed", planning_only, True)

            self._transition("COMPLETE", {"plans": [p.__dict__ for p in plans]})
            return TouchMissionResult(
                True,
                "COMPLETE",
                None,
                planning_only=planning_only,
                physical_motion_performed=physical,
                outputs=self._report(marker, plans, execution_blocked=not physical),
            )
        except ValueError as exc:
            return self._fail(self.transitions[-1] if self.transitions else "IDLE", str(exc), str(exc), planning_only, False)

    def _preflight(self) -> None:
        self._transition("IDLE", {"backend": self.backend.describe()})
        if self.config.marker_dictionary != EXPECTED_MARKER_DICTIONARY:
            raise ValueError(f"configured marker dictionary must be {EXPECTED_MARKER_DICTIONARY}")
        if self.config.marker_id != EXPECTED_MARKER_ID:
            raise ValueError("configured marker ID must be 6")
        if abs(self.config.marker_size_m - EXPECTED_MARKER_SIZE_M) > 1e-9:
            raise ValueError("configured marker size must be 0.100 m")
        if self.config.press_distance_m <= 0.0 or self.config.press_distance_m > 0.020:
            raise ValueError("press distance must be positive and no more than 0.020 m before operator review")
        if self.config.cartesian_fraction_threshold != 1.0:
            raise ValueError("fixed MVP requires Cartesian path fraction threshold of 1.0")
        if self.config.velocity_scaling > 0.05 or self.config.acceleration_scaling > 0.05:
            raise ValueError("velocity and acceleration scaling must remain <= 0.05")
        state = self.backend.read_joint_state()
        if state.age_s > self.config.max_joint_state_age_s:
            raise ValueError(TouchFailure.STALE_JOINT_STATE)
        validate_joint_values(state.joint_names, state.positions, self.config.joint_limits)
        for pose_name in [self.config.home_pose_name, self.config.pre_touch_pose_name]:
            pose = self._pose(pose_name)
            validate_joint_values(pose.joint_names, pose.positions, self.config.joint_limits)

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

    def _validate_plan(self, plan: PlanSummary, *, require_cartesian: bool) -> None:
        if not plan.success:
            raise ValueError(plan.reason or TouchFailure.MOVEIT_PLANNING_FAILURE)
        if require_cartesian and plan.path_fraction < self.config.cartesian_fraction_threshold:
            raise ValueError(f"Cartesian path fraction {plan.path_fraction:.6f} below required {self.config.cartesian_fraction_threshold:.6f}")
        if plan.maximum_joint_delta_rad > self.config.max_joint_jump_rad:
            raise ValueError(f"joint jump {plan.maximum_joint_delta_rad:.6f} exceeds limit {self.config.max_joint_jump_rad:.6f}")

    def _transition(self, state: str, outputs: dict[str, Any]) -> None:
        self.transitions.append(state)
        self.logger.append({"mission_id": self.mission_id, "event": "state_transition", "state": state, **outputs})

    def _fail(self, state: str, reason: str, message: str, planning_only: bool, physical_motion_performed: bool) -> TouchMissionResult:
        self.logger.append({"mission_id": self.mission_id, "event": "failure", "state": state, "reason": reason, "message": message})
        return TouchMissionResult(False, state, reason, planning_only, physical_motion_performed, {"message": message, "transitions": self.transitions})

    def _report(self, marker: MarkerStatus, plans: list[PlanSummary], *, execution_blocked: bool) -> dict[str, Any]:
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
