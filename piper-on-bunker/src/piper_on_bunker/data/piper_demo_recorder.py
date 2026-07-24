from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

from piper_on_bunker.configuration import PipelineConfig, load_config
from piper_on_bunker.hardware.joint_state import DEFAULT_ARM_JOINT_NAMES, map_joint_state
from piper_on_bunker.hardware.piper_ros_arm import PiperRosArm

from .action_schema import EndPoseMetadata, PiperCommandSample, PiperFrameRecord, PiperStateSample
from .episode_writer import EpisodeSession, EpisodeWriter


DEFAULT_CAMERA_TOPIC = "/table_camera/color/image_raw"
DEFAULT_END_POSE_TOPIC = "/end_pose"
DEFAULT_COMMAND_ECHO_TOPIC = "/piper_joint_commands"
DEFAULT_GRIPPER_MIN_M = 0.0
DEFAULT_GRIPPER_MAX_M = 0.06
LIVE_DEMO_MODE = "piper_demo_collection"


def parse_manual_command(command: str) -> Tuple[str, int]:
    token = command.strip().lower()
    if token == "g+":
        return "gripper", 1
    if token == "g-":
        return "gripper", -1
    if len(token) == 3 and token[0] == "j" and token[1].isdigit() and token[2] in "+-":
        joint_index = int(token[1])
        if not 1 <= joint_index <= 6:
            raise ValueError("joint command must target j1..j6")
        return f"joint{joint_index}", 1 if token[2] == "+" else -1
    raise ValueError("unsupported command; use j1+/j1-/.../j6+/j6-/g+/g-")


def build_target_from_command(
    current_joints_rad,
    current_gripper_m: float,
    command: str,
    joint_step_rad: float,
    gripper_step_m: float,
    gripper_min_m: float = DEFAULT_GRIPPER_MIN_M,
    gripper_max_m: float = DEFAULT_GRIPPER_MAX_M,
) -> Tuple[list[float], float]:
    joint_name, direction = parse_manual_command(command)
    joints = [float(value) for value in current_joints_rad]
    gripper = float(current_gripper_m)
    if joint_name == "gripper":
        gripper = min(gripper_max_m, max(gripper_min_m, gripper + direction * float(gripper_step_m)))
    else:
        joints[int(joint_name[-1]) - 1] += direction * float(joint_step_rad)
    return joints, gripper


@dataclass
class LiveSnapshot:
    image_rgb: np.ndarray
    image_topic: str
    image_frame_id: str
    image_timestamp_s: float
    image_age_s: float
    state: PiperStateSample
    end_pose: Optional[EndPoseMetadata]


class RosSnapshotProvider:
    def __init__(
        self,
        camera_topic: str = DEFAULT_CAMERA_TOPIC,
        end_pose_topic: str = DEFAULT_END_POSE_TOPIC,
        expected_joint_names=None,
    ) -> None:
        try:
            import rospy
            from cv_bridge import CvBridge
            from geometry_msgs.msg import PoseStamped
            from sensor_msgs.msg import Image, JointState
        except Exception as exc:
            raise RuntimeError("PiPER demo recorder requires rospy, sensor_msgs, geometry_msgs, and cv_bridge") from exc
        self.rospy = rospy
        self.CvBridge = CvBridge
        self.Image = Image
        self.JointState = JointState
        self.PoseStamped = PoseStamped
        self.camera_topic = camera_topic
        self.end_pose_topic = end_pose_topic
        self.expected_joint_names = list(expected_joint_names or DEFAULT_ARM_JOINT_NAMES)
        self.bridge = CvBridge()
        if not rospy.get_node_uri():
            rospy.init_node("piper_demo_recorder", anonymous=True, disable_signals=True)

    def read_snapshot(self, state_timeout_s: float, image_timeout_s: float, max_state_age_s: float, max_image_age_s: float) -> LiveSnapshot:
        joint_msg = self.rospy.wait_for_message("/joint_states_single", self.JointState, timeout=state_timeout_s)
        mapped = map_joint_state(
            joint_msg.name,
            joint_msg.position,
            float(joint_msg.header.stamp.to_sec()),
            self.expected_joint_names,
        )
        if mapped.gripper_position is None:
            raise ValueError("live joint state does not expose gripper position")
        if mapped.age_s > max_state_age_s:
            raise ValueError(f"joint state is stale: {mapped.age_s:.3f}s > {max_state_age_s:.3f}s")
        state = PiperStateSample(
            joint_names=list(mapped.arm_joint_names),
            joint_positions_rad=list(mapped.arm_positions),
            gripper_value=float(mapped.gripper_position),
            timestamp_s=float(mapped.stamp_s),
            age_s=float(mapped.age_s),
        )
        image_msg = self.rospy.wait_for_message(self.camera_topic, self.Image, timeout=image_timeout_s)
        image_timestamp_s = float(image_msg.header.stamp.to_sec()) if image_msg.header.stamp else 0.0
        now_s = float(self.rospy.Time.now().to_sec()) if self.rospy.Time.now() else time.time()
        image_age_s = max(0.0, now_s - image_timestamp_s) if image_timestamp_s else float("inf")
        if image_age_s > max_image_age_s:
            raise ValueError(f"camera image is stale: {image_age_s:.3f}s > {max_image_age_s:.3f}s")
        image_bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        image_rgb = image_bgr[:, :, ::-1].copy()
        end_pose = None
        try:
            pose_msg = self.rospy.wait_for_message(self.end_pose_topic, self.PoseStamped, timeout=0.5)
            end_pose = EndPoseMetadata(
                position_m=[pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z],
                quaternion_xyzw=[
                    pose_msg.pose.orientation.x,
                    pose_msg.pose.orientation.y,
                    pose_msg.pose.orientation.z,
                    pose_msg.pose.orientation.w,
                ],
                timestamp_s=float(pose_msg.header.stamp.to_sec()),
                frame_id=pose_msg.header.frame_id,
            )
        except Exception:
            pass
        return LiveSnapshot(
            image_rgb=image_rgb,
            image_topic=self.camera_topic,
            image_frame_id=image_msg.header.frame_id,
            image_timestamp_s=image_timestamp_s,
            image_age_s=image_age_s,
            state=state,
            end_pose=end_pose,
        )

    def read_command_echo(self, timeout_s: float = 0.5) -> Optional[Dict[str, object]]:
        try:
            msg = self.rospy.wait_for_message(DEFAULT_COMMAND_ECHO_TOPIC, self.JointState, timeout=timeout_s)
        except Exception:
            return None
        return {
            "topic": DEFAULT_COMMAND_ECHO_TOPIC,
            "names": list(msg.name),
            "positions": [float(value) for value in msg.position],
            "stamp_s": float(msg.header.stamp.to_sec()) if msg.header.stamp else 0.0,
        }


class RecorderTeleopController:
    def __init__(self, config: PipelineConfig) -> None:
        self.config = config
        self.arm = PiperRosArm(
            physical_motion_enabled=config.physical_motion_enabled,
            named_poses=config.named_poses,
            safety=config.safety,
            expected_joint_names=config.expected_joint_names,
        )

    @property
    def expected_joint_names(self) -> list[str]:
        return list(self.arm.expected_joint_names)

    def execute(self, joint_target_rad, gripper_target_m: float):
        return self.arm.move_to_joint_target(joint_target_rad, gripper_target_m, label="record_demo_step")

    def preview_settings(self) -> dict:
        speed = float(self.arm.safety.get("max_speed_scaling", 0.1))
        acceleration = float(self.arm.safety.get("max_acceleration_scaling", speed))
        return {
            "moveit_service": "/joint_moveit_ctrl_piper",
            "joint_step_rad": float(self.arm.safety.get("demo_joint_step_rad", 0.05)),
            "gripper_step_m": float(self.arm.safety.get("demo_gripper_step_m", 0.005)),
            "max_velocity_scaling": speed,
            "max_acceleration_scaling": acceleration,
        }


class PiperDemoRecorder:
    def __init__(
        self,
        config: PipelineConfig,
        dataset_root: str | Path,
        camera_topic: str = DEFAULT_CAMERA_TOPIC,
    ) -> None:
        self.config = config
        self.writer = EpisodeWriter(dataset_root)
        self.controller = RecorderTeleopController(config)
        self.snapshots = RosSnapshotProvider(camera_topic=camera_topic, expected_joint_names=self.controller.expected_joint_names)
        self.joint_step_rad = float(config.safety.get("demo_joint_step_rad", 0.05))
        self.gripper_step_m = float(config.safety.get("demo_gripper_step_m", 0.005))
        self.max_state_age_s = float(config.safety.get("max_state_age_s", 1.0))
        self.max_image_age_s = float(config.safety.get("max_camera_age_s", 1.0))
        self.gripper_min_m = float(config.safety.get("gripper_min_m", DEFAULT_GRIPPER_MIN_M))
        self.gripper_max_m = float(config.safety.get("gripper_max_m", DEFAULT_GRIPPER_MAX_M))

    def start_episode(self, task_instruction: str) -> EpisodeSession:
        return self.writer.start_episode(task_instruction)

    def record_command(self, session: EpisodeSession, command: str, note: Optional[str] = None) -> PiperFrameRecord:
        if not self.config.physical_motion_enabled:
            raise RuntimeError("physical demo collection is disabled for this configuration")
        snapshot = self.snapshots.read_snapshot(
            state_timeout_s=float(self.config.safety.get("state_read_timeout_s", 2.0)),
            image_timeout_s=float(self.config.safety.get("camera_read_timeout_s", 2.0)),
            max_state_age_s=self.max_state_age_s,
            max_image_age_s=self.max_image_age_s,
        )
        target_joints, target_gripper = build_target_from_command(
            snapshot.state.joint_positions_rad,
            snapshot.state.gripper_value,
            command,
            joint_step_rad=self.joint_step_rad,
            gripper_step_m=self.gripper_step_m,
            gripper_min_m=self.gripper_min_m,
            gripper_max_m=self.gripper_max_m,
        )
        command_timestamp_s = time.time()
        result = self.controller.execute(target_joints, target_gripper)
        if not result.success:
            raise RuntimeError(result.message)
        echo = self.snapshots.read_command_echo(timeout_s=0.5)
        preview = dict(result.outputs.get("moveit_request_preview") or {})
        verification = dict(result.outputs.get("pose_reached_verification") or {})
        preview["execution_allowed"] = bool(self.config.physical_motion_enabled)
        frame = PiperFrameRecord(
            frame_index=len(session.frames),
            task_instruction=session.task_instruction,
            camera_path="",
            camera_topic=snapshot.image_topic,
            camera_frame_id=snapshot.image_frame_id,
            image_timestamp_s=snapshot.image_timestamp_s,
            image_age_s=snapshot.image_age_s,
            state=snapshot.state,
            command=PiperCommandSample(
                command_joint_names=list(self.controller.expected_joint_names),
                target_joint_positions_rad=list(target_joints),
                target_gripper_value=float(target_gripper),
                timestamp_s=command_timestamp_s,
                source="manual_joint_step_teleop_via_moveit_service",
                max_velocity=float(preview.get("max_velocity", 0.0)),
                max_acceleration=float(preview.get("max_acceleration", 0.0)),
                moveit_service=str(preview.get("service", "/joint_moveit_ctrl_piper")),
                execution_mode="physical_execution_verified",
                execution_allowed=bool(self.config.physical_motion_enabled),
                physically_executed=True,
                physical_execution_verified=True,
                service_response_success=bool(result.outputs.get("service_response_success", True)),
                target_reached_verified=bool(verification),
                gripper_result_verified=verification.get("gripper_result_verified"),
                state_fresh_before_command=snapshot.state.age_s <= self.max_state_age_s,
                image_fresh_before_command=snapshot.image_age_s <= self.max_image_age_s,
                moveit_request_preview=preview,
                bridge_command_echo=echo,
            ),
            end_pose=snapshot.end_pose,
            receive_timestamp_s=time.time(),
            notes=note,
        )
        session.append_frame(frame, snapshot.image_rgb)
        return frame


def load_demo_config(path: str | Path) -> PipelineConfig:
    return load_config(path)


def is_live_demo_collection_mode(config: PipelineConfig) -> bool:
    return config.mode == LIVE_DEMO_MODE


def require_interactive_live_demo_session(config: PipelineConfig, config_path: str | Path) -> None:
    if not is_live_demo_collection_mode(config):
        return
    if not (os.isatty(0) and os.isatty(1)):
        raise RuntimeError("live demo collection requires an interactive terminal; refuse noninteractive/background invocation")
    if not config.physical_motion_enabled:
        local_path = Path(config_path).with_name(Path(config_path).stem + ".local" + Path(config_path).suffix)
        raise RuntimeError(
            "live demo collection config requires explicit local activation before any motion: "
            + str(local_path)
        )
