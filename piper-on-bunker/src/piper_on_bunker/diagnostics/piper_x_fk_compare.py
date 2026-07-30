"""Offline FK comparison helpers for PiPER-X hand-eye diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class PoseTransform:
    translation: np.ndarray
    quaternion_xyzw: np.ndarray

    def matrix(self) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_quat(self.quaternion_xyzw).as_matrix()
        transform[:3, 3] = self.translation
        return transform


@dataclass(frozen=True)
class CapturedPose:
    path: str
    joint_states_single: list[float]
    relayed_joint_states: list[float]
    controller_end_pose: PoseTransform
    base_to_gripper: PoseTransform
    endpoint_transforms: dict[str, PoseTransform]
    base_to_marker: PoseTransform | None


_FLOAT = r"[-+]?(?:\d+\.\d*|\d*\.\d+|\d+)(?:[eE][-+]?\d+)?"
_TRANSLATION_RE = re.compile(r"Translation:\s*\[(" + _FLOAT + r"),\s*(" + _FLOAT + r"),\s*(" + _FLOAT + r")\]")
_QUAT_RE = re.compile(
    r"Rotation: in Quaternion\s*\[(" + _FLOAT + r"),\s*(" + _FLOAT + r"),\s*(" + _FLOAT + r"),\s*(" + _FLOAT + r")\]"
)
_POSITION_LIST_RE = re.compile(r"position:\s*\[([^\]]+)\]")


def parse_diagnostic_file(path: str | Path) -> CapturedPose:
    text = Path(path).read_text(encoding="utf-8")
    sections = _split_sections(text)
    joint_states_single = _parse_position_list(_require_section(sections, "JOINT STATES SINGLE"))
    relayed_joint_states = _parse_position_list(_require_section(sections, "RELAYED JOINT STATES"))
    endpoint_transforms = {
        "gripper_base": _parse_tf_section(_require_section(sections, "BASE TO GRIPPER")),
    }
    optional_sections = {
        "link6": "BASE TO LINK6",
        "gripper_tcp": "BASE TO GRIPPER TCP",
    }
    for frame_name, section_name in optional_sections.items():
        if section_name in sections:
            endpoint_transforms[frame_name] = _parse_tf_section(sections[section_name])
    return CapturedPose(
        path=str(path),
        joint_states_single=joint_states_single,
        relayed_joint_states=relayed_joint_states,
        controller_end_pose=_parse_controller_end_pose(_require_section(sections, "CONTROLLER END POSE")),
        base_to_gripper=endpoint_transforms["gripper_base"],
        endpoint_transforms=endpoint_transforms,
        base_to_marker=_parse_optional_tf_section(sections.get("BASE TO MARKER", "")),
    )


def parse_diagnostic_path(path: str | Path) -> CapturedPose:
    path_obj = Path(path)
    if path_obj.suffix.lower() == ".json":
        return parse_capture_json_file(path_obj)
    return parse_diagnostic_file(path_obj)


def parse_capture_json_file(path: str | Path) -> CapturedPose:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    ros = data.get("ros", {})
    endpoint_transforms = {}
    endpoint_keys = {
        "link6": "base_to_link6",
        "gripper_base": "base_to_gripper_base",
        "gripper_tcp": "base_to_gripper_tcp",
    }
    for frame_name, key in endpoint_keys.items():
        stdout = ros.get(key, {}).get("stdout", "")
        try:
            endpoint_transforms[frame_name] = _parse_tf_section(stdout)
        except ValueError:
            pass
    if "gripper_base" not in endpoint_transforms:
        raise ValueError("missing gripper_base TF in JSON diagnostic")
    return CapturedPose(
        path=str(path),
        joint_states_single=_parse_position_list(ros.get("joint_states_single", {}).get("stdout", "")),
        relayed_joint_states=_parse_position_list(ros.get("joint_states", {}).get("stdout", "")),
        controller_end_pose=_parse_controller_end_pose(ros.get("end_pose", {}).get("stdout", "")),
        base_to_gripper=endpoint_transforms["gripper_base"],
        endpoint_transforms=endpoint_transforms,
        base_to_marker=None,
    )


def analyze_captures(captures: Iterable[CapturedPose], residual_warn_translation_m: float = 0.02, residual_warn_angle_deg: float = 5.0) -> dict:
    poses = list(captures)
    per_pose = []
    candidate_deltas: dict[str, list[np.ndarray]] = {}
    candidate_errors: dict[str, list[tuple[float, float]]] = {}
    marker_positions = []
    for pose in poses:
        controller = pose.controller_end_pose.matrix()
        candidate_report = {}
        for frame_name, endpoint in pose.endpoint_transforms.items():
            endpoint_matrix = endpoint.matrix()
            delta = np.linalg.inv(endpoint_matrix) @ controller
            candidate_deltas.setdefault(frame_name, []).append(delta)
            translation_error = float(np.linalg.norm(pose.controller_end_pose.translation - endpoint.translation))
            angular_error = _angular_distance_deg(pose.controller_end_pose.quaternion_xyzw, endpoint.quaternion_xyzw)
            candidate_errors.setdefault(frame_name, []).append((translation_error, angular_error))
            candidate_report[frame_name] = {
                "translation_error_m": translation_error,
                "angular_error_deg": angular_error,
                "urdf_position_m": endpoint.translation.tolist(),
                "endpoint_to_controller_delta_translation_m": delta[:3, 3].tolist(),
                "endpoint_to_controller_delta_quaternion_xyzw": Rotation.from_matrix(delta[:3, :3]).as_quat().tolist(),
            }
        gripper_error = candidate_report["gripper_base"]
        joint_copy_exact = np.allclose(pose.joint_states_single[:6], pose.relayed_joint_states[:6], atol=0.0, rtol=0.0)
        if pose.base_to_marker is not None:
            marker_positions.append(pose.base_to_marker.translation)
        per_pose.append(
            {
                "path": pose.path,
                "joint_copy_exact_first_6": bool(joint_copy_exact),
                "controller_vs_urdf_gripper_translation_error_m": gripper_error["translation_error_m"],
                "controller_vs_urdf_gripper_angular_error_deg": gripper_error["angular_error_deg"],
                "controller_position_m": pose.controller_end_pose.translation.tolist(),
                "urdf_base_to_gripper_position_m": pose.base_to_gripper.translation.tolist(),
                "base_to_marker_position_m": None if pose.base_to_marker is None else pose.base_to_marker.translation.tolist(),
                "gripper_to_controller_delta_translation_m": gripper_error["endpoint_to_controller_delta_translation_m"],
                "gripper_to_controller_delta_quaternion_xyzw": gripper_error["endpoint_to_controller_delta_quaternion_xyzw"],
                "candidate_endpoint_mappings": candidate_report,
            }
        )

    candidate_mapping_summary = {}
    for frame_name, deltas in sorted(candidate_deltas.items()):
        residuals = _constant_transform_residuals(deltas)
        errors = candidate_errors[frame_name]
        candidate_mapping_summary[frame_name] = {
            "pose_count": len(deltas),
            "mean_translation_error_m": float(np.mean([item[0] for item in errors])),
            "max_translation_error_m": float(np.max([item[0] for item in errors])),
            "mean_angular_error_deg": float(np.mean([item[1] for item in errors])),
            "max_angular_error_deg": float(np.max([item[1] for item in errors])),
            "constant_endpoint_to_controller_transform": {
                "verified": bool(
                    residuals["max_translation_residual_m"] <= residual_warn_translation_m
                    and residuals["max_angular_residual_deg"] <= residual_warn_angle_deg
                ),
                "max_translation_residual_m": residuals["max_translation_residual_m"],
                "max_angular_residual_deg": residuals["max_angular_residual_deg"],
                "mean_delta_translation_m": residuals["mean_translation"].tolist(),
                "mean_delta_quaternion_xyzw": residuals["mean_quaternion_xyzw"].tolist(),
                "translation_threshold_m": residual_warn_translation_m,
                "angular_threshold_deg": residual_warn_angle_deg,
            },
        }
    residuals = candidate_mapping_summary["gripper_base"]["constant_endpoint_to_controller_transform"]
    marker_displacement = None
    if len(marker_positions) >= 2:
        marker_displacement = float(max(np.linalg.norm(a - b) for a in marker_positions for b in marker_positions))
    constant_ok = (
        residuals["max_translation_residual_m"] <= residual_warn_translation_m
        and residuals["max_angular_residual_deg"] <= residual_warn_angle_deg
    )
    return {
        "pose_count": len(poses),
        "per_pose": per_pose,
        "all_first_six_joints_copied_exactly": all(item["joint_copy_exact_first_6"] for item in per_pose),
        "constant_gripper_to_controller_transform": {
            "verified": bool(constant_ok),
            "max_translation_residual_m": residuals["max_translation_residual_m"],
            "max_angular_residual_deg": residuals["max_angular_residual_deg"],
            "mean_delta_translation_m": residuals["mean_delta_translation_m"],
            "mean_delta_quaternion_xyzw": residuals["mean_delta_quaternion_xyzw"],
            "translation_threshold_m": residual_warn_translation_m,
            "angular_threshold_deg": residual_warn_angle_deg,
        },
        "candidate_endpoint_mappings": candidate_mapping_summary,
        "fixed_marker_false_motion": {
            "visible_pose_count": len(marker_positions),
            "max_pairwise_displacement_m": marker_displacement,
        },
        "fk_verified": False,
        "rejection_reason": (
            "controller /end_pose and URDF TF disagree in a configuration-dependent way; "
            "a single constant TCP transform does not explain the captured poses"
            if not constant_ok
            else "controller/URDF constant transform residuals are small, but physical model and SDK FK remain unverified"
        ),
    }


def _split_sections(text: str) -> dict[str, str]:
    matches = list(re.finditer(r"^=====\s+(.+?)\s+=====\s*$", text, re.MULTILINE))
    sections: dict[str, str] = {}
    for index, match in enumerate(matches):
        name = match.group(1).strip()
        start = match.end()
        end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
        sections[name] = text[start:end]
    return sections


def _require_section(sections: dict[str, str], name: str) -> str:
    if name not in sections:
        raise ValueError(f"missing section {name!r}")
    return sections[name]


def _parse_position_list(section: str) -> list[float]:
    match = _POSITION_LIST_RE.search(section)
    if not match:
        raise ValueError("missing position list")
    return [float(value.strip()) for value in match.group(1).split(",")]


def _parse_controller_end_pose(section: str) -> PoseTransform:
    position_match = re.search(r"position:\s*(.*?)orientation:", section, re.DOTALL)
    orientation_match = re.search(r"orientation:\s*(.*)$", section, re.DOTALL)
    if not position_match or not orientation_match:
        raise ValueError("missing controller position/orientation")

    def scalar(block: str, name: str) -> float:
        match = re.search(rf"\b{name}:\s*({_FLOAT})", block)
        if not match:
            raise ValueError(f"missing controller scalar {name}")
        return float(match.group(1))

    return PoseTransform(
        translation=np.asarray([scalar(position_match.group(1), "x"), scalar(position_match.group(1), "y"), scalar(position_match.group(1), "z")], dtype=float),
        quaternion_xyzw=np.asarray(
            [
                scalar(orientation_match.group(1), "x"),
                scalar(orientation_match.group(1), "y"),
                scalar(orientation_match.group(1), "z"),
                scalar(orientation_match.group(1), "w"),
            ],
            dtype=float,
        ),
    )


def _parse_tf_section(section: str) -> PoseTransform:
    translation_match = _TRANSLATION_RE.search(section)
    quat_match = _QUAT_RE.search(section)
    if not translation_match or not quat_match:
        raise ValueError("missing TF transform")
    return PoseTransform(
        translation=np.asarray([float(translation_match.group(i)) for i in range(1, 4)], dtype=float),
        quaternion_xyzw=np.asarray([float(quat_match.group(i)) for i in range(1, 5)], dtype=float),
    )


def _parse_optional_tf_section(section: str) -> PoseTransform | None:
    try:
        return _parse_tf_section(section)
    except ValueError:
        return None


def _angular_distance_deg(q1: np.ndarray, q2: np.ndarray) -> float:
    r1 = Rotation.from_quat(q1)
    r2 = Rotation.from_quat(q2)
    return float((r1.inv() * r2).magnitude() * 180.0 / math.pi)


def _constant_transform_residuals(deltas: list[np.ndarray]) -> dict:
    if not deltas:
        return {
            "max_translation_residual_m": math.inf,
            "max_angular_residual_deg": math.inf,
            "mean_translation": np.zeros(3),
            "mean_quaternion_xyzw": np.asarray([0.0, 0.0, 0.0, 1.0]),
        }
    translations = np.asarray([delta[:3, 3] for delta in deltas], dtype=float)
    rotations = Rotation.from_matrix([delta[:3, :3] for delta in deltas])
    mean_translation = translations.mean(axis=0)
    mean_rotation = rotations.mean()
    translation_residuals = np.linalg.norm(translations - mean_translation, axis=1)
    angular_residuals = [(mean_rotation.inv() * rotation).magnitude() * 180.0 / math.pi for rotation in rotations]
    return {
        "max_translation_residual_m": float(np.max(translation_residuals)),
        "max_angular_residual_deg": float(np.max(angular_residuals)),
        "mean_translation": mean_translation,
        "mean_quaternion_xyzw": mean_rotation.as_quat(),
    }
