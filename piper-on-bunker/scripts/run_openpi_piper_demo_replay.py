#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from time import monotonic

import numpy as np


PIPER_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PIPER_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from piper_on_bunker.control.piper_joint_phase_executor import CommandAuthorityLock
from piper_on_bunker.hardware.live_openpi_observation import read_live_openpi_observation
from piper_on_bunker.policies.openpi_piper_policy import PIPER_JOINT_NAMES


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _iter_jsonl(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                yield json.loads(line)


def _select_episode(episode_root: Path, phase_id: str) -> Path:
    if (episode_root / "metadata.json").exists():
        metadata = _load_json(episode_root / "metadata.json")
        if metadata.get("phase_id") != phase_id:
            raise SystemExit(f"episode phase_id is {metadata.get('phase_id')!r}, not {phase_id!r}")
        return episode_root
    matches = []
    for path in sorted(episode_root.iterdir()):
        if not path.is_dir() or not (path / "metadata.json").exists():
            continue
        metadata = _load_json(path / "metadata.json")
        if metadata.get("phase_id") == phase_id:
            matches.append(path)
    if not matches:
        raise SystemExit(f"no recorded episode for phase {phase_id!r} under {episode_root}")
    if len(matches) > 1:
        names = ", ".join(path.name for path in matches)
        raise SystemExit(f"multiple episodes match phase {phase_id!r}; pass one episode directory explicitly: {names}")
    return matches[0]


def _load_actions(episode_dir: Path, *, max_samples: int | None, stride: int) -> tuple[np.ndarray, dict, dict]:
    metadata = _load_json(episode_dir / "metadata.json")
    summary = _load_json(episode_dir / "summary.json")
    if metadata.get("joint_order") != list(PIPER_JOINT_NAMES):
        raise SystemExit(f"{episode_dir}: joint_order does not match PiPER direct joint schema")
    rows = []
    for row in _iter_jsonl(episode_dir / "frames.jsonl"):
        if row.get("valid") is True:
            rows.append(row)
    if not rows:
        raise SystemExit(f"{episode_dir}: no valid recorded frames")
    rows = rows[:: max(1, int(stride))]
    if max_samples is not None:
        rows = rows[: max(2, int(max_samples))]
    actions = np.asarray([row["action"] for row in rows], dtype=np.float64)
    if actions.ndim != 2 or actions.shape[1] != 7 or len(actions) < 2:
        raise SystemExit(f"{episode_dir}: expected at least two 7D actions, got shape {actions.shape}")
    if not np.all(np.isfinite(actions)):
        raise SystemExit(f"{episode_dir}: recorded actions contain non-finite values")
    return actions, metadata, summary


def _make_publisher(topic: str):
    try:
        import rospy
        from sensor_msgs.msg import JointState
        from piper_msgs.srv import Enable
    except Exception as exc:
        raise RuntimeError("demo replay requires rospy, sensor_msgs, and piper_msgs") from exc
    if not rospy.get_node_uri():
        rospy.init_node("openpi_piper_demo_replay", anonymous=True, disable_signals=True)
    publisher = rospy.Publisher(topic, JointState, queue_size=1)

    def enable_arm():
        rospy.wait_for_service("/enable_srv", timeout=5.0)
        response = rospy.ServiceProxy("/enable_srv", Enable)(True)
        return bool(response.enable_response)

    def publish(sample):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(PIPER_JOINT_NAMES[:6])
        msg.position = [float(value) for value in sample[:6]]
        publisher.publish(msg)

    return rospy, enable_arm, publish


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Replay recorded PiPER OpenPI demo actions through the direct /piper_joint_commands path."
    )
    parser.add_argument("--episode-root", default="piper-on-bunker/data/local/openpi_episodes")
    parser.add_argument("--phase-id", default="approach")
    parser.add_argument("--command-topic", default="/piper_joint_commands")
    parser.add_argument("--joint-topic", default="/joint_states_single")
    parser.add_argument("--exterior-image-topic", default="/table_camera/color/image_raw")
    parser.add_argument("--no-wrist", action="store_true")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--publish-frequency-hz", type=float, default=50.0)
    parser.add_argument("--max-samples", type=int, default=120, help="Limit replay length. Use 0 for the full episode.")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--max-start-delta-rad", type=float, default=0.40)
    parser.add_argument("--lock-path", default="/tmp/piper_openpi_command_authority.lock")
    parser.add_argument("--execute", action="store_true", help="Required to publish recorded commands.")
    args = parser.parse_args()

    if args.publish_frequency_hz <= 0:
        raise SystemExit("--publish-frequency-hz must be positive")
    episode_dir = _select_episode(Path(args.episode_root), args.phase_id)
    max_samples = None if args.max_samples == 0 else args.max_samples
    actions, metadata, summary = _load_actions(episode_dir, max_samples=max_samples, stride=args.stride)

    live = read_live_openpi_observation(
        timeout_s=args.timeout,
        joint_topic=args.joint_topic,
        exterior_image_topic=args.exterior_image_topic,
        wrist_image_topic=None if args.no_wrist else "/cam_left_wrist",
    )
    start_delta = float(np.max(np.abs(actions[0, :6] - live.state[:6])))
    report = {
        "mode": "recorded_demo_replay_not_pi05",
        "episode_dir": str(episode_dir),
        "episode_id": metadata.get("episode_id"),
        "phase_id": metadata.get("phase_id"),
        "phase_prompt": metadata.get("phase_prompt"),
        "summary": summary,
        "loaded_action_samples": int(actions.shape[0]),
        "publish_frequency_hz": float(args.publish_frequency_hz),
        "estimated_duration_s": float((actions.shape[0] - 1) / args.publish_frequency_hz),
        "live_observation": live.to_summary(),
        "max_start_delta_rad": start_delta,
        "start_delta_allowed_rad": float(args.max_start_delta_rad),
        "execution_allowed": False,
        "published_commands": 0,
        "physical_motion_performed": False,
    }

    if start_delta > args.max_start_delta_rad:
        report["reason"] = (
            f"current arm is too far from the recorded phase start: {start_delta:.6f} rad > "
            f"{args.max_start_delta_rad:.6f} rad"
        )
        print(json.dumps(report, indent=2, sort_keys=True))
        return 2

    if not args.execute:
        report["reason"] = "preflight only; pass --execute to replay recorded commands"
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0

    rospy, enable_arm, publish = _make_publisher(args.command_topic)
    enable_ok = enable_arm()
    if not enable_ok:
        raise SystemExit("/enable_srv did not return enable_response=True")
    rate = rospy.Rate(args.publish_frequency_hz)
    published = 0
    start = monotonic()
    with CommandAuthorityLock(args.lock_path):
        for sample in actions:
            if rospy.is_shutdown():
                break
            if not all(math.isfinite(float(value)) for value in sample[:6]):
                raise SystemExit("non-finite recorded command encountered during replay")
            publish(sample)
            published += 1
            rate.sleep()

    report["execution_allowed"] = True
    report["published_commands"] = published
    report["physical_motion_performed"] = bool(published)
    report["actual_duration_s"] = monotonic() - start
    report["enable_response"] = enable_ok
    report["reason"] = "recorded demo commands replayed through direct PiPER joint publisher"
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
