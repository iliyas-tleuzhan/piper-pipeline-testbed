from __future__ import annotations

import argparse
from pathlib import Path
import sys

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from _bootstrap import add_repo_src_to_syspath
else:
    from ._bootstrap import add_repo_src_to_syspath

add_repo_src_to_syspath()

from piper_on_bunker.data.piper_demo_recorder import PiperDemoRecorder, load_demo_config
from piper_on_bunker.data.piper_demo_recorder import require_interactive_live_demo_session


HELP = """
Commands:
  start               start a new episode
  status              print current frame count
  j1+ / j1- ... j6+   send one bounded joint-step command and record it
  g+ / g-             send one bounded gripper-step command and record it
  note TEXT           attach a note to the active episode
  save                save the active episode as success=true
  abort               discard the active episode
  reset               discard the active episode and immediately start a new one
  quit                exit without saving an active episode
""".strip()


def main() -> None:
    parser = argparse.ArgumentParser(description="Manual PiPER demonstration recorder for the first X-VLA dataset.")
    parser.add_argument("--config", required=True)
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--task", default="Move the gripper toward the marked target.")
    args = parser.parse_args()

    config = load_demo_config(args.config)
    require_interactive_live_demo_session(config, args.config)
    recorder = PiperDemoRecorder(config=config, dataset_root=args.dataset_root)
    session = None

    print("PiPER X-VLA demonstration recorder")
    print("configured_physical_motion_enabled:", config.configured_physical_motion_enabled)
    print("physical_motion_enabled:", config.physical_motion_enabled)
    if config.mode == "piper_demo_collection":
        settings = recorder.controller.preview_settings()
        print("WARNING: PHYSICAL MOTION IS ENABLED FOR LIVE DEMONSTRATION COLLECTION")
        print("MoveIt service:", settings["moveit_service"])
        print("joint_step_rad:", settings["joint_step_rad"])
        print("gripper_step_m:", settings["gripper_step_m"])
        print("max_velocity_scaling:", settings["max_velocity_scaling"])
        print("max_acceleration_scaling:", settings["max_acceleration_scaling"])
        print("dataset_root:", args.dataset_root)
        snapshot = recorder.snapshots.read_snapshot(
            state_timeout_s=float(config.safety.get("state_read_timeout_s", 2.0)),
            image_timeout_s=float(config.safety.get("camera_read_timeout_s", 2.0)),
            max_state_age_s=float(config.safety.get("max_state_age_s", 1.0)),
            max_image_age_s=float(config.safety.get("max_camera_age_s", 1.0)),
        )
        print(
            "live_state_check:",
            {
                "joint_state_age_s": snapshot.state.age_s,
                "image_age_s": snapshot.image_age_s,
                "camera_topic": snapshot.image_topic,
                "joint_names": snapshot.state.joint_names,
            },
        )
        checklist = [
            "Confirm the Bunker is immobilized [y/N]: ",
            "Confirm the workspace is clear [y/N]: ",
            "Confirm the emergency stop is reachable [y/N]: ",
            "Confirm the camera and joint state are live [y/N]: ",
        ]
        for prompt in checklist:
            answer = input(prompt).strip().lower()
            if answer not in {"y", "yes"}:
                raise SystemExit("Live demonstration collection aborted before any motion.")
    print(HELP)

    while True:
        try:
            raw = input("> ").strip()
        except EOFError:
            print("stdin closed; exiting without sending any robot command")
            if session and session.frames:
                session.discard()
            break
        if not raw:
            continue
        if raw == "quit":
            if session and session.frames:
                print("unsaved episode discarded")
                session.discard()
            break
        if raw == "start":
            if session is not None:
                print("episode already active")
                continue
            session = recorder.start_episode(args.task)
            print(f"started episode_{session.episode_index:04d}")
            continue
        if raw == "status":
            if session is None:
                print("no active episode")
            else:
                print(f"episode_{session.episode_index:04d} frames={len(session.frames)}")
            continue
        if raw.startswith("note "):
            if session is None:
                print("no active episode")
            else:
                session.add_note(raw[5:].strip())
                print("note added")
            continue
        if raw == "save":
            if session is None:
                print("no active episode")
                continue
            path = session.save(success=True, software={"config_mode": config.mode})
            print(f"saved {path}")
            session = None
            continue
        if raw == "abort":
            if session is None:
                print("no active episode")
                continue
            session.discard()
            print("episode discarded")
            session = None
            continue
        if raw == "reset":
            if session is not None:
                session.discard()
            session = recorder.start_episode(args.task)
            print(f"reset to episode_{session.episode_index:04d}")
            continue
        if session is None:
            print("start an episode first")
            continue
        try:
            frame = recorder.record_command(session, raw)
            print(
                f"recorded frame {frame.frame_index} "
                f"target={frame.command.target_joint_positions_rad + [frame.command.target_gripper_value]}"
            )
        except Exception as exc:
            print(f"command failed: {exc!r}")


if __name__ == "__main__":
    main()
