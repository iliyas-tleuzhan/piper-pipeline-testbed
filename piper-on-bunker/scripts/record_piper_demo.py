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
    recorder = PiperDemoRecorder(config=config, dataset_root=args.dataset_root)
    session = None

    print("PiPER X-VLA demonstration recorder")
    print("physical_motion_enabled:", config.physical_motion_enabled)
    print(HELP)

    while True:
        raw = input("> ").strip()
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
