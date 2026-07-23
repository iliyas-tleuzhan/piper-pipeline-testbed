from pathlib import Path
import sys


def safe_print(text: str) -> None:
    sys.stdout.buffer.write((text + "\n").encode("utf-8", errors="backslashreplace"))

names = [
    r"C:\Users\user\ABot-Claw",
    r"C:\Users\user\.codex\reference\ABot-Claw-piper",
    r"C:\Users\user\OneDrive\Документы\upstream_questVR_ws\src\Piper_ros",
]
for name in names:
    path = Path(name)
    safe_print(f"{name}: {'found' if path.exists() else 'missing'}")
