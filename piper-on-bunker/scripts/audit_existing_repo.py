from pathlib import Path

paths = [
    Path(r"C:\Users\user\ABot-Claw"),
    Path(r"C:\Users\user\.codex\reference\ABot-Claw-piper"),
]
for path in paths:
    print(f"{path}: {'found' if path.exists() else 'missing'}")
print("Verified GitHub reference during project creation: iliyas-tleuzhan/ABot-Claw-piper, visibility PUBLIC, default branch main.")
