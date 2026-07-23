import shutil
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[2]
out = root / "deployment" / "piper-pipeline-testbed.zip"
out.parent.mkdir(parents=True, exist_ok=True)
if out.exists():
    out.unlink()
shutil.make_archive(str(out.with_suffix("")), "zip", root, ".")
sys.stdout.buffer.write((str(out) + "\n").encode("utf-8", errors="backslashreplace"))
