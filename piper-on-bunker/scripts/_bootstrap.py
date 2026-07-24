from __future__ import annotations

import sys
from pathlib import Path


def add_repo_src_to_syspath() -> None:
    script_dir = Path(__file__).resolve().parent
    repo_src = script_dir.parent / "src"
    repo_src_str = str(repo_src)
    if repo_src_str not in sys.path:
        sys.path.insert(0, repo_src_str)
