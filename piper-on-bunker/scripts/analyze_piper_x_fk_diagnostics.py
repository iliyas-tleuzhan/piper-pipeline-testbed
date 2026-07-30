#!/usr/bin/env python3
"""Analyze captured PiPER-X FK mismatch diagnostics without touching hardware."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from piper_on_bunker.diagnostics.piper_x_fk_compare import analyze_captures
from piper_on_bunker.diagnostics.piper_x_fk_compare import parse_diagnostic_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", help="Captured pose diagnostic text files")
    parser.add_argument("--output-json", help="Optional path for machine-readable report")
    args = parser.parse_args()

    ordered_paths = sorted((Path(path) for path in args.paths), key=lambda path: (path.stat().st_mtime_ns, str(path)))
    captures = [parse_diagnostic_path(path) for path in ordered_paths]
    report = analyze_captures(captures)
    text = json.dumps(report, indent=2, sort_keys=True)
    print(text)
    if args.output_json:
        Path(args.output_json).write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
