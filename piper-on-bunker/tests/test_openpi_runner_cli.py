import subprocess
import sys
from pathlib import Path


def test_live_shadow_accepts_phase_id_help():
    script = Path("piper-on-bunker/scripts/run_openpi_piper_live_shadow.py")
    result = subprocess.run([sys.executable, str(script), "--help"], check=True, text=True, stdout=subprocess.PIPE)
    assert "--phase-id" in result.stdout


def test_guarded_physical_preflight_does_not_require_checkpoint_metadata_help():
    script = Path("piper-on-bunker/scripts/run_openpi_piper_guarded_physical.py")
    result = subprocess.run([sys.executable, str(script), "--help"], check=True, text=True, stdout=subprocess.PIPE)
    assert "--checkpoint-metadata CHECKPOINT_METADATA" in result.stdout
    assert "--no-wrist" in result.stdout
