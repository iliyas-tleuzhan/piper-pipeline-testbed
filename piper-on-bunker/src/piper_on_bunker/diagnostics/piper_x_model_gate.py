"""Fail-closed PiPER-X model and URDF selection helpers."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
import re


STANDARD_PIPER_NEW_URDF = "piper_description.urdf"
STANDARD_PIPER_OLD_URDF = "piper_description_old.urdf"
PIPER_X_URDF_NAME = "piper_x_description.urdf"


@dataclass(frozen=True)
class UrdfSelection:
    selected: bool
    selected_urdf_path: str | None
    selected_urdf_sha256: str | None
    physical_model_id: str | None
    firmware_version: str | None
    fk_verified: bool
    blockers: list[str]

    def to_dict(self) -> dict:
        return {
            "selected": self.selected,
            "selected_urdf_path": self.selected_urdf_path,
            "selected_urdf_sha256": self.selected_urdf_sha256,
            "physical_model_id": self.physical_model_id,
            "firmware_version": self.firmware_version,
            "fk_verified": self.fk_verified,
            "blockers": list(self.blockers),
        }


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def select_urdf_for_calibration(
    *,
    physical_model_id: str | None,
    firmware_version: str | None,
    robot_urdf_path: str | None,
    expected_sha256: str | None = None,
    fk_verified: bool = False,
) -> UrdfSelection:
    """Select a calibration URDF only when the model has already been verified.

    This intentionally does not infer that PiPER-X is compatible with the normal
    PiPER URDF. The calibration launcher must not publish FK for an unresolved
    model because that directly contaminates hand-eye samples.
    """

    model = (physical_model_id or "").strip() or None
    firmware = (firmware_version or "").strip() or None
    path_text = (robot_urdf_path or "").strip() or None
    blockers: list[str] = []

    if not model:
        blockers.append("physical model ID is unresolved")
    if not firmware:
        blockers.append("firmware version is unresolved")
    if not path_text:
        blockers.append("ROBOT_URDF_PATH is required; do not use a blind default")

    path = Path(path_text).expanduser() if path_text else None
    actual_hash = None
    if path is not None:
        if not path.exists():
            blockers.append(f"selected URDF does not exist: {path}")
        elif not path.is_file():
            blockers.append(f"selected URDF is not a file: {path}")
        else:
            actual_hash = sha256_file(path)
            if expected_sha256 and actual_hash != expected_sha256:
                blockers.append(f"selected URDF hash mismatch: expected {expected_sha256}, got {actual_hash}")

    if model and "piper_x" in model.lower():
        if path is not None and path.name != PIPER_X_URDF_NAME:
            blockers.append(f"PiPER-X calibration requires an explicit PiPER-X URDF; got {path.name}")
    elif model:
        blockers.append(f"physical model {model!r} is not the PiPER-X calibration target")

    if not fk_verified:
        blockers.append("FK verification has not passed across multiple stopped poses")

    selected = not blockers
    return UrdfSelection(
        selected=selected,
        selected_urdf_path=str(path) if selected and path is not None else None,
        selected_urdf_sha256=actual_hash if selected else None,
        physical_model_id=model,
        firmware_version=firmware,
        fk_verified=bool(fk_verified),
        blockers=blockers,
    )


def official_standard_piper_urdf_for_firmware(firmware_version: str) -> str | None:
    """Return the official normal-PiPER URDF name for a firmware string.

    Official piper_ros guidance: firmware older than S-V1.6-3 uses
    piper_description_old.urdf; firmware S-V1.6-3 or newer uses
    piper_description.urdf. This rule is documented here for auditability but
    is not sufficient to authorize PiPER-X.
    """

    parsed = _parse_s_version(firmware_version)
    if parsed is None:
        return None
    return STANDARD_PIPER_NEW_URDF if parsed >= (1, 6, 3) else STANDARD_PIPER_OLD_URDF


def _parse_s_version(value: str) -> tuple[int, int, int] | None:
    match = re.search(r"S-V(\d+)\.(\d+)-(\d+)", value)
    if not match:
        return None
    return tuple(int(part) for part in match.groups())
