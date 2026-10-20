"""Old recordings deleted so that a folder stays under a size limit.

Application Settings, Video: "Auto-delete saved recordings" and "Max storage
usage (MB)". They did nothing until 2026-10-20. Now, when the first is on, each
saved recording is followed by this clean-up of the folder it was saved in.

Only VGCS's own recordings are touched: files named the way VGCS suggests
(recording_YYYYMMDD_HHMMSS.mp4, .mkv or .mov). The oldest go first, by the date
and time in their names. The newest one and the one just saved are never
deleted, and no other file ever is. 0 MB means no limit.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

# The name VGCS suggests in the Save dialog (vgcs.video.pipeline.suggested_recording_filename).
RECORDING_NAME = re.compile(r"^recording_(\d{8})_(\d{6})\.(mp4|mkv|mov)$", re.IGNORECASE)
BYTES_PER_MB = 1024 * 1024


@dataclass(frozen=True)
class Pruned:
    """What a clean-up did."""

    deleted: tuple[Path, ...] = ()
    failed: tuple[Path, ...] = ()   # could not be deleted (open in a player, read-only)
    total_bytes: int = 0            # VGCS's recordings in the folder afterwards

    def words(self, max_mb: int) -> str:
        """A sentence for the operator, or "" when nothing was deleted or tried."""
        parts = []
        if self.deleted:
            n = len(self.deleted)
            parts.append(f"Deleted {n} old recording{'' if n == 1 else 's'} to stay under {max_mb} MB.")
        if self.failed:
            n = len(self.failed)
            parts.append(f"Could not delete {n} old recording{'' if n == 1 else 's'}: {self.failed[0].name}.")
        return " ".join(parts)


def vgcs_recordings(folder: str | Path) -> list[Path]:
    """VGCS's recordings in a folder, oldest first (by the date and time in their names)."""
    try:
        entries = list(Path(folder).iterdir())
    except OSError:
        return []
    found = []
    for path in entries:
        m = RECORDING_NAME.match(path.name)
        if m is not None and path.is_file():
            found.append((m.group(1) + m.group(2), path.name, path))
    found.sort()
    return [path for _stamp, _name, path in found]


def prune_recordings(folder: str | Path, max_mb: int, *, keep: str | Path | None = None) -> Pruned:
    """Delete VGCS's oldest recordings in `folder` until they fit `max_mb`.

    `keep`: the recording just saved, never deleted (also when its name is not the newest).
    """
    files = vgcs_recordings(folder)
    sizes: dict[Path, int] = {}
    for path in files:
        try:
            sizes[path] = path.stat().st_size
        except OSError:
            sizes[path] = 0
    total = sum(sizes.values())
    if int(max_mb) <= 0 or not files:
        return Pruned(total_bytes=total)
    limit = int(max_mb) * BYTES_PER_MB
    kept_name = Path(keep).name.lower() if keep is not None else None
    deleted: list[Path] = []
    failed: list[Path] = []
    for path in files[:-1]:          # never the newest
        if total <= limit:
            break
        if kept_name is not None and path.name.lower() == kept_name:
            continue
        try:
            path.unlink()
        except FileNotFoundError:
            total -= sizes[path]
            continue
        except OSError:
            failed.append(path)
            continue
        total -= sizes[path]
        deleted.append(path)
    return Pruned(deleted=tuple(deleted), failed=tuple(failed), total_bytes=total)


__all__ = ["BYTES_PER_MB", "Pruned", "RECORDING_NAME", "prune_recordings", "vgcs_recordings"]
