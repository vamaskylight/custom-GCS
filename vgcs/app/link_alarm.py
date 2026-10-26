"""The alarm sound while the link to a drone is lost (2026-10-21).

The client's certification extract (DGCA, end of clause 6) asks for a sound and a
screen alert to the remote pilot on link loss. The screen alert was there: the
red header with the drone's last known position. This is the sound.

A drone counts as lost while its link is lost (it was up, and nothing came for
the link loss time), or after its link closed by itself (not by Disconnect).
The fleet says which drones that is (Fleet.lost_links). The alarm repeats while
a drone is lost and the pilot has not silenced it. Silence stops it for the
drones lost now; a drone that is lost again later sounds again. The alarm stops
by itself when every link is back, or when the lost drones are disconnected.

The sound is a short two-tone alarm, played by Windows every 2 seconds. On the
off-screen platform of the tests, off Windows, and with VGCS_NO_SOUND set,
nothing plays: the player only counts.
"""

from __future__ import annotations

import io
import math
import os
import struct
import sys
import tempfile
import wave
from pathlib import Path

from PySide6.QtCore import QObject, QTimer, Signal

# The alarm plays again every REPEAT_MS while it sounds (the sound itself is about 1 s).
REPEAT_MS = 2000
SAMPLE_RATE = 22050
# High, then lower, twice: a sound that is not one of Windows' own.
_TONES = ((988.0, 0.22), (0.0, 0.04), (740.0, 0.22), (0.0, 0.08), (988.0, 0.22), (0.0, 0.04), (740.0, 0.22))
_LEVEL = 0.55        # of full scale
_FADE_S = 0.008      # at both ends of each tone, against clicks
WAV_NAME = "vgcs-link-lost-alarm.wav"


def alarm_wav_bytes() -> bytes:
    """The alarm as a WAV file: 16-bit mono PCM. The same bytes every time."""
    frames = bytearray()
    for freq, seconds in _TONES:
        count = int(round(seconds * SAMPLE_RATE))
        fade = max(1, int(_FADE_S * SAMPLE_RATE))
        for i in range(count):
            if freq <= 0.0:
                value = 0.0
            else:
                ramp = min(1.0, i / fade, (count - 1 - i) / fade)
                value = _LEVEL * ramp * math.sin(2.0 * math.pi * freq * i / SAMPLE_RATE)
            frames += struct.pack("<h", int(round(value * 32767)))
    out = io.BytesIO()
    with wave.open(out, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(bytes(frames))
    return out.getvalue()


class SilentPlayer:
    """Plays nothing, and counts: for the tests, and where no sound can play."""

    def __init__(self) -> None:
        self.playing = False
        self.starts = 0
        self.stops = 0

    def start(self) -> None:
        self.playing = True
        self.starts += 1

    def stop(self) -> None:
        if self.playing:
            self.stops += 1
        self.playing = False


class WindowsPlayer(QObject):
    """Plays the alarm with Windows' own sound player, again every REPEAT_MS while it sounds.

    Played again each time rather than in a loop, so that another sound (Windows plays
    one sound at a time) cannot end the alarm for good.
    """

    def __init__(self, parent: QObject | None = None, *, sound_module=None, folder: Path | None = None) -> None:
        super().__init__(parent)
        self._sound = sound_module
        self._folder = Path(folder) if folder is not None else Path(tempfile.gettempdir())
        self._path: Path | None = None
        self.playing = False
        self._timer = QTimer(self)
        self._timer.setInterval(REPEAT_MS)
        self._timer.timeout.connect(self._play_once)

    def _module(self):
        if self._sound is None:
            import winsound  # Windows only; this player is made only there

            self._sound = winsound
        return self._sound

    def _wav_path(self) -> Path:
        if self._path is None:
            data = alarm_wav_bytes()
            path = self._folder / WAV_NAME
            try:
                if not path.is_file() or path.read_bytes() != data:
                    path.write_bytes(data)
            except OSError:
                pass
            self._path = path
        return self._path

    def _play_once(self) -> None:
        try:
            s = self._module()
            s.PlaySound(str(self._wav_path()), s.SND_FILENAME | s.SND_ASYNC | s.SND_NODEFAULT)
        except Exception:
            pass

    def start(self) -> None:
        if self.playing:
            return
        self.playing = True
        self._play_once()
        self._timer.start()

    def stop(self) -> None:
        if not self.playing:
            return
        self.playing = False
        self._timer.stop()
        try:
            self._module().PlaySound(None, 0)   # stops the sound that plays now
        except Exception:
            pass


def make_player(parent: QObject | None = None):
    """The player for this computer: Windows' sound, or a silent one (see the module text)."""
    try:
        from PySide6.QtGui import QGuiApplication

        offscreen = QGuiApplication.platformName() == "offscreen"
    except Exception:
        offscreen = False
    if sys.platform != "win32" or offscreen or os.environ.get("VGCS_NO_SOUND"):
        return SilentPlayer()
    return WindowsPlayer(parent)


class LinkAlarm(QObject):
    """Whether the alarm sounds, from the drones whose link is lost. See the module text."""

    changed = Signal()

    def __init__(self, player, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.player = player
        self._lost: dict[str, str] = {}      # vid: name
        self._silenced: set[str] = set()
        self._sounding = False

    @property
    def sounding(self) -> bool:
        return self._sounding

    def lost_names(self) -> list[str]:
        return list(self._lost.values())

    def update(self, lost: dict[str, str]) -> None:
        """The drones whose link is lost now ({vid: name})."""
        lost = {str(k): str(v) for k, v in dict(lost or {}).items()}
        # A drone that is back (or gone) is forgotten: lost again later, it sounds again.
        self._silenced &= set(lost)
        sound = bool(set(lost) - self._silenced)
        before = (self._sounding, self._lost)
        self._lost = lost
        if sound and not self._sounding:
            self._sounding = True
            self.player.start()
        elif not sound and self._sounding:
            self._sounding = False
            self.player.stop()
        if before != (self._sounding, self._lost):
            self.changed.emit()

    def silence(self) -> None:
        """The pilot heard it: no sound for the drones lost now."""
        self._silenced |= set(self._lost)
        self.update(self._lost)

    def stop(self) -> None:
        """No sound at all (VGCS is closing)."""
        self.update({})
