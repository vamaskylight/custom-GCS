"""MAVLink 2 command signing (milestone M16, "secure command transmission").

With signing, every packet VGCS sends carries a signature made with a 256-bit
secret key (SHA-256, truncated to 48 bits, with a timestamp against replays).
A drone that has been given the key drops every unsigned or wrongly signed
command on its radio links. So only a ground station that holds the key can
arm it, change its mode, upload a mission or change a setting. Telemetry still
reaches everyone in radio range: signing proves who sent a command, it does not
hide anything. Encrypting the link is the radio's job (see DOCS/M16-SECURITY.md).

What ArduPilot does (checked in its source, GCS_Signing.cpp, 4.6 and 4.7):

- It takes a key from SETUP_SIGNING at any time while disarmed ("ERROR: Won't
  setup signing when armed" otherwise), keeps it in its own storage, and from
  then on signs its own packets on every link.
- It drops unsigned packets on every link except channel 0 (USB on a flight
  controller; port 5760 in the simulator) and RADIO_STATUS from the radio.
  So a key can always be changed or removed over USB.
- A key of all zeros with timestamp 0 switches signing off again. Once signing
  is on, that message itself must be signed with the current key (or come over USB).

The key comes from a passphrase exactly the way QGroundControl 5.1.5 makes it
(PBKDF2-HMAC-SHA256, its fixed salt, 600,000 rounds), so the same passphrase
gives the same key in VGCS and in the VAMA APK (QGC's MAVLink signing keys).

Pure Python, no Qt and no pymavlink, so it is tested without a link.
"""

from __future__ import annotations

import hashlib
import time
from typing import Callable

KEY_BYTES = 32
# QGroundControl's derivation (apk/qgc-src/src/MAVLink/Signing/MAVLinkSigningKeys.h).
PBKDF2_SALT = b"QGroundControl-MAVLink-Signing-v1"
PBKDF2_ITERATIONS = 600_000
MIN_PASSPHRASE = 8

# MAVLink signing timestamps count 10 microsecond steps since 1 January 2015.
_EPOCH_2015 = 1420070400

# Drone signing states, from the heartbeats it sends.
OFF = "off"                        # no key on this laptop: VGCS does not sign
WAITING = "waiting"                # key here, nothing heard from the drone yet
DRONE_SIGNS = "drone_signs"        # the drone signs with this key: commands are protected
DRONE_UNSIGNED = "drone_unsigned"  # the drone does not sign: it has no key, anyone can command it
KEY_MISMATCH = "key_mismatch"      # the drone signs with another key: it ignores VGCS's commands
MAVLINK1 = "mavlink1"              # the link speaks MAVLink 1, which cannot be signed

# A drone that changes key or stops signing shows it within a heartbeat or
# two. A single odd packet is not a state change.
_AGREE_COUNT = 2


def key_from_passphrase(passphrase: str) -> bytes:
    """The signing key for a passphrase, the same as in QGroundControl."""
    text = str(passphrase or "")
    if len(text) < MIN_PASSPHRASE:
        raise ValueError(f"the passphrase needs at least {MIN_PASSPHRASE} characters")
    return hashlib.pbkdf2_hmac("sha256", text.encode("utf-8"), PBKDF2_SALT, PBKDF2_ITERATIONS, KEY_BYTES)


def key_from_hex(text: str) -> bytes:
    """A key typed or pasted as 64 hexadecimal characters (QGroundControl's raw key)."""
    clean = "".join(str(text or "").split()).replace("-", "")
    try:
        key = bytes.fromhex(clean)
    except ValueError as e:
        raise ValueError("a raw key is 64 hexadecimal characters (0-9, a-f)") from e
    if len(key) != KEY_BYTES:
        raise ValueError(f"a raw key is 64 hexadecimal characters, this one has {len(clean)}")
    return key


def fingerprint(key: bytes) -> str:
    """A short name for a key, to compare two laptops without showing the key itself."""
    digest = hashlib.sha256(b"VGCS key fingerprint" + bytes(key)).hexdigest().upper()
    return f"{digest[0:4]}-{digest[4:8]}"


def timestamp_now(clock: Callable[[], float] = time.time) -> int:
    """Signing timestamp for now: 10 microsecond steps since 1 January 2015."""
    return int(max(0.0, clock() - _EPOCH_2015) * 100_000)


def is_zero_key(key: bytes | None) -> bool:
    return key is None or not any(key)


class SigningMonitor:
    """Whether the drone signs, and with which key, read from its heartbeats.

    note(present, valid): present means the packet carried a signature, valid
    that it checked out with this laptop's key.
    """

    def __init__(self) -> None:
        self.key_set = False
        self.state = OFF
        self._candidate = OFF
        self._count = 0

    def set_key(self, key_set: bool) -> None:
        self.key_set = bool(key_set)
        self.state = WAITING if self.key_set else OFF
        self._candidate = self.state
        self._count = 0

    def note(self, present: bool, valid: bool) -> bool:
        """One heartbeat from the drone. True when the state changed."""
        if not self.key_set:
            seen = OFF
        elif valid:
            seen = DRONE_SIGNS
        elif present:
            seen = KEY_MISMATCH
        else:
            seen = DRONE_UNSIGNED
        if seen == self.state:
            self._candidate, self._count = seen, 0
            return False
        if seen != self._candidate:
            self._candidate, self._count = seen, 0
        self._count += 1
        if self._count >= _AGREE_COUNT or self.state in (WAITING, OFF):
            self.state = seen
            self._candidate, self._count = seen, 0
            return True
        return False

    def mavlink1(self) -> None:
        if self.key_set:
            self.state = MAVLINK1


def state_text(state: str, key_fp: str = "") -> str:
    """The signing state in words, for the dashboard and the settings."""
    key = f" (key {key_fp})" if key_fp else ""
    return {
        OFF: "Off: commands are not signed",
        WAITING: f"On{key}: waiting for the drone",
        DRONE_SIGNS: f"On{key}: the drone signs with this key, only signed commands reach it",
        DRONE_UNSIGNED: f"On{key}, but the drone does not sign: anyone in range can command it. Send it the key.",
        KEY_MISMATCH: f"The drone signs with another key{key}: it ignores VGCS's commands",
        MAVLINK1: "The link speaks MAVLink 1, which cannot be signed",
    }.get(state, state)


def state_level(state: str) -> str:
    """ok, warn or bad, for the dashboard colour."""
    if state == DRONE_SIGNS:
        return "ok"
    if state in (KEY_MISMATCH, MAVLINK1):
        return "bad"
    if state == DRONE_UNSIGNED:
        return "warn"
    return "na"


class KeyTransfer:
    """Giving the drone the key (or taking it away), done when the drone's heartbeats show it.

    on_done(ok, detail) runs once. The drone signs with the new key from its
    next packet, or stops signing after a zero key, so a heartbeat or two
    tells. It refuses while armed and says so in a STATUSTEXT.
    """

    RESEND_S = 2.0
    GIVE_UP_S = 5.0

    def __init__(self, enable: bool, now: float, on_done: Callable[[bool, str], None]) -> None:
        self.enable = bool(enable)
        self.started = float(now)
        self.resent = False
        self.done = False
        self.on_done = on_done

    def _finish(self, ok: bool, detail: str) -> None:
        if not self.done:
            self.done = True
            self.on_done(ok, detail)

    def heard(self, state: str) -> None:
        if self.enable and state == DRONE_SIGNS:
            self._finish(True, "the drone took the key and signs with it")
        elif not self.enable and state == DRONE_UNSIGNED:
            self._finish(True, "the drone no longer signs: signing is off on the drone")

    def heard_text(self, text: str) -> None:
        low = str(text or "").lower()
        if "signing" in low and ("error" in low or "not enabled" in low):
            self._finish(False, f"the drone refused: {text}")

    def tick(self, now: float) -> bool:
        """True when the request should be sent once more."""
        if self.done:
            return False
        waited = now - self.started
        if waited >= self.GIVE_UP_S:
            if self.enable:
                self._finish(False, "the drone did not start signing with this key. It may already have "
                                    "another key: then send the key over USB, which the drone always accepts")
            else:
                self._finish(False, "the drone still signs. It may hold another key: remove it over USB")
            return False
        if not self.resent and waited >= self.RESEND_S:
            self.resent = True
            return True
        return False
