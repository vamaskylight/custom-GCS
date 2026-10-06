"""License key for the VGCS exe.

Asked 2026-10-05 (Abhi): the exe must not run without a valid key. It has to
work offline, because customer laptops never connect to the internet. The app
shows a machine code, and a key made for that code works only on that laptop,
so a copied exe does not run on another one.

How it works:

- Machine code: 16 characters shown in the app, for example ABCD-EFGH-IJKL-MNOP.
  It is a hash of the computer's hardware ID (the system UUID in the firmware's
  SMBIOS table), or of the Windows installation ID (MachineGuid) when the board
  has no usable UUID. The ID itself is never shown. The last two characters
  are a check, so a mistyped code is refused before a key is made for it.
- License key: that hash, a serial number, the issue date and an optional end
  date, signed with VAMA's private key (Ed25519). The app holds only the public
  key, so it can check keys but cannot make them. The private key stays with
  whoever issues keys (packaging/vgcs_license_tool.py) and is never in the repo.
- The key is saved in the user's settings when it is accepted, and checked
  again at every start.

What it does not do, so nobody relies on more than it gives: it stops a
copied exe or a shared key from running on another laptop. It does not stop
someone who changes the program itself; no check inside a program can. Setting
the computer's clock back would also get past an end date.

The check runs only in the packaged exe. Running from source ("python -m vgcs")
is how developers work and has no check. VGCS_REQUIRE_LICENSE=1 turns it on
there, to try it.
"""

from __future__ import annotations

import base64
import enum
import hashlib
import os
import sys
from dataclasses import dataclass
from datetime import date, timedelta

from vgcs.app import ed25519

# VAMA's license public key. The matching private key is kept outside the
# repo (see packaging/vgcs_license_tool.py). Changing this makes every key
# issued so far invalid.
LICENSE_PUBLIC_KEY = bytes.fromhex("d5bd5e5523238b32d6b885a4006f36c1dacef1893e2fd7e439e89dc43ba12798")

KEY_VERSION = 1
ALL_FEATURES = 0xFF
FINGERPRINT_BYTES = 9

_EPOCH = date(2026, 1, 1)  # dates in a key are days after this
_SIGNED_PREFIX = b"VGCS license key v1\x00"
_PAYLOAD_BYTES = 1 + FINGERPRINT_BYTES + 2 + 2 + 2 + 1
_SIGNATURE_BYTES = 64
_KEY_BYTES = _PAYLOAD_BYTES + _SIGNATURE_BYTES

_QS_KEY = "license/key"

# Characters people type for ones the code never uses (base32 has no 0, 1 or 8).
_LOOK_ALIKES = str.maketrans({"0": "O", "1": "I", "8": "B"})

# A placeholder some boards report instead of a real UUID
# (03000200-0400-0500-0006-000700080009, as stored in the table).
_PLACEHOLDER_UUID = bytes.fromhex("00020003000400050006000700080009")


class Status(enum.Enum):
    OK = "ok"
    MISSING = "missing"              # no key entered yet
    DAMAGED = "damaged"              # not a complete key
    INVALID = "invalid"              # not signed by VAMA
    WRONG_MACHINE = "wrong_machine"  # a real key, for another computer
    EXPIRED = "expired"
    NO_MACHINE_ID = "no_machine_id"  # this computer's ID cannot be read


@dataclass(frozen=True)
class LicenseInfo:
    serial: int
    issued: date
    expires: date | None
    features: int
    fingerprint: bytes


@dataclass(frozen=True)
class CheckResult:
    status: Status
    info: LicenseInfo | None = None

    @property
    def ok(self) -> bool:
        return self.status is Status.OK


def license_required(*, frozen: bool | None = None, env=None) -> bool:
    """True in the packaged exe, or when VGCS_REQUIRE_LICENSE asks for it."""
    if frozen is None:
        frozen = bool(getattr(sys, "frozen", False))
    if frozen:
        return True
    env = os.environ if env is None else env
    return str(env.get("VGCS_REQUIRE_LICENSE", "")).strip().lower() in ("1", "true", "yes", "on")


# --- This computer ---------------------------------------------------------------


def smbios_system_uuid(raw: bytes) -> bytes | None:
    """The 16-byte UUID of the SMBIOS type 1 structure, or None.

    ``raw`` is what GetSystemFirmwareTable('RSMB') returns: an 8-byte header
    (calling method, versions, table length) and then the structures.
    """
    if len(raw) < 8:
        return None
    length = int.from_bytes(raw[4:8], "little")
    table = raw[8:8 + length]
    i = 0
    while i + 4 <= len(table):
        kind, size = table[i], table[i + 1]
        if size < 4:
            return None
        if kind == 1:
            if size < 0x19 or i + 24 > len(table):
                return None
            uuid = bytes(table[i + 8:i + 24])
            return uuid if _usable_uuid(uuid) else None
        if kind == 127:  # end of table
            return None
        # The formatted part, then strings ending with two zero bytes.
        j = i + size
        while j + 1 < len(table) and not (table[j] == 0 and table[j + 1] == 0):
            j += 1
        i = j + 2
    return None


def _usable_uuid(uuid: bytes) -> bool:
    return len(set(uuid)) > 1 and uuid != _PLACEHOLDER_UUID


def read_smbios_uuid() -> bytes | None:
    """The firmware's system UUID on Windows, read without WMI (fast, no extra process)."""
    if sys.platform != "win32":
        return None
    try:
        import ctypes

        get_table = ctypes.windll.kernel32.GetSystemFirmwareTable
        get_table.restype = ctypes.c_uint
        get_table.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint]
        provider = int.from_bytes(b"RSMB", "big")
        size = get_table(provider, 0, None, 0)
        if size == 0:
            return None
        buffer = ctypes.create_string_buffer(size)
        if get_table(provider, 0, buffer, size) != size:
            return None
        return smbios_system_uuid(buffer.raw)
    except Exception:
        return None


def read_machine_guid() -> str | None:
    """The Windows installation ID (changes when Windows is installed again)."""
    if sys.platform != "win32":
        return None
    try:
        import winreg

        with winreg.OpenKey(
            winreg.HKEY_LOCAL_MACHINE,
            r"SOFTWARE\Microsoft\Cryptography",
            0,
            winreg.KEY_READ | winreg.KEY_WOW64_64KEY,
        ) as key:
            value, _kind = winreg.QueryValueEx(key, "MachineGuid")
        text = str(value).strip().lower()
        return text or None
    except OSError:
        return None


def machine_fingerprint(read_uuid=read_smbios_uuid, read_guid=read_machine_guid) -> bytes | None:
    """The 9 bytes that tie a key to this computer, or None if no ID can be read."""
    uuid = read_uuid()
    if uuid:
        source = b"smbios-uuid:" + uuid
    else:
        guid = read_guid()
        if not guid:
            return None
        source = b"windows-machine-guid:" + guid.encode("ascii", "ignore")
    return hashlib.sha256(b"VGCS machine code v1\x00" + source).digest()[:FINGERPRINT_BYTES]


def _code_check(fingerprint: bytes) -> bytes:
    return hashlib.sha256(b"VGCS machine code check\x00" + fingerprint).digest()[:1]


def machine_code(fingerprint: bytes) -> str:
    """The code the user sends to VAMA: 16 characters in four groups."""
    text = base64.b32encode(fingerprint + _code_check(fingerprint)).decode("ascii")
    return "-".join(text[i:i + 4] for i in range(0, 16, 4))


def _normalise(text: str) -> str:
    kept = "".join(ch for ch in str(text or "").upper() if ch.isalnum())
    return kept.translate(_LOOK_ALIKES)


def _b32decode(text: str) -> bytes | None:
    try:
        return base64.b32decode(text + "=" * (-len(text) % 8))
    except ValueError:  # binascii.Error is a ValueError
        return None


def parse_machine_code(text: str) -> bytes | None:
    """The fingerprint in a machine code, or None if it is mistyped."""
    raw = _b32decode(_normalise(text))
    if raw is None or len(raw) != FINGERPRINT_BYTES + 1:
        return None
    fingerprint, check = raw[:FINGERPRINT_BYTES], raw[FINGERPRINT_BYTES:]
    return fingerprint if check == _code_check(fingerprint) else None


# --- Keys ------------------------------------------------------------------------


def _day_number(day: date) -> int:
    number = (day - _EPOCH).days
    if not 0 <= number <= 0xFFFF:
        raise ValueError(f"date out of range: {day}")
    return number


def make_key(
    secret: bytes,
    fingerprint: bytes,
    *,
    serial: int,
    issued: date,
    expires: date | None = None,
    features: int = ALL_FEATURES,
) -> str:
    """A license key for one computer. Needs the private key: vendor side only."""
    if len(fingerprint) != FINGERPRINT_BYTES:
        raise ValueError("fingerprint must be 9 bytes")
    if not 1 <= serial <= 0xFFFF:
        raise ValueError("serial must be 1 to 65535")
    if expires is not None and expires <= _EPOCH:
        raise ValueError("end date must be after 2026-01-01")
    payload = (
        bytes([KEY_VERSION])
        + fingerprint
        + serial.to_bytes(2, "big")
        + _day_number(issued).to_bytes(2, "big")
        + (_day_number(expires) if expires else 0).to_bytes(2, "big")
        + bytes([features & 0xFF])
    )
    signature = ed25519.sign(secret, _SIGNED_PREFIX + payload)
    text = base64.b32encode(payload + signature).decode("ascii").rstrip("=")
    return "-".join(text[i:i + 5] for i in range(0, len(text), 5))


def check_key(
    text: str,
    fingerprint: bytes | None,
    *,
    today: date | None = None,
    public_key: bytes | None = None,
) -> CheckResult:
    """Whether a key may run VGCS on the computer with this fingerprint."""
    if fingerprint is None:
        return CheckResult(Status.NO_MACHINE_ID)
    normalised = _normalise(text)
    if not normalised:
        return CheckResult(Status.MISSING)
    raw = _b32decode(normalised)
    if raw is None or len(raw) != _KEY_BYTES or raw[0] != KEY_VERSION:
        return CheckResult(Status.DAMAGED)
    payload, signature = raw[:_PAYLOAD_BYTES], raw[_PAYLOAD_BYTES:]
    key = LICENSE_PUBLIC_KEY if public_key is None else public_key
    # The signature first: a made-up key must never be told which computer it is for.
    if not ed25519.verify(key, _SIGNED_PREFIX + payload, signature):
        return CheckResult(Status.INVALID)
    expires_day = int.from_bytes(payload[14:16], "big")
    info = LicenseInfo(
        serial=int.from_bytes(payload[10:12], "big"),
        issued=_EPOCH + timedelta(days=int.from_bytes(payload[12:14], "big")),
        expires=(_EPOCH + timedelta(days=expires_day)) if expires_day else None,
        features=payload[16],
        fingerprint=payload[1:1 + FINGERPRINT_BYTES],
    )
    if info.fingerprint != fingerprint:
        return CheckResult(Status.WRONG_MACHINE, info)
    if info.expires is not None and (today or date.today()) > info.expires:
        return CheckResult(Status.EXPIRED, info)
    return CheckResult(Status.OK, info)


# --- The saved key ---------------------------------------------------------------


def _default_settings():
    from PySide6.QtCore import QSettings

    from vgcs.map.app_settings import QS_APP, QS_ORG

    return QSettings(QS_ORG, QS_APP)


def stored_key(settings=None) -> str:
    settings = settings or _default_settings()
    return str(settings.value(_QS_KEY, "") or "")


def store_key(text: str, settings=None) -> None:
    settings = settings or _default_settings()
    settings.setValue(_QS_KEY, str(text or "").strip())
    settings.sync()


def check_stored_license(
    settings=None,
    *,
    fingerprint: bytes | None = None,
    today: date | None = None,
    public_key: bytes | None = None,
) -> CheckResult:
    """The saved key, checked for this computer."""
    if fingerprint is None:
        fingerprint = machine_fingerprint()
    return check_key(stored_key(settings), fingerprint, today=today, public_key=public_key)


def describe(info: LicenseInfo) -> str:
    """One line for the log, for example "serial 12, no end date"."""
    end = f"ends {info.expires.isoformat()}" if info.expires else "no end date"
    return f"serial {info.serial}, {end}"
