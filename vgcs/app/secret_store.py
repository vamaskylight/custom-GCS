"""Secrets in VGCS's settings, encrypted for this Windows user (milestone M16).

VGCS keeps its settings in the Windows registry (QSettings). A secret, such as
the MAVLink signing key, is never written there as it is. It goes through the
Windows Data Protection API (DPAPI, CryptProtectData), which encrypts it with a
key that Windows derives from this user's logon secret (AES-256 on current
Windows). Only the same Windows user on the same computer can read it back. A
copy of the registry, a settings backup, or another user account gets a blob
it cannot open.

Each secret is bound to its purpose (DPAPI's "entropy"), so a blob stored for
one setting cannot be passed off as another.

Pure Python with ctypes, no extra package. Windows only: elsewhere `available()`
is False and storing a secret raises SecretStoreError, so a caller can keep the
secret for the session only and say so, instead of writing it in the clear.
"""

from __future__ import annotations

import base64
import ctypes
import sys

PREFIX = "dpapi:"
_CRYPTPROTECT_UI_FORBIDDEN = 0x1


class SecretStoreError(Exception):
    """A secret could not be stored or read back."""


def available() -> bool:
    return sys.platform == "win32"


if sys.platform == "win32":
    from ctypes import wintypes

    class _Blob(ctypes.Structure):
        _fields_ = [("cbData", wintypes.DWORD), ("pbData", ctypes.POINTER(ctypes.c_char))]

    _crypt32 = ctypes.WinDLL("crypt32", use_last_error=True)
    _kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _crypt32.CryptProtectData.argtypes = [
        ctypes.POINTER(_Blob), wintypes.LPCWSTR, ctypes.POINTER(_Blob),
        ctypes.c_void_p, ctypes.c_void_p, wintypes.DWORD, ctypes.POINTER(_Blob),
    ]
    _crypt32.CryptProtectData.restype = wintypes.BOOL
    _crypt32.CryptUnprotectData.argtypes = [
        ctypes.POINTER(_Blob), ctypes.c_void_p, ctypes.POINTER(_Blob),
        ctypes.c_void_p, ctypes.c_void_p, wintypes.DWORD, ctypes.POINTER(_Blob),
    ]
    _crypt32.CryptUnprotectData.restype = wintypes.BOOL
    _kernel32.LocalFree.argtypes = [ctypes.c_void_p]
    _kernel32.LocalFree.restype = ctypes.c_void_p

    def _blob(data: bytes):
        buf = ctypes.create_string_buffer(bytes(data), len(data))
        return _Blob(len(data), ctypes.cast(buf, ctypes.POINTER(ctypes.c_char))), buf

    def _take(out: "_Blob") -> bytes:
        try:
            return ctypes.string_at(out.pbData, out.cbData)
        finally:
            _kernel32.LocalFree(ctypes.cast(out.pbData, ctypes.c_void_p))


def protect(data: bytes, purpose: str) -> str:
    """Encrypt for this Windows user. Returns text for the settings, starting with "dpapi:"."""
    if not available():
        raise SecretStoreError("protected storage needs Windows")
    in_blob, _in_buf = _blob(data)
    ent_blob, _ent_buf = _blob(str(purpose).encode("utf-8"))
    out = _Blob()
    if not _crypt32.CryptProtectData(ctypes.byref(in_blob), "VGCS", ctypes.byref(ent_blob),
                                     None, None, _CRYPTPROTECT_UI_FORBIDDEN, ctypes.byref(out)):
        raise SecretStoreError(f"Windows could not protect the secret (error {ctypes.get_last_error()})")
    return PREFIX + base64.b64encode(_take(out)).decode("ascii")


def unprotect(text: str, purpose: str) -> bytes:
    """Read back what protect() wrote, for the same purpose."""
    if not available():
        raise SecretStoreError("protected storage needs Windows")
    text = str(text or "")
    if not text.startswith(PREFIX):
        raise SecretStoreError("this setting is not a protected secret")
    try:
        raw = base64.b64decode(text[len(PREFIX):].encode("ascii"), validate=True)
    except Exception as e:
        raise SecretStoreError(f"the protected secret is damaged ({e})") from e
    in_blob, _in_buf = _blob(raw)
    ent_blob, _ent_buf = _blob(str(purpose).encode("utf-8"))
    out = _Blob()
    if not _crypt32.CryptUnprotectData(ctypes.byref(in_blob), None, ctypes.byref(ent_blob),
                                       None, None, _CRYPTPROTECT_UI_FORBIDDEN, ctypes.byref(out)):
        raise SecretStoreError(
            "this secret was stored by another Windows user or computer, or it is damaged "
            f"(error {ctypes.get_last_error()})"
        )
    return _take(out)


def store(settings, key: str, data: bytes | None) -> None:
    """Keep a secret in the settings, protected. None removes it."""
    if data is None:
        settings.remove(key)
        return
    settings.setValue(key, protect(data, key))
    try:
        settings.sync()
    except Exception:
        pass


def load(settings, key: str) -> bytes | None:
    """The secret stored under key, or None if there is none. Raises SecretStoreError if unreadable."""
    value = settings.value(key, "")
    if value is None or str(value) == "":
        return None
    return unprotect(str(value), key)
