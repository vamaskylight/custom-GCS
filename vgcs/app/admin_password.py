r"""The admin password, asked before the camera list can be changed (task AA, VGCS part).

The client asked (2026-10-16) that only the admin changes such settings. The APK asks for an
admin password before Advanced Mode (task Z, apk/custom/src/AdminPassword.cc). VGCS asks for
the same password, by the same rules, before its camera editor. Choosing a camera stays open
to everyone.

Only a salted hash of the password is kept: PBKDF2 with HMAC-SHA256, as the APK checks it.
A customer exe has the team's password: apk\make_admin_password.ps1 makes its salt and hash
once, in %USERPROFILE%\.vama\vama-admin-password.txt (never in the repo), and
"packaging\build_exe.py --customer" puts them into the exe as vgcs_admin_password.txt. A test
exe, and VGCS run from source, has the test password.

What it does not do, said plainly: the hash can be read out of the exe and guesses tried away
from it, so the password must be long. And the camera files are plain files in the user's
folder: whoever can write there can add a camera without VGCS. The password guards the editor.
"""

from __future__ import annotations

import hashlib
import hmac
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# The password of test exes (it is in their test steps), and its salt and hash (as the APK's).
TEST_ADMIN_PASSWORD = "vama1234"
_TEST_SALT_HEX = "c4196712152590a95a9e20e7573dff17"
_TEST_HASH_HEX = "79483abd3d0626ff731367fc33bf9cecbeaa384176928a86528bd5d6ed2b39d2"
_TEST_ITERATIONS = 100000

_HASH_BYTES = 32
# The file a customer exe carries (packaging/build_exe.py --customer).
SECRET_FILE = "vgcs_admin_password.txt"


@dataclass(frozen=True)
class AdminSecret:
    """A password's salt and PBKDF2 hash, and the number of rounds."""

    salt: bytes
    hash: bytes
    iterations: int

    @property
    def usable(self) -> bool:
        return self.iterations > 0 and bool(self.salt) and len(self.hash) == _HASH_BYTES


# A secret no password matches: a customer exe whose file is broken (never the test password).
NO_SECRET = AdminSecret(b"", b"", 0)


def secret_for_test_builds() -> AdminSecret:
    return AdminSecret(bytes.fromhex(_TEST_SALT_HEX), bytes.fromhex(_TEST_HASH_HEX), _TEST_ITERATIONS)


def derive_hash(password: str, salt: bytes, iterations: int) -> bytes:
    """PBKDF2 with HMAC-SHA256 over the password's UTF-8 bytes: 32 bytes."""
    return hashlib.pbkdf2_hmac("sha256", password.encode("utf-8"), salt, iterations, _HASH_BYTES)


def parse_secret(text: str) -> AdminSecret | None:
    """The secret in a file of apk\\make_admin_password.ps1 (salt=, hash=, iterations= lines), or None."""
    values: dict[str, str] = {}
    for line in str(text).splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip()
    try:
        secret = AdminSecret(
            bytes.fromhex(values["salt"]), bytes.fromhex(values["hash"]), int(values["iterations"])
        )
    except (KeyError, ValueError):
        return None
    return secret if secret.usable else None


def secret_text(secret: AdminSecret) -> str:
    """The lines a customer exe carries."""
    return f"salt={secret.salt.hex()}\nhash={secret.hash.hex()}\niterations={secret.iterations}\n"


def built_in_secret(bundle: Path | None = None) -> tuple[AdminSecret, str]:
    """The secret in this VGCS and where it comes from: "team", "test" or "broken".

    "broken": a customer exe whose file cannot be read. Then no password opens the editor,
    as the test password must never open a customer exe.
    """
    if bundle is None and getattr(sys, "frozen", False):
        bundle = Path(getattr(sys, "_MEIPASS", Path(sys.executable).parent))
    if bundle is None:
        return secret_for_test_builds(), "test"
    path = Path(bundle) / SECRET_FILE
    if not path.exists():
        return secret_for_test_builds(), "test"
    try:
        secret = parse_secret(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError):
        secret = None
    if secret is None:
        return NO_SECRET, "broken"
    return secret, "team"


class AdminPassword:
    """Checks passwords, and makes the next try wait after a wrong one (the APK's rules)."""

    RIGHT = "right"
    WRONG = "wrong"
    WAIT = "wait"  # a try during a wait: not checked

    # After a wrong password the next try waits this long, and after this many wrong ones in a row longer.
    WRONG_WAIT_S = 5.0
    MANY_WRONG = 5
    MANY_WRONG_WAIT_S = 60.0

    def __init__(self, secret: AdminSecret, *, clock=time.monotonic) -> None:
        self._secret = secret
        self._clock = clock
        self._wrong_in_a_row = 0
        self._next_try = 0.0
        self._any_try = False

    def check(self, password: str) -> str:
        now = float(self._clock())
        if self._any_try and now < self._next_try:
            return self.WAIT
        self._any_try = True
        password = str(password or "")
        if (
            self._secret.usable
            and password
            # The same time for any password: no early end at the first byte that differs.
            and hmac.compare_digest(derive_hash(password, self._secret.salt, self._secret.iterations), self._secret.hash)
        ):
            self._wrong_in_a_row = 0
            self._next_try = now
            return self.RIGHT
        self._wrong_in_a_row += 1
        wait = self.MANY_WRONG_WAIT_S if self._wrong_in_a_row >= self.MANY_WRONG else self.WRONG_WAIT_S
        self._next_try = now + wait
        return self.WRONG

    def wait_seconds(self) -> int:
        """Seconds until the next try may be made. 0: now."""
        now = float(self._clock())
        if not self._any_try or now >= self._next_try:
            return 0
        return max(1, math.ceil(self._next_try - now))


_shared: AdminPassword | None = None


def shared_admin_password() -> AdminPassword:
    """The one checker of this VGCS, so that a wait outlasts a closed window."""
    global _shared
    if _shared is None:
        _shared = AdminPassword(built_in_secret()[0])
    return _shared


__all__ = [
    "AdminPassword",
    "AdminSecret",
    "NO_SECRET",
    "SECRET_FILE",
    "TEST_ADMIN_PASSWORD",
    "built_in_secret",
    "derive_hash",
    "parse_secret",
    "secret_for_test_builds",
    "secret_text",
    "shared_admin_password",
]
