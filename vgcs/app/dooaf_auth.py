"""Password gate on the artillery position in DOOAF.

Requested 2026-09-10: "add authentication on DOOAF, I mean when a user is
trying to select the artillery one popup should come and password is required."

What this is, stated plainly so nobody relies on more than it gives. It is a
gate in the interface, not encryption. It stops the wrong person at the console
from moving the gun, which is the stated need. It does not stop anyone who has
the laptop and wants in: the stored value can be cleared from the settings, and
the aircraft's own data is untouched by it. Anything stronger needs the
artillery position kept somewhere this application cannot read, which is a
different job.

What it does do properly:

- The password is never stored. A random salt and PBKDF2-HMAC-SHA256 are, and
  the check is a constant-time comparison, so the stored value cannot be read
  back into a password and a wrong guess cannot be timed.
- Repeated wrong guesses are slowed down, so the gate is not a free oracle for
  someone trying a list.
- Unlocking lasts for the session only. Closing VGCS locks it again.

Forgetting the password means clearing it in the settings on that machine.
There is deliberately no reset inside the application, because a reset anyone
can press is not a gate at all.
"""

from __future__ import annotations

import hashlib
import hmac
import os
import time

# Cost is a trade against how long an operator waits at the popup. This lands
# around a tenth of a second on the field laptops, which nobody notices once
# and which makes a guessing run expensive.
PBKDF2_ITERATIONS = 240_000
_SALT_BYTES = 16
_KEY_BYTES = 32

MIN_PASSWORD_LEN = 4

_QS_SALT = "dooaf/artillery_pw_salt"
_QS_HASH = "dooaf/artillery_pw_hash"
_QS_ITERATIONS = "dooaf/artillery_pw_iterations"

# Wrong guesses start costing time from here, doubling to a ceiling.
FAILURES_BEFORE_DELAY = 3
_FIRST_DELAY_S = 5.0
_MAX_DELAY_S = 300.0


def _default_settings():
    from PySide6.QtCore import QSettings

    from vgcs.map.app_settings import QS_APP, QS_ORG

    return QSettings(QS_ORG, QS_APP)


def derive_key(password: str, salt: bytes, iterations: int) -> bytes:
    """PBKDF2-HMAC-SHA256 of a password. Pure, so the maths can be tested."""
    return hashlib.pbkdf2_hmac(
        "sha256",
        str(password or "").encode("utf-8"),
        bytes(salt),
        max(1, int(iterations)),
        dklen=_KEY_BYTES,
    )


def password_is_set(settings=None) -> bool:
    st = settings if settings is not None else _default_settings()
    return bool(str(st.value(_QS_HASH, "") or "").strip()) and bool(
        str(st.value(_QS_SALT, "") or "").strip()
    )


def set_password(password: str, settings=None) -> bool:
    """Store a new password. False when it is too short to be worth having."""
    pw = str(password or "")
    if len(pw) < MIN_PASSWORD_LEN:
        return False
    st = settings if settings is not None else _default_settings()
    salt = os.urandom(_SALT_BYTES)
    key = derive_key(pw, salt, PBKDF2_ITERATIONS)
    st.setValue(_QS_SALT, salt.hex())
    st.setValue(_QS_HASH, key.hex())
    st.setValue(_QS_ITERATIONS, int(PBKDF2_ITERATIONS))
    return True


def verify_password(password: str, settings=None) -> bool:
    """True when the password matches the stored one."""
    st = settings if settings is not None else _default_settings()
    if not password_is_set(st):
        return False
    try:
        salt = bytes.fromhex(str(st.value(_QS_SALT, "") or ""))
        expected = bytes.fromhex(str(st.value(_QS_HASH, "") or ""))
        iterations = int(st.value(_QS_ITERATIONS, PBKDF2_ITERATIONS) or PBKDF2_ITERATIONS)
    except (TypeError, ValueError):
        return False
    if not salt or not expected:
        return False
    got = derive_key(str(password or ""), salt, iterations)
    # Constant time: a plain == leaks how much of the hash matched.
    return hmac.compare_digest(got, expected)


def clear_password(settings=None) -> None:
    st = settings if settings is not None else _default_settings()
    for key in (_QS_SALT, _QS_HASH, _QS_ITERATIONS):
        try:
            st.remove(key)
        except Exception:
            st.setValue(key, "")


def lockout_delay_s(failures: int) -> float:
    """How long to refuse after this many consecutive wrong guesses."""
    n = int(failures)
    if n <= FAILURES_BEFORE_DELAY:
        return 0.0
    steps = n - FAILURES_BEFORE_DELAY - 1
    return min(_MAX_DELAY_S, _FIRST_DELAY_S * (2.0 ** max(0, steps)))


class ArtilleryLock:
    """Whether the artillery position may be changed right now.

    Locked again on every restart, on purpose: a session that outlives the
    operator at the console is the thing this is meant to prevent.
    """

    def __init__(self, settings=None, *, clock=time.monotonic) -> None:
        self._settings = settings
        self._clock = clock
        self._unlocked = False
        self._failures = 0
        self._blocked_until = 0.0

    # ------------------------------------------------------------------ state
    @property
    def is_unlocked(self) -> bool:
        return self._unlocked

    def password_is_set(self) -> bool:
        return password_is_set(self._settings)

    def seconds_until_retry(self) -> float:
        return max(0.0, self._blocked_until - self._clock())

    def is_rate_limited(self) -> bool:
        return self.seconds_until_retry() > 0.0

    # ---------------------------------------------------------------- actions
    def set_password(self, password: str) -> bool:
        """Set the password and unlock. Used the first time, when none exists."""
        if not set_password(password, self._settings):
            return False
        self._unlocked = True
        self._failures = 0
        self._blocked_until = 0.0
        return True

    def unlock(self, password: str) -> bool:
        """Try a password. Refuses outright while rate limited."""
        if self.is_rate_limited():
            return False
        if verify_password(password, self._settings):
            self._unlocked = True
            self._failures = 0
            self._blocked_until = 0.0
            return True
        self._failures += 1
        delay = lockout_delay_s(self._failures)
        if delay > 0.0:
            self._blocked_until = self._clock() + delay
        return False

    def lock(self) -> None:
        self._unlocked = False
