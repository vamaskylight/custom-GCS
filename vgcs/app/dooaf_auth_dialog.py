"""The popup that asks for the artillery password.

Two modes, chosen by whether a password exists yet. The first time it asks the
operator to choose one, with a confirmation box, because a gate nobody has set
up is not a gate. After that it asks for it.

Wrong answers are refused in place rather than closing the dialog, and after a
few of them the dialog says how long it will not listen for, so the delay reads
as a decision rather than the application hanging.
"""

from __future__ import annotations

from PySide6.QtWidgets import (
    QDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
)

from vgcs.app.dooaf_auth import MIN_PASSWORD_LEN

CREATE_PROMPT = (
    "Set a password for the artillery position.\n"
    "It will be asked for before the gun position can be changed."
)
ENTER_PROMPT = "Password required to change the artillery position."
MISMATCH_TEXT = "The two entries do not match."
TOO_SHORT_TEXT = f"Use at least {MIN_PASSWORD_LEN} characters."
WRONG_TEXT = "Wrong password."


class ArtilleryPasswordDialog(QDialog):
    """Ask for the artillery password, or set one if there is none yet."""

    def __init__(self, lock, parent=None) -> None:
        super().__init__(parent)
        self._lock = lock
        self._creating = not lock.password_is_set()
        self.setWindowTitle("Artillery position")
        self.setObjectName("artilleryPasswordDialog")
        self.setModal(True)

        layout = QVBoxLayout(self)
        prompt = QLabel(CREATE_PROMPT if self._creating else ENTER_PROMPT)
        prompt.setWordWrap(True)
        layout.addWidget(prompt)

        self._pw = QLineEdit()
        self._pw.setObjectName("artilleryPasswordEdit")
        self._pw.setEchoMode(QLineEdit.EchoMode.Password)
        self._pw.textChanged.connect(self._clear_error)
        layout.addWidget(self._pw)

        self._confirm = QLineEdit()
        self._confirm.setObjectName("artilleryPasswordConfirmEdit")
        self._confirm.setEchoMode(QLineEdit.EchoMode.Password)
        self._confirm.setPlaceholderText("Repeat the password")
        self._confirm.textChanged.connect(self._clear_error)
        self._confirm.setVisible(self._creating)
        layout.addWidget(self._confirm)

        self._error = QLabel("")
        self._error.setObjectName("artilleryPasswordError")
        self._error.setWordWrap(True)
        self._error.setStyleSheet("color: #ff6b6b;")
        self._error.hide()
        layout.addWidget(self._error)

        buttons = QHBoxLayout()
        buttons.addStretch(1)
        self._cancel = QPushButton("Cancel")
        self._cancel.setObjectName("artilleryPasswordCancelBtn")
        self._cancel.clicked.connect(self.reject)
        buttons.addWidget(self._cancel)
        self._ok = QPushButton("Set password" if self._creating else "Unlock")
        self._ok.setObjectName("artilleryPasswordOkBtn")
        self._ok.setDefault(True)
        self._ok.clicked.connect(self._on_ok)
        buttons.addWidget(self._ok)
        layout.addLayout(buttons)

        self._pw.setFocus()
        self._refresh_rate_limit()

    # ------------------------------------------------------------------ helpers
    @property
    def is_creating(self) -> bool:
        return self._creating

    def error_text(self) -> str:
        return "" if self._error.isHidden() else str(self._error.text())

    def set_password_text(self, text: str) -> None:
        self._pw.setText(str(text or ""))

    def set_confirm_text(self, text: str) -> None:
        self._confirm.setText(str(text or ""))

    def _clear_error(self, *_a) -> None:
        self._error.hide()
        self._error.setText("")

    def _show_error(self, text: str) -> None:
        self._error.setText(str(text))
        self._error.show()

    def _refresh_rate_limit(self) -> None:
        """Refuse to listen while the lock is counting down, and say so."""
        wait = self._lock.seconds_until_retry()
        if wait <= 0.0:
            self._ok.setEnabled(True)
            return
        self._ok.setEnabled(False)
        self._show_error(f"Too many attempts. Try again in {wait:.0f} s.")

    # ------------------------------------------------------------------ action
    def _on_ok(self) -> None:
        pw = self._pw.text()
        if self._creating:
            if len(pw) < MIN_PASSWORD_LEN:
                self._show_error(TOO_SHORT_TEXT)
                return
            if pw != self._confirm.text():
                self._show_error(MISMATCH_TEXT)
                return
            if not self._lock.set_password(pw):
                self._show_error(TOO_SHORT_TEXT)
                return
            self.accept()
            return
        if self._lock.unlock(pw):
            self.accept()
            return
        # Stays open on a wrong answer: closing it makes a typo feel like a
        # refusal by the application rather than a wrong password.
        self._show_error(WRONG_TEXT)
        self._pw.selectAll()
        self._refresh_rate_limit()


def request_artillery_unlock(lock, parent=None) -> bool:
    """Show the popup and report whether the artillery position may be changed."""
    if lock.is_unlocked:
        return True
    dlg = ArtilleryPasswordDialog(lock, parent)
    dlg.exec()
    return bool(lock.is_unlocked)
