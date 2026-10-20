"""The window that asks for the admin password (admin_password.py).

A wrong password is refused in place. The window then counts down the wait before the next
try, with Unlock off, so that a wait does not look like VGCS hanging.
"""

from __future__ import annotations

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QDialog, QHBoxLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout

from vgcs.app.admin_password import AdminPassword, shared_admin_password

CAMERAS_PROMPT = "Only the admin changes the camera list. Type the admin password."
WRONG_TEXT = "Wrong password."


class AdminPasswordDialog(QDialog):
    """Asks for the admin password. Accepted: the password was right."""

    def __init__(self, checker: AdminPassword, prompt: str = CAMERAS_PROMPT, parent=None) -> None:
        super().__init__(parent)
        self._checker = checker
        self._wrong = False
        self.setWindowTitle("Admin password")
        self.setObjectName("adminPasswordDialog")
        self.setModal(True)

        layout = QVBoxLayout(self)
        text = QLabel(prompt)
        text.setWordWrap(True)
        layout.addWidget(text)

        self._pw = QLineEdit()
        self._pw.setObjectName("adminPasswordEdit")
        self._pw.setEchoMode(QLineEdit.EchoMode.Password)
        layout.addWidget(self._pw)

        self._error = QLabel("")
        self._error.setObjectName("adminPasswordError")
        self._error.setWordWrap(True)
        self._error.setStyleSheet("color: #ff6b6b;")
        self._error.hide()
        layout.addWidget(self._error)

        buttons = QHBoxLayout()
        buttons.addStretch(1)
        cancel = QPushButton("Cancel")
        cancel.setObjectName("adminPasswordCancelBtn")
        cancel.clicked.connect(self.reject)
        buttons.addWidget(cancel)
        self._ok = QPushButton("Unlock")
        self._ok.setObjectName("adminPasswordOkBtn")
        self._ok.setDefault(True)
        self._ok.clicked.connect(self._on_ok)
        buttons.addWidget(self._ok)
        layout.addLayout(buttons)

        # Counts the wait down while there is one.
        self._timer = QTimer(self)
        self._timer.setInterval(250)
        self._timer.timeout.connect(self._show_wait)

        self._pw.setFocus()
        self._show_wait()

    def error_text(self) -> str:
        return "" if self._error.isHidden() else str(self._error.text())

    def _show_wait(self) -> None:
        wait = self._checker.wait_seconds()
        if wait <= 0:
            self._timer.stop()
            self._ok.setEnabled(True)
            self._error.setText(WRONG_TEXT)
            self._error.setVisible(self._wrong)
            return
        self._ok.setEnabled(False)
        self._error.setText((WRONG_TEXT + " " if self._wrong else "") + f"Try again in {wait} s.")
        self._error.show()
        if not self._timer.isActive():
            self._timer.start()

    def _on_ok(self) -> None:
        result = self._checker.check(self._pw.text())
        if result == AdminPassword.RIGHT:
            self.accept()
            return
        if result == AdminPassword.WRONG:
            self._wrong = True
            self._pw.selectAll()
        self._show_wait()


def ask_admin_password(parent=None, *, checker: AdminPassword | None = None, prompt: str = CAMERAS_PROMPT) -> bool:
    """Shows the window. True: the right password was typed."""
    dlg = AdminPasswordDialog(checker or shared_admin_password(), prompt, parent)
    return dlg.exec() == QDialog.DialogCode.Accepted


__all__ = ["AdminPasswordDialog", "CAMERAS_PROMPT", "WRONG_TEXT", "ask_admin_password"]
