"""The window that asks for a license key before VGCS starts.

It shows this computer's machine code with a Copy button, takes the key VAMA
sends back (pasted, with or without the dashes), and checks it in place. A
wrong key is explained in the window instead of closing it, so the user knows
whether the key is incomplete, for another computer, or past its end date.

The rules and the key format are in vgcs/app/license_key.py.
"""

from __future__ import annotations

from datetime import date

from PySide6.QtCore import QTimer
from PySide6.QtGui import QFont, QGuiApplication
from PySide6.QtWidgets import (
    QDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
)

from vgcs.app import license_key
from vgcs.app.license_key import CheckResult, Status

NEED_KEY_TEXT = "This copy of VGCS needs a license key for this computer."
SEND_CODE_TEXT = "Send this machine code to VAMA. You will get a license key back. Paste it below."
NO_MACHINE_ID_TEXT = "VGCS cannot read this computer's ID, so it cannot be activated here. Please contact VAMA."


def status_text(result: CheckResult) -> str:
    """Why a key was not accepted, in words for the operator."""
    if result.status is Status.MISSING:
        return "Paste the license key first."
    if result.status is Status.DAMAGED:
        return "This key is not complete. Copy the whole key again."
    if result.status is Status.INVALID:
        return "This is not a valid VGCS license key."
    if result.status is Status.WRONG_MACHINE:
        return "This key is for another computer. Send the machine code above to VAMA for a key for this one."
    if result.status is Status.EXPIRED and result.info is not None and result.info.expires is not None:
        return f"This key ended on {result.info.expires.isoformat()}. Ask VAMA for a new key."
    if result.status is Status.NO_MACHINE_ID:
        return NO_MACHINE_ID_TEXT
    return ""


class LicenseDialog(QDialog):
    """Ask for a key; accepted only when the key fits this computer."""

    def __init__(
        self,
        fingerprint: bytes | None,
        previous: CheckResult | None = None,
        *,
        settings=None,
        today: date | None = None,
        public_key: bytes | None = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._fingerprint = fingerprint
        self._settings = settings
        self._today = today
        self._public_key = public_key
        self.accepted_result: CheckResult | None = None
        self.setWindowTitle("VGCS license")
        self.setObjectName("licenseDialog")
        self.setModal(True)
        self.setMinimumWidth(560)

        layout = QVBoxLayout(self)
        prompt = QLabel(NEED_KEY_TEXT)
        prompt.setWordWrap(True)
        layout.addWidget(prompt)

        mono = QFont("Consolas")
        mono.setStyleHint(QFont.StyleHint.Monospace)

        code_row = QHBoxLayout()
        code_row.addWidget(QLabel("Machine code"))
        self._code = QLineEdit(license_key.machine_code(fingerprint) if fingerprint else "")
        self._code.setObjectName("licenseMachineCode")
        self._code.setReadOnly(True)
        bold_mono = QFont(mono)
        bold_mono.setBold(True)
        self._code.setFont(bold_mono)
        code_row.addWidget(self._code, 1)
        self._copy = QPushButton("Copy")
        self._copy.setObjectName("licenseCopyBtn")
        self._copy.clicked.connect(self._copy_code)
        code_row.addWidget(self._copy)
        layout.addLayout(code_row)

        hint = QLabel(SEND_CODE_TEXT)
        hint.setWordWrap(True)
        layout.addWidget(hint)

        # QTextEdit, not QPlainTextEdit: VGCS's style sheet gives it the text field look.
        self._key = QTextEdit()
        self._key.setObjectName("licenseKeyEdit")
        self._key.setAcceptRichText(False)
        self._key.setFont(mono)
        self._key.setPlaceholderText("License key")
        self._key.setFixedHeight(96)
        self._key.textChanged.connect(self._clear_error)
        layout.addWidget(self._key)

        self._error = QLabel("")
        self._error.setObjectName("licenseError")
        self._error.setWordWrap(True)
        self._error.setStyleSheet("color: #ff6b6b;")
        self._error.hide()
        layout.addWidget(self._error)

        buttons = QHBoxLayout()
        self._paste = QPushButton("Paste")
        self._paste.setObjectName("licensePasteBtn")
        self._paste.clicked.connect(self._paste_key)
        buttons.addWidget(self._paste)
        buttons.addStretch(1)
        self._quit = QPushButton("Quit")
        self._quit.setObjectName("licenseQuitBtn")
        self._quit.clicked.connect(self.reject)
        buttons.addWidget(self._quit)
        self._activate = QPushButton("Activate")
        self._activate.setObjectName("licenseActivateBtn")
        self._activate.setDefault(True)
        self._activate.clicked.connect(self._on_activate)
        buttons.addWidget(self._activate)
        layout.addLayout(buttons)

        if fingerprint is None:
            self._activate.setEnabled(False)
            self._paste.setEnabled(False)
            self._copy.setEnabled(False)
            self._show_error(NO_MACHINE_ID_TEXT)
        elif previous is not None and previous.status not in (Status.OK, Status.MISSING):
            # A saved key that stopped fitting: say why, so it is not a mystery.
            self._show_error(status_text(previous))
        self._key.setFocus()

    # ------------------------------------------------------------------ helpers
    def machine_code_text(self) -> str:
        return self._code.text()

    def error_text(self) -> str:
        return "" if self._error.isHidden() else str(self._error.text())

    def set_key_text(self, text: str) -> None:
        self._key.setPlainText(str(text or ""))

    def _clear_error(self) -> None:
        if self._fingerprint is not None:
            self._error.hide()

    def _show_error(self, text: str) -> None:
        self._error.setText(text)
        self._error.show()

    def _copy_code(self) -> None:
        QGuiApplication.clipboard().setText(self._code.text())
        self._copy.setText("Copied")
        QTimer.singleShot(1500, lambda: self._copy.setText("Copy"))

    def _paste_key(self) -> None:
        self._key.setPlainText(QGuiApplication.clipboard().text())

    def _on_activate(self) -> None:
        text = self._key.toPlainText()
        result = license_key.check_key(
            text, self._fingerprint, today=self._today, public_key=self._public_key
        )
        if not result.ok:
            self._show_error(status_text(result))
            return
        license_key.store_key(text, self._settings)
        self.accepted_result = result
        self.accept()


def ensure_license(settings=None, *, fingerprint: bytes | None = None, today: date | None = None) -> bool:
    """True when a valid key is saved, or the user enters one now.

    False means the user chose Quit: VGCS must not start.
    """
    if fingerprint is None:
        fingerprint = license_key.machine_fingerprint()
    saved = license_key.check_stored_license(settings, fingerprint=fingerprint, today=today)
    if saved.ok:
        print(f"[VGCS] license OK: {license_key.describe(saved.info)}", flush=True)
        return True
    dialog = LicenseDialog(fingerprint, saved, settings=settings, today=today)
    if dialog.exec() != QDialog.DialogCode.Accepted or dialog.accepted_result is None:
        print("[VGCS] no license key: closing", flush=True)
        return False
    print(f"[VGCS] license activated: {license_key.describe(dialog.accepted_result.info)}", flush=True)
    return True
