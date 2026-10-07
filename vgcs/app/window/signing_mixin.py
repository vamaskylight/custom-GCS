"""MAVLink 2 command signing in the window (milestone M16). See vgcs/link/signing.py.

- The key comes from a passphrase, the same way as in QGroundControl, so the
  VAMA APK and every VGCS laptop that should command a drone use one passphrase.
- The passphrase is never stored. The key is, encrypted for this Windows user
  (vgcs/app/secret_store.py).
- With a key, every drone link VGCS opens signs its packets (each link of a
  fleet too). The dashboard's "Command signing" value says whether the drone
  on screen signs with the same key, so only signed commands reach it.
- "Send key to the drone" and "Remove key from the drone" act on the drone on
  screen, while it is disarmed, and are reported once the drone shows it.
"""

from __future__ import annotations

import secrets

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
)

from vgcs.app import secret_store
from vgcs.link.signing import (
    MIN_PASSPHRASE,
    OFF,
    WAITING,
    fingerprint,
    key_from_passphrase,
    state_level,
    state_text,
)

KEY_SIGNING_KEY = "signing/key"            # protected (DPAPI), never the passphrase
KEY_SIGNING_LINK_ID = "signing/link_id"    # this laptop's MAVLink signing link id, 1 to 254


class MainWindowSigningMixin:
    """Command signing for every drone link of the window."""

    def _init_signing(self) -> None:
        self._signing_key = None
        self._signing_state = OFF
        self._signing_note = ""
        try:
            link_id = int(self._settings.value(KEY_SIGNING_LINK_ID, 0) or 0)
        except (TypeError, ValueError):
            link_id = 0
        if not 1 <= link_id <= 254:
            # Fixed per laptop: the drone tracks a signing stream per link id,
            # and a new id on every connect would use up its stream slots.
            link_id = secrets.randbelow(254) + 1
            self._settings.setValue(KEY_SIGNING_LINK_ID, link_id)
        self._signing_link_id = link_id
        try:
            self._signing_key = secret_store.load(self._settings, KEY_SIGNING_KEY)
        except secret_store.SecretStoreError as e:
            self._signing_key = None
            self._signing_note = f"The stored signing key cannot be read on this account ({e}). Set the passphrase again."
        if self._signing_key is not None and len(self._signing_key) != 32:
            self._signing_key = None
            self._signing_note = "The stored signing key is damaged. Set the passphrase again."
        self._signing_state = WAITING if self._signing_key else OFF

    def _make_link_thread(self, connection: str, timeout_s: float):
        """A drone link for the fleet, signing with this laptop's key when there is one."""
        from vgcs.link.mavlink_thread import MavlinkThread

        thread = MavlinkThread(connection, timeout_s=timeout_s)
        if self._signing_key:
            thread.queue_signing_key(self._signing_key, self._signing_link_id)
        return thread

    def _signing_fingerprint(self) -> str:
        return fingerprint(self._signing_key) if self._signing_key else ""

    # --- the key ------------------------------------------------------------
    def _signing_use_key(self, key: bytes | None) -> str:
        """Keep a new key (or none), and sign with it on every open link.

        Returns "" when stored protected, or a note when it could only be kept
        for this session (no protected storage on this system).
        """
        note = ""
        try:
            secret_store.store(self._settings, KEY_SIGNING_KEY, key)
        except secret_store.SecretStoreError as e:
            note = f"Kept for this session only: {e}."
        self._signing_key = key
        self._signing_note = note
        for vehicle in self._fleet.vehicles():
            if vehicle.is_running():
                vehicle.thread.queue_signing_key(key, self._signing_link_id)
        self._signing_state = WAITING if key else OFF
        self._publish_signing({})
        if key:
            self._append_log(f"Signing: key {fingerprint(key)} set. VGCS signs its commands to every drone.")
        else:
            self._append_log("Signing: key forgotten on this laptop. VGCS no longer signs its commands.")
        return note

    def _signing_set_passphrase(self, parent=None) -> None:
        parent = parent or self
        text, ok = QInputDialog.getText(
            parent, "Command signing",
            f"Passphrase (at least {MIN_PASSPHRASE} characters, longer is safer).\n"
            "Use the same passphrase in the VAMA APK and on every VGCS laptop that should command the drone.",
            QLineEdit.EchoMode.Password,
        )
        if not ok or not text:
            return
        again, ok = QInputDialog.getText(parent, "Command signing", "The same passphrase once more:", QLineEdit.EchoMode.Password)
        if not ok:
            return
        if again != text:
            QMessageBox.warning(parent, "Command signing", "The two passphrases differ. Nothing was changed.")
            return
        try:
            QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
            try:
                key = key_from_passphrase(text)
            finally:
                QApplication.restoreOverrideCursor()
        except ValueError as e:
            QMessageBox.warning(parent, "Command signing", str(e).capitalize() + ". Nothing was changed.")
            return
        note = self._signing_use_key(key)
        QMessageBox.information(
            parent, "Command signing",
            f"Key {fingerprint(key)} is set. VGCS now signs its commands.\n"
            "A drone that has no key yet still takes them, and so does it take everyone else's. "
            "Send the key to each drone to protect it."
            + (f"\n\n{note}" if note else ""),
        )

    def _signing_forget(self, parent=None) -> None:
        parent = parent or self
        if not self._signing_key:
            return
        answer = QMessageBox.question(
            parent, "Command signing",
            "Forget the key on this laptop?\n"
            "A drone that holds this key will then ignore VGCS's commands, until the same passphrase is set again.",
        )
        if answer == QMessageBox.StandardButton.Yes:
            self._signing_use_key(None)

    # --- the drone on screen ----------------------------------------------------
    def _signing_to_drone(self, enable: bool, parent=None, *, confirm: bool = True) -> bool:
        parent = parent or self
        if self._thread is None or not self._thread.isRunning():
            QMessageBox.warning(parent, "Command signing", "Connect the drone first.")
            return False
        if not self._signing_key:
            QMessageBox.warning(parent, "Command signing", "Set a passphrase first.")
            return False
        if confirm:
            if enable:
                text = (
                    f"Give the drone on screen key {self._signing_fingerprint()}?\n\n"
                    "From then on it obeys only ground stations with this key: this laptop, the APK and other "
                    "laptops with the same passphrase. Mission Planner or QGroundControl without the key can "
                    "still see it, but not command it.\n\n"
                    "The drone must be disarmed. Over USB the drone always accepts a new key, "
                    "so a lost passphrase can be replaced there."
                )
            else:
                text = (
                    "Switch signing off on the drone on screen?\n\n"
                    "It will then obey any ground station in radio range again."
                )
            if QMessageBox.question(parent, "Command signing", text) != QMessageBox.StandardButton.Yes:
                return False
        self._thread.queue_signing_to_drone(bool(enable))
        self._append_log("Signing: key sent to the drone" if enable else "Signing: asked the drone to stop signing")
        return True

    # --- what the dashboard shows -----------------------------------------------
    def _publish_signing(self, data: dict) -> None:
        """The "Command signing" value. data is a SIGNING payload from the link, or {} for this laptop's state."""
        field = getattr(self, "_fields", {}).get("signing")
        if data:
            self._signing_state = str(data.get("state") or self._signing_state)
            text = str(data.get("text") or "")
            level = str(data.get("level") or "na")
        else:
            self._signing_state = WAITING if self._signing_key else OFF
            text = state_text(self._signing_state, self._signing_fingerprint())
            if self._signing_key and not getattr(self, "_heartbeat_seen", False):
                text = f"On (key {self._signing_fingerprint()}): no drone connected"
            level = state_level(self._signing_state)
        if field is None:
            return
        if field.text() != text:
            field.setText(text)
        self._apply_state_style(field, level)

    def _build_signing_settings_group(self) -> QGroupBox:
        group = QGroupBox("Command signing (MAVLink 2)")
        grid = QGridLayout()
        status = QLabel()
        status.setObjectName("signingStatus")
        status.setWordWrap(True)

        def refresh() -> None:
            if self._signing_key:
                line = f"Key {self._signing_fingerprint()} is set on this laptop. " + state_text(
                    self._signing_state, self._signing_fingerprint()
                ) + "."
            else:
                line = "No key: VGCS does not sign its commands, and a drone takes commands from anyone in range."
            if self._signing_note:
                line += f" {self._signing_note}"
            status.setText(line)

        refresh()
        grid.addWidget(status, 0, 0, 1, 4)
        b_pass = QPushButton("Set passphrase...")
        b_send = QPushButton("Send key to the drone")
        b_remove = QPushButton("Remove key from the drone")
        b_forget = QPushButton("Forget key")
        b_pass.clicked.connect(lambda: (self._signing_set_passphrase(group.window()), refresh()))
        b_send.clicked.connect(lambda: (self._signing_to_drone(True, group.window()), refresh()))
        b_remove.clicked.connect(lambda: (self._signing_to_drone(False, group.window()), refresh()))
        b_forget.clicked.connect(lambda: (self._signing_forget(group.window()), refresh()))
        for col, b in enumerate((b_pass, b_send, b_remove, b_forget)):
            grid.addWidget(b, 1, col)
        hint = QLabel(
            "With a key, VGCS signs every command it sends (256-bit key, MAVLink 2 signing). A drone that holds "
            "the key ignores commands that are not signed with it, so nobody else in radio range can arm it, "
            "change its mode or its mission. It does not encrypt: telemetry and video can still be received. "
            "The same passphrase works in the VAMA APK (Application Settings, MAVLink signing keys). "
            "The passphrase is not stored. The key is, encrypted for this Windows user."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #aab4c8; font-size: 11px;")
        grid.addWidget(hint, 2, 0, 1, 4)
        group.setLayout(grid)
        return group
