"""Minimal main window — connection string, connect/disconnect, log."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from vgcs.link.mavlink_thread import MavlinkThread


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("VGCS — link test")
        self.resize(640, 420)

        self._thread: MavlinkThread | None = None

        conn_label = QLabel("MAVLink connection string:")
        self._conn_edit = QLineEdit()
        # Default: many ArduPilot SITL setups target this UDP endpoint (see README: Connect to ArduPilot SITL)
        self._conn_edit.setText("udp:127.0.0.1:14550")

        self._btn_connect = QPushButton("Connect")
        self._btn_disconnect = QPushButton("Disconnect")
        self._btn_disconnect.setEnabled(False)

        self._status = QLabel("Status: disconnected")
        self._hb = QLabel("Last HEARTBEAT: —")

        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setPlaceholderText("Connection log…")

        row = QHBoxLayout()
        row.addWidget(self._btn_connect)
        row.addWidget(self._btn_disconnect)

        layout = QVBoxLayout()
        layout.addWidget(conn_label)
        layout.addWidget(self._conn_edit)
        layout.addLayout(row)
        layout.addWidget(self._status)
        layout.addWidget(self._hb)
        layout.addWidget(self._log)

        central = QWidget()
        central.setLayout(layout)
        self.setCentralWidget(central)

        self._btn_connect.clicked.connect(self._on_connect)
        self._btn_disconnect.clicked.connect(self._on_disconnect)

    def _append_log(self, line: str) -> None:
        self._log.append(line)
        self._log.verticalScrollBar().setValue(self._log.verticalScrollBar().maximum())

    def _on_connect(self) -> None:
        if self._thread is not None and self._thread.isRunning():
            QMessageBox.warning(self, "VGCS", "Already connected.")
            return

        cs = self._conn_edit.text().strip()
        if not cs:
            QMessageBox.warning(self, "VGCS", "Enter a connection string.")
            return

        self._thread = MavlinkThread(cs)
        self._thread.log_line.connect(self._append_log)
        self._thread.error.connect(self._on_link_error)
        self._thread.link_up.connect(self._on_link_up)
        self._thread.link_down.connect(self._on_link_down)
        self._thread.heartbeat.connect(self._on_heartbeat)
        self._thread.finished.connect(self._on_thread_finished)

        self._btn_connect.setEnabled(False)
        self._conn_edit.setEnabled(False)
        self._status.setText("Status: connecting…")
        self._thread.start()

    def _on_disconnect(self) -> None:
        if self._thread is not None:
            self._thread.stop()
            if self._thread.isRunning():
                self._thread.wait(3000)

    def _on_link_up(self) -> None:
        self._status.setText("Status: connected")
        self._btn_disconnect.setEnabled(True)

    def _on_link_down(self) -> None:
        self._status.setText("Status: disconnected")
        self._hb.setText("Last HEARTBEAT: —")
        self._btn_connect.setEnabled(True)
        self._conn_edit.setEnabled(True)
        self._btn_disconnect.setEnabled(False)

    def _on_heartbeat(self, sysid: int, compid: int, mav_ver: int) -> None:
        self._hb.setText(
            f"Last HEARTBEAT: sys={sysid} comp={compid} mavlink_ver={mav_ver}"
        )

    def _on_link_error(self, text: str) -> None:
        self._append_log(f"Error: {text}")

    def _on_thread_finished(self) -> None:
        self._thread = None

    def closeEvent(self, event) -> None:  # noqa: N802 — Qt API
        self._on_disconnect()
        super().closeEvent(event)
