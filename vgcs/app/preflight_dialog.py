"""The connect-time readiness popup.

Requested 2026-09-01: "when our drone is connected then one popup should come
and in that popup battery voltage, gps sats and motors are ready or not ... if
sats are good then in front of sats a right button ✅ automatically visible".

Two decisions worth stating, because both were driven by how the operator
actually uses this:

**It is live, not a snapshot.** On connect the autopilot has told us nothing —
no fix, no battery, no arming verdict. A checklist rendered once at that moment
would read "unknown" across the board and be useless. So it subscribes to the
same telemetry the dashboard uses and re-renders as answers arrive, which is
what "the tick appears automatically" means.

**It is modeless.** A modal dialog over a GCS during pre-flight is a hazard: it
blocks the map, the video and the flight controls while the operator is stood
next to an armed aircraft. This one can be left open or dismissed, and never
takes control away.

The verdict logic lives in :mod:`vgcs.app.preflight_checklist` and is tested
without Qt. This file is presentation only.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from vgcs.app.preflight_checklist import (
    STATUS_FAIL,
    STATUS_PASS,
    PreflightCheck,
    battery_is_present,
    build_preflight_checks,
    checklist_is_ready,
    checklist_summary,
)

_REFRESH_MS = 500

_TICK_STYLE = {
    STATUS_PASS: ("✔", "#16a34a"),
    STATUS_FAIL: ("✖", "#dc2626"),
}
_TICK_UNKNOWN = ("…", "#94a3b8")

_MOTOR_TEST_TOOLTIP = (
    "Spins each motor briefly at low throttle, in board output order."
    + chr(10) + "REMOVE PROPELLERS FIRST."
)
_MOTOR_TEST_NO_BATTERY = (
    "Connect the flight battery first - the ESCs are unpowered on USB."
)


class PreflightDialog(QDialog):
    """Live pre-arm checklist. `provider` returns the kwargs for
    :func:`build_preflight_checks`, so this widget never touches telemetry."""

    def __init__(self, provider, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._provider = provider
        self.setWindowTitle("Pre-flight check")
        self.setModal(False)              # never block the operator
        self.setMinimumWidth(430)

        root = QVBoxLayout(self)
        root.setContentsMargins(18, 16, 18, 14)
        root.setSpacing(10)

        self._summary = QLabel("Waiting for the autopilot…")
        self._summary.setWordWrap(True)
        self._summary.setStyleSheet("font-size:15px;font-weight:700;")
        root.addWidget(self._summary)

        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet("color:#e2e8f0;")
        root.addWidget(line)

        self._rows_host = QVBoxLayout()
        self._rows_host.setSpacing(8)
        root.addLayout(self._rows_host)
        self._rows: dict[str, tuple[QLabel, QLabel, QLabel]] = {}

        root.addStretch(1)
        note = QLabel(
            "Only the vehicle's own arming checks decide whether it can fly. "
            "GPS and battery here are for information."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color:#64748b;font-size:11px;")
        root.addWidget(note)

        buttons = QHBoxLayout()
        # Spinning motors is an operator decision, never a side effect of the
        # popup opening. Requested 2026-09-02: "when it comes motor/ESC check
        # then motor should rotate accordingly".
        self._motor_test_btn = QPushButton("Test motors...")
        self._motor_test_btn.setToolTip(_MOTOR_TEST_TOOLTIP)
        self._motor_test_btn.clicked.connect(self._on_motor_test_clicked)
        self._motor_test_running = False
        self._motor_test_end = QTimer(self)
        self._motor_test_end.setSingleShot(True)
        self._motor_test_end.timeout.connect(lambda: self._set_motor_test_running(False))
        buttons.addWidget(self._motor_test_btn)
        buttons.addStretch(1)
        close = QPushButton("Close")
        close.clicked.connect(self.close)
        buttons.addWidget(close)
        root.addLayout(buttons)

        self._timer = QTimer(self)
        self._timer.setInterval(_REFRESH_MS)
        self._timer.timeout.connect(self.refresh)
        self._timer.start()
        self.refresh()

    # ---------------------------------------------------------------- #

    def _on_motor_test_clicked(self) -> None:
        """Confirm, then spin each motor in turn — or stop a run in progress.

        The confirmation is not a formality. This is the only control in VGCS
        that makes the aircraft move on the ground, and the person pressing it
        is usually stood over it. While the sequence runs the same button
        becomes the abort, because that is where the operator is already
        looking when they see a motor turn the wrong way.
        """
        if self._motor_test_running:
            self._stop_motor_test()
            return
        parent = self.parent()
        runner = getattr(parent, "run_motor_test", None)
        if not callable(runner):
            QMessageBox.information(
                self, "Motor test", "Motor test is not available on this link."
            )
            return
        answer = QMessageBox.warning(
            self,
            "Motor test - remove propellers",
            "This will spin each motor in turn at low throttle.\n\n"
            "REMOVE THE PROPELLERS before continuing, and keep clear of the "
            "aircraft.\n\nWatch that each motor turns, in the right order and "
            "the right direction.\n\nStart the test?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        try:
            seconds = float(runner() or 0.0)
        except Exception as e:
            QMessageBox.critical(self, "Motor test", f"Could not start the test:\n{e}")
            return
        if seconds <= 0.0:
            return
        self._set_motor_test_running(True)
        # Return the button to its idle state when the sequence has finished,
        # so a stale "Stop" never suggests motors are still turning.
        self._motor_test_end.setInterval(int(seconds * 1000) + 500)
        self._motor_test_end.start()

    def _stop_motor_test(self) -> None:
        self._motor_test_end.stop()
        stopper = getattr(self.parent(), "stop_motor_test", None)
        if callable(stopper):
            try:
                stopper()
            except Exception:
                pass
        self._set_motor_test_running(False)

    def _set_motor_test_available(self, available: bool) -> None:
        if self._motor_test_running:
            return                      # never disable the abort mid-run
        self._motor_test_btn.setEnabled(bool(available))
        self._motor_test_btn.setToolTip(
            _MOTOR_TEST_TOOLTIP if available else _MOTOR_TEST_NO_BATTERY
        )

    def _set_motor_test_running(self, running: bool) -> None:
        self._motor_test_running = bool(running)
        self._motor_test_btn.setText("Stop motor test" if running else "Test motors...")

    def refresh(self) -> None:
        try:
            kwargs = dict(self._provider() or {})
        except Exception:
            return
        try:
            checks = build_preflight_checks(**kwargs)
        except Exception:
            return
        # ESCs run off the flight pack, so on USB alone the test would command
        # motors that have no power to turn. Say that instead of letting the
        # operator press it and conclude the motors are dead.
        self._set_motor_test_available(
            battery_is_present(kwargs.get("battery_voltage_v")) is not False
        )
        self._render(checks)

    def _render(self, checks: list[PreflightCheck]) -> None:
        for check in checks:
            if check.key not in self._rows:
                self._rows[check.key] = self._add_row()
            tick, label, detail = self._rows[check.key]
            glyph, colour = _TICK_STYLE.get(check.status, _TICK_UNKNOWN)
            tick.setText(glyph)
            tick.setStyleSheet(f"font-size:19px;font-weight:800;color:{colour};")
            label.setText(check.label)
            detail.setText(check.detail)

        ready = checklist_is_ready(checks)
        self._summary.setText(checklist_summary(checks))
        self._summary.setStyleSheet(
            "font-size:15px;font-weight:700;color:"
            + ("#15803d" if ready else "#b45309")
            + ";"
        )

    def _add_row(self) -> tuple[QLabel, QLabel, QLabel]:
        row = QHBoxLayout()
        row.setSpacing(12)

        tick = QLabel("…")
        tick.setFixedWidth(26)
        tick.setAlignment(Qt.AlignmentFlag.AlignCenter)
        row.addWidget(tick)

        text = QVBoxLayout()
        text.setSpacing(1)
        label = QLabel("")
        label.setStyleSheet("font-weight:600;")
        detail = QLabel("")
        detail.setWordWrap(True)
        detail.setStyleSheet("color:#475569;font-size:12px;")
        text.addWidget(label)
        text.addWidget(detail)
        row.addLayout(text, 1)

        self._rows_host.addLayout(row)
        return (tick, label, detail)

    def closeEvent(self, event) -> None:  # pragma: no cover - Qt lifecycle
        try:
            self._timer.stop()
        except Exception:
            pass
        # Closing the window must not leave motors spinning with no visible
        # control to stop them.
        if self._motor_test_running:
            try:
                self._stop_motor_test()
            except Exception:
                pass
        super().closeEvent(event)
