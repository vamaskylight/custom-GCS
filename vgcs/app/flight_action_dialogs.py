"""Ask before the aircraft arms, climbs, comes home or lands.

Requested 2026-09-10: "our drone needs to be fully autonomy, user can click on
the take off button then one popup should come for adding altitude and some
specific msg on that popup, after that drone will Arm automatically and will go
for that particular height which is added by user, after that same for land or
return button."

So the takeoff popup takes the height rather than only confirming one chosen
somewhere else, and each popup says plainly what the aircraft is about to do.
These are the last screens between an operator and spinning propellers, so the
message names the consequence rather than asking "are you sure".

Text builders are pure so the wording can be tested without a display.
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDialog,
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
)

# Matches the dashboard's takeoff altitude control, so the two cannot disagree.
TAKEOFF_MIN_ALT_M = 1.0
TAKEOFF_MAX_ALT_M = 200.0
TAKEOFF_DEFAULT_ALT_M = 15.0


def clamp_takeoff_alt_m(alt_m: float | None) -> float:
    """A height the aircraft can actually be asked for."""
    try:
        alt = float(alt_m)
    except (TypeError, ValueError):
        return TAKEOFF_DEFAULT_ALT_M
    if alt != alt:  # NaN
        return TAKEOFF_DEFAULT_ALT_M
    return max(TAKEOFF_MIN_ALT_M, min(TAKEOFF_MAX_ALT_M, alt))


def takeoff_message(alt_m: float) -> str:
    """What pressing Takeoff is about to do, in the order it happens."""
    alt = clamp_takeoff_alt_m(alt_m)
    return (
        f"The aircraft will arm itself and climb to {alt:.1f} m, then hold there.\n\n"
        "Propellers start turning as soon as you confirm. Stand clear and make "
        "sure the area above the aircraft is empty."
    )


def return_message() -> str:
    return (
        "The aircraft will fly back to the launch point and land there.\n\n"
        "It climbs to its configured return altitude first, so check nothing is "
        "in the way between here and home."
    )


def land_message() -> str:
    return (
        "The aircraft will descend and land where it is now.\n\n"
        "It does not return home first. Make sure the ground below it is clear."
    )


class TakeoffDialog(QDialog):
    """Height entry plus the consequence, before anything arms."""

    def __init__(self, parent=None, *, default_alt_m: float = TAKEOFF_DEFAULT_ALT_M) -> None:
        super().__init__(parent)
        self.setWindowTitle("Takeoff")
        self.setObjectName("takeoffDialog")
        self.setModal(True)

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Takeoff altitude"))

        row = QHBoxLayout()
        self._alt = QDoubleSpinBox()
        self._alt.setObjectName("takeoffAltitudeSpin")
        self._alt.setRange(TAKEOFF_MIN_ALT_M, TAKEOFF_MAX_ALT_M)
        self._alt.setDecimals(1)
        self._alt.setSingleStep(1.0)
        self._alt.setSuffix(" m")
        self._alt.setValue(clamp_takeoff_alt_m(default_alt_m))
        self._alt.valueChanged.connect(self._refresh_message)
        row.addWidget(self._alt, 1)
        layout.addLayout(row)

        self._message = QLabel("")
        self._message.setObjectName("takeoffMessage")
        self._message.setWordWrap(True)
        self._message.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        layout.addWidget(self._message)

        buttons = QHBoxLayout()
        buttons.addStretch(1)
        self._cancel = QPushButton("Cancel")
        self._cancel.setObjectName("takeoffCancelBtn")
        self._cancel.clicked.connect(self.reject)
        buttons.addWidget(self._cancel)
        self._go = QPushButton("Arm and take off")
        self._go.setObjectName("takeoffConfirmBtn")
        # Not the default button: the default is the one an accidental Return
        # key presses, and that must not be the one that flies the aircraft.
        self._go.setDefault(False)
        self._go.setAutoDefault(False)
        self._cancel.setDefault(True)
        self._go.clicked.connect(self.accept)
        buttons.addWidget(self._go)
        layout.addLayout(buttons)

        self._refresh_message()

    def _refresh_message(self) -> None:
        self._message.setText(takeoff_message(self._alt.value()))

    def message_text(self) -> str:
        return str(self._message.text())

    def altitude_m(self) -> float:
        return clamp_takeoff_alt_m(self._alt.value())

    def set_altitude_m(self, alt_m: float) -> None:
        self._alt.setValue(clamp_takeoff_alt_m(alt_m))


def ask_takeoff_altitude(parent, default_alt_m: float) -> float | None:
    """Height to climb to, or None if the operator backed out."""
    dlg = TakeoffDialog(parent, default_alt_m=default_alt_m)
    if dlg.exec() != QDialog.DialogCode.Accepted:
        return None
    return dlg.altitude_m()


def _confirm(parent, title: str, text: str, ok_label: str) -> bool:
    box = QMessageBox(parent)
    box.setWindowTitle(title)
    box.setText(text)
    box.setIcon(QMessageBox.Icon.Warning)
    go = box.addButton(ok_label, QMessageBox.ButtonRole.AcceptRole)
    cancel = box.addButton("Cancel", QMessageBox.ButtonRole.RejectRole)
    box.setDefaultButton(cancel)
    box.exec()
    return box.clickedButton() is go


def confirm_return(parent) -> bool:
    return _confirm(parent, "Return", return_message(), "Return to launch")


def confirm_land(parent) -> bool:
    return _confirm(parent, "Land", land_message(), "Land here")


def disarm_message() -> str:
    return (
        "The motors will stop and the aircraft will be disarmed.\n\n"
        "The aircraft refuses this while it still believes it is flying, so it "
        "is for after landing. To stop the motors in an emergency, use "
        "EMERGENCY STOP instead."
    )


def confirm_disarm(parent) -> bool:
    return _confirm(parent, "Disarm", disarm_message(), "Disarm")
