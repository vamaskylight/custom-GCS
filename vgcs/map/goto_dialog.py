"""Ask for a position, then go there or make a waypoint of it.

Requested 2026-09-08: "We want to add the latlong then we can see the latlong
and I should select that latlong in the waypoint 1 or 2 etc."

This replaced a plain QInputDialog. Two things it could not do matter here. It
had one OK button, so a typed position could only be looked at and never used;
and when the text could not be read it closed anyway and reported into the map
status line, which dashboard mode hides, so a mistyped position looked exactly
like a working one that had gone somewhere unseen.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
)

from vgcs.map.coordinate_input import describe_formats, parse_location

BAD_POSITION_TEXT = (
    "Could not read that position. Use one of the formats above."
)


class GotoLocationDialog(QDialog):
    """Position entry with both of the things an operator wants to do with one."""

    go_requested = Signal(str)
    waypoint_requested = Signal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Go to position")
        self.setObjectName("gotoLocationDialog")

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Latitude and longitude, or a grid reference:"))
        formats = QLabel(describe_formats())
        formats.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(formats)

        self._edit = QLineEdit()
        self._edit.setObjectName("gotoLocationEdit")
        self._edit.setPlaceholderText("20.4101472, 72.8798915")
        self._edit.textChanged.connect(self._on_text_changed)
        layout.addWidget(self._edit)

        # Its own label rather than the map status line, which is hidden in
        # dashboard mode: a refusal has to be readable where it happened.
        self._error = QLabel("")
        self._error.setObjectName("gotoLocationError")
        self._error.setWordWrap(True)
        self._error.setStyleSheet("color: #ff6b6b;")
        self._error.hide()
        layout.addWidget(self._error)

        buttons = QHBoxLayout()
        buttons.addStretch(1)
        self._btn_waypoint = QPushButton("Add waypoint")
        self._btn_waypoint.setObjectName("gotoAddWaypointBtn")
        self._btn_waypoint.setToolTip(
            "Add a waypoint at this position and select it in the mission"
        )
        self._btn_waypoint.clicked.connect(self._on_add_waypoint)
        buttons.addWidget(self._btn_waypoint)

        self._btn_go = QPushButton("Show on map")
        self._btn_go.setObjectName("gotoShowBtn")
        self._btn_go.setDefault(True)
        self._btn_go.clicked.connect(self._on_go)
        buttons.addWidget(self._btn_go)

        self._btn_cancel = QPushButton("Cancel")
        self._btn_cancel.setObjectName("gotoCancelBtn")
        self._btn_cancel.clicked.connect(self.reject)
        buttons.addWidget(self._btn_cancel)
        layout.addLayout(buttons)

        self._edit.setFocus()

    # ---------------------------------------------------------------- helpers
    def text(self) -> str:
        return str(self._edit.text() or "").strip()

    def set_text(self, text: str) -> None:
        self._edit.setText(str(text or ""))

    def error_text(self) -> str:
        return "" if self._error.isHidden() else str(self._error.text())

    def _on_text_changed(self, _text: str) -> None:
        # Clear the refusal as soon as they start fixing it.
        self._error.hide()
        self._error.setText("")

    def _accepted_position(self) -> bool:
        """True when the box holds something readable; complains in place if not."""
        if parse_location(self.text()) is not None:
            return True
        self._error.setText(BAD_POSITION_TEXT)
        self._error.show()
        return False

    # ---------------------------------------------------------------- actions
    def _on_go(self) -> None:
        if not self._accepted_position():
            return
        self.go_requested.emit(self.text())
        self.accept()

    def _on_add_waypoint(self) -> None:
        if not self._accepted_position():
            return
        self.waypoint_requested.emit(self.text())
        self.accept()
