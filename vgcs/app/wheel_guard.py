"""Number and list boxes never take the mouse wheel: it goes on to the page (row B31).

Qt gives a spin box or a combo box every wheel step under the pointer, with or without
the keyboard focus, and the wheel gives such a box the focus on its way (Qt.WheelFocus).
In a page that scrolls, the wheel meant for the page changes each box that passes under
the pointer. On 2026-11-06 the client's V20 gimbal port went from 5000 to 4981, then
4977, while Application Settings scrolled, and Apply saved it: the gimbal and the torch
stopped. The same wheel can change a waypoint's height in Plan Flight, the takeoff
height, a fence or a parameter value.

Taking the wheel only with the focus is not enough. A box the operator has just clicked
to type in is still under the pointer when the page is scrolled next. On PySide6 6.11.1,
with real wheel events, such a port box went from 5000 to 4997 in 3 steps.

So in all of VGCS a QAbstractSpinBox or a QComboBox never takes a wheel step. The wheel
goes on to the widget behind it, so the page scrolls, and it no longer moves the keyboard
focus onto such a box (Qt.StrongFocus: a click or Tab gives it). Values change by typing,
the arrow buttons, the arrow keys, or the list. An open list still scrolls with the wheel.

One event filter on the QApplication does it, so boxes made later (Plan Flight's rows, new
dialogs) are covered too. It adds 1 to 2 us to each event in the main thread (this PC,
2026-11-06: 0.8 to 0.9 us per delivered event without it, 2.0 to 2.6 us with it).

VGCS_WHEEL_IN_BOXES=1 leaves Qt's own wheel steps, as before.
"""

from __future__ import annotations

import os

from PySide6.QtCore import QCoreApplication, QEvent, QObject, Qt
from PySide6.QtWidgets import QAbstractSpinBox, QApplication, QComboBox

_BOXES = (QAbstractSpinBox, QComboBox)
_WHEEL = QEvent.Type.Wheel
_POLISH = QEvent.Type.Polish


def wheel_in_boxes_wanted() -> bool:
    """True when VGCS_WHEEL_IN_BOXES=1 asks for Qt's own wheel steps in number and list boxes."""
    return str(os.environ.get("VGCS_WHEEL_IN_BOXES", "") or "").strip().lower() in ("1", "true", "yes", "on")


def _no_wheel_focus(box: QAbstractSpinBox | QComboBox) -> None:
    if box.focusPolicy() == Qt.FocusPolicy.WheelFocus:
        box.setFocusPolicy(Qt.FocusPolicy.StrongFocus)


class WheelGuard(QObject):
    """The filter: a wheel event for a number or list box is passed on, never taken."""

    def eventFilter(self, obj, event) -> bool:
        kind = event.type()
        if kind == _WHEEL:
            if isinstance(obj, _BOXES):
                # Ignored and not delivered: Qt then offers the wheel to the widgets behind
                # the box, up to the scroll area, which scrolls the page.
                event.ignore()
                return True
        elif kind == _POLISH and isinstance(obj, _BOXES):
            # Every widget is polished once before it is first shown.
            _no_wheel_focus(obj)
        return False


_guard: WheelGuard | None = None


def install_wheel_guard(app: QCoreApplication | None = None) -> WheelGuard | None:
    """Put the guard on the application, once. Returns it, or None without an application
    or with VGCS_WHEEL_IN_BOXES=1."""
    global _guard
    app = app if app is not None else QCoreApplication.instance()
    if app is None or wheel_in_boxes_wanted():
        return None
    if _guard is not None:
        try:
            if _guard.parent() is app:
                return _guard
        except RuntimeError:  # its application is gone
            pass
        _guard = None
    guard = WheelGuard(app)  # a child of the application: it lives as long as the application
    app.installEventFilter(guard)
    if isinstance(app, QApplication):
        # Boxes polished before the guard came lose the wheel's focus now.
        for widget in QApplication.allWidgets():
            if isinstance(widget, _BOXES):
                _no_wheel_focus(widget)
    _guard = guard
    return guard


def remove_wheel_guard() -> None:
    """Take the guard off the application again (for the tests). Boxes keep Qt.StrongFocus."""
    global _guard
    guard, _guard = _guard, None
    if guard is None:
        return
    try:
        app = guard.parent()
        if app is not None:
            app.removeEventFilter(guard)
        guard.deleteLater()
    except RuntimeError:  # its application is gone, and the guard with it
        pass
