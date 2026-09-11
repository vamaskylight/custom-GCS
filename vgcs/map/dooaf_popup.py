"""The DOOAF read-out an operator actually sees when they mark a point.

Requested 2026-09-09 from a competitor's screen: "When I set the target then
one popup will come on the screen with latlong, IMGR and their altitude ...
After set the target and fall of shot then this popup will come with
correction. This is a simple way to do DOOAF."

VGCS already computed every one of those numbers. It reported them into the map
status line, which ``set_dashboard_mode`` hides and VGCS turns on at startup,
so in practice the answer was written somewhere the operator never looks. That
is the same defect as the map-click coordinates (2026-09-02) and the waypoint
reached notice (2026-09-08), which is why this puts it on the screen instead.

The text builders here are pure so the wording and the arithmetic can be tested
without a display.
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDialog,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
)

TARGET_HEADING = "Target"
IMPACT_HEADING = "Fall of Shot"
UNKNOWN = "unknown"


def _grid_reference(lat: float, lon: float) -> str:
    """Grid reference for a point, or empty when it cannot be produced.

    One call, so the Indian military grid can replace MGRS here alone once the
    crew send the DSM specification, rather than the popup being rebuilt.
    """
    try:
        from vgcs.observe.grid_reference import format_grid_reference

        return str(format_grid_reference(lat, lon) or "")
    except Exception:
        return ""


def point_block(heading: str, point: object) -> str:
    """Position, grid reference and height for one marked point."""
    if point is None:
        return ""
    try:
        lat = float(getattr(point, "lat"))
        lon = float(getattr(point, "lon"))
    except (TypeError, ValueError, AttributeError):
        return ""
    lines = [f"{heading}:", f"Lat, Lon: {lat:.5f}, {lon:.5f}"]
    gr = _grid_reference(lat, lon)
    if gr:
        lines.append(f"GR (MGRS): {gr}")
    alt = getattr(point, "alt_m", None)
    # Said plainly rather than shown as 0, which reads as sea level.
    lines.append(
        f"Alt: {float(alt):.0f} m MSL" if alt is not None else f"Alt: {UNKNOWN}"
    )
    return "\n".join(lines)


def correction_block(correction: object) -> str:
    """What to change so the next round lands on the target.

    Signs follow FireCorrection, where the correction fields are already the
    change to apply rather than the miss: positive deflection is right,
    positive range is add. Getting this backwards would send rounds twice as
    far wrong as doing nothing, so it is stated in words, never as a signed
    number the operator has to interpret under time pressure.
    """
    if correction is None:
        return ""
    lines = ["Correction:"]
    defl = getattr(correction, "deflection_correction_m", None)
    if defl is not None:
        side = "Right" if float(defl) >= 0 else "Left"
        lines.append(f"{side}: {abs(float(defl)):.0f} m")
    rng = getattr(correction, "range_correction_m", None)
    if rng is not None:
        way = "Add" if float(rng) >= 0 else "Drop"
        lines.append(f"{way}: {abs(float(rng)):.0f} m")
    miss = getattr(correction, "impact_to_intended_m", None)
    if miss is not None:
        lines.append(f"Miss: {float(miss):.0f} m")
    vert = getattr(correction, "elevation_correction_m", None)
    if vert is None:
        vert = getattr(correction, "miss_vertical_m", None)
    if vert is not None:
        lines.append(f"Altitude: {float(vert):.0f} m")
    return "\n".join(lines) if len(lines) > 1 else ""


def mean_correction_block(session: object) -> str:
    """The correction averaged over every round marked so far.

    Requested 2026-09-11: "we should be able to mark 2 impact Target and get
    the mean of that two correction in the final popup."

    Averaged as components along the firing line, never as Left and Add
    magnitudes. Two rounds 19 m and 13 m out average to 16 m by that method,
    but if they missed in opposite directions the real bias is nearer 3, and
    correcting by 16 would throw the next round further out than doing nothing.

    The spread is printed beside it because the two demand opposite responses.
    The mean is bias, and the correction cancels it. The spread is scatter, it
    is not correctable, and an operator who adjusts for it is chasing noise.
    """
    averaged = getattr(session, "averaged", None)
    if averaged is None:
        return ""
    rounds = int(getattr(averaged, "rounds", 0) or 0)
    if rounds < 2:
        # One round says nothing about a mean, and labelling it as one would
        # suggest a confidence a single observation cannot carry.
        return ""
    correction = getattr(session, "correction", None)
    bearing = getattr(correction, "bearing_gun_to_intended_deg", None)
    try:
        along, right = averaged.along_across(float(bearing or 0.0))
    except Exception:
        return ""
    # The stored values are the miss; the correction is its opposite, exactly
    # as FireCorrection derives range_correction_m = -along.
    corr_along = -float(along)
    corr_right = -float(right)
    lines = [f"Mean of {rounds} rounds:"]
    lines.append(
        f"{'Right' if corr_right >= 0 else 'Left'}: {abs(corr_right):.0f} m"
    )
    lines.append(f"{'Add' if corr_along >= 0 else 'Drop'}: {abs(corr_along):.0f} m")
    spread = getattr(averaged, "dispersion_m", None)
    if spread is not None:
        lines.append(f"Spread: {float(spread):.0f} m")
    return "\n".join(lines)


def dooaf_popup_text(session: object) -> str:
    """Everything known so far: target, fall of shot, and the correction.

    Marking only the target gives one block, which is what the operator sees
    first; marking the fall of shot adds the second and the correction. Nothing
    is invented to fill a gap.
    """
    if session is None:
        return ""
    blocks = [
        point_block(TARGET_HEADING, getattr(session, "intended", None)),
        point_block(IMPACT_HEADING, getattr(session, "impact", None)),
        correction_block(getattr(session, "correction", None)),
        # Last, because the correction above is the newest round and is what
        # gets applied now; the mean is the shoot as a whole.
        mean_correction_block(session),
    ]
    return "\n\n".join(b for b in blocks if b)


def gun_note(session: object) -> str:
    """States when the firing line was assumed rather than surveyed.

    Left in on purpose. An operator reading Left and Add off this screen is
    entitled to know the direction they are measured along was taken on trust.
    """
    if session is None or not getattr(session, "gun_is_assumed", False):
        return ""
    return "Artillery position not surveyed: correction is along an assumed firing line."


class DooafPopup(QDialog):
    """Modeless, so a read-out can never block the aircraft controls.

    The competitor's is modal with an OK button. This keeps the OK button and
    the look, and drops the blocking, because a dialog that has to be dismissed
    before anything else can be touched is the wrong thing to put in front of
    someone flying.
    """

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("DOOAF")
        self.setObjectName("dooafPopup")
        self.setModal(False)
        self.setWindowFlag(Qt.WindowType.Tool, True)

        layout = QVBoxLayout(self)
        self._body = QLabel("")
        self._body.setObjectName("dooafPopupBody")
        self._body.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        self._body.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self._body)

        self._note = QLabel("")
        self._note.setObjectName("dooafPopupNote")
        self._note.setWordWrap(True)
        self._note.hide()
        layout.addWidget(self._note)

        row = QHBoxLayout()
        row.addStretch(1)
        self._ok = QPushButton("OK")
        self._ok.setObjectName("dooafPopupOk")
        self._ok.clicked.connect(self.hide)
        row.addWidget(self._ok)
        row.addStretch(1)
        layout.addLayout(row)

    def body_text(self) -> str:
        return str(self._body.text())

    def note_text(self) -> str:
        return "" if self._note.isHidden() else str(self._note.text())

    def show_text(self, text: str, note: str = "") -> None:
        self._body.setText(str(text or ""))
        if note:
            self._note.setText(str(note))
            self._note.show()
        else:
            self._note.setText("")
            self._note.hide()
        self.adjustSize()
        self.show()
        self.raise_()
