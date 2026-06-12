"""DOOAF setup — enter military-supplied artillery and target coordinates."""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
)

from vgcs.observe.dooaf import DooafSettings, validate_dooaf_settings

DOOAF_PICK_GUN = "gun"
DOOAF_PICK_TARGET = "target"


def _coord_edit(value: float | None = None) -> QLineEdit:
    edit = QLineEdit()
    edit.setPlaceholderText("e.g. 12.9716000")
    edit.setClearButtonEnabled(True)
    edit.setMinimumWidth(160)
    if value is not None:
        edit.setText(f"{float(value):.7f}")
    return edit


def _parse_coord(text: str) -> float | None:
    t = str(text or "").strip()
    if not t:
        return None
    try:
        v = float(t)
    except ValueError:
        return None
    if not (-180.0 <= v <= 180.0):
        return None
    return v


def _optional_alt_spin() -> QDoubleSpinBox:
    spin = QDoubleSpinBox()
    spin.setRange(-500.0, 12000.0)
    spin.setDecimals(1)
    spin.setSingleStep(1.0)
    spin.setSpecialValueText("—")
    spin.setMinimum(-500.0)
    spin.setValue(-500.0)
    spin.setSuffix(" m")
    spin.setMinimumWidth(160)
    return spin


def _set_optional_alt(spin: QDoubleSpinBox, value: float | None) -> None:
    if value is None:
        spin.setValue(spin.minimum())
    else:
        spin.setValue(float(value))


def _optional_alt_value(spin: QDoubleSpinBox) -> float | None:
    if spin.value() <= spin.minimum() + 0.01:
        return None
    return float(spin.value())


def settings_from_edits(
    *,
    gun_lat: QLineEdit,
    gun_lon: QLineEdit,
    gun_alt: QDoubleSpinBox,
    tgt_lat: QLineEdit,
    tgt_lon: QLineEdit,
    tgt_alt: QDoubleSpinBox,
) -> DooafSettings:
    glat = _parse_coord(gun_lat.text())
    glon = _parse_coord(gun_lon.text())
    tlat = _parse_coord(tgt_lat.text())
    tlon = _parse_coord(tgt_lon.text())
    if glat is not None and not (-90.0 <= glat <= 90.0):
        glat = None
    if tlat is not None and not (-90.0 <= tlat <= 90.0):
        tlat = None
    has_gun = glat is not None and glon is not None
    has_tgt = tlat is not None and tlon is not None
    return DooafSettings(
        gun_lat=glat if has_gun else None,
        gun_lon=glon if has_gun else None,
        gun_alt_m=_optional_alt_value(gun_alt) if has_gun else None,
        target_lat=tlat if has_tgt else None,
        target_lon=tlon if has_tgt else None,
        target_alt_m=_optional_alt_value(tgt_alt) if has_tgt else None,
    )


class DooafSetupDialog(QDialog):
    """Popup for fixed artillery position and actual target lat/lon (military grid)."""

    pick_point_requested = Signal(str)
    pick_video_requested = Signal(str)
    coordinates_changed = Signal(str)

    def __init__(
        self,
        parent=None,
        *,
        settings: DooafSettings | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("DOOAF Setup")
        self.setModal(True)
        self.resize(480, 380)
        self.setObjectName("dooafSetupDialog")

        root = QVBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(10)

        intro = QLabel(
            "Enter coordinates from military staff, pick on the map, or pick on the live "
            "video (geo from GPS + gimbal + DEM). Altitude (MSL) is auto-filled from your "
            "DEM file when omitted. The drone marks fall of shot on video for range and "
            "deflection correction."
        )
        intro.setWordWrap(True)
        root.addWidget(intro)

        s = settings or DooafSettings()

        gun_box = QGroupBox("Artillery position (gun origin)")
        gun_form = QFormLayout(gun_box)
        self._gun_lat = _coord_edit(s.gun_lat)
        self._gun_lon = _coord_edit(s.gun_lon)
        self._gun_alt = _optional_alt_spin()
        _set_optional_alt(self._gun_alt, s.gun_alt_m)
        gun_form.addRow("Latitude", self._gun_lat)
        gun_form.addRow("Longitude", self._gun_lon)
        gun_form.addRow("Altitude (optional)", self._gun_alt)
        gun_actions = QHBoxLayout()
        gun_actions.setContentsMargins(0, 0, 0, 0)
        gun_actions.setSpacing(6)
        btn_pick_gun = QPushButton("Pick on map")
        btn_pick_gun.setToolTip("Hide this dialog and click the map for gun position.")
        btn_pick_gun.clicked.connect(
            lambda: self.pick_point_requested.emit(DOOAF_PICK_GUN)
        )
        btn_pick_gun_vid = QPushButton("Pick on video")
        btn_pick_gun_vid.setToolTip(
            "Hide this dialog and click the ground in the live video feed "
            "(uses GPS + gimbal + DEM like a video mark)."
        )
        btn_pick_gun_vid.clicked.connect(
            lambda: self.pick_video_requested.emit(DOOAF_PICK_GUN)
        )
        btn_clear_gun = QPushButton("Clear")
        btn_clear_gun.clicked.connect(self._clear_gun)
        gun_actions.addWidget(btn_pick_gun)
        gun_actions.addWidget(btn_pick_gun_vid)
        gun_actions.addWidget(btn_clear_gun)
        gun_actions.addStretch(1)
        gun_form.addRow("", gun_actions)
        root.addWidget(gun_box)

        tgt_box = QGroupBox("Actual target point (officer coordinates)")
        tgt_form = QFormLayout(tgt_box)
        self._tgt_lat = _coord_edit(s.target_lat)
        self._tgt_lon = _coord_edit(s.target_lon)
        self._tgt_alt = _optional_alt_spin()
        _set_optional_alt(self._tgt_alt, s.target_alt_m)
        tgt_form.addRow("Latitude", self._tgt_lat)
        tgt_form.addRow("Longitude", self._tgt_lon)
        tgt_form.addRow("Altitude (optional)", self._tgt_alt)
        tgt_actions = QHBoxLayout()
        tgt_actions.setContentsMargins(0, 0, 0, 0)
        tgt_actions.setSpacing(6)
        btn_pick_tgt = QPushButton("Pick on map")
        btn_pick_tgt.setToolTip("Hide this dialog and click the map for target position.")
        btn_pick_tgt.clicked.connect(
            lambda: self.pick_point_requested.emit(DOOAF_PICK_TARGET)
        )
        btn_pick_tgt_vid = QPushButton("Pick on video")
        btn_pick_tgt_vid.setToolTip(
            "Hide this dialog and click the intended target on the live video feed."
        )
        btn_pick_tgt_vid.clicked.connect(
            lambda: self.pick_video_requested.emit(DOOAF_PICK_TARGET)
        )
        btn_clear_tgt = QPushButton("Clear")
        btn_clear_tgt.clicked.connect(self._clear_target)
        tgt_actions.addWidget(btn_pick_tgt)
        tgt_actions.addWidget(btn_pick_tgt_vid)
        tgt_actions.addWidget(btn_clear_tgt)
        tgt_actions.addStretch(1)
        tgt_form.addRow("", tgt_actions)
        root.addWidget(tgt_box)

        buttons = QDialogButtonBox()
        btn_clear_all = buttons.addButton(
            "Clear all", QDialogButtonBox.ButtonRole.ResetRole
        )
        btn_clear_all.clicked.connect(self._clear_all)
        buttons.addButton(QDialogButtonBox.StandardButton.Ok)
        buttons.addButton(QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self._on_accept)
        buttons.rejected.connect(self.reject)
        root.addWidget(buttons)

    def _clear_gun(self) -> None:
        self._gun_lat.clear()
        self._gun_lon.clear()
        _set_optional_alt(self._gun_alt, None)
        self.coordinates_changed.emit("gun")

    def _clear_target(self) -> None:
        self._tgt_lat.clear()
        self._tgt_lon.clear()
        _set_optional_alt(self._tgt_alt, None)
        self.coordinates_changed.emit("target")

    def _clear_all(self) -> None:
        self._gun_lat.clear()
        self._gun_lon.clear()
        _set_optional_alt(self._gun_alt, None)
        self._tgt_lat.clear()
        self._tgt_lon.clear()
        _set_optional_alt(self._tgt_alt, None)
        self.coordinates_changed.emit("all")

    def set_point_coords(
        self,
        role: str,
        lat: float,
        lon: float,
        *,
        alt_m: float | None = None,
    ) -> None:
        text_lat = f"{float(lat):.7f}"
        text_lon = f"{float(lon):.7f}"
        if role == DOOAF_PICK_GUN:
            self._gun_lat.setText(text_lat)
            self._gun_lon.setText(text_lon)
            if alt_m is not None:
                _set_optional_alt(self._gun_alt, float(alt_m))
        elif role == DOOAF_PICK_TARGET:
            self._tgt_lat.setText(text_lat)
            self._tgt_lon.setText(text_lon)
            if alt_m is not None:
                _set_optional_alt(self._tgt_alt, float(alt_m))

    def _on_accept(self) -> None:
        err = validate_dooaf_settings(self.result_settings())
        if err:
            QMessageBox.warning(self, "DOOAF Setup", err)
            return
        self.accept()

    def result_settings(self) -> DooafSettings:
        return settings_from_edits(
            gun_lat=self._gun_lat,
            gun_lon=self._gun_lon,
            gun_alt=self._gun_alt,
            tgt_lat=self._tgt_lat,
            tgt_lon=self._tgt_lon,
            tgt_alt=self._tgt_alt,
        )
