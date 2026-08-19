"""DOOAF setup — enter military-supplied artillery and target coordinates."""

from __future__ import annotations

from PySide6.QtCore import QSettings, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QInputDialog,
    QPushButton,
    QVBoxLayout,
)

from vgcs.observe.dooaf import (
    ASSUMED_GUN_DIRECTIONS,
    DooafPreset,
    DooafSettings,
    delete_dooaf_preset,
    load_dooaf_presets,
    upsert_dooaf_preset,
    validate_dooaf_settings,
)

DOOAF_PICK_GUN = "gun"
DOOAF_PICK_TARGET = "target"

# Video pick modes (DOOAF Setup → Pick on video)
DOOAF_VIDEO_PICK_GROUND = "ground"
DOOAF_VIDEO_PICK_FACADE_LRF = "facade_lrf"


def _coord_edit(value: float | None = None) -> QLineEdit:
    edit = QLineEdit()
    edit.setPlaceholderText("e.g. 12.9716000")
    edit.setClearButtonEnabled(True)
    edit.setMinimumWidth(160)
    if value is not None:
        edit.setText(f"{float(value):.7f}")
    return edit


def _parse_coord(text: str) -> float | None:
    t = str(text or "").strip().replace(",", ".")
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
    assumed_gun_bearing_deg: float | None = None,
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
    if assumed_gun_bearing_deg is not None:
        # The gun is not being surveyed; drop any coordinate left in the fields
        # so a stale one from an earlier session cannot silently win over the
        # assumed direction in build_dooaf_session.
        glat = glon = None
        has_gun = False
    return DooafSettings(
        # Keep whichever of lat/lon parsed even if its sibling didn't, so
        # validate_dooaf_settings' "needs both latitude and longitude" check
        # can actually see and report the partial input instead of it being
        # silently nulled out here first.
        gun_lat=glat,
        gun_lon=glon,
        gun_alt_m=_optional_alt_value(gun_alt) if has_gun else None,
        target_lat=tlat,
        target_lon=tlon,
        target_alt_m=_optional_alt_value(tgt_alt) if has_tgt else None,
        assumed_gun_bearing_deg=assumed_gun_bearing_deg,
    )


class DooafSetupDialog(QDialog):
    """Popup for fixed artillery position and actual target lat/lon (military grid)."""

    pick_point_requested = Signal(str)
    pick_video_requested = Signal(str)
    pick_video_facade_lrf_requested = Signal(str)
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

        preset_box = QGroupBox("Saved positions")
        preset_lay = QHBoxLayout(preset_box)
        preset_lay.setContentsMargins(8, 8, 8, 8)
        self._preset_combo = QComboBox()
        self._preset_combo.setMinimumWidth(180)
        preset_lay.addWidget(self._preset_combo, 1)
        btn_save_preset = QPushButton("Save…")
        btn_save_preset.setToolTip("Save current gun + target coordinates as a named preset.")
        btn_save_preset.clicked.connect(self._save_preset)
        btn_delete_preset = QPushButton("Delete")
        btn_delete_preset.clicked.connect(self._delete_preset)
        preset_lay.addWidget(btn_save_preset)
        preset_lay.addWidget(btn_delete_preset)
        root.addWidget(preset_box)
        self._reload_presets()
        self._preset_combo.currentIndexChanged.connect(self._on_preset_selected)

        s = settings or DooafSettings()

        gun_box = QGroupBox("Artillery position (gun origin)")
        gun_form = QFormLayout(gun_box)
        # Skip the gun survey entirely: mark only target and impact and take the
        # artillery as sitting on a known side. Client request 2026-08-19 — the
        # gun is fixed to the south, firing north, so range/deflection come out
        # as north-south / east-west without a gun pick.
        self._gun_assumed_chk = QCheckBox("No gun position — artillery is:")
        self._gun_assumed_chk.setToolTip(
            "Mark only the target and the impact. The artillery is taken to sit on "
            "the chosen side, and range/deflection are given along that firing line.\n"
            "Gun-to-target distance is unknown in this mode and is not reported."
        )
        self._gun_assumed_dir = QComboBox()
        for label, deg in ASSUMED_GUN_DIRECTIONS:
            self._gun_assumed_dir.addItem(label, float(deg))
        assumed_row = QHBoxLayout()
        assumed_row.setContentsMargins(0, 0, 0, 0)
        assumed_row.setSpacing(6)
        assumed_row.addWidget(self._gun_assumed_chk)
        assumed_row.addWidget(self._gun_assumed_dir, 1)
        gun_form.addRow("", assumed_row)
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
            "Click gun position on video — mark stays on your click. "
            "Uses GPS + DEM ray (open ground / hills; no LRF slew)."
        )
        btn_pick_gun_vid.clicked.connect(
            lambda: self.pick_video_requested.emit(DOOAF_PICK_GUN)
        )
        btn_pick_gun_lrf = QPushButton("LRF lock (facade)")
        btn_pick_gun_lrf.setToolTip(
            "Click a point on a building face — camera slews to centre and "
            "one LRF lock enables fast TARGET/IMPACT picks. If you already picked "
            "the gun on open ground, the gun position is kept and only slant is stored."
        )
        btn_pick_gun_lrf.clicked.connect(
            lambda: self.pick_video_facade_lrf_requested.emit(DOOAF_PICK_GUN)
        )
        btn_clear_gun = QPushButton("Clear")
        btn_clear_gun.clicked.connect(self._clear_gun)
        gun_actions.addWidget(btn_pick_gun)
        gun_actions.addWidget(btn_pick_gun_vid)
        gun_actions.addWidget(btn_pick_gun_lrf)
        gun_actions.addWidget(btn_clear_gun)
        gun_actions.addStretch(1)
        gun_form.addRow("", gun_actions)
        root.addWidget(gun_box)

        self._gun_coord_widgets = (
            self._gun_lat,
            self._gun_lon,
            self._gun_alt,
            btn_pick_gun,
            btn_pick_gun_vid,
            btn_pick_gun_lrf,
            btn_clear_gun,
        )
        self._gun_assumed_chk.toggled.connect(self._on_gun_assumed_toggled)
        if s.assumed_gun_bearing_deg is not None:
            idx = self._gun_assumed_dir.findData(float(s.assumed_gun_bearing_deg))
            if idx >= 0:
                self._gun_assumed_dir.setCurrentIndex(idx)
            self._gun_assumed_chk.setChecked(True)
        self._on_gun_assumed_toggled(self._gun_assumed_chk.isChecked())

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
            "Click target on the building face — mark at your click. "
            "After a facade LRF lock: fast pick on the same face. "
            "For open ground / hills only (not walls), use after LRF slant is set "
            "or when no building is involved."
        )
        btn_pick_tgt_vid.clicked.connect(
            lambda: self.pick_video_requested.emit(DOOAF_PICK_TARGET)
        )
        btn_pick_tgt_lrf = QPushButton("LRF lock (facade slant)")
        btn_pick_tgt_lrf.setToolTip(
            "Gun on open ground? Click the building face — one LRF lock records "
            "slant range for fast TARGET/IMPACT picks without moving the gun mark."
        )
        btn_pick_tgt_lrf.clicked.connect(
            lambda: self.pick_video_facade_lrf_requested.emit(DOOAF_PICK_TARGET)
        )
        btn_clear_tgt = QPushButton("Clear")
        btn_clear_tgt.clicked.connect(self._clear_target)
        tgt_actions.addWidget(btn_pick_tgt)
        tgt_actions.addWidget(btn_pick_tgt_vid)
        tgt_actions.addWidget(btn_pick_tgt_lrf)
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

    def _settings_store(self) -> QSettings:
        return QSettings("VGCS", "VGCS")

    def _reload_presets(self) -> None:
        self._preset_combo.blockSignals(True)
        self._preset_combo.clear()
        self._preset_combo.addItem("— Load preset —", "")
        for preset in load_dooaf_presets(self._settings_store()):
            self._preset_combo.addItem(preset.name, preset.name)
        self._preset_combo.blockSignals(False)

    def _apply_settings_to_form(self, settings: DooafSettings) -> None:
        self._gun_lat.setText(
            f"{float(settings.gun_lat):.7f}" if settings.gun_lat is not None else ""
        )
        self._gun_lon.setText(
            f"{float(settings.gun_lon):.7f}" if settings.gun_lon is not None else ""
        )
        _set_optional_alt(self._gun_alt, settings.gun_alt_m)
        self._tgt_lat.setText(
            f"{float(settings.target_lat):.7f}" if settings.target_lat is not None else ""
        )
        self._tgt_lon.setText(
            f"{float(settings.target_lon):.7f}" if settings.target_lon is not None else ""
        )
        _set_optional_alt(self._tgt_alt, settings.target_alt_m)

    def _on_preset_selected(self, _idx: int) -> None:
        name = str(self._preset_combo.currentData() or "").strip()
        if not name:
            return
        for preset in load_dooaf_presets(self._settings_store()):
            if preset.name == name:
                self._apply_settings_to_form(preset.settings)
                self.coordinates_changed.emit("all")
                break

    def _save_preset(self) -> None:
        name, ok = QInputDialog.getText(self, "Save preset", "Preset name:")
        if not ok:
            return
        label = str(name or "").strip()
        if not label:
            return
        settings = settings_from_edits(
            gun_lat=self._gun_lat,
            gun_lon=self._gun_lon,
            gun_alt=self._gun_alt,
            tgt_lat=self._tgt_lat,
            tgt_lon=self._tgt_lon,
            tgt_alt=self._tgt_alt,
            assumed_gun_bearing_deg=self._assumed_gun_bearing_deg(),
        )
        err = validate_dooaf_settings(settings)
        if err:
            QMessageBox.warning(self, "DOOAF Setup", err)
            return
        upsert_dooaf_preset(self._settings_store(), DooafPreset(name=label, settings=settings))
        self._reload_presets()
        idx = self._preset_combo.findData(label)
        if idx >= 0:
            self._preset_combo.setCurrentIndex(idx)

    def _delete_preset(self) -> None:
        name = str(self._preset_combo.currentData() or "").strip()
        if not name:
            return
        delete_dooaf_preset(self._settings_store(), name)
        self._reload_presets()

    def _assumed_gun_bearing_deg(self) -> float | None:
        """Gun→target firing bearing, or None when a real gun is being surveyed."""
        try:
            if not self._gun_assumed_chk.isChecked():
                return None
            val = self._gun_assumed_dir.currentData()
            return None if val is None else float(val)
        except Exception:
            return None

    def _on_gun_assumed_toggled(self, on: bool) -> None:
        """Grey out the gun coordinate entry when it is not being used.

        Leaving those fields live would let an operator type a gun position that
        is then silently discarded, which reads as the app ignoring their input.
        """
        assumed = bool(on)
        try:
            self._gun_assumed_dir.setEnabled(assumed)
            for w in getattr(self, "_gun_coord_widgets", ()):
                w.setEnabled(not assumed)
        except Exception:
            pass

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
            _set_optional_alt(self._gun_alt, float(alt_m) if alt_m is not None else None)
        elif role == DOOAF_PICK_TARGET:
            self._tgt_lat.setText(text_lat)
            self._tgt_lon.setText(text_lon)
            _set_optional_alt(self._tgt_alt, float(alt_m) if alt_m is not None else None)

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
            assumed_gun_bearing_deg=self._assumed_gun_bearing_deg(),
        )
