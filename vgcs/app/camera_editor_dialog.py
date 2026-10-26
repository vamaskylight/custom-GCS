"""The cameras VGCS knows, for the admin: the camera editor (task AA, VGCS part).

Application Settings opens it ("Cameras...", beside the camera profile) after the admin
password (admin_password.py). It is the APK's editor (VamaCameraEditor.qml) in VGCS:
 - A built-in camera (V12, V13, V14 Pro) is copied, never changed.
 - An added camera is a file in VGCS's folder of cameras (camera_files.default_camera_folder()).
   The APK reads the same files: a camera made here works there once its file is copied over,
   and the other way round.
 - Every camera speaks Skydroid's TOP protocol.
The form is read as the APK reads its form: through the file reader (camera_files.from_json),
so the same values give the same camera and the same sentences in both apps.

VGCS uses an added camera's addresses, lenses and zoom, photo and recording. Its tilt and
turn limits, laser and own tracker are kept in the file for the APK; VGCS does not use them.
"""

from __future__ import annotations

import ipaddress
import math
from dataclasses import replace
from typing import Callable
from urllib.parse import urlparse

from PySide6.QtCore import Qt, QUrl
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from vgcs.skydroid.camera_files import (
    FORMAT,
    MAX_LENSES,
    PROTOCOL,
    SKYDROID_HOST,
    TRANSPORT_TCP,
    TRANSPORT_UDP,
    ZOOM_COUNTED,
    ZOOM_NONE,
    ZOOM_REPORTED,
    Camera,
    CameraStore,
    from_json,
    profile_id_for,
)

# How the control commands reach the camera, in the order of the box (as the APK's).
TRANSPORT_CHOICES = (
    (TRANSPORT_UDP, "UDP (as the V12, V13 and V14 Pro)"),
    (TRANSPORT_TCP, "TCP (as the V20)"),
)

# The zoom kinds, in the order of the zoom box (as the APK's).
ZOOM_CHOICES = (
    (ZOOM_NONE, "No zoom"),
    (ZOOM_COUNTED, "Steps of 0.1x (as the V12 and V13)"),
    (ZOOM_REPORTED, "The camera reports its zoom step (as the V14 Pro)"),
)

UNUSED_NOTE = (
    "VGCS uses the addresses, the lenses and zoom, photo and recording. "
    "The tilt and turn, the laser and the own tracker are kept for the app on the remote control. "
    "VGCS does not use them yet."
)

# (host, control port, day video, thermal video)
Addresses = tuple[str, int, str, str]


def _number(text: str) -> float:
    """A number as typed: a comma counts as the decimal point. Anything else is not a number."""
    t = str(text).strip().replace(",", ".")
    if not t:
        return math.nan
    try:
        return float(t)
    except ValueError:
        return math.nan


def _shown(value) -> str:
    """A number as the form shows it: 83.4, 30, 1000. Not a number: empty."""
    if isinstance(value, int) and not isinstance(value, bool):
        return str(value)
    try:
        v = float(value)
    except (TypeError, ValueError):
        return ""
    return format(v, ".10g") if math.isfinite(v) else ""


def _host_of(url: str) -> str:
    try:
        host = urlparse(str(url).strip()).hostname or ""
        ipaddress.ip_address(host)
    except ValueError:
        return ""
    return host


class CameraEditorDialog(QDialog):
    """The list of cameras (Copy, Change, Delete, Open folder), and the form of one camera."""

    def __init__(
        self,
        store: CameraStore | None = None,
        *,
        in_use_profile_id: str = "",
        addresses_in_use: Callable[[], Addresses] | None = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Cameras")
        self.setObjectName("cameraEditorDialog")
        self.setModal(True)
        self.resize(760, 680)
        self._store = store if store is not None else CameraStore()
        self._in_use = str(in_use_profile_id or "")
        self._addresses_in_use = addresses_in_use
        self._editing: Camera | None = None  # the camera in the form (id "": a new one)
        self._lens_rows: list[tuple[list[QWidget], list[QLineEdit]]] = []  # each row's widgets, and its fields
        # What changed while the window was open: Application Settings refreshes from these.
        self.saved_ids: set[str] = set()
        self.deleted_ids: set[str] = set()

        outer = QVBoxLayout(self)
        self._pages = QStackedWidget()
        outer.addWidget(self._pages)
        self._pages.addWidget(self._make_list_page())
        self._pages.addWidget(self._make_form_page())
        self._reload()

    # ------------------------------------------------------------------ the list
    def _make_list_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        title = QLabel("Cameras")
        title.setStyleSheet("font-weight: bold;")
        layout.addWidget(title)
        self._about = QLabel("")
        self._about.setObjectName("cameraAbout")
        self._about.setWordWrap(True)
        self._about.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self._about)
        self._notes = QLabel("")
        self._notes.setObjectName("cameraNotes")
        self._notes.setWordWrap(True)
        self._notes.setStyleSheet("color: #ffb454;")
        layout.addWidget(self._notes)

        self._list = QListWidget()
        self._list.setObjectName("cameraList")
        self._list.currentRowChanged.connect(lambda _row: self._enable_list_buttons())
        layout.addWidget(self._list, 1)

        self._list_problem = QLabel("")
        self._list_problem.setObjectName("cameraListProblem")
        self._list_problem.setWordWrap(True)
        self._list_problem.setStyleSheet("color: #ff6b6b;")
        self._list_problem.hide()
        layout.addWidget(self._list_problem)

        buttons = QHBoxLayout()
        self._copy_btn = self._button("Copy", "cameraCopyBtn", lambda: self._edit(as_copy=True))
        self._change_btn = self._button("Change", "cameraChangeBtn", lambda: self._edit(as_copy=False))
        self._delete_btn = self._button("Delete", "cameraDeleteBtn", self._delete)
        for b in (self._copy_btn, self._change_btn, self._delete_btn):
            buttons.addWidget(b)
        buttons.addStretch(1)
        buttons.addWidget(self._button("Open folder", "cameraOpenFolderBtn", self._open_folder))
        buttons.addWidget(self._button("Close", "cameraCloseBtn", self.accept))
        layout.addLayout(buttons)
        return page

    def _button(self, text: str, name: str, slot) -> QPushButton:
        b = QPushButton(text)
        b.setObjectName(name)
        b.setAutoDefault(False)
        b.clicked.connect(slot)
        return b

    def _reload(self, select_id: str = "") -> None:
        """Reads the folder again (files copied in meanwhile) and shows the list."""
        keep = select_id or self._selected_id()
        self._store.load()
        self._about.setText(
            "Copy a camera to add a new one. "
            "Every camera here speaks Skydroid's TOP protocol: a camera with another protocol needs a VGCS update. "
            f"Cameras you add are files in {self._store.folder}. "
            "Copy such a file into that folder on another computer, or into the Cameras folder of the app "
            "on a remote control, and the camera is in its list too."
        )
        notes = self._store.notes
        self._notes.setText("Files that are not read:\n" + "\n".join(notes) if notes else "")
        self._notes.setVisible(bool(notes))
        self._list.clear()
        for cam in self._store.cameras:
            text = cam.name
            if cam.built_in:
                text += "  (built in)"
            if self._in_use and profile_id_for(cam) == self._in_use:
                text += "  (in use)"
            item = QListWidgetItem(text)
            item.setData(Qt.ItemDataRole.UserRole, cam.id)
            self._list.addItem(item)
        row = next((i for i, c in enumerate(self._store.cameras) if c.id == keep), 0)
        self._list.setCurrentRow(row if self._list.count() else -1)
        self._enable_list_buttons()

    def _selected_id(self) -> str:
        item = self._list.currentItem()
        return str(item.data(Qt.ItemDataRole.UserRole)) if item is not None else ""

    def _selected(self) -> Camera | None:
        return self._store.find(self._selected_id())

    def _enable_list_buttons(self) -> None:
        cam = self._selected()
        self._copy_btn.setEnabled(cam is not None)
        self._change_btn.setEnabled(cam is not None and not cam.built_in)
        self._delete_btn.setEnabled(cam is not None and not cam.built_in)

    def _show_list_problem(self, text: str) -> None:
        self._list_problem.setText(text)
        self._list_problem.setVisible(bool(text))

    def _delete(self) -> None:
        cam = self._selected()
        if cam is None or cam.built_in:
            return
        question = "Delete this camera? Its file is removed from this computer."
        if self._in_use and profile_id_for(cam) == self._in_use:
            question += "\nIt is the camera in use: VGCS then uses the V13's profile."
        answer = QMessageBox.question(
            self,
            "Delete camera",
            question,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        why = self._store.remove(cam.id)
        if not why:
            self.deleted_ids.add(cam.id)
            self.saved_ids.discard(cam.id)
        self._reload()
        self._show_list_problem(why)

    def _open_folder(self) -> None:
        try:
            self._store.folder.mkdir(parents=True, exist_ok=True)
        except OSError:
            self._show_list_problem(f"The folder for cameras cannot be made: {self._store.folder}")
            return
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(self._store.folder)))

    # ------------------------------------------------------------------ the form
    def _make_form_page(self) -> QWidget:
        page = QWidget()
        outer = QVBoxLayout(page)
        self._form_title = QLabel("")
        self._form_title.setObjectName("cameraFormTitle")
        self._form_title.setStyleSheet("font-weight: bold;")
        outer.addWidget(self._form_title)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        body = QWidget()
        form = QVBoxLayout(body)
        scroll.setWidget(body)
        outer.addWidget(scroll, 1)

        grid = QGridLayout()
        form.addLayout(grid)
        row = 0

        def line(label: str, name: str, placeholder: str = "") -> QLineEdit:
            nonlocal row
            edit = QLineEdit()
            edit.setObjectName(name)
            if placeholder:
                edit.setPlaceholderText(placeholder)
            grid.addWidget(QLabel(label), row, 0)
            grid.addWidget(edit, row, 1, 1, 2)
            row += 1
            return edit

        def pair(label: str, first: str, second: str) -> tuple[QLabel, QLineEdit, QLineEdit]:
            nonlocal row
            text = QLabel(label)
            a, b = QLineEdit(), QLineEdit()
            a.setObjectName(first)
            b.setObjectName(second)
            grid.addWidget(text, row, 0)
            grid.addWidget(a, row, 1)
            grid.addWidget(b, row, 2)
            row += 1
            return text, a, b

        self._name = line("Name", "cameraName")
        self._host = line("Camera IP address", "cameraHost")
        self._port = line("Control port", "cameraPort")
        self._transport = QComboBox()
        self._transport.setObjectName("cameraTransport")
        for kind, text in TRANSPORT_CHOICES:
            self._transport.addItem(text, kind)
        grid.addWidget(QLabel("Control link"), row, 0)
        grid.addWidget(self._transport, row, 1, 1, 2)
        row += 1
        self._day = line("Day video", "cameraDayVideo")
        self._thermal = line("Thermal video", "cameraThermalVideo", "Empty: no thermal picture")
        self._take = self._button("Take the addresses in use now", "cameraTakeAddresses", self._take_addresses)
        self._take.setVisible(self._addresses_in_use is not None)
        grid.addWidget(self._take, row, 1, 1, 2)
        row += 1

        self._zoom = QComboBox()
        self._zoom.setObjectName("cameraZoom")
        for kind, text in ZOOM_CHOICES:
            self._zoom.addItem(text, kind)
        self._zoom.currentIndexChanged.connect(lambda _i: self._show_zoom_fields())
        grid.addWidget(QLabel("Zoom"), row, 0)
        grid.addWidget(self._zoom, row, 1, 1, 2)
        row += 1
        self._fov_label, self._fov_h, self._fov_v = pair(
            "Day lens, across and up-down (degrees)", "cameraFovH", "cameraFovV"
        )
        self._max_zoom_label = QLabel("Highest zoom (times)")
        self._max_zoom = QLineEdit()
        self._max_zoom.setObjectName("cameraMaxZoom")
        grid.addWidget(self._max_zoom_label, row, 0)
        grid.addWidget(self._max_zoom, row, 1)
        row += 1

        # The lenses of a camera that reports its zoom step.
        self._lens_box = QWidget()
        lens_layout = QVBoxLayout(self._lens_box)
        lens_layout.setContentsMargins(0, 0, 0, 0)
        about_lenses = QLabel(
            "Each lens: its view across and up-down (degrees) at its first zoom step, then its first and "
            "last step. Each step crops the sensor picture by the points below."
        )
        about_lenses.setWordWrap(True)
        lens_layout.addWidget(about_lenses)
        self._lens_grid = QGridLayout()
        for column, heading in enumerate(("Across", "Up-down", "First step", "Last step"), start=1):
            self._lens_grid.addWidget(QLabel(heading), 0, column)
        lens_layout.addLayout(self._lens_grid)
        self._add_lens_btn = self._button("Add a lens", "cameraAddLens", self._add_lens)
        lens_layout.addWidget(self._add_lens_btn, 0, Qt.AlignmentFlag.AlignLeft)
        sensor_grid = QGridLayout()
        self._sensor_w, self._sensor_h = QLineEdit(), QLineEdit()
        self._sensor_w.setObjectName("cameraSensorW")
        self._sensor_h.setObjectName("cameraSensorH")
        self._crop_w, self._crop_h = QLineEdit(), QLineEdit()
        self._crop_w.setObjectName("cameraCropW")
        self._crop_h.setObjectName("cameraCropH")
        sensor_grid.addWidget(QLabel("Sensor points"), 0, 0)
        sensor_grid.addWidget(self._sensor_w, 0, 1)
        sensor_grid.addWidget(self._sensor_h, 0, 2)
        sensor_grid.addWidget(QLabel("Crop of a step"), 1, 0)
        sensor_grid.addWidget(self._crop_w, 1, 1)
        sensor_grid.addWidget(self._crop_h, 1, 2)
        lens_layout.addLayout(sensor_grid)
        form.addWidget(self._lens_box)

        grid2 = QGridLayout()
        form.addLayout(grid2)
        self._thermal_fov_h, self._thermal_fov_v = QLineEdit(), QLineEdit()
        self._thermal_fov_h.setObjectName("cameraThermalFovH")
        self._thermal_fov_v.setObjectName("cameraThermalFovV")
        grid2.addWidget(QLabel("Thermal lens, across and up-down (degrees)"), 0, 0)
        grid2.addWidget(self._thermal_fov_h, 0, 1)
        grid2.addWidget(self._thermal_fov_v, 0, 2)
        self._thermal_sure = QCheckBox("The thermal lens is measured")
        self._thermal_sure.setObjectName("cameraThermalSure")
        grid2.addWidget(self._thermal_sure, 1, 1, 1, 2)
        not_sure = QLabel("If it is not, VGCS takes its view as an estimate (from the data sheet).")
        not_sure.setWordWrap(True)
        grid2.addWidget(not_sure, 2, 1, 1, 2)
        self._pitch_min, self._pitch_max = QLineEdit(), QLineEdit()
        self._pitch_min.setObjectName("cameraPitchMin")
        self._pitch_max.setObjectName("cameraPitchMax")
        grid2.addWidget(QLabel("Tilt from, to (degrees)"), 3, 0)
        grid2.addWidget(self._pitch_min, 3, 1)
        grid2.addWidget(self._pitch_max, 3, 2)
        self._yaw_limit = QLineEdit()
        self._yaw_limit.setObjectName("cameraYawLimit")
        grid2.addWidget(QLabel("Turn either way (degrees)"), 4, 0)
        grid2.addWidget(self._yaw_limit, 4, 1)
        self._laser = QCheckBox("Laser")
        self._laser.setObjectName("cameraLaser")
        self._laser_max = QLineEdit()
        self._laser_max.setObjectName("cameraLaserMax")
        self._laser.toggled.connect(self._laser_max.setEnabled)
        grid2.addWidget(self._laser, 5, 0)
        grid2.addWidget(QLabel("Longest reading (m)"), 5, 1)
        grid2.addWidget(self._laser_max, 5, 2)
        self._photo = QCheckBox("Photo")
        self._photo.setObjectName("cameraPhoto")
        self._record = QCheckBox("Video recording")
        self._record.setObjectName("cameraRecord")
        grid2.addWidget(self._photo, 6, 0)
        grid2.addWidget(self._record, 6, 1)
        self._own_tracker = QCheckBox("Its own object tracker")
        self._own_tracker.setObjectName("cameraOwnTracker")
        self._upper_header = QCheckBox("Upper-case gimbal frames only (as the V14 Pro)")
        self._upper_header.setObjectName("cameraUpperHeader")
        grid2.addWidget(self._own_tracker, 7, 0)
        grid2.addWidget(self._upper_header, 7, 1, 1, 2)
        unused = QLabel(UNUSED_NOTE)
        unused.setWordWrap(True)
        form.addWidget(unused)
        form.addStretch(1)

        self._problem = QLabel("")
        self._problem.setObjectName("cameraProblem")
        self._problem.setWordWrap(True)
        self._problem.setStyleSheet("color: #ff6b6b;")
        self._problem.hide()
        outer.addWidget(self._problem)

        buttons = QHBoxLayout()
        buttons.addStretch(1)
        buttons.addWidget(self._button("Cancel", "cameraCancelBtn", self._back_to_list))
        save = self._button("Save", "cameraSaveBtn", self._save)
        buttons.addWidget(save)
        outer.addLayout(buttons)
        return page

    def _edit(self, *, as_copy: bool) -> None:
        """The form with a camera: an added camera's own fields, or a copy of any camera."""
        cam = self._selected()
        if cam is None or (cam.built_in and not as_copy):
            return
        if as_copy:
            cam = replace(cam, id="", name=f"{cam.name} copy", built_in=False)
        self._editing = cam
        self._form_title.setText("New camera" if not cam.id else "Change camera")
        self._fill(cam)
        self._show_problem("")
        self._pages.setCurrentIndex(1)
        self._name.setFocus()

    def _fill(self, cam: Camera) -> None:
        self._name.setText(cam.name)
        self._host.setText(cam.host)
        self._port.setText(_shown(cam.port))
        self._transport.setCurrentIndex(max(0, self._transport.findData(cam.transport)))
        self._day.setText(cam.day_video_url)
        self._thermal.setText(cam.thermal_video_url)
        self._zoom.setCurrentIndex(max(0, self._zoom.findData(cam.zoom)))
        self._fov_h.setText(_shown(cam.fov_h_deg))
        self._fov_v.setText(_shown(cam.fov_v_deg))
        self._max_zoom.setText(_shown(cam.max_zoom))
        self._set_lens_rows([
            [_shown(lens.fov_h_deg), _shown(lens.fov_v_deg), _shown(lens.first_step), _shown(lens.last_step)]
            for lens in cam.lenses
        ])
        self._sensor_w.setText(_shown(cam.sensor_w))
        self._sensor_h.setText(_shown(cam.sensor_h))
        self._crop_w.setText(_shown(cam.crop_step_w))
        self._crop_h.setText(_shown(cam.crop_step_h))
        self._thermal_fov_h.setText(_shown(cam.thermal_fov_h_deg))
        self._thermal_fov_v.setText(_shown(cam.thermal_fov_v_deg))
        self._thermal_sure.setChecked(cam.thermal_fov_sure)
        self._pitch_min.setText(_shown(cam.pitch_min_deg))
        self._pitch_max.setText(_shown(cam.pitch_max_deg))
        self._yaw_limit.setText(_shown(cam.yaw_limit_deg))
        self._laser.setChecked(cam.laser)
        self._laser_max.setText(_shown(cam.laser_max_m))
        self._laser_max.setEnabled(cam.laser)
        self._photo.setChecked(cam.photo)
        self._record.setChecked(cam.record)
        self._own_tracker.setChecked(cam.own_tracker)
        self._upper_header.setChecked(cam.upper_gimbal_header)
        self._show_zoom_fields()

    def _show_zoom_fields(self) -> None:
        kind = self._zoom.currentData()
        for w in (self._fov_label, self._fov_h, self._fov_v):
            w.setVisible(kind != ZOOM_REPORTED)
        self._max_zoom_label.setVisible(kind == ZOOM_COUNTED)
        self._max_zoom.setVisible(kind == ZOOM_COUNTED)
        self._lens_box.setVisible(kind == ZOOM_REPORTED)

    # The lens rows: across, up-down, first step, last step, and Remove.
    def _lens_texts(self) -> list[list[str]]:
        return [[edit.text() for edit in edits] for _row, edits in self._lens_rows]

    def _set_lens_rows(self, rows: list[list[str]]) -> None:
        for widgets, _edits in self._lens_rows:
            for widget in widgets:
                self._lens_grid.removeWidget(widget)
                widget.setParent(None)
                widget.deleteLater()
        self._lens_rows = []
        for i, texts in enumerate(rows):
            row = i + 1  # under the headings
            label = QLabel(f"Lens {i + 1}")
            self._lens_grid.addWidget(label, row, 0)
            edits = []
            for j, text in enumerate(texts):
                edit = QLineEdit(text)
                edit.setObjectName(f"cameraLens{i + 1}_{j}")
                self._lens_grid.addWidget(edit, row, j + 1)
                edits.append(edit)
            remove = self._button("Remove", f"cameraLensRemove{i + 1}", lambda _c=False, n=i: self._remove_lens(n))
            self._lens_grid.addWidget(remove, row, len(texts) + 1)
            self._lens_rows.append(([label, *edits, remove], edits))
        self._add_lens_btn.setEnabled(len(rows) < MAX_LENSES)

    def _add_lens(self) -> None:
        rows = self._lens_texts()
        if len(rows) >= MAX_LENSES:
            return
        if rows:
            fov_h, fov_v, _first, last = (_number(t) for t in rows[-1])
            rows.append([_shown(fov_h / 2), _shown(fov_v / 2), _shown(last + 1), _shown(last + 50)])
        else:
            rows.append(["60", "40", "0", "50"])
        self._set_lens_rows(rows)

    def _remove_lens(self, index: int) -> None:
        rows = self._lens_texts()
        if 0 <= index < len(rows):
            rows.pop(index)
        self._set_lens_rows(rows)

    def _take_addresses(self) -> None:
        if self._addresses_in_use is None:
            return
        host, port, day, thermal = self._addresses_in_use()
        # No host set: VGCS talks to the day video's camera, else to Skydroid's address.
        self._host.setText(str(host).strip() or _host_of(day) or SKYDROID_HOST)
        self._port.setText(_shown(int(port)))
        self._day.setText(str(day).strip())
        self._thermal.setText(str(thermal).strip())

    def form_fields(self) -> dict:
        """The form as the file's fields (numbers as typed: not a number when they are not one)."""
        return {
            "format": FORMAT,
            "protocol": PROTOCOL,
            "name": self._name.text(),
            "host": self._host.text(),
            "port": _number(self._port.text()),
            "transport": self._transport.currentData(),
            "dayVideoUrl": self._day.text(),
            "thermalVideoUrl": self._thermal.text(),
            "zoom": self._zoom.currentData(),
            "dayFovH": _number(self._fov_h.text()),
            "dayFovV": _number(self._fov_v.text()),
            "maxZoom": _number(self._max_zoom.text()),
            "lenses": [
                {"fovH": _number(h), "fovV": _number(v), "firstStep": _number(a), "lastStep": _number(b)}
                for h, v, a, b in self._lens_texts()
            ],
            "sensorW": _number(self._sensor_w.text()),
            "sensorH": _number(self._sensor_h.text()),
            "cropStepW": _number(self._crop_w.text()),
            "cropStepH": _number(self._crop_h.text()),
            "thermalFovH": _number(self._thermal_fov_h.text()),
            "thermalFovV": _number(self._thermal_fov_v.text()),
            "thermalFovSure": self._thermal_sure.isChecked(),
            "pitchMin": _number(self._pitch_min.text()),
            "pitchMax": _number(self._pitch_max.text()),
            "yawLimit": _number(self._yaw_limit.text()),
            "laser": self._laser.isChecked(),
            "laserMaxM": _number(self._laser_max.text()),
            "photo": self._photo.isChecked(),
            "record": self._record.isChecked(),
            "ownTracker": self._own_tracker.isChecked(),
            "upperGimbalHeader": self._upper_header.isChecked(),
        }

    def _show_problem(self, text: str) -> None:
        self._problem.setText(text)
        self._problem.setVisible(bool(text))

    def _save(self) -> None:
        if self._editing is None:
            return
        # Read as the file reader reads a file: the values a zoom kind does not use are not kept.
        cam, why = from_json(self.form_fields(), self._editing.id)
        if cam is None:
            self._show_problem(why[:1].upper() + why[1:])
            return
        why, saved = self._store.save(cam)
        if why:
            self._show_problem(why)
            return
        self.saved_ids.add(saved)
        self._editing = None
        self._show_problem("")
        self._pages.setCurrentIndex(0)
        self._reload(select_id=saved)

    def _back_to_list(self) -> None:
        self._editing = None
        self._show_problem("")
        self._pages.setCurrentIndex(0)

    def showing_form(self) -> bool:
        return self._pages.currentIndex() == 1


__all__ = ["CameraEditorDialog", "UNUSED_NOTE", "ZOOM_CHOICES"]
