"""The window "Obstacle sensor settings": the drone's LiDAR, keeping away from objects, the path planner (A7 item 1).

Opened from Vehicle Configuration. It reads the drone's settings, shows them as
choices, and on "Write to drone" says first what changes (and what to know:
a port that speaks MAVLink now, the restart), writes only those settings, each
confirmed by the drone, and reads them back. "Restart the drone" restarts the
flight controller, which takes the sensor, its port and the planner only then.
The rules are in vgcs/app/obstacle_setup.py.
"""

from __future__ import annotations

from collections.abc import Callable

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from vgcs.app.obstacle_setup import (
    MARGIN_RANGE_M,
    MODELS,
    PLANNER_CHOICES,
    RANGE_LIMIT_M,
    YAW_RANGE_DEG,
    Change,
    Choices,
    choices_from_drone,
    clean_values,
    mode_switch,
    plan_writes,
    port_words,
    ports_on_drone,
    read_back_mismatches,
    read_names,
    taken_at_start,
)

# A read of every setting is answered within a few seconds; after this long the page says what it has.
READ_GIVE_UP_MS = 20000
LINK_CLOSED_WORDS = ("The link to the drone is closed (a restart over USB or TCP closes it). Close this "
                     "window, connect again, and open it again.")


class ObstacleSetupDialog(QDialog):
    """``link``: the active drone's link (MavlinkThread): reads, writes and the restart go through it."""

    def __init__(self, parent: QWidget | None, link, is_armed: Callable[[], bool],
                 log: Callable[[str], None] | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("obstacleSetupDialog")
        self.setWindowTitle("Obstacle sensor settings")
        self.resize(640, 760)
        self._link = link
        self._is_armed = is_armed
        self._log = log or (lambda _text: None)
        self._values: dict[str, float] = {}
        self._missing: set[str] = set()
        self._asked: set[str] = set()
        self._reading = False
        self._read_purpose = ""              # "load" or "check"
        self._pending: dict[str, Change] = {}
        self._results: dict[str, tuple[bool, str]] = {}
        self._written: list[Change] = []
        self._filling = False
        self._read_timer = QTimer(self)
        self._read_timer.setSingleShot(True)
        self._read_timer.timeout.connect(self._read_gave_up)
        self._build()
        link.params_snapshot.connect(self._on_values)
        link.param_set_result.connect(self._on_param_set)
        link.action_result.connect(self._on_action)
        self.finished.connect(self._disconnect)
        self._read("load")

    # -- the page ------------------------------------------------------------
    def _build(self) -> None:
        root = QVBoxLayout(self)
        self._status = QLabel("")
        self._status.setObjectName("obstacleSetupStatus")
        self._status.setWordWrap(True)
        root.addWidget(self._status)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        # Long choices (the ports, the planner) shrink with the window instead of scrolling it sideways.
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        inner = QWidget()
        lay = QVBoxLayout(inner)
        scroll.setWidget(inner)
        root.addWidget(scroll, 1)

        lidar = QGroupBox("LiDAR, 360 degrees (the first proximity sensor)")
        form = QFormLayout(lidar)
        self._lidar_on = QCheckBox("An RPLidar is on the drone (PRX1_TYPE 5)")
        form.addRow(self._lidar_on)
        self._model = QComboBox()
        for key, words, _baud in MODELS:
            self._model.addItem(words, key)
        form.addRow("Model", self._model)
        self._port = QComboBox()
        self._port.setObjectName("obstacleSetupPort")
        form.addRow("Port", self._port)
        self._mount = QComboBox()
        self._mount.addItem("On top, cable to the rear (PRX1_ORIENT 0)", False)
        self._mount.addItem("Upside down, below the drone (PRX1_ORIENT 1)", True)
        form.addRow("Mounted", self._mount)
        self._yaw = QSpinBox()
        self._yaw.setRange(int(YAW_RANGE_DEG[0]), int(YAW_RANGE_DEG[1]))
        self._yaw.setSuffix(" deg")
        self._yaw.setToolTip("PRX1_YAW_CORR: how far the LiDAR's front is turned from the drone's nose, to the right.")
        form.addRow("Turned", self._yaw)
        self._min = QDoubleSpinBox()
        self._max = QDoubleSpinBox()
        for box, name in ((self._min, "PRX1_MIN"), (self._max, "PRX1_MAX")):
            box.setRange(*RANGE_LIMIT_M)
            box.setDecimals(1)
            box.setSingleStep(0.1)
            box.setSuffix(" m")
            box.setSpecialValueText("none")
            box.setToolTip(f"{name}: readings beyond it are left out (the drone's own legs, far noise). none: 0.")
        form.addRow("Leave out nearer than", self._min)
        form.addRow("Leave out farther than", self._max)
        others = QLabel("The TF02 Pro and the MTF-01P are not set here yet: their ports and what they are for are "
                        "not known.")
        others.setWordWrap(True)
        form.addRow(others)
        lay.addWidget(lidar)

        avoid = QGroupBox("Keep away from objects (Loiter, AltHold, FlowHold, ZigZag)")
        form = QFormLayout(avoid)
        self._keep_away = QCheckBox("Use the sensor (AVOID_ENABLE: 2 added)")
        form.addRow(self._keep_away)
        self._stop_m = QDoubleSpinBox()
        self._stop_m.setRange(*MARGIN_RANGE_M)
        self._stop_m.setDecimals(1)
        self._stop_m.setSingleStep(0.5)
        self._stop_m.setSuffix(" m")
        form.addRow("Stop distance (AVOID_MARGIN)", self._stop_m)
        self._behave = QComboBox()
        self._behave.addItem("Stop before it (AVOID_BEHAVE 1)", True)
        self._behave.addItem("Slide along it (AVOID_BEHAVE 0)", False)
        form.addRow("In Loiter", self._behave)
        lay.addWidget(avoid)

        mission = QGroupBox("Fly around objects (Auto, Guided, RTL, SmartRTL)")
        form = QFormLayout(mission)
        self._planner = QComboBox()
        for value, words in PLANNER_CHOICES:
            self._planner.addItem(f"{words} (OA_TYPE {value})", value)
        form.addRow("Path planner", self._planner)
        lay.addWidget(mission)

        switch = QGroupBox("The mode switch, with these settings")
        box = QVBoxLayout(switch)
        self._switch = QLabel("")
        self._switch.setObjectName("obstacleSetupModeSwitch")
        self._switch.setWordWrap(True)
        box.addWidget(self._switch)
        lay.addWidget(switch)

        self._preview = QLabel("")
        self._preview.setObjectName("obstacleSetupPreview")
        self._preview.setWordWrap(True)
        lay.addWidget(self._preview)
        lay.addStretch(1)

        buttons = QHBoxLayout()
        self._btn_read = QPushButton("Read from drone")
        self._btn_write = QPushButton("Write to drone")
        self._btn_write.setObjectName("obstacleSetupWrite")
        self._btn_restart = QPushButton("Restart the drone")
        self._btn_restart.setObjectName("obstacleSetupRestart")
        self._btn_restart.setToolTip("The flight controller restarts. It takes the sensor, its port and the path "
                                     "planner only then. Refused while armed.")
        close = QPushButton("Close")
        for b in (self._btn_read, self._btn_write, self._btn_restart):
            buttons.addWidget(b)
        buttons.addStretch(1)
        buttons.addWidget(close)
        root.addLayout(buttons)
        self._btn_read.clicked.connect(lambda: self._read("load"))
        self._btn_write.clicked.connect(self._write)
        self._btn_restart.clicked.connect(self._restart)
        close.clicked.connect(self.reject)

        for box in (self._model, self._port, self._mount, self._behave, self._planner):
            box.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
            box.setMinimumContentsLength(18)
        for w in (self._lidar_on, self._keep_away):
            w.toggled.connect(self._changed)
        for w in (self._model, self._port, self._mount, self._behave, self._planner):
            w.currentIndexChanged.connect(self._changed)
        for w in (self._yaw, self._min, self._max, self._stop_m):
            w.valueChanged.connect(self._changed)
        self._set_busy(True)

    def _set_busy(self, busy: bool) -> None:
        self._btn_write.setEnabled(not busy and bool(self._values))
        self._btn_read.setEnabled(not busy)

    def choices(self) -> Choices:
        """What the page shows now."""
        return Choices(
            lidar_on=self._lidar_on.isChecked(),
            model=str(self._model.currentData() or MODELS[0][0]),
            port=int(self._port.currentData() or 0),
            upside_down=bool(self._mount.currentData()),
            yaw_corr_deg=float(self._yaw.value()),
            min_m=float(self._min.value()),
            max_m=float(self._max.value()),
            keep_away=self._keep_away.isChecked(),
            stop_m=float(self._stop_m.value()),
            stop=bool(self._behave.currentData()),
            planner=int(self._planner.currentData() or 0),
        )

    def _show(self, c: Choices) -> None:
        """The drone's settings onto the page."""
        self._filling = True
        try:
            self._lidar_on.setChecked(c.lidar_on)
            self._model.setCurrentIndex(max(0, self._model.findData(c.model)))
            self._port.clear()
            for port in ports_on_drone(self._values):
                self._port.addItem(port_words(port, self._values), port)
            self._port.setCurrentIndex(max(0, self._port.findData(c.port)))
            self._mount.setCurrentIndex(1 if c.upside_down else 0)
            self._yaw.setValue(round(c.yaw_corr_deg))
            self._min.setValue(c.min_m)
            self._max.setValue(c.max_m)
            self._keep_away.setChecked(c.keep_away)
            self._stop_m.setValue(c.stop_m)
            self._behave.setCurrentIndex(0 if c.stop else 1)
            self._planner.setCurrentIndex(max(0, self._planner.findData(c.planner)))
        finally:
            self._filling = False
        self._changed()

    def _changed(self, *_args) -> None:
        """The preview of what Write would change, and the mode switch with those settings."""
        if self._filling:
            return
        for w in (self._model, self._port, self._mount, self._yaw, self._min, self._max):
            w.setEnabled(self._lidar_on.isChecked())
        for w in (self._stop_m, self._behave):
            w.setEnabled(self._keep_away.isChecked())
        if not self._values:
            self._preview.setText("")
            self._switch.setText("")
            return
        plan = plan_writes(self._values, self.choices(), armed=False)
        if plan.refusal:
            self._preview.setText(plan.refusal)
        elif not plan.changes:
            self._preview.setText("The drone has these settings already.")
        else:
            text = "Write to drone would change: " + "; ".join(c.words() for c in plan.changes) + "."
            if plan.restart_names:
                text += " The drone takes " + ", ".join(plan.restart_names) + " only at its next start."
            self._preview.setText(text)
        positions = mode_switch(plan.values_after(self._values))
        if not positions:
            self._switch.setText("This drone has no mode switch settings (FLTMODE1 to FLTMODE6).")
        else:
            rows = []
            for p in positions:
                row = f"Position {p.position} ({p.pwm}): {p.words}"
                if p.avoids is False:
                    row = f"<span style='color:#d97706'>{row}</span>"
                rows.append(row)
            self._switch.setText("<br>".join(rows))

    # -- reading ---------------------------------------------------------------
    def _link_closed(self) -> bool:
        """The link ended (a restart over USB or TCP closes it): nothing sent now would be answered."""
        running = getattr(self._link, "isRunning", None)
        if running is None or running():
            return False
        self._status.setText(LINK_CLOSED_WORDS)
        self._set_busy(False)
        self._btn_write.setEnabled(False)
        return True

    def _read(self, purpose: str) -> None:
        if self._link_closed():
            return
        self._asked = set(read_names())
        self._reading = True
        self._read_purpose = purpose
        self._set_busy(True)
        self._status.setText("Reading the drone's settings...")
        self._link.queue_params_fetch(sorted(self._asked))
        self._read_timer.start(READ_GIVE_UP_MS)

    def _on_values(self, payload: object) -> None:
        values = clean_values(payload if isinstance(payload, dict) else {})
        mine = {k: v for k, v in values.items() if k in self._asked or k in self._values}
        self._values.update(mine)
        # The answer to this page's read holds only names it asked for (other reads hold others too).
        if not self._reading or not values or not set(values) <= self._asked:
            return
        self._reading = False
        self._read_timer.stop()
        self._missing = self._asked - set(values)
        self._read_done()

    def _read_gave_up(self) -> None:
        if not self._reading:
            return
        self._reading = False
        self._missing = self._asked - set(self._values)
        if not self._values:
            self._set_busy(False)
            self._status.setText("The drone answered none of these settings. Is it connected, and is it a copter?")
            return
        self._read_done()

    def _read_done(self) -> None:
        self._set_busy(False)
        if self._read_purpose == "check":
            self._check_written()
            return
        self._show(choices_from_drone(self._values))
        missing = sorted(n for n in self._missing if not n.startswith(("BRD_SER", "SERIAL")))
        text = "Read from the drone."
        if missing:
            text += " This drone has no " + ", ".join(missing) + "."
        self._status.setText(text)

    # -- writing ---------------------------------------------------------------
    def _write(self) -> None:
        if self._link_closed():
            return
        plan = plan_writes(self._values, self.choices(), armed=bool(self._is_armed()))
        if plan.refusal:
            QMessageBox.warning(self, "Obstacle sensor settings", plan.refusal)
            return
        if not plan.changes:
            QMessageBox.information(self, "Obstacle sensor settings", "The drone has these settings already.")
            return
        lines = ["Write these settings to the drone?", ""] + [c.words() for c in plan.changes]
        if plan.warnings:
            lines += [""] + plan.warnings
        if plan.restart_names:
            lines += ["", "The drone takes " + ", ".join(plan.restart_names) + " only at its next start: "
                      "press Restart the drone after the write, or switch it off and on."]
        answer = QMessageBox.question(self, "Obstacle sensor settings", "\n".join(lines),
                                      QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
                                      QMessageBox.StandardButton.Cancel)
        if answer != QMessageBox.StandardButton.Yes:
            return
        self._pending = {c.name: c for c in plan.changes}
        self._results = {}
        self._written = list(plan.changes)
        self._set_busy(True)
        self._status.setText(f"Writing {len(plan.changes)} settings...")
        self._log("Obstacle settings: writing " + "; ".join(c.words() for c in plan.changes))
        for c in plan.changes:
            self._link.queue_param_set(c.name, c.new)

    def _on_param_set(self, name: str, ok: bool, detail: str) -> None:
        key = str(name).strip().upper()
        if key not in self._pending:
            return
        self._results[key] = (bool(ok), str(detail))
        del self._pending[key]
        if self._pending:
            self._status.setText(f"Writing... {len(self._results)} of {len(self._written)} confirmed by the drone.")
            return
        failed = [f"{n}: {d}" for n, (good, d) in self._results.items() if not good]
        if failed:
            self._set_busy(False)
            self._status.setText("Not all settings were written: " + "; ".join(failed) + ". Write again.")
            self._log("Obstacle settings: not written: " + "; ".join(failed))
            return
        self._read("check")

    def _check_written(self) -> None:
        wrong = read_back_mismatches(self._written, self._values)
        if wrong:
            self._status.setText("Written, but the drone holds something else: " + "; ".join(wrong) + ".")
            self._log("Obstacle settings: read back wrong: " + "; ".join(wrong))
            return
        restart = [c.name for c in self._written if taken_at_start(c.name)]
        text = f"Written and read back: {len(self._written)} settings."
        if restart:
            text += (" The drone takes " + ", ".join(restart) + " only at its next start: press Restart the drone, "
                     "or switch it off and on.")
        self._status.setText(text)
        self._log("Obstacle settings: " + text)
        self._show(choices_from_drone(self._values))

    # -- restarting ------------------------------------------------------------
    def _restart(self) -> None:
        if self._is_armed():
            QMessageBox.warning(self, "Obstacle sensor settings", "The drone is armed. It restarts on the ground only.")
            return
        answer = QMessageBox.question(
            self, "Obstacle sensor settings",
            "Restart the drone's flight controller now?\n\nOver a radio link (UDP) VGCS hears it again by itself "
            "after a few seconds. Over USB or TCP the drone's port closes with the restart: VGCS closes its link, and "
            "you connect again when the drone is back.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel, QMessageBox.StandardButton.Cancel)
        if answer != QMessageBox.StandardButton.Yes:
            return
        self._status.setText("Restart asked for...")
        self._link.queue_reboot()

    def _on_action(self, action: str, ok: bool, detail: str) -> None:
        if action != "reboot":
            return
        if ok:
            self._status.setText(f"{detail}. When it is back (over USB or TCP: connect again), press Read from "
                                 "drone.")
        else:
            self._status.setText(f"Not restarted: {detail}.")
        self._log(f"Obstacle settings: restart: {detail}")

    def _disconnect(self, *_args) -> None:
        self._read_timer.stop()
        for signal, slot in ((self._link.params_snapshot, self._on_values),
                             (self._link.param_set_result, self._on_param_set),
                             (self._link.action_result, self._on_action)):
            try:
                signal.disconnect(slot)
            except (RuntimeError, TypeError):
                pass
