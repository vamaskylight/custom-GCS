"""Several drones in one VGCS window (milestone M15, client requirement 14).

The fleet itself is in vgcs/link/fleet.py: one link thread per drone, all
running. This mixin ties it to the window:

- The window shows one drone, the active one, exactly as before, and every
  existing button commands it. `self._thread` is the active drone's thread.
- Making another drone active rewires the window's slots to that drone's
  thread and clears the view the way a link drop does (without announcing a
  lost link: the old drone is still connected and still flying).
- The other drones are drawn on the map with their names, and listed in the
  Fleet panel (logo menu, "Fleet") with their link, mode, height, battery and
  last message. Hold, return home and land go to one drone or to all.
- A warning from a drone that is not on screen (link lost, a low battery, a
  fence breach) is logged and shown in the message line, with its name.

Missions stay one per drone: plan, make the drone active, upload. VGCS never
sends one route to several drones, which would fly them into each other.
"""

from __future__ import annotations

import time
import warnings

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QAbstractItemView,
    QDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
)

from vgcs.link.fleet import LINK_LOST, LINK_UP, LINK_WAITING, Fleet, row_cells

FLEET_COLUMNS = ["Drone", "Link", "Mode", "Armed", "Height", "Battery", "GPS", "Mission", "Last message"]
FLEET_REFRESH_MS = 500

# The window slots each drone's link thread is wired to while it is active.
_LINK_SLOTS = (
    ("log_line", "_append_log"),
    ("error", "_on_link_error"),
    ("link_up", "_on_link_up"),
    ("link_down", "_on_link_down"),
    ("heartbeat", "_on_heartbeat"),
    ("telemetry", "_on_telemetry"),
    ("link_timeout", "_on_link_timeout"),
    ("mission_uploaded", "_on_mission_uploaded"),
    ("mission_downloaded", "_on_mission_downloaded"),
    ("mission_progress", "_on_mission_progress"),
    ("mode_changed", "_on_mode_change_result"),
    ("action_result", "_on_action_result"),
    ("geofence_result", "_on_geofence_result"),
    ("params_snapshot", "_on_params_snapshot"),
    ("param_set_result", "_on_param_set_result"),
    ("finished", "_on_thread_finished"),
)

FLEET_COMMANDS = {
    "hold": "Hold position (BRAKE, else LOITER)",
    "rtl": "Return home (RTL)",
    "land": "Land where it is (LAND)",
}


class MainWindowFleetMixin:
    """Fleet of drones for the main window. See the module docstring."""

    def _init_fleet(self) -> None:
        self._fleet = Fleet()
        self._fleet.alert.connect(self._on_fleet_alert)
        self._fleet_dialog = None
        self._fleet_table = None
        self._fleet_status_label = None
        self._suppress_preflight_popup = False
        self._pending_fleet_name = ""
        self._base_window_title = self.windowTitle()
        self._fleet_timer = QTimer(self)
        self._fleet_timer.setInterval(FLEET_REFRESH_MS)
        self._fleet_timer.timeout.connect(self._refresh_fleet_view)
        self._fleet_timer.start()

    # --- wiring the window to the active drone ---------------------------
    def _wire_link_signals(self, thread) -> None:
        for signal_name, slot_name in _LINK_SLOTS:
            getattr(thread, signal_name).connect(getattr(self, slot_name))

    def _unwire_link_signals(self, thread) -> None:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            for signal_name, slot_name in _LINK_SLOTS:
                try:
                    getattr(thread, signal_name).disconnect(getattr(self, slot_name))
                except (RuntimeError, TypeError):
                    pass

    def _switch_active_vehicle(self, vid: str) -> bool:
        """Show and command another drone of the fleet. The others keep running."""
        vehicle = self._fleet.get(vid)
        if vehicle is None:
            return False
        if vehicle.thread is self._thread:
            self._fleet.set_active(vid)
            return True
        if not vehicle.is_running():
            return False
        self._close_preflight_dialog()
        old = self._thread
        if old is not None:
            self._unwire_link_signals(old)
        # The view now describes another drone. Clear it the way a link drop
        # does, but without "LINK LOST": the old drone is still connected.
        self._heartbeat_seen = False
        self._on_link_down()
        last_params = getattr(self, "_last_params", None)
        if last_params is not None:
            last_params.clear()
        self._recent_statustext.clear()
        self._connect_attempt_active = False
        self._thread = vehicle.thread
        self._wire_link_signals(self._thread)
        self._fleet.set_active(vid)
        # The next heartbeat goes through the first-heartbeat path again (it
        # reads this drone's settings), but a switch is not a new connection,
        # so the pre-flight popup is not shown for it.
        self._suppress_preflight_popup = True
        self._conn_edit.setText(vehicle.summary.connection)
        for widget in (self._btn_connect, self._conn_edit, self._timeout_spin, self._theme_combo):
            widget.setEnabled(False)
        state = vehicle.summary.link
        if state in (LINK_WAITING, LINK_UP, LINK_LOST):
            self._on_link_up()
        if state == LINK_LOST:
            since = vehicle.summary.lost_since_mono
            self._on_link_timeout(0.0 if since is None else time.monotonic() - since)
        self._append_log(f"Active drone: {vehicle.name} ({vehicle.summary.connection})")
        self._refresh_fleet_view()
        return True

    # --- adding and removing drones ----------------------------------------
    def _fleet_add_vehicle(self, connection: str, name: str = "") -> str:
        """Connect one more drone. Returns "" when it went, or the reason it did not.

        With no drone on screen the new one becomes the active drone, through
        the same path as the Connect button. Otherwise it runs in the
        background until it is made active.
        """
        connection = str(connection or "").strip()
        if self._thread is None or not self._thread.isRunning():
            self._conn_edit.setText(connection)
            self._pending_fleet_name = str(name or "").strip()
            before = len(self._fleet.vehicles())
            self._on_connect()
            self._pending_fleet_name = ""
            return "" if len(self._fleet.vehicles()) >= before and self._thread is not None else "not connected"
        try:
            vehicle = self._fleet.add(connection, name=name, timeout_s=float(self._timeout_s))
        except ValueError as e:
            return str(e)
        self._append_log(f"Fleet: {vehicle.name} connecting on {connection}")
        self._refresh_fleet_view()
        return ""

    def _fleet_remove_vehicle(self, vid: str) -> None:
        vehicle = self._fleet.get(vid)
        if vehicle is None:
            return
        if vehicle.thread is self._thread:
            self._on_disconnect()
        self._fleet.remove(vid)
        self._append_log(f"Fleet: {vehicle.name} disconnected")
        self._refresh_fleet_view()

    def _stop_fleet(self) -> None:
        """Stop every drone's link (the window is closing)."""
        try:
            self._fleet.stop_all()
        except Exception:
            pass

    # --- commands ----------------------------------------------------------
    def _fleet_command(self, command: str, vid: str = "") -> list[str]:
        """Hold, return home or land: one drone (vid) or every connected drone."""
        if vid:
            vehicle = self._fleet.get(vid)
            if vehicle is None or vehicle not in self._fleet.connected():
                return []
            t = vehicle.thread
            {"hold": t.queue_mission_pause, "rtl": lambda: t.queue_mode_change("RTL"),
             "land": t.queue_auto_land}[command]()
            vehicle.summary.last_result = f"{command.upper()} sent"
            sent = [vehicle.name]
        else:
            sent = self._fleet.command_all(command)
        if sent:
            self._append_log(f"Fleet: {FLEET_COMMANDS[command]} sent to {', '.join(sent)}")
        return sent

    # --- what the drones that are not on screen say ------------------------
    def _on_fleet_alert(self, vid: str, text: str) -> None:
        vehicle = self._fleet.get(vid)
        if vehicle is None or vid == self._fleet.active_id:
            return    # the window already shows the active drone's own messages
        line = f"{vehicle.name}: {text}"
        self._append_log(f"[Fleet] {line}")
        try:
            self._post_gcs_notice(line)
        except Exception:
            pass

    # --- the views ---------------------------------------------------------
    def _fleet_map_items(self) -> list[dict]:
        items = []
        for vehicle in self._fleet.vehicles():
            s = vehicle.summary
            label = s.name if s.alt_rel_m is None else f"{s.name}  {s.alt_rel_m:.0f} m"
            item = {"label": label, "link_ok": s.link == LINK_UP, "active": vehicle.vid == self._fleet.active_id}
            if s.lat is not None and s.lon is not None:
                item.update(lat=s.lat, lon=s.lon, heading_deg=s.heading_deg or 0.0)
            items.append(item)
        return items

    def _refresh_fleet_view(self) -> None:
        vehicles = self._fleet.vehicles()
        several = len(vehicles) >= 2
        try:
            self._map_widget.set_fleet_vehicles(self._fleet_map_items() if several else [])
        except Exception:
            pass
        active = self._fleet.active()
        title = self._base_window_title
        if several:
            on_screen = active.name if active is not None else "no drone on screen"
            title = f"{title}  [{on_screen}, fleet of {len(vehicles)}]"
        if self.windowTitle() != title:
            self.setWindowTitle(title)
        if self._fleet_dialog is not None and self._fleet_dialog.isVisible():
            self._fill_fleet_table()

    def _fill_fleet_table(self) -> None:
        table = self._fleet_table
        if table is None:
            return
        vehicles = self._fleet.vehicles()
        selected = self._fleet_selected_vid()
        table.setRowCount(len(vehicles))
        now = time.monotonic()
        for row, vehicle in enumerate(vehicles):
            cells = row_cells(vehicle.summary, now)
            if vehicle.vid == self._fleet.active_id:
                cells[0] = f"{cells[0]} (on screen)"
            for col, text in enumerate(cells):
                item = table.item(row, col)
                if item is None:
                    item = QTableWidgetItem()
                    table.setItem(row, col, item)
                if item.text() != text:
                    item.setText(text)
                item.setData(Qt.ItemDataRole.UserRole, vehicle.vid)
            if vehicle.vid == selected:
                table.selectRow(row)

    def _fleet_selected_vid(self) -> str:
        table = self._fleet_table
        if table is None:
            return ""
        rows = table.selectionModel().selectedRows() if table.selectionModel() else []
        if not rows:
            return ""
        item = table.item(rows[0].row(), 0)
        return str(item.data(Qt.ItemDataRole.UserRole) or "") if item is not None else ""

    def _show_fleet_dialog(self) -> None:
        if self._fleet_dialog is None:
            self._build_fleet_dialog()
        self._fill_fleet_table()
        self._fleet_dialog.show()
        self._fleet_dialog.raise_()

    def _fleet_say(self, text: str) -> None:
        if self._fleet_status_label is not None:
            self._fleet_status_label.setText(text)

    def _build_fleet_dialog(self) -> None:
        dlg = QDialog(self)
        dlg.setWindowTitle("Fleet")
        dlg.setModal(False)
        dlg.resize(980, 420)
        lay = QVBoxLayout(dlg)
        lay.addWidget(QLabel(
            "Each drone needs its own connection (its own port or address). "
            "The drone on screen is the one the window's buttons command. "
            "Missions are one per drone: make a drone active, then plan and upload."
        ))

        table = QTableWidget(0, len(FLEET_COLUMNS))
        table.setHorizontalHeaderLabels(FLEET_COLUMNS)
        table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        table.verticalHeader().setVisible(False)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        table.horizontalHeader().setStretchLastSection(True)
        table.doubleClicked.connect(lambda _index: self._fleet_on_make_active())
        lay.addWidget(table, 1)
        self._fleet_table = table

        add_row = QHBoxLayout()
        conn_edit = QLineEdit()
        conn_edit.setPlaceholderText("Connection, for example udpin:0.0.0.0:14551")
        name_edit = QLineEdit()
        name_edit.setPlaceholderText("Name (optional)")
        name_edit.setMaximumWidth(180)
        btn_add = QPushButton("Connect drone")
        add_row.addWidget(QLabel("Add a drone"))
        add_row.addWidget(conn_edit, 1)
        add_row.addWidget(name_edit)
        add_row.addWidget(btn_add)
        lay.addLayout(add_row)

        def add() -> None:
            why = self._fleet_add_vehicle(conn_edit.text(), name_edit.text())
            if why:
                self._fleet_say(f"Not connected: {why}")
            else:
                self._fleet_say(f"Connecting {conn_edit.text().strip()}")
                conn_edit.clear()
                name_edit.clear()
            self._fill_fleet_table()

        btn_add.clicked.connect(add)

        sel_row = QHBoxLayout()
        sel_row.addWidget(QLabel("Selected drone"))
        btn_active = QPushButton("Show and command it")
        btn_active.clicked.connect(self._fleet_on_make_active)
        sel_row.addWidget(btn_active)
        for command, label in (("hold", "Hold"), ("rtl", "Return home"), ("land", "Land")):
            b = QPushButton(label)
            b.setToolTip(FLEET_COMMANDS[command])
            b.clicked.connect(lambda _c=False, cmd=command: self._fleet_on_command(cmd, selected=True))
            sel_row.addWidget(b)
        btn_remove = QPushButton("Disconnect")
        btn_remove.clicked.connect(self._fleet_on_remove)
        sel_row.addWidget(btn_remove)
        sel_row.addStretch(1)
        lay.addLayout(sel_row)

        all_row = QHBoxLayout()
        all_row.addWidget(QLabel("All connected drones"))
        for command, label in (("hold", "Hold all"), ("rtl", "Return home all"), ("land", "Land all")):
            b = QPushButton(label)
            b.setToolTip(FLEET_COMMANDS[command])
            b.clicked.connect(lambda _c=False, cmd=command: self._fleet_on_command(cmd, selected=False))
            all_row.addWidget(b)
        all_row.addStretch(1)
        lay.addLayout(all_row)

        status = QLabel("")
        status.setWordWrap(True)
        lay.addWidget(status)
        self._fleet_status_label = status
        self._fleet_dialog = dlg

    def _fleet_on_make_active(self) -> None:
        vid = self._fleet_selected_vid()
        if not vid:
            self._fleet_say("Select a drone first.")
            return
        vehicle = self._fleet.get(vid)
        if self._switch_active_vehicle(vid):
            self._fleet_say(f"{vehicle.name} is on screen. The window's buttons command it.")
        else:
            self._fleet_say(f"{vehicle.name} is not connected. Disconnect it and connect it again.")
        self._fill_fleet_table()

    def _fleet_on_remove(self) -> None:
        vid = self._fleet_selected_vid()
        vehicle = self._fleet.get(vid) if vid else None
        if vehicle is None:
            self._fleet_say("Select a drone first.")
            return
        if vehicle.summary.armed:
            answer = QMessageBox.question(
                self._fleet_dialog, "Disconnect",
                f"{vehicle.name} is armed. Disconnect it anyway? It keeps flying, and its own failsafes decide what it does.",
            )
            if answer != QMessageBox.StandardButton.Yes:
                return
        self._fleet_remove_vehicle(vid)
        self._fleet_say(f"{vehicle.name} disconnected.")
        self._fill_fleet_table()

    def _fleet_on_command(self, command: str, *, selected: bool) -> None:
        if selected:
            vid = self._fleet_selected_vid()
            vehicle = self._fleet.get(vid) if vid else None
            if vehicle is None:
                self._fleet_say("Select a drone first.")
                return
            targets = [vehicle.name]
        else:
            vid = ""
            targets = [v.name for v in self._fleet.connected()]
        if not targets:
            self._fleet_say("No connected drone.")
            return
        answer = QMessageBox.question(
            self._fleet_dialog, "Fleet",
            f"{FLEET_COMMANDS[command]}: {', '.join(targets)}?",
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        sent = self._fleet_command(command, vid)
        self._fleet_say(f"{FLEET_COMMANDS[command]} sent to {', '.join(sent)}." if sent else "Nothing sent: the drone is not connected.")
        self._fill_fleet_table()
