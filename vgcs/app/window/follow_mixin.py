"""Follow Me in the main window: the drone follows the object VGCS's tracker has locked, and keeps its distance.

The pieces are in vgcs/app/object_follow.py (the clock and the operator's start and stop), vgcs/app/follow_guide.py
(every rule), vgcs/app/follow_drone.py (the active drone's link) and vgcs/map/observation/follow_lock.py (the
tracker). This part makes them, keeps the drone's link up to date, and connects the Follow button of the TRACK block.
"""

from __future__ import annotations

from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QLabel,
    QMessageBox,
    QSpinBox,
)

from vgcs.app import follow_guide as fg
from vgcs.app.flight_action_dialogs import confirm_follow, follow_message
from vgcs.app.follow_drone import LinkFollowDrone
from vgcs.app.object_follow import ObjectFollow

_START_TIP = "Follow Me: the drone follows the tracked object and keeps its distance (it asks first)."


class MainWindowFollowMixin:
    def _setup_object_follow(self) -> None:
        """Make Follow Me (after the map widget, and after self._thread exists)."""
        self._follow_drone = LinkFollowDrone(self)
        self._object_follow = ObjectFollow(self._map_widget.follow_lock(), self._follow_drone, self)
        self._follow_last_message = ""
        self._object_follow.changed.connect(self._on_object_follow_changed)
        self._map_widget.follow_object_requested.connect(self._on_follow_object_requested)
        self._sync_follow_drone()
        self._on_object_follow_changed()

    def _sync_follow_drone(self) -> None:
        """The window's active drone changed (connect, a fleet switch, a link that ended): Follow's drone too.
        Follow never goes on with another drone than the one it was started on (follow_guide.Why.DRONE_CHANGED)."""
        drone = getattr(self, "_follow_drone", None)
        if drone is not None:
            drone.set_thread(getattr(self, "_thread", None))

    def _on_follow_object_requested(self) -> None:
        follow = getattr(self, "_object_follow", None)
        if follow is None:
            return
        if follow.active():
            follow.stop()
            self._append_log("Follow: stopped by the operator.")
            self._on_object_follow_changed()
            return
        follow.refresh()  # decide on the state now, not on one worked out before
        if not follow.ready():
            QMessageBox.information(self, "Follow", follow.not_ready_text() or "Follow cannot start now.")
            self._on_object_follow_changed()
            return
        text = follow_message(follow.distance_m(), follow.speed_limit(), follow.range_limit(), fg.MIN_DISTANCE_M,
                              follow.flies_on_s())
        if not confirm_follow(self, text):
            self._on_object_follow_changed()
            return
        follow.start()
        if follow.active():
            self._append_log(f"Follow: started, keeping {follow.kept_distance_m():.0f} m to the tracked object.")
        else:
            QMessageBox.information(self, "Follow", follow.message() or "Follow did not start.")
        self._on_object_follow_changed()

    def _on_object_follow_changed(self) -> None:
        follow = getattr(self, "_object_follow", None)
        if follow is None:
            return
        tip = follow.message() if follow.active() else (follow.not_ready_text() or _START_TIP)
        self._map_widget.set_follow_object_state(follow.active(), tip)
        message = follow.message()
        if message != self._follow_last_message:
            self._follow_last_message = message
            if message:
                self._append_log(f"Follow: {message}")
                self._post_gcs_notice(message)

    def _build_follow_settings_group(self):
        """Application Settings, General: Follow Me's two limits. Returns the group and what saves it (on OK)."""
        group = QGroupBox("Follow Me")
        grid = QGridLayout()
        follow = getattr(self, "_object_follow", None)
        speed = QDoubleSpinBox()
        speed.setObjectName("followSpeedLimitSpin")
        speed.setRange(fg.SPEED_LIMIT_LOW_MPS, fg.SPEED_LIMIT_HIGH_MPS)
        speed.setDecimals(1)
        speed.setSingleStep(0.5)
        speed.setValue(follow.speed_limit() if follow is not None else fg.SPEED_LIMIT_DEFAULT_MPS)
        reach = QSpinBox()
        reach.setObjectName("followRangeLimitSpin")
        reach.setRange(int(fg.RANGE_LIMIT_LOW_M), int(fg.RANGE_LIMIT_HIGH_M))
        reach.setSingleStep(50)
        reach.setValue(round(follow.range_limit() if follow is not None else fg.RANGE_LIMIT_DEFAULT_M))
        grid.addWidget(QLabel("Speed limit (m/s)"), 0, 0)
        grid.addWidget(speed, 0, 1)
        grid.addWidget(QLabel("Range limit (m)"), 1, 0)
        grid.addWidget(reach, 1, 1)
        hint = QLabel(
            "Follow (the TRACK block, beside Track) flies the drone after the object that VGCS's own tracker has "
            f"locked. It starts only {fg.START_DISTANCE_M:.0f} m or more from the object, never flies nearer than "
            f"{fg.MIN_DISTANCE_M:.0f} m, never changes the height, and stops when the lock is lost. The speed limit "
            "is the fastest it flies the drone. Follow ends when the drone is the range limit away from where it "
            "started."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #aab4c8; font-size: 11px;")
        grid.addWidget(hint, 2, 0, 1, 2)
        grid.setColumnStretch(1, 1)
        group.setLayout(grid)

        def save() -> None:
            now = getattr(self, "_object_follow", None)
            if now is not None:
                now.set_speed_limit(float(speed.value()))
                now.set_range_limit(float(reach.value()))

        return group, save

    def _shutdown_object_follow(self) -> None:
        """Never leave the drone flying a speed when VGCS closes."""
        follow = getattr(self, "_object_follow", None)
        if follow is not None:
            follow.shutdown()
