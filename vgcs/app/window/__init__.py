"""Composable mixins for :class:`vgcs.app.main_window.MainWindow`."""

from __future__ import annotations

from vgcs.app.window.flight_commands_mixin import MainWindowFlightCommandsMixin
from vgcs.app.window.flight_status_mixin import MainWindowFlightStatusMixin
from vgcs.app.window.link_mixin import MainWindowLinkMixin
from vgcs.app.window.map_chrome_mixin import MainWindowMapChromeMixin
from vgcs.app.window.params_mixin import MainWindowParamsMixin
from vgcs.app.window.plan_mission_mixin import MainWindowPlanMissionMixin
from vgcs.app.window.settings_dialogs_mixin import MainWindowSettingsDialogsMixin
from vgcs.app.window.telemetry_mixin import MainWindowTelemetryMixin
from vgcs.app.window.ui_layout_mixin import MainWindowUiLayoutMixin
from vgcs.app.window.window_lifecycle_mixin import MainWindowLifecycleMixin


class MainWindowMixins(
    MainWindowUiLayoutMixin,
    MainWindowPlanMissionMixin,
    MainWindowMapChromeMixin,
    MainWindowSettingsDialogsMixin,
    MainWindowFlightStatusMixin,
    MainWindowLinkMixin,
    MainWindowTelemetryMixin,
    MainWindowFlightCommandsMixin,
    MainWindowParamsMixin,
    MainWindowLifecycleMixin,
):
    """Mixin bundle for the GCS main window."""


__all__ = [
    "MainWindowMixins",
    "MainWindowFlightCommandsMixin",
    "MainWindowFlightStatusMixin",
    "MainWindowLifecycleMixin",
    "MainWindowLinkMixin",
    "MainWindowMapChromeMixin",
    "MainWindowParamsMixin",
    "MainWindowPlanMissionMixin",
    "MainWindowSettingsDialogsMixin",
    "MainWindowTelemetryMixin",
    "MainWindowUiLayoutMixin",
]
