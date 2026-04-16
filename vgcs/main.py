"""VGCS application entrypoint."""

import sys

from PySide6.QtWidgets import QApplication

from vgcs.app.gcs_style import gcs_stylesheet
from vgcs.app.main_window import MainWindow
from vgcs.app.runtime_ui import (
    apply_qt_scale_override,
    build_base_font,
    configure_high_dpi_policy,
    select_font_profile,
)


def main() -> int:
    # Must happen before QApplication to affect Qt layout metrics.
    apply_qt_scale_override()
    configure_high_dpi_policy()
    app = QApplication(sys.argv)
    ui_scale = 1.0
    profile = select_font_profile()
    app.setFont(build_base_font(profile, ui_scale=ui_scale))
    app.setStyle("Fusion")
    app.setStyleSheet(gcs_stylesheet(mono_family=profile.mono_family, ui_scale=ui_scale))
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
