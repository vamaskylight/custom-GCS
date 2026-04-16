"""VGCS application entrypoint."""

import sys

from PySide6.QtWidgets import QApplication

from vgcs.app.gcs_style import gcs_stylesheet
from vgcs.app.main_window import MainWindow
from vgcs.app.runtime_ui import (
    build_base_font,
    configure_high_dpi_policy,
    select_font_profile,
)


def main() -> int:
    configure_high_dpi_policy()
    app = QApplication(sys.argv)
    profile = select_font_profile()
    app.setFont(build_base_font(profile))
    app.setStyle("Fusion")
    app.setStyleSheet(gcs_stylesheet(mono_family=profile.mono_family))
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
