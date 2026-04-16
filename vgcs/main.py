"""VGCS application entrypoint."""

import sys

from PySide6.QtWidgets import QApplication

from vgcs.app.gcs_style import gcs_stylesheet
from vgcs.app.main_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(gcs_stylesheet())
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
