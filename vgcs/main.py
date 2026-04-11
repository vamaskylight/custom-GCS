"""VGCS application entrypoint."""

import sys

from PySide6.QtWidgets import QApplication

from vgcs.app.main_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
