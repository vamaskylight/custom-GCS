"""The GPS in QGC's toolbar with our VehicleGPSIndicator.qml (A8 step 4, 2026-10-30).

Our file (src/VehicleGPSIndicator.qml, served by custom.qrc at QGC's address) puts "CAM"
beside QGC's own GPS indicator while the drone flies on the down camera's position (GPS 2),
and opens QGC's GPS page with a "Position source" part above QGC's own content. It is not
in the folder of QGC's toolbar files, so it imports them as QGC's module.

This renders it offscreen with Qt 6.11.1 (the app's Qt, through PySide6), from a folder
outside QGC's module as in the app, with QGC 5.1.5's own GPSIndicator.qml,
GPSIndicatorPage.qml, ToolIndicatorPage.qml, SettingsGroupLayout.qml and LabelledLabel.qml
from apk/qgc-src. QGC's C++ parts (the QGroundControl object, the palette, ScreenTools, the
drone and its facts, the drawer of mainWindow) and the app's store (src/PositionSourceStore.h)
are stood in for by small QML files in a temporary folder. The store itself is checked by
test/link/position_source_test.cc.

QGC's own VehicleGPSIndicator.qml is shown beside ours, for its size and to show that it
has no tag.

Usage:
    py apk/custom/test/gps_indicator_view_test.py

Needs apk/qgc-src (fetch_qgc.ps1) and PySide6. Nothing leaves this computer.
"""
from __future__ import annotations

import os
import pathlib
import shutil
import sys
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

HERE = pathlib.Path(__file__).resolve().parent
QGC_SRC = HERE.parents[1] / "qgc-src" / "src"
OURS = HERE.parent / "src" / "VehicleGPSIndicator.qml"

ORANGE = "#ff9800"
RED = "#e1544c"
TEXT = "#ffffff"
CAMERA_TEXT = "The camera (GPS 2), 3D, about 4.2 m (GPS 1 has no 3D fix)"
CAMERA_DETAILS = ("Where the drone's position comes from: GPS 1 while it has a 3D fix, else GPS 2.\n"
                  "GPS 2 is the down camera's position (GPS-denied navigation, vama_nav on the Pi).")


class Report:
    def __init__(self) -> None:
        self.lines: list[str] = []
        self.failed = 0

    def check(self, ok: bool, text: str) -> None:
        self.lines.append(("    ok    " if ok else "    FAIL  ") + text)
        if not ok:
            self.failed += 1


def write(path: pathlib.Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def stand_ins(root: pathlib.Path) -> None:
    """QGC's modules as far as the indicator and its page use them."""
    gc = root / "QGroundControl"
    write(gc / "qmldir", "module QGroundControl\nsingleton QGroundControl 1.0 QGroundControl.qml\n"
                         "singleton PositionSourceStore 1.0 PositionSourceStore.qml\n"
                         "BaseModeDefinition 1.0 BaseModeDefinition.qml\n")
    # QGC's enum of the RTK base's modes (RTKSettings.h), which the page's RTK settings read.
    write(gc / "BaseModeDefinition.qml",
          "import QtQuick\nQtObject {\n    enum Mode { BaseSurveyIn = 0, BaseFixed = 1 }\n}\n")
    fact = ("QtObject { property var value: 0; property string valueString: \"0\"; property var rawValue: 0; "
            "property bool userVisible: true }")
    write(gc / "QGroundControl.qml", f"""pragma Singleton
import QtQuick
QtObject {{
    property QtObject multiVehicleManager: QtObject {{
        property var activeVehicle: null
        property bool parameterReadyVehicleAvailable: true
    }}
    property QtObject gpsRtk: QtObject {{
        property QtObject connected: QtObject {{ property bool value: false }}
        property QtObject active: QtObject {{ property bool value: false }}
        property QtObject numSatellites: QtObject {{ property int value: 0 }}
        property QtObject currentDuration: QtObject {{ property int value: 0 }}
        property QtObject valid: QtObject {{ property bool value: false }}
        property QtObject currentAccuracy: {fact}
        property QtObject currentLatitude: {fact}
        property QtObject currentLongitude: {fact}
        property QtObject currentAltitude: {fact}
    }}
    property QtObject settingsManager: QtObject {{
        property QtObject rtkSettings: QtObject {{
            property QtObject useFixedBasePosition: {fact}
            property QtObject baseReceiverManufacturers: {fact}
            property QtObject surveyInAccuracyLimit: QtObject {{ property bool userVisible: true }}
            property QtObject surveyInMinObservationDuration: QtObject {{ property bool userVisible: true }}
            property QtObject fixedBasePositionLatitude: QtObject {{ property string shortDescription: "Latitude" }}
            property QtObject fixedBasePositionLongitude: QtObject {{ property string shortDescription: "Longitude" }}
            property QtObject fixedBasePositionAltitude: QtObject {{ property string shortDescription: "Altitude" }}
            property QtObject fixedBasePositionAccuracy: QtObject {{ property string shortDescription: "Accuracy" }}
        }}
        property QtObject autoConnectSettings: QtObject {{
            property QtObject autoConnectRTKGPS: QtObject {{ property bool userVisible: true }}
        }}
    }}
    property QtObject unitsConversion: QtObject {{ property string appSettingsHorizontalDistanceUnitsString: "m" }}
    property QtObject globalPalette: QtObject {{ property color groupBorder: "#808080" }}
}}
""")
    # The app's store (src/PositionSourceStore.h): forVehicle gives the scene's stand-in for the drone.
    write(gc / "PositionSourceStore.qml",
          "pragma Singleton\nimport QtQuick\nQtObject {\n    id: store\n    property var current: null\n"
          "    function forVehicle(vehicleId) { return store.current }\n}\n")

    controls = gc / "Controls"
    write(controls / "qmldir", "module QGroundControl.Controls\nsingleton ScreenTools 1.0 ScreenTools.qml\n"
                               "QGCLabel 1.0 QGCLabel.qml\nQGCColoredImage 1.0 QGCColoredImage.qml\n"
                               "QGCPalette 1.0 QGCPalette.qml\nQGCRadioButton 1.0 QGCRadioButton.qml\n"
                               "LabelledButton 1.0 LabelledButton.qml\nToolIndicatorPage 1.0 ToolIndicatorPage.qml\n"
                               "SettingsGroupLayout 1.0 SettingsGroupLayout.qml\nLabelledLabel 1.0 LabelledLabel.qml\n")
    write(controls / "ScreenTools.qml",
          "pragma Singleton\nimport QtQuick\nQtObject {\n    property real defaultFontPixelWidth: 8\n"
          "    property real defaultFontPixelHeight: 16\n    property real defaultFontPointSize: 12\n"
          "    property real smallFontPointSize: 10\n}\n")
    write(controls / "QGCLabel.qml", "import QtQuick\nText { color: \"white\"; font.pointSize: 12 }\n")
    # QGC's coloured image only as far as the indicator sets it: nothing is loaded.
    write(controls / "QGCColoredImage.qml",
          "import QtQuick\nItem {\n    property url source\n    property int fillMode\n    property size sourceSize\n"
          "    property color color\n}\n")
    write(controls / "QGCPalette.qml",
          f"import QtQuick\nQtObject {{\n    property color text: \"{TEXT}\"\n    property color colorOrange: \"{ORANGE}\"\n"
          f"    property color colorRed: \"{RED}\"\n}}\n")
    write(controls / "QGCRadioButton.qml", "import QtQuick\nItem { property string text; property bool checked; signal clicked() }\n")
    write(controls / "LabelledButton.qml",
          "import QtQuick\nItem { property string label; property string buttonText; signal clicked() }\n")
    for name in ("ToolIndicatorPage.qml", "SettingsGroupLayout.qml", "LabelledLabel.qml"):
        shutil.copy2(QGC_SRC / "QmlControls" / name, controls / name)

    facts = gc / "FactControls"
    write(facts / "qmldir", "module QGroundControl.FactControls\nFactCheckBoxSlider 1.0 FactCheckBoxSlider.qml\n"
                            "FactComboBox 1.0 FactComboBox.qml\nFactSlider 1.0 FactSlider.qml\n"
                            "LabelledFactTextField 1.0 LabelledFactTextField.qml\n")
    write(facts / "FactCheckBoxSlider.qml", "import QtQuick\nItem { property var fact; property string text }\n")
    write(facts / "FactComboBox.qml", "import QtQuick\nItem { property var fact }\n")
    write(facts / "FactSlider.qml", "import QtQuick\nItem { property var fact; property string label; property real majorTickStepSize }\n")
    write(facts / "LabelledFactTextField.qml", "import QtQuick\nItem { property var fact; property string label }\n")

    toolbar = gc / "Toolbar"
    toolbar.mkdir(parents=True, exist_ok=True)
    for name in ("GPSIndicator.qml", "GPSIndicatorPage.qml", "VehicleGPSIndicator.qml"):
        shutil.copy2(QGC_SRC / "Toolbar" / name, toolbar / name)
    write(toolbar / "qmldir", "module QGroundControl.Toolbar\nGPSIndicator 1.0 GPSIndicator.qml\n"
                              "GPSIndicatorPage 1.0 GPSIndicatorPage.qml\nVehicleGPSIndicator 1.0 VehicleGPSIndicator.qml\n")


def scene(folder: pathlib.Path, ours: pathlib.Path, qgc: pathlib.Path) -> pathlib.Path:
    """A toolbar with our indicator and QGC's own, a made-up drone, and mainWindow's drawer."""
    body = f"""import QtQuick
import QGroundControl
Rectangle {{
    id: root
    width: 900; height: 700; color: "#202428"

    property alias theDrone: drone
    property alias theSource: source
    property alias drawer: mainWindow
    // In the app the toolbar's indicators are made once there is a drone.
    property bool ready: false

    QtObject {{
        id: drone
        property int id: 1
        property QtObject gps: QtObject {{
            property bool telemetryAvailable: true
            property QtObject count: QtObject {{ property real value: 0; property string valueString: "0" }}
            property QtObject lock: QtObject {{ property int value: 1; property string enumStringValue: "None" }}
            property QtObject hdop: QtObject {{ property real value: 99.9; property string valueString: "99.9" }}
            property QtObject vdop: QtObject {{ property real value: 99.9; property string valueString: "99.9" }}
            property QtObject courseOverGround: QtObject {{ property real value: 0; property string valueString: "0.0" }}
            property QtObject systemErrors: QtObject {{ property int value: 0 }}
        }}
        property QtObject parameterManager: QtObject {{ property bool parameterDownloadSkipped: false }}
    }}
    // What the app's store gives for the drone (src/PositionSourceStore.h).
    QtObject {{
        id: source
        property bool hasGps2: false
        property string tag: ""
        property string text: "N/A"
        property string level: "na"
        property string details: ""
    }}
    Component.onCompleted: {{
        PositionSourceStore.current = source
        QGroundControl.multiVehicleManager.activeVehicle = drone
        ready = true
    }}

    // The toolbar: QGC loads each indicator by its address (FirmwarePlugin::toolIndicators).
    Row {{
        id: toolbar
        height: 48
        spacing: 40
        Loader {{ objectName: "oursLoader"; height: parent.height; active: root.ready; source: "{ours.as_uri()}" }}
        Loader {{ objectName: "qgcLoader"; height: parent.height; active: root.ready; source: "{qgc.as_uri()}" }}
    }}

    // MainWindow.qml's drawer, as far as an indicator uses it.
    Item {{
        id: mainWindow
        objectName: "drawer"
        y: 60
        width: parent.width
        height: parent.height - 60
        property var page: null
        property bool expanded: false
        property int opened: 0
        function showIndicatorDrawer(drawerComponent, indicatorItem) {{
            if (page) {{
                page.destroy()
            }}
            page = drawerComponent.createObject(mainWindow, {{ expanded: mainWindow.expanded }})
            opened += 1
        }}
    }}
}}
"""
    path = folder / "scene.qml"
    path.write_text(body, encoding="utf-8")
    return path


def main() -> int:
    if not (QGC_SRC / "Toolbar" / "GPSIndicator.qml").exists():
        print(f"SKIPPED: QGC's source is not in {QGC_SRC.parent} (run apk/fetch_qgc.ps1)")
        return 0
    from PySide6.QtCore import QPoint, QPointF, Qt, QTimer, QUrl, qInstallMessageHandler
    from PySide6.QtGui import QColor, QGuiApplication
    from PySide6.QtQuick import QQuickView
    from PySide6.QtTest import QTest

    app = QGuiApplication.instance() or QGuiApplication([])
    warnings: list[str] = []
    qInstallMessageHandler(lambda _kind, _context, message: warnings.append(message))
    work = pathlib.Path(tempfile.mkdtemp(prefix="vama-gps-indicator-"))
    r = Report()

    def wait(ms: int) -> None:
        waited: list[int] = []
        QTimer.singleShot(ms, lambda: waited.append(1))
        while not waited:
            app.processEvents()

    try:
        stand_ins(work / "imports")
        # As in the app, our file is not in the folder of QGC's toolbar files (custom.qrc puts it
        # under /Custom), so QGC's GPSIndicator comes from QGC's module only.
        ours = work / "Custom" / "qml" / "QGroundControl" / "Toolbar" / "VehicleGPSIndicator.qml"
        ours.parent.mkdir(parents=True)
        shutil.copy2(OURS, ours)
        qgc = work / "imports" / "QGroundControl" / "Toolbar" / "VehicleGPSIndicator.qml"

        view = QQuickView()
        view.engine().addImportPath(str(work / "imports"))
        view.setSource(QUrl.fromLocalFile(str(scene(work, ours, qgc))))
        view.show()
        wait(200)
        r.check(not [e.toString() for e in view.errors()], f"the scene loads {[e.toString() for e in view.errors()][:1]}")
        root = view.rootObject()
        drone, source, drawer = root.property("theDrone"), root.property("theSource"), root.property("drawer")

        def walk(item):
            for child in item.childItems():
                yield child
                yield from walk(child)

        def find(name: str, under=None):
            for item in walk(under or root):
                if item.objectName() == name:
                    return item
            return None

        def loaded(name: str):
            loader = find(name)
            return loader.property("item") if loader is not None else None

        def texts(under) -> list[str]:
            if under is None:
                return []
            return [item.property("text") for item in walk(under)
                    if isinstance(item.property("text"), str) and item.property("text") and item.isVisible()]

        def colour(item) -> str:
            return QColor(item.property("color")).name() if item is not None else "none"

        def tap(item) -> None:
            point = item.mapToScene(QPointF(item.width() / 2.0, item.height() / 2.0))
            QTest.mouseClick(view, Qt.MouseButton.LeftButton, Qt.KeyboardModifier.NoModifier,
                             QPoint(round(point.x()), round(point.y())))
            wait(100)

        indicator, qgc_indicator = loaded("oursLoader"), loaded("qgcLoader")
        r.check(indicator is not None and indicator.objectName() == "toolbar_gpsIndicator",
                "our file loads in the toolbar, with QGC's object name")
        tag = find("vamaPositionSourceTag", indicator) if indicator is not None else None
        if indicator is None or tag is None:
            r.check(False, f"our indicator and its tag are there ({[e.toString() for e in view.errors()][:1]}, {warnings[:2]})")
            print("\n".join(r.lines))
            return 1

        # --- a drone without GPS 2: as before --------------------------------------------------
        r.check(tag is not None and not tag.isVisible(), "no GPS 2: no tag")
        r.check(abs(indicator.width() - qgc_indicator.width()) < 0.5 and indicator.width() > 0,
                f"no GPS 2: as wide as QGC's own ({indicator.width():.0f} and {qgc_indicator.width():.0f} px)")
        r.check(indicator.property("showIndicator") is True and qgc_indicator.property("showIndicator") is True,
                "no GPS 2: shown, as QGC's own")
        r.check(find("vamaPositionSourceTag", qgc_indicator) is None, "(QGC's own file has no tag)")
        tap(indicator)
        page = drawer.property("page")
        r.check(drawer.property("opened") == 1 and page is not None, "a tap on it opens the GPS page")
        group = find("vamaPositionSourceGroup", page) if page else None
        r.check(group is not None and not group.isVisible(), "no GPS 2: the page has no position source part")
        shown = texts(page) if page else []
        r.check("Vehicle GPS Status" in shown and "Satellites" in shown and "None" in shown,
                f"QGC's own content is in the page, with the drone's values ({shown[:6]})")

        # --- the camera gives the position -----------------------------------------------------
        source.setProperty("hasGps2", True)
        source.setProperty("tag", "CAM")
        source.setProperty("level", "warn")
        source.setProperty("text", CAMERA_TEXT)
        source.setProperty("details", CAMERA_DETAILS)
        wait(100)
        r.check(tag.isVisible() and tag.property("text") == "CAM", "the camera: \"CAM\" beside the satellites")
        r.check(colour(tag) == ORANGE, f"the camera: the tag is orange ({colour(tag)})")
        r.check(indicator.width() > qgc_indicator.width() + tag.width(),
                f"the camera: the indicator is wider by the tag ({indicator.width():.0f} px, QGC's {qgc_indicator.width():.0f})")
        tag_left = tag.mapToScene(QPointF(0, 0)).x()
        qgc_part = indicator.childItems()[0].childItems()[0]
        r.check(tag_left >= qgc_part.mapToScene(QPointF(qgc_part.width(), 0)).x(),
                "the camera: the tag is after QGC's icon and numbers")
        tap(tag)
        page = drawer.property("page")
        r.check(drawer.property("opened") == 2 and page is not None, "a tap on the tag opens the page too")
        group = find("vamaPositionSourceGroup", page)
        text = find("vamaPositionSourceText", page)
        details = find("vamaPositionSourceDetails", page)
        r.check(group is not None and group.isVisible(), "the camera: the page has the position source part")
        r.check(text is not None and text.property("text") == CAMERA_TEXT and colour(text) == ORANGE,
                f"the camera: VGCS's line, orange ({text.property('text') if text else None}, {colour(text)})")
        r.check(details is not None and details.isVisible() and details.property("text") == CAMERA_DETAILS,
                "the camera: what GPS 2 is, under it")
        shown = texts(page)
        headings = [t for t in shown if t in ("Position source", "Vehicle GPS Status")]
        r.check(headings == ["Position source", "Vehicle GPS Status"],
                f"the position source is first, QGC's content under it, once each ({headings})")
        status = [item for item in walk(page) if item.property("text") == "Vehicle GPS Status"]
        r.check(group is not None and bool(status) and
                status[0].mapToScene(QPointF(0, 0)).y() >= group.mapToScene(QPointF(0, group.height())).y() - 0.5,
                "and QGC's content lies under it, not over it")

        source.setProperty("level", "bad")
        source.setProperty("text", "None: GPS 1 and the camera (GPS 2) have no 3D fix")
        source.setProperty("tag", "")
        wait(100)
        r.check(colour(text) == RED and not tag.isVisible(), f"no position: the line is red, no tag ({colour(text)})")
        source.setProperty("level", "ok")
        source.setProperty("text", "GPS 1 (the camera (GPS 2): 3D, about 1.5 m)")
        wait(100)
        r.check(colour(text) == TEXT, f"GPS 1: the line in the toolbar's colour ({colour(text)})")

        # QGC's content reads the drone through names of the indicator around it (its _activeVehicle):
        # its "GPS Error" line shows only when that works.
        drone.property("gps").property("systemErrors").setProperty("value", 4)
        wait(100)
        shown = texts(page)
        r.check("GPS Error" in shown and "Software" in shown, f"QGC's content finds the drone, as in QGC ({shown[-3:]})")
        drone.property("gps").property("systemErrors").setProperty("value", 0)

        # --- QGC's expanded part: the RTK settings, once ----------------------------------------
        drawer.setProperty("expanded", True)
        tap(indicator)
        page = drawer.property("page")
        wait(100)
        shown = texts(page)
        r.check(shown.count("RTK GPS Settings") == 1 and "Position source" in shown,
                f"expanded: QGC's RTK settings once, the position source still there ({shown.count('RTK GPS Settings')})")
        drawer.setProperty("expanded", False)

        # --- when it shows ------------------------------------------------------------------------
        drone.property("gps").setProperty("telemetryAvailable", False)
        wait(50)
        r.check(indicator.property("showIndicator") is True and qgc_indicator.property("showIndicator") is False,
                "a drone that reports only GPS 2: ours shows, QGC's own did not")
        source.setProperty("hasGps2", False)
        wait(50)
        r.check(indicator.property("showIndicator") is False, "no GPS report at all: not shown")
        drone.property("gps").setProperty("telemetryAvailable", True)

        # (PySide6 brings no fonts: Qt says so once. Not a QML warning.)
        bad = [w for w in warnings if not w.startswith("QFontDatabase:")]
        r.check(not bad, f"no QML warnings ({bad[:3]})")
        view.close()
        view.deleteLater()
        app.processEvents()
    finally:
        qInstallMessageHandler(None)
        shutil.rmtree(work, ignore_errors=True)

    print("\n".join(r.lines))
    print(f"{len(r.lines) - r.failed} of {len(r.lines)} checks passed")
    return 1 if r.failed else 0


if __name__ == "__main__":
    sys.exit(main())
