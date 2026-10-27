"""QGC's obstacle radar, on the map and on the video, with our ProximityRadarValues.qml (A7, 2026-10-27).

Renders QGC 5.1.5's own ProximityRadarMapView.qml (on an offline map) and
ProximityRadarVideoView.qml from apk/qgc-src, offscreen, with Qt 6.11.1 (the
app's Qt, through PySide6), and a made-up drone whose obstacle sensor sends
what ArduPilot sends: a distance in each of 8 directions, an empty direction as
the sensor's maximum, and 0 as the maximum of a sensor whose range ArduPilot
does not know (the client's RPLidar A3).

QGC's own ProximityRadarValues.qml put a "0.00" label in the middle of the
video for each empty direction of such a sensor. The checks below run our copy
(src/ProximityRadarValues.qml, served by custom.qrc in place of QGC's file),
and QGC's own file once, to show that the copy is still needed.

QGC's own modules (ScreenTools, QGCLabel) are stood in for by small QML files
in a temporary folder.

Usage:
    py apk/custom/test/proximity_view_test.py

Needs apk/qgc-src (fetch_qgc.ps1) and PySide6. Nothing leaves this computer:
the map uses the offline "itemsoverlay" plugin, no tiles.
"""
from __future__ import annotations

import math
import os
import pathlib
import shutil
import sys
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

HERE = pathlib.Path(__file__).resolve().parent
QGC_SRC = HERE.parents[1] / "qgc-src" / "src"
OURS = HERE.parent / "src" / "ProximityRadarValues.qml"

ROTATIONS = ["rotationNone", "rotationYaw45", "rotationYaw90", "rotationYaw135", "rotationYaw180",
             "rotationYaw225", "rotationYaw270", "rotationYaw315"]
MAP_SIZE = (640, 480)
VIDEO_SIZE = (640, 360)
RED = (255, 0, 0)
AMBER_LIKE_WHITE = (255, 255, 255)


class Report:
    def __init__(self) -> None:
        self.lines: list[str] = []
        self.failed = 0

    def check(self, ok: bool, text: str) -> None:
        self.lines.append(("    ok    " if ok else "    FAIL  ") + text)
        if not ok:
            self.failed += 1


def stand_ins(root: pathlib.Path, values_file: pathlib.Path) -> None:
    """QGC's modules as far as the two views use them, with the given ProximityRadarValues.qml."""
    gc = root / "QGroundControl"
    for sub in ("Controls", "FlyView", "FlightMap"):
        (gc / sub).mkdir(parents=True, exist_ok=True)
    (gc / "qmldir").write_text("module QGroundControl\nsingleton ScreenTools 1.0 ScreenTools.qml\n")
    (gc / "ScreenTools.qml").write_text(
        "pragma Singleton\nimport QtQuick\nQtObject { property real defaultFontPixelHeight: 16 }\n")
    (gc / "Controls" / "qmldir").write_text("module QGroundControl.Controls\nQGCLabel 1.0 QGCLabel.qml\n")
    (gc / "Controls" / "QGCLabel.qml").write_text("import QtQuick\nText { color: \"white\"; font.pixelSize: 16 }\n")
    shutil.copy2(values_file, gc / "FlyView" / "ProximityRadarValues.qml")
    shutil.copy2(QGC_SRC / "FlyView" / "ProximityRadarVideoView.qml", gc / "FlyView" / "ProximityRadarVideoView.qml")
    (gc / "FlyView" / "qmldir").write_text(
        "module QGroundControl.FlyView\nProximityRadarValues 1.0 ProximityRadarValues.qml\n"
        "ProximityRadarVideoView 1.0 ProximityRadarVideoView.qml\n")
    shutil.copy2(QGC_SRC / "FlightMap" / "MapItems" / "ProximityRadarMapView.qml",
                 gc / "FlightMap" / "ProximityRadarMapView.qml")
    (gc / "FlightMap" / "qmldir").write_text(
        "module QGroundControl.FlightMap\nProximityRadarMapView 1.0 ProximityRadarMapView.qml\n")


def drone_qml(sectors: list, max_m: float | None, heading: float, telemetry: bool = True) -> str:
    """A made-up vehicle with QGC's distanceSensors fact group: each fact a value and its text."""
    def fact(value):
        number = "NaN" if value is None else repr(float(value))
        return (f"QtObject {{ property real value: {number}; "
                "property string valueString: isNaN(value) ? \"--.--\" : value.toFixed(2) }")
    rows = "\n".join(f"        property QtObject {name}: {fact(sectors[i])}" for i, name in enumerate(ROTATIONS))
    return ("QtObject {\n"
            f"    property QtObject heading: QtObject {{ property real value: {heading} }}\n"
            "    property QtObject distanceSensors: QtObject {\n"
            f"        property bool telemetryAvailable: {'true' if telemetry else 'false'}\n"
            f"{rows}\n"
            f"        property QtObject maxDistance: {fact(max_m)}\n"
            "    }\n"
            "}\n")


def scene(folder: pathlib.Path, kind: str, drone: str) -> pathlib.Path:
    if kind == "map":
        body = f"""import QtQuick
import QtLocation
import QtPositioning
import QGroundControl.FlightMap
Rectangle {{
    width: {MAP_SIZE[0]}; height: {MAP_SIZE[1]}; color: "#202428"
    Map {{
        id: theMap
        anchors.fill: parent
        plugin: Plugin {{ name: "itemsoverlay" }}
        center: QtPositioning.coordinate(20.4347, 72.8696)
        zoomLevel: 19
        color: "transparent"
        ProximityRadarMapView {{
            objectName: "radar"
            vehicle: theDrone
            coordinate: theMap.center
            map: theMap
        }}
    }}
    property QtObject theDrone: {drone}
}}
"""
    else:
        body = f"""import QtQuick
import QGroundControl.FlyView
Rectangle {{
    width: {VIDEO_SIZE[0]}; height: {VIDEO_SIZE[1]}; color: "#202428"
    ProximityRadarVideoView {{
        objectName: "radar"
        vehicle: theDrone
    }}
    property QtObject theDrone: {drone}
}}
"""
    path = folder / f"scene_{kind}.qml"
    path.write_text(body, encoding="utf-8")
    return path


class Rendered:
    """What a view drew: the picture's red and white pixels, and its visible labels."""

    def __init__(self, image, labels: list[tuple[str, float, float]], errors: list[str]) -> None:
        import numpy as np

        rgba = image.convertToFormat(image.Format.Format_RGBA8888)
        width, height = rgba.width(), rgba.height()
        data = np.frombuffer(rgba.constBits(), dtype=np.uint8, count=rgba.sizeInBytes())
        pixels = data.reshape(height, rgba.bytesPerLine())[:, : width * 4].reshape(height, width, 4).astype(int)
        r, g, b = pixels[..., 0], pixels[..., 1], pixels[..., 2]
        self.size = (width, height)
        ys, xs = np.nonzero((r > 110) & (g < 80) & (b < 80) & (r - g > 60))
        self.red = list(zip(xs.tolist(), ys.tolist()))
        self.white = int(np.count_nonzero((r > 120) & (g > 120) & (b > 120)))
        self.labels = labels
        self.errors = errors

    def red_angles_deg(self) -> list[float]:
        """Where the red pixels are, seen from the middle: 0 up (north on the map, ahead on the video), clockwise."""
        cx, cy = self.size[0] / 2.0, self.size[1] / 2.0
        return [math.degrees(math.atan2(x - cx, cy - y)) % 360.0 for x, y in self.red]

    def red_distance_px(self) -> list[float]:
        cx, cy = self.size[0] / 2.0, self.size[1] / 2.0
        return [math.hypot(x - cx, y - cy) for x, y in self.red]


def render(app, folder: pathlib.Path, kind: str, drone: str, change: dict | None = None) -> Rendered:
    """Show the view and take its picture. ``change`` ({fact name: metres}): new readings after the first
    picture, and the picture after them (a view that draws only once would keep its first)."""
    from PySide6.QtCore import QPointF, QTimer, QUrl
    from PySide6.QtGui import QColor
    from PySide6.QtQuick import QQuickView

    def wait(ms: int) -> None:
        waited: list[int] = []
        QTimer.singleShot(ms, lambda: waited.append(1))
        while not waited:
            app.processEvents()

    view = QQuickView()
    view.engine().addImportPath(str(folder / "imports"))
    view.setColor(QColor("#202428"))
    view.setSource(QUrl.fromLocalFile(str(scene(folder, kind, drone))))
    errors = [e.toString() for e in view.errors()]
    size = MAP_SIZE if kind == "map" else VIDEO_SIZE
    view.resize(*size)
    view.show()
    # The map view sizes itself 100 ms after the map has (its scaleTimer).
    wait(500)
    image = view.grabWindow()
    if change:
        sensors = view.rootObject().property("theDrone").property("distanceSensors")
        for name, metres in change.items():
            sensors.property(name).setProperty("value", float(metres))
        wait(300)
        image = view.grabWindow()

    def walk(item):
        for child in item.childItems():
            yield child
            yield from walk(child)

    labels = []
    root = view.rootObject()
    for item in ([] if root is None else walk(root)):
        text = item.property("text")
        if isinstance(text, str) and text and item.isVisible():
            p = item.mapToScene(QPointF(item.width() / 2.0, item.height() / 2.0))
            labels.append((text, p.x() - size[0] / 2.0, p.y() - size[1] / 2.0))
    out = Rendered(image, labels, errors)
    view.close()
    view.deleteLater()
    app.processEvents()
    return out


def near(value: float, wanted: float, tolerance: float) -> bool:
    return abs(value - wanted) <= tolerance


def within_deg(angles: list[float], centre: float, half_width: float) -> bool:
    return bool(angles) and all(abs((a - centre + 180.0) % 360.0 - 180.0) <= half_width for a in angles)


def main() -> int:
    if not (QGC_SRC / "FlyView" / "ProximityRadarVideoView.qml").exists():
        print(f"SKIPPED: QGC's source is not in {QGC_SRC.parent} (run apk/fetch_qgc.ps1)")
        return 0
    from PySide6.QtGui import QGuiApplication

    app = QGuiApplication.instance() or QGuiApplication([])
    work = pathlib.Path(tempfile.mkdtemp(prefix="vama-proximity-view-"))
    r = Report()
    try:
        def with_values(values_file: pathlib.Path) -> pathlib.Path:
            folder = work / values_file.parent.name
            if not (folder / "imports").exists():
                stand_ins(folder / "imports", values_file)
            return folder

        ours = with_values(OURS)
        qgc = with_values(QGC_SRC / "FlyView" / "ProximityRadarValues.qml")
        a3_ahead = drone_qml([5.0, 0, 0, 0, 0, 0, 0, 0], 0.0, 0.0)       # the A3: range 0, 5 m ahead

        # --- why the copy exists: QGC's own file ---------------------------------------------
        video = render(app, qgc, "video", a3_ahead)
        piled = [t for t, x, y in video.labels if abs(x) < 2 and abs(y) < 2]
        r.check(len(piled) == 7, f"QGC's own file: 7 labels in the middle of the video for the A3's empty directions "
                                 f"({piled}); when this fails QGC has changed and the copy may not be needed")

        # --- ours, the A3 (range 0) -------------------------------------------------------------
        video = render(app, ours, "video", a3_ahead)
        r.check(not video.errors, f"the views load with our file {video.errors[:1]}")
        r.check([t for t, _x, _y in video.labels] == ["5.00"],
                f"video, the A3: only the reading is labelled, nothing in the middle ({video.labels})")
        if video.labels:
            _t, x, y = video.labels[0]
            # QGC's video radar shows 6 m to the picture's half height: 30 px a metre at 360 px.
            r.check(near(x, 0.0, 2.0) and near(y, -150.0, 3.0), f"and the label is ahead, 5 m out ({x:.0f}, {y:.0f})")
        # Stretched across the picture, the arc's 38 degrees look like 62 from the middle.
        r.check(within_deg(video.red_angles_deg(), 0.0, 40.0), "video, the A3: the one arc is ahead")

        shown = render(app, ours, "map", a3_ahead)
        r.check(not shown.errors, f"the map view loads with our file {shown.errors[:1]}")
        distances = shown.red_distance_px()
        r.check(within_deg(shown.red_angles_deg(), 0.0, 30.0) and bool(distances)
                and near(sum(distances) / len(distances), 18.0, 4.0),
                f"map, the A3: one red arc, ahead of the drone, 5 m out at 3.6 px a metre "
                f"({len(shown.red)} red pixels, mean {sum(distances) / max(1, len(distances)):.1f} px)")
        r.check(shown.white == 0, "map, the A3: no range circle (ArduPilot knows no range for it)")

        # The object comes nearer: both views follow (the video radar redraws only when told to).
        video = render(app, ours, "video", a3_ahead, change={"rotationNone": 2.0})
        r.check([(t, round(y)) for t, _x, y in video.labels] == [("2.00", -60)]
                and bool(video.red) and max(video.red_distance_px()) < 100.0,
                f"video, the A3: the object comes to 2 m, label and arc follow ({video.labels}, "
                f"arc out to {max(video.red_distance_px(), default=0.0):.0f} px)")
        shown = render(app, ours, "map", a3_ahead, change={"rotationNone": 2.0})
        distances = shown.red_distance_px()
        r.check(bool(distances) and near(sum(distances) / len(distances), 7.2, 3.0),
                f"map, the A3: the arc follows to 2 m (mean {sum(distances) / max(1, len(distances)):.1f} px)")
        video = render(app, ours, "video", a3_ahead, change={"rotationNone": 0.0})
        r.check(not video.labels and not video.red, f"video, the A3: the object gone, nothing left ({video.labels})")

        a3_two = drone_qml([0, 0, 3.0, 0, 4.0, 0, 0, 0], 0.0, 0.0)       # 3 m right, 4 m behind
        video = render(app, ours, "video", a3_two)
        got = sorted((t, round(x), round(y)) for t, x, y in video.labels)
        # The video radar is stretched to the picture's width: 30 px a metre up and down, 53 across.
        r.check(len(got) == 2 and got[0][0] == "3.00" and near(got[0][1], 160, 3) and near(got[0][2], 0, 3)
                and got[1][0] == "4.00" and near(got[1][1], 0, 3) and near(got[1][2], 120, 3),
                f"video, the A3: 3 m right and 4 m behind, where they are ({got})")

        # --- ours, a sensor with a known range (40 m) --------------------------------------------
        ranged = drone_qml([5.0, 40, 40, 40, 40, 40, 40, 40], 40.0, 0.0)
        shown = render(app, ours, "map", ranged)
        r.check(within_deg(shown.red_angles_deg(), 0.0, 30.0) and len(shown.red) < 150,
                f"map, 40 m range: only the reading is red, the empty directions are not drawn on the circle "
                f"({len(shown.red)} red pixels)")
        r.check(shown.white > 1000, f"map, 40 m range: the range circle is drawn ({shown.white} white pixels)")
        video = render(app, ours, "video", ranged)
        r.check([t for t, _x, _y in video.labels] == ["5.00"], f"video, 40 m range: one label ({video.labels})")

        qgc_map = render(app, qgc, "map", ranged)
        r.check(len(qgc_map.red) > 5 * max(1, len(shown.red)),
                f"(QGC's own file drew the 7 empty directions as red arcs on the circle: {len(qgc_map.red)} red pixels)")

        near_max = drone_qml([39.9, 40, 40, 40, 40, 40, 40, 40], 40.0, 0.0)
        video = render(app, ours, "video", near_max)
        r.check([t for t, _x, _y in video.labels] == ["39.90"],
                f"a reading just inside the range still counts ({video.labels})")

        # --- the drone's heading, and nothing before the first reading -----------------------------
        east = drone_qml([5.0, 0, 0, 0, 0, 0, 0, 0], 0.0, 90.0)
        shown = render(app, ours, "map", east)
        r.check(within_deg(shown.red_angles_deg(), 90.0, 30.0),
                "map: the arc ahead of a drone facing east is east of it (QGC turns the radar with the heading)")
        nothing = drone_qml([None] * 8, None, 0.0)
        video = render(app, ours, "video", nothing)
        r.check(not video.labels and not video.red, "before the first reading: no label, no arc")
        silent = drone_qml([5.0, 0, 0, 0, 0, 0, 0, 0], 0.0, 0.0, telemetry=False)
        video = render(app, ours, "video", silent)
        r.check(not video.labels and not video.red, "no DISTANCE_SENSOR from the drone: the radar is not shown")
    finally:
        shutil.rmtree(work, ignore_errors=True)

    print("\n".join(r.lines))
    print(f"{len(r.lines) - r.failed} of {len(r.lines)} checks passed")
    return 1 if r.failed else 0


if __name__ == "__main__":
    sys.exit(main())
