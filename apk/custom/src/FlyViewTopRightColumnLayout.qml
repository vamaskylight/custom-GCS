// VAMA GCS: replaces QGC's FlyViewTopRightColumnLayout.qml (see custom.qrc).
//
// QGC shows its own photo and video buttons here (PhotoVideoControl). They
// send MAVLink camera commands, which the Skydroid camera does not use, so
// they did nothing next to our camera controls in FlyViewCustomLayer.qml.
// Only the terrain download progress is kept.

import QtQuick
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import QGroundControl.FlyView
import QGroundControl.FlightMap

ColumnLayout {
    spacing: ScreenTools.defaultFontPixelHeight / 2

    TerrainProgress {
        Layout.fillWidth: true
    }
}
