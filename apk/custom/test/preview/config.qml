// Stand-in for QGC's MainWindow around Vehicle Configuration (the client, 2026-10-30: outside
// Advanced Mode only Summary, Radio and Sensors). The Loader opens QGC's own address, as
// MainWindow's showTool does, so the preview also proves that the VAMA copy in custom.qrc
// replaces QGC's VehicleConfigView.qml.

import QtQuick
import QtQuick.Window

Window {
    id:      window
    width:   1280
    height:  720
    visible: true
    color:   "black"

    // The MainWindow name the page uses.
    QtObject {
        id: mainWindow
        function allowViewSwitch() { return true }
    }

    Loader {
        objectName:   "vamaConfigLoader"
        anchors.fill: parent
        source:       "qrc:/qml/QGroundControl/VehicleSetup/VehicleConfigView.qml"
    }
}
