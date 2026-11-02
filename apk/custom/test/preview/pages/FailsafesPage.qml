// Stand-in for the Failsafes setup page of the preview's made-up ArduPilot plugin, at the address
// its component names (qrc:/preview/vehicle/APMFailsafesComponent.qml). The check opens it in
// Advanced Mode, then turns Advanced Mode off: the page must close.

import QtQuick

import QGroundControl.Controls

Item {
    objectName: "previewFailsafesPage"

    QGCLabel {
        text: "Failsafes setup (stand-in)"
    }
}
