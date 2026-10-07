// Stand-in for QGC's HelpSettings.qml, the page with links to QGC, PX4,
// ArduPilot and Discord. The VAMA app must never open it (client feedback
// 2026-10-07, task F), so loading it is a preview problem.

import QtQuick

import QGroundControl.Controls

Item {
    property int sectionFilter: -1

    QGCLabel {
        text: "QGC's Help page with its links"
    }

    Component.onCompleted: console.warn("QGC's Help page was opened")
}
