import QtQuick
import QtQuick.Controls

// Stand-in for QGC's ConfigButton, a line of the Vehicle Configuration list (VehicleConfigView.qml).
Button {
    property bool setupComplete: true
    property bool expandable:    false
    property bool expanded:      false

    signal toggleExpand()
}
