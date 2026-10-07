// Copy of QGC 5.1.5's src/QmlControls/SidebarDivider.qml for the preview.
// The line between page groups of Application Settings.

import QtQuick
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls

/// Horizontal separator between groups in a sidebar navigation list
Item {
    Layout.fillWidth:       true
    Layout.preferredHeight: ScreenTools.defaultFontPixelHeight / 2

    QGCPalette { id: qgcPal }

    Rectangle {
        anchors.left:           parent.left
        anchors.right:          parent.right
        anchors.verticalCenter: parent.verticalCenter
        height:                 1
        color:                  qgcPal.windowShade
    }
}
