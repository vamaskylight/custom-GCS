import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls

// VAMA (B19, the client 2026-10-30: "like i set the range to 5 metre"; B25 2026-10-31: the round view
// replaced the bars): how far the round proximity view reaches (VamaProximityWindow.qml, its rings
// every tenth of it), saved by the store (ProximityStore.h). The app menu
// (SelectViewDropdown.qml) shows this in place of its buttons, as it shows the voice: a window made
// from the menu would stop working when the menu closes (VamaAdminPasswordPanel.qml). `finished`
// closes the menu.
ColumnLayout {
    id:      root
    spacing: ScreenTools.defaultFontPixelHeight / 2

    property var store: ProximityStore

    property real _width: ScreenTools.defaultFontPixelWidth * 44

    signal finished()

    QGCLabel { text: qsTr("Proximity range"); font.bold: true }

    QGCLabel {
        Layout.preferredWidth: root._width
        wrapMode:              Text.WordWrap
        text:                  qsTr("How far the round proximity view reaches. An object farther away is not drawn. It changes nothing on the drone.")
    }

    QGCComboBox {
        objectName:            "vamaObstacleRangeBox"
        Layout.preferredWidth: root._width
        model:                 root.store.barRangeChoices.map(function(metres) { return qsTr("%1 m").arg(metres) })
        currentIndex:          root.store.barRangeChoices.indexOf(root.store.barRange)
        onActivated:           (index) => { root.store.barRange = root.store.barRangeChoices[index] }
    }

    QGCButton {
        objectName:       "vamaObstacleRangeDone"
        Layout.alignment: Qt.AlignRight
        text:             qsTr("Done")
        onClicked:        root.finished()
    }
}
