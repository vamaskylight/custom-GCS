// Stand-in for QGC's MainWindow around Application Settings (client feedback
// 2026-10-07, tasks E and F). showSettingsTool() gives the tool drawer's
// Loader QGC's own address, as MainWindow.showSettingsTool() does, so the
// preview also proves that the VAMA copy in custom.qrc replaces QGC's file.

import QtQuick
import QtQuick.Window

Window {
    id:      window
    width:   1920
    height:  1080
    visible: true
    color:   "black"

    // The two MainWindow names the settings screen uses.
    QtObject {
        id: globals
        property bool commingFromRIDIndicator: false
    }
    QtObject {
        id: mainWindow
        function allowViewSwitch() { return true }
    }

    Rectangle {
        id:     toolDrawerToolbar
        width:  parent.width
        height: 92
        color:  "#e0000000"
        Text { anchors.centerIn: parent; color: "white"; text: "QGC toolbar: Application Settings"; font.pixelSize: 30 }
    }

    Item {
        id:             toolDrawer
        anchors.left:   parent.left
        anchors.right:  parent.right
        anchors.top:    toolDrawerToolbar.bottom
        anchors.bottom: parent.bottom

        property alias toolSource: toolDrawerLoader.source

        Loader {
            id:           toolDrawerLoader
            objectName:   "toolDrawerLoader"
            anchors.fill: parent
        }
    }

    function showSettingsTool(settingsPage) {
        toolDrawer.toolSource = "qrc:/qml/QGroundControl/Controls/AppSettings.qml"
        if (settingsPage !== "") {
            toolDrawerLoader.item.showSettingsPage(settingsPage)
        }
    }
}
