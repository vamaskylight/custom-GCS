// Stand-in for QGC's MainWindow around the app menu (task Z, the admin password for
// Advanced Mode). The drawer is built as in QGC 5.1.5's MainWindow.qml: the menu is made when
// the drawer opens and destroyed when it closes. The menu is QGC's own address, so the
// preview also proves that the VAMA copy in custom.qrc replaces QGC's file.

import QtQuick
import QtQuick.Controls
import QtQuick.Window

import QGroundControl.Controls

Window {
    id:      window
    width:   1920
    height:  1080
    visible: true
    color:   "black"

    readonly property bool menuOpen: indicatorDrawer.opened
    property int offQuestions: 0  // QGC's "Turn off Advanced Mode?" was asked
    property int flyViews: 0      // the menu switched to the fly view (task AG)

    function openMenu() { indicatorDrawer.open() }

    // The MainWindow names the menu uses.
    QtObject {
        id: mainWindow
        function allowViewSwitch() { return true }
        function closeIndicatorDrawer() { indicatorDrawer.close() }
        function showFlyView() { window.flyViews++ }
        function showPlanView() {}
        function showAnalyzeTool() {}
        function showVehicleConfig() {}
        function showSettingsTool() {}
        function close() {}
    }
    QtObject {
        id: advancedModeOffConfirmation
        function open() { window.offQuestions++ }
    }
    QtObject {
        id: showTouchAreasNotification
        function open() {}
    }

    Rectangle {
        width:  parent.width
        height: 92
        color:  "#e0000000"
        Text { anchors.centerIn: parent; color: "white"; text: "QGC toolbar: the app menu"; font.pixelSize: 30 }
    }

    Popup {
        id:          indicatorDrawer
        x:           10
        y:           102
        padding:     20
        modal:       true
        focus:       true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside

        onOpened: indicatorDrawerLoader.source = "qrc:/qml/QGroundControl/Toolbar/SelectViewDropdown.qml"
        onClosed: indicatorDrawerLoader.source = ""

        background: Rectangle {
            color:  "#212529"
            radius: 8
        }

        contentItem: Loader {
            id:         indicatorDrawerLoader
            objectName: "indicatorDrawerLoader"
        }
    }
}
