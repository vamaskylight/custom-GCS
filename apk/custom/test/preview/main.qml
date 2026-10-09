// Stand-in for QGC's FlyView on the RC screen (1920x1080): the video, the
// spots where QGC draws its own widgets (grey boxes), and our real camera
// layer loaded from the custom resources, built the way FlyView.qml builds it.
// The video is the VAMA copy of QGC's FlightDisplayViewVideoOutput.qml, over a
// stand-in VideoOutput that plays a day or a thermal picture (stubs/QtMultimedia).

import QtQuick
import QtQuick.Window

import QGroundControl
import QGroundControl.Controls

Window {
    id:      window
    width:   1920
    height:  1080
    visible: true
    color:   "black"

    readonly property real _margin: ScreenTools.defaultFontPixelWidth * 0.75

    // True hides everything over the video, so a check can read its pixels.
    property bool videoOnly: false
    // True makes the map the main view (QGC's "full" state), as after a tap on the small map.
    property bool mapIsMain: false

    Item {
        id:           mapHolder
        anchors.fill: parent

        // QGC's own address: the app's override rule (main.cpp) answers it with
        // the VAMA copy. Without that copy nothing would load here.
        Loader {
            anchors.fill: parent
            source:       "qrc:/qml/QGroundControl/FlyView/FlightDisplayViewVideoOutput.qml"
        }

        // QGC's widget layer: tool strip top left, instruments bottom right.
        Item {
            id:                widgetLayer
            visible:           !window.videoOnly
            anchors.fill:      parent
            anchors.margins:   _margin
            anchors.topMargin: toolbar.height + _margin
            z:                 2

            Rectangle {
                id: toolStrip
                width: 130; height: 300; radius: 10; color: "#b0303030"
                Text { anchors.centerIn: parent; color: "white"; text: "QGC\ntools"; font.pixelSize: 26 }
            }
            Rectangle {
                id: instruments
                anchors.right: parent.right; anchors.bottom: parent.bottom
                width: 760; height: 170; radius: 10; color: "#b0303030"
                Text { anchors.centerIn: parent; color: "white"; text: "QGC telemetry and compass"; font.pixelSize: 26 }
            }

            // QGC's widget layer knows the map; its warning texts read it from here.
            property var mapControl: fakeMap

            property QtObject totalToolInsets: QGCToolInsets {
                leftEdgeTopInset:      toolStrip.width + _margin
                leftEdgeCenterInset:   toolStrip.width + _margin
                leftEdgeBottomInset:   pip.width + _margin
                topEdgeLeftInset:      toolStrip.height + _margin
                bottomEdgeLeftInset:   pip.height + _margin
                bottomEdgeCenterInset: instruments.height + _margin
                bottomEdgeRightInset:  instruments.height + _margin
                rightEdgeBottomInset:  instruments.width + _margin
            }
        }

        // The map, small in the corner while the video is the main view.
        Rectangle {
            id: pip
            visible: !window.videoOnly
            x: _margin; y: parent.height - height - _margin
            width: 430; height: 242; z: 100; color: "#6b8e5a"; border.color: "white"
            Text { anchors.centerIn: parent; color: "white"; text: "Map"; font.pixelSize: 30 }
        }
    }

    Rectangle {
        id: toolbar
        visible: !window.videoOnly
        width: parent.width; height: 92; color: "#e0000000"
        Text { anchors.centerIn: parent; color: "white"; text: "QGC toolbar"; font.pixelSize: 30 }
    }

    QtObject {
        id: fakeMap
        property QtObject pipState: QtObject {
            property string state:              window.mapIsMain ? "full" : "pip"
            readonly property string fullState: "full"
        }
        function addMapItem(item) { }
    }

    property Item layer: null

    Component.onCompleted: {
        var component = Qt.createComponent("qrc:/Custom/qml/QGroundControl/FlyView/FlyViewCustomLayer.qml")
        if (component.status !== Component.Ready) {
            console.error("LOAD ERROR: " + component.errorString())
            return
        }
        layer = component.createObject(mapHolder, { parentToolInsets: widgetLayer.totalToolInsets, mapControl: fakeMap })
        layer.anchors.fill = widgetLayer
        layer.z = 2
        layer.visible = Qt.binding(function() { return !window.videoOnly })

        // QGC's warning texts ("No GPS Lock for Vehicle", pre-arm errors), by QGC's
        // own address and placed as FlyViewWidgetLayer.qml places them: in the
        // middle of the widget layer. The app's override rule answers with the
        // VAMA copy (VehicleWarnings.qml).
        var warnings = Qt.createComponent("qrc:/qml/QGroundControl/FlyView/VehicleWarnings.qml")
        if (warnings.status !== Component.Ready) {
            console.error("LOAD ERROR: " + warnings.errorString())
            return
        }
        var warningsItem = warnings.createObject(widgetLayer, {})
        warningsItem.anchors.centerIn = widgetLayer
        warningsItem.z = QGroundControl.zOrderTopMost
    }
}
