// Stand-in for QGC's FlyView on the RC screen (1920x1080): a fake video, the
// spots where QGC draws its own widgets (grey boxes), and our real camera
// layer loaded from the custom resources, built the way FlyView.qml builds it.

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

    Item {
        id:           mapHolder
        anchors.fill: parent

        // Fake camera picture: bright sky over darker ground, the hard case for white icons.
        Rectangle {
            anchors.fill: parent
            gradient: Gradient {
                GradientStop { position: 0.0;  color: "#dfeaf4" }
                GradientStop { position: 0.45; color: "#b9cfe0" }
                GradientStop { position: 0.46; color: "#7d8a6a" }
                GradientStop { position: 1.0;  color: "#4f5a40" }
            }
            Rectangle { x: parent.width * 0.55; y: parent.height * 0.3; width: 260; height: 200; color: "#c9c2b5" }
            Rectangle { x: parent.width * 0.18; y: parent.height * 0.36; width: 140; height: 120; color: "#efefef" }
        }

        // QGC's widget layer: tool strip top left, instruments bottom right.
        Item {
            id:                widgetLayer
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
            x: _margin; y: parent.height - height - _margin
            width: 430; height: 242; z: 100; color: "#6b8e5a"; border.color: "white"
            Text { anchors.centerIn: parent; color: "white"; text: "Map"; font.pixelSize: 30 }
        }
    }

    Rectangle {
        id: toolbar
        width: parent.width; height: 92; color: "#e0000000"
        Text { anchors.centerIn: parent; color: "white"; text: "QGC toolbar"; font.pixelSize: 30 }
    }

    QtObject {
        id: fakeMap
        property QtObject pipState: QtObject {
            property string state:              "pip"
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
    }
}
