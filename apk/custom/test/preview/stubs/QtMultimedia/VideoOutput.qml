// Stand-in for Qt Multimedia's VideoOutput (the preview's Qt has no Qt
// Multimedia). It plays the camera: the day picture, or while QGC's video
// address is the thermal address, a grey (white hot) thermal picture of 5:4,
// like the 640 x 512 thermal stream, with black bars beside it. contentRect
// says where the picture is, as in the real VideoOutput.

import QtQuick

import QGroundControl

Item {
    id: root

    // Qt's values.
    enum FillMode { Stretch, PreserveAspectFit, PreserveAspectCrop }

    property int fillMode: VideoOutput.PreserveAspectFit

    readonly property bool _thermal:   QGroundControl.settingsManager.videoSettings.rtspUrl.rawValue === SkydroidLink.thermalVideoUrl
    readonly property rect contentRect: _thermal ? Qt.rect(thermalPicture.x, thermalPicture.y, thermalPicture.width, thermalPicture.height)
                                                 : Qt.rect(0, 0, width, height)

    // Day: bright sky over darker ground, the hard case for white icons.
    Rectangle {
        anchors.fill: parent
        visible:      !root._thermal
        gradient: Gradient {
            GradientStop { position: 0.0;  color: "#dfeaf4" }
            GradientStop { position: 0.45; color: "#b9cfe0" }
            GradientStop { position: 0.46; color: "#7d8a6a" }
            GradientStop { position: 1.0;  color: "#4f5a40" }
        }
        Rectangle { x: parent.width * 0.55; y: parent.height * 0.3; width: 260; height: 200; color: "#c9c2b5" }
        Rectangle { x: parent.width * 0.18; y: parent.height * 0.36; width: 140; height: 120; color: "#efefef" }
    }

    // Thermal: black bars, and the picture in the middle.
    Rectangle {
        anchors.fill: parent
        visible:      root._thermal
        color:        "black"

        Item {
            id:               thermalPicture
            objectName:       "previewThermalPicture"
            height:           parent.height
            width:            Math.min(parent.width, height * 1.25)
            anchors.centerIn: parent

            // Top half: 64 grey levels, coldest on the left (the preview's colour check reads them).
            Row {
                width:  parent.width
                height: parent.height / 2

                Repeater {
                    model: 64

                    Rectangle {
                        required property int index
                        readonly property real level: Math.round(index * 255 / 63) / 255

                        width:  thermalPicture.width / 64
                        height: parent.height
                        color:  Qt.rgba(level, level, level, 1)
                    }
                }
            }

            // Bottom half: a cool scene with a building, a warm car with its hot engine, and a person.
            Rectangle {
                y:      parent.height / 2
                width:  parent.width
                height: parent.height / 2
                color:  "#323232"

                Rectangle { x: parent.width * 0.08; y: parent.height * 0.2;  width: parent.width * 0.3;  height: parent.height * 0.6;  color: "#6e6e6e" }
                Rectangle { x: parent.width * 0.5;  y: parent.height * 0.55; width: parent.width * 0.28; height: parent.height * 0.25; radius: 12; color: "#aaaaaa" }
                Rectangle { x: parent.width * 0.52; y: parent.height * 0.6;  width: parent.width * 0.07; height: parent.height * 0.12; radius: 6;  color: "#f5f5f5" }
                Rectangle { x: parent.width * 0.85; y: parent.height * 0.4;  width: parent.width * 0.04; height: parent.height * 0.4;  radius: 10; color: "#c8c8c8" }
            }
        }
    }
}
