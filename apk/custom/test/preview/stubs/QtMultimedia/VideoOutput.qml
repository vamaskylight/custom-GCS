// Stand-in for Qt Multimedia's VideoOutput (the preview's Qt has no Qt
// Multimedia). It plays the camera: the day picture, or while QGC's video
// address is the thermal address, a grey (white hot) thermal picture of 5:4,
// like the 640 x 512 thermal stream, with black bars beside it. contentRect
// says where the picture is, as in the real VideoOutput.
//
// The day picture is a look into a world that stands still: sky, ground, a few
// buildings and a walker. Where the fake camera looks (previewCamera, set by
// main.cpp) decides which part of it shows, so the picture moves when the
// gimbal turns, as the real one does. The object lock is tried on this: the
// app has to find the walker in the picture and turn the fake camera after it.

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

    // Where the camera looks. Its yaw counts to the left, as the C13 reports it.
    readonly property var  _camera:    (typeof previewCamera !== "undefined") ? previewCamera : null
    readonly property real _lookRight: _camera ? -_camera.yaw : 0
    readonly property real _lookUp:    _camera ? _camera.pitch : 0
    // The C13's view, as the app counts it: 83.4 degrees across, 46.9 up and down.
    readonly property real _perDegX:   width / 83.4
    readonly property real _perDegY:   height / 46.9

    // Day: a world of 240 by 120 degrees. Its point (0, 0) is straight ahead and level.
    Item {
        id:      day
        visible: !root._thermal
        anchors.fill: parent
        clip:    true

        Rectangle { anchors.fill: parent; color: "#4f5a40" }

        Item {
            id: world
            objectName: "previewWorld"
            x: root.width / 2 - root._lookRight * root._perDegX
            y: root.height / 2 + root._lookUp * root._perDegY

            // Sky above the level line, ground below.
            Rectangle {
                x: -120 * root._perDegX; width: 240 * root._perDegX
                y: -70 * root._perDegY;  height: 70 * root._perDegY
                gradient: Gradient {
                    GradientStop { position: 0.0; color: "#c5d9ea" }
                    GradientStop { position: 1.0; color: "#e4edf4" }
                }
            }
            Rectangle {
                x: -120 * root._perDegX; width: 240 * root._perDegX
                y: 0;                    height: 70 * root._perDegY
                gradient: Gradient {
                    GradientStop { position: 0.0; color: "#8a9673" }
                    GradientStop { position: 1.0; color: "#4f5a40" }
                }
            }

            // Fields and roofs, so the ground is not one plain colour: a patch every 6 degrees.
            Repeater {
                model: 38 * 16

                Rectangle {
                    required property int index
                    readonly property int column: index % 38
                    readonly property int row:    Math.floor(index / 38)
                    // The same patchwork on every run.
                    readonly property int seed:   (column * 7919 + row * 104729 + column * row * 31) % 97
                    // How far below the level line its top is, in degrees.
                    readonly property real below: 1 + row * 5 + (seed % 7) * 0.3

                    x:      (-114 + column * 6 + (seed % 5) * 0.4) * root._perDegX
                    y:      below * root._perDegY
                    width:  (2 + seed % 4) * root._perDegX
                    height: (1.5 + seed % 3) * root._perDegY
                    color:  Qt.rgba(0.30 + (seed % 9) * 0.05, 0.34 + (seed % 7) * 0.05, 0.25 + (seed % 5) * 0.05, 1)
                }
            }

            // Buildings: a light one and a grey one near where the preview starts
            // (looking 12 degrees right and 31 down), and two near the level line.
            Rectangle { x: (12.4 + 8) * root._perDegX;  y: (31.5 - 6) * root._perDegY; width: 11 * root._perDegX; height: 9 * root._perDegY; color: "#c9c2b5" }
            Rectangle {
                objectName: "previewLightBuilding"
                x: (12.4 - 24) * root._perDegX; y: (31.5 - 3) * root._perDegY; width: 6 * root._perDegX;  height: 5 * root._perDegY; color: "#efefef"
            }
            Rectangle { x: (12.4 - 30) * root._perDegX; y: -1 * root._perDegY; width: 7 * root._perDegX; height: 5 * root._perDegY; color: "#d8d2c4" }
            Rectangle { x: (12.4 + 22) * root._perDegX; y: 2 * root._perDegY;  width: 9 * root._perDegX; height: 4 * root._perDegY; color: "#9a948a" }

            // The walker: 3 degrees wide and 9 high, head, shirt and trousers.
            Item {
                id:         walker
                objectName: "previewWalker"
                visible:    root._camera ? root._camera.walkerVisible : false
                width:      3 * root._perDegX
                height:     9 * root._perDegY
                x:          (root._camera ? root._camera.walkerRight : 0) * root._perDegX - width / 2
                y:          -(root._camera ? root._camera.walkerUp : 0) * root._perDegY - height / 2

                Rectangle { x: parent.width * 0.25; y: 0;                    width: parent.width * 0.5; height: parent.height * 0.18; radius: width / 2; color: "#5b4636" }
                Rectangle { x: 0;                   y: parent.height * 0.2;  width: parent.width;       height: parent.height * 0.4;  color: "#d9e6f5" }
                Rectangle { x: parent.width * 0.1;  y: parent.height * 0.6;  width: parent.width * 0.8; height: parent.height * 0.4;  color: "#27324a" }
                Rectangle { x: parent.width * 0.3;  y: parent.height * 0.3;  width: parent.width * 0.4; height: parent.height * 0.12; color: "#8a1c1c" }
            }
        }
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
