// VAMA: replaces QGC 5.1.5's src/FlyView/ProximityRadarVideoView.qml (custom.qrc). It draws nothing.
//
// QGC's file drew each obstacle reading on the video at its distance from the middle, on a fixed 6 m
// scale: a farther reading moved up and left the picture (the client, 2026-10-30, B19: "make that
// stable at one position"). The app shows four bars at fixed places instead (FlyViewCustomLayer.qml).
// QGC's FlyViewVideo.qml still makes this item and gives it the vehicle.

import QtQuick

Item {
    objectName: "vamaNoVideoRadar"
    visible:    false

    property var  vehicle
    property real range: 6
}
