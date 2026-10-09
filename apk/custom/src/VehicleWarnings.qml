// VAMA copy of QGC 5.1.5's src/FlyView/VehicleWarnings.qml (the large "No GPS
// Lock for Vehicle" and pre-arm texts of the fly view). custom.qrc serves it in
// place of QGC's file. Compare it with QGC's file again after every QGC update.
// The changes are marked "VAMA".
//
// QGC puts these texts in the middle of the fly view. With the video as the
// main view that is the middle of the camera picture. In the field video of
// test build 6 (2026-10-09, indoors, no GPS) three large lines lay over the
// person the operator was drawing the lock box around, for the whole test.
// So while the video is the main view the same warnings show small, in one
// box at the bottom of the picture: the middle stays free. With the map as
// the main view nothing changes.

import QtQuick

import QGroundControl
import QGroundControl.Controls

Rectangle {
    id:                 warnings  // VAMA
    objectName:         "vamaVehicleWarnings"
    anchors.margins:    -ScreenTools.defaultFontPixelHeight
    height:             warningsCol.height
    width:              warningsCol.width
    color:              Qt.rgba(1, 1, 1, _small ? 0.6 : 0.5)
    radius:             ScreenTools.defaultFontPixelWidth / 2
    visible:            _noGPSLockVisible || _prearmErrorVisible

    property var  _activeVehicle:       QGroundControl.multiVehicleManager.activeVehicle
    property bool _noGPSLockVisible:    _activeVehicle && _activeVehicle.requiresGpsFix && !_activeVehicle.coordinate.isValid
    property bool _prearmErrorVisible:  _activeVehicle && !_activeVehicle.armed && _activeVehicle.prearmError && !_activeVehicle.healthAndArmingCheckReport.supported

    // VAMA: the video is the main view when the map is the small one (the same
    // test as _videoIsMain in FlyViewCustomLayer.qml). The parent is QGC's
    // widget layer, which knows the map.
    readonly property var  _mapControl: parent && parent.mapControl ? parent.mapControl : null
    readonly property bool _small:      _mapControl && _mapControl.pipState ? _mapControl.pipState.state !== _mapControl.pipState.fullState : false
    readonly property real _bottomInset: parent && parent.totalToolInsets ? parent.totalToolInsets.bottomEdgeCenterInset : 0
    readonly property real _fontSize:   _small ? ScreenTools.smallFontPointSize : ScreenTools.largeFontPointSize

    // VAMA: QGC centres this item in the fly view. Small, it is moved from
    // there down to just above QGC's bottom row.
    transform: Translate {
        y: warnings._small && warnings.parent
           ? Math.max(0, warnings.parent.height / 2 - warnings.height / 2 - warnings._bottomInset - ScreenTools.defaultFontPixelHeight * 1.5)
           : 0
    }

    Column {
        id:         warningsCol
        spacing:    warnings._small ? ScreenTools.defaultFontPixelHeight / 4 : ScreenTools.defaultFontPixelHeight  // VAMA
        padding:    warnings._small ? ScreenTools.defaultFontPixelHeight / 3 : 0                                    // VAMA

        QGCLabel {
            anchors.horizontalCenter:   parent.horizontalCenter
            visible:                    _noGPSLockVisible
            color:                      "black"
            font.pointSize:             warnings._fontSize  // VAMA
            text:                       qsTr("No GPS Lock for Vehicle")
        }

        QGCLabel {
            anchors.horizontalCenter:   parent.horizontalCenter
            visible:                    _prearmErrorVisible
            color:                      "black"
            font.pointSize:             warnings._fontSize  // VAMA
            text:                       _activeVehicle ? _activeVehicle.prearmError : ""
        }

        QGCLabel {
            objectName:                 "vamaVehicleWarningsAdvice"
            anchors.horizontalCenter:   parent.horizontalCenter
            // VAMA: small, the advice is one short line. The error itself is the line above.
            visible:                    _prearmErrorVisible
            width:                      warnings._small ? implicitWidth : ScreenTools.defaultFontPixelWidth * 50
            horizontalAlignment:        Text.AlignHCenter
            wrapMode:                   warnings._small ? Text.NoWrap : Text.WordWrap
            color:                      "black"
            font.pointSize:             warnings._small ? ScreenTools.smallFontPointSize : ScreenTools.largeFontPointSize
            text:                       warnings._small ? qsTr("Fix this before the vehicle can be armed.")
                                                        : qsTr("The vehicle has failed a pre-arm check. In order to arm the vehicle, resolve the failure.")
        }
    }
}
