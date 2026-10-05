// VAMA GCS fly view overlay: Skydroid camera controls, laser range and the
// target position. Replaces QGC's empty FlyViewCustomLayer.qml (see custom.qrc).

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QtLocation
import QtPositioning

import QGroundControl
import QGroundControl.Controls
import QGroundControl.FlyView
import QGroundControl.FlightMap

Item {
    id: _root

    property var parentToolInsets
    property var totalToolInsets: _toolInsets
    property var mapControl

    property real _margin: ScreenTools.defaultFontPixelWidth
    property real _padButton: ScreenTools.defaultFontPixelHeight * 2.2
    property var _link: SkydroidLink

    QGCPalette { id: qgcPal; colorGroupEnabled: true }

    // Our panel sits on the right edge; tell QGC so its own tools keep clear.
    QGCToolInsets {
        id:                     _toolInsets
        leftEdgeTopInset:       parentToolInsets.leftEdgeTopInset
        leftEdgeCenterInset:    parentToolInsets.leftEdgeCenterInset
        leftEdgeBottomInset:    parentToolInsets.leftEdgeBottomInset
        rightEdgeTopInset:      parentToolInsets.rightEdgeTopInset
        rightEdgeCenterInset:   cameraPanel.visible ? cameraPanel.width + _margin : parentToolInsets.rightEdgeCenterInset
        rightEdgeBottomInset:   parentToolInsets.rightEdgeBottomInset
        topEdgeLeftInset:       parentToolInsets.topEdgeLeftInset
        topEdgeCenterInset:     parentToolInsets.topEdgeCenterInset
        topEdgeRightInset:      parentToolInsets.topEdgeRightInset
        bottomEdgeLeftInset:    parentToolInsets.bottomEdgeLeftInset
        bottomEdgeCenterInset:  parentToolInsets.bottomEdgeCenterInset
        bottomEdgeRightInset:   parentToolInsets.bottomEdgeRightInset
    }

    function _fmt(value, decimals) {
        return Number(value).toFixed(decimals)
    }

    // --- Camera panel --------------------------------------------------------
    Rectangle {
        id:                     cameraPanel
        anchors.right:          parent.right
        anchors.rightMargin:    _margin + parentToolInsets.rightEdgeCenterInset
        anchors.verticalCenter: parent.verticalCenter
        width:                  panelColumn.width + _margin * 2
        height:                 panelColumn.height + _margin * 2
        radius:                 ScreenTools.defaultBorderRadius
        color:                  Qt.rgba(qgcPal.window.r, qgcPal.window.g, qgcPal.window.b, 0.85)

        ColumnLayout {
            id:                 panelColumn
            anchors.centerIn:   parent
            spacing:            _margin / 2

            // Header: status light, title, settings
            RowLayout {
                Layout.fillWidth: true
                spacing:          _margin / 2

                Rectangle {
                    width:  ScreenTools.defaultFontPixelHeight * 0.6
                    height: width
                    radius: width / 2
                    color:  !_link.enabled ? qgcPal.colorGrey : (_link.answering ? qgcPal.colorGreen : qgcPal.colorRed)
                }
                QGCLabel {
                    Layout.fillWidth: true
                    text: !_link.enabled ? qsTr("Camera off")
                                         : (_link.answering ? _link.model : qsTr("No reply"))
                }
                QGCButton {
                    text:      qsTr("Set")
                    onClicked: settingsPopup.open()
                }
            }

            QGCButton {
                Layout.fillWidth: true
                visible:   !_link.enabled
                text:      qsTr("Turn camera on")
                onClicked: _link.enabled = true
            }

            // Gimbal pad: hold to move, release to stop
            GridLayout {
                Layout.alignment: Qt.AlignHCenter
                visible:          _link.enabled
                columns:          3
                rowSpacing:       _margin / 3
                columnSpacing:    _margin / 3

                Item { width: _padButton; height: _padButton }
                QGCButton {
                    Layout.preferredWidth: _padButton; Layout.preferredHeight: _padButton
                    text: "▲"
                    onPressed:  _link.ptz("up")
                    onReleased: _link.stopGimbal()
                    onCanceled: _link.stopGimbal()
                }
                Item { width: _padButton; height: _padButton }

                QGCButton {
                    Layout.preferredWidth: _padButton; Layout.preferredHeight: _padButton
                    text: "◀"
                    onPressed:  _link.ptz("left")
                    onReleased: _link.stopGimbal()
                    onCanceled: _link.stopGimbal()
                }
                QGCButton {
                    Layout.preferredWidth: _padButton; Layout.preferredHeight: _padButton
                    text: "◎"
                    onClicked: _link.center()
                }
                QGCButton {
                    Layout.preferredWidth: _padButton; Layout.preferredHeight: _padButton
                    text: "▶"
                    onPressed:  _link.ptz("right")
                    onReleased: _link.stopGimbal()
                    onCanceled: _link.stopGimbal()
                }

                Item { width: _padButton; height: _padButton }
                QGCButton {
                    Layout.preferredWidth: _padButton; Layout.preferredHeight: _padButton
                    text: "▼"
                    onPressed:  _link.ptz("down")
                    onReleased: _link.stopGimbal()
                    onCanceled: _link.stopGimbal()
                }
                Item { width: _padButton; height: _padButton }
            }

            QGCLabel {
                Layout.alignment: Qt.AlignHCenter
                visible: _link.enabled
                text: _link.attitudeValid
                      ? qsTr("Yaw %1°  Pitch %2°").arg(_fmt(_link.gimbalYaw, 1)).arg(_fmt(_link.gimbalPitch, 1))
                      : qsTr("No gimbal angles")
                font.pointSize: ScreenTools.smallFontPointSize
            }

            // Zoom
            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                visible: _link.enabled
                QGCButton { text: "−"; onClicked: _link.zoom(-1) }
                QGCButton { text: qsTr("1x"); onClicked: _link.zoomHome() }
                QGCButton { text: "+"; onClicked: _link.zoom(1) }
            }
            QGCLabel {
                Layout.alignment: Qt.AlignHCenter
                visible: _link.enabled && _link.zoomStep >= 0
                text: qsTr("Zoom step %1").arg(_link.zoomStep)
                font.pointSize: ScreenTools.smallFontPointSize
            }

            // Photo and record
            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                visible: _link.enabled
                QGCButton { text: qsTr("Photo"); onClicked: _link.takePhoto() }
                QGCButton {
                    text:      _link.recording ? qsTr("Stop rec") : qsTr("Record")
                    onClicked: _link.toggleRecord()
                }
            }

            // Laser and target
            QGCButton {
                Layout.fillWidth: true
                visible:   _link.enabled
                enabled:   !_link.laserBusy
                primary:   true
                text:      _link.laserBusy ? qsTr("Measuring...") : qsTr("Laser")
                onClicked: _link.fireLaser()
            }
            QGCLabel {
                Layout.maximumWidth: _padButton * 3.5
                visible:  _link.enabled && (_link.laserValid || _link.laserMessage !== "")
                wrapMode: Text.WordWrap
                text: _link.laserValid ? qsTr("Distance %1 m").arg(_fmt(_link.laserRangeM, 1)) : _link.laserMessage
            }
            QGCLabel {
                Layout.maximumWidth: _padButton * 3.5
                visible:  _link.enabled && _link.targetValid
                wrapMode: Text.WordWrap
                text: qsTr("Target %1, %2").arg(_fmt(_link.targetLat, 6)).arg(_fmt(_link.targetLon, 6))
                      + (_link.targetHasAlt ? qsTr("\nHeight %1 m MSL").arg(_fmt(_link.targetAltMsl, 0)) : "")
            }
            QGCLabel {
                Layout.maximumWidth: _padButton * 3.5
                visible:  _link.enabled && _link.targetMessage !== ""
                wrapMode: Text.WordWrap
                color:    qgcPal.warningText
                text:     _link.targetMessage
                font.pointSize: ScreenTools.smallFontPointSize
            }
        }
    }

    // --- Camera settings -----------------------------------------------------
    Popup {
        id:          settingsPopup
        anchors.centerIn: parent
        modal:       true
        focus:       true
        padding:     _margin

        background: Rectangle {
            color:  qgcPal.window
            radius: ScreenTools.defaultBorderRadius
            border.color: qgcPal.text
        }

        ColumnLayout {
            spacing: _margin

            QGCLabel { text: qsTr("Camera settings"); font.bold: true }

            QGCCheckBox {
                text:      qsTr("Camera link on")
                checked:   _link.enabled
                onClicked: _link.enabled = checked
            }

            QGCLabel { text: qsTr("Camera model") }
            QGCComboBox {
                Layout.fillWidth: true
                model:        _link.models
                currentIndex: _link.models.indexOf(_link.model)
                onActivated: (index) => { _link.model = _link.models[index] }
            }

            QGCLabel { text: qsTr("Camera IP address") }
            QGCTextField {
                id:               hostField
                Layout.fillWidth: true
                text:             _link.host
                onEditingFinished: _link.host = text
            }

            QGCLabel { text: qsTr("Control port (usually 5000)") }
            QGCTextField {
                id:               portField
                Layout.fillWidth: true
                text:             _link.port
                inputMethodHints: Qt.ImhDigitsOnly
                onEditingFinished: _link.port = parseInt(text)
            }

            QGCLabel {
                Layout.maximumWidth: ScreenTools.defaultFontPixelWidth * 40
                wrapMode: Text.WordWrap
                font.pointSize: ScreenTools.smallFontPointSize
                text: qsTr("Video: set the camera RTSP URL in Application Settings, Video.")
            }

            QGCButton {
                Layout.alignment: Qt.AlignRight
                text:      qsTr("Close")
                onClicked: {
                    _link.host = hostField.text
                    _link.port = parseInt(portField.text)
                    settingsPopup.close()
                }
            }
        }
    }

    // --- Target on the map -------------------------------------------------------
    MapQuickItem {
        id:            targetMarker
        visible:       _link.targetValid
        coordinate:    QtPositioning.coordinate(_link.targetLat, _link.targetLon)
        anchorPoint.x: sourceItem.width / 2
        anchorPoint.y: sourceItem.height / 2

        sourceItem: Item {
            width:  ScreenTools.defaultFontPixelHeight * 2
            height: width

            Rectangle {
                anchors.fill: parent
                radius:       width / 2
                color:        "transparent"
                border.color: "red"
                border.width: 2
            }
            Rectangle { anchors.centerIn: parent; width: parent.width; height: 2; color: "red" }
            Rectangle { anchors.centerIn: parent; width: 2; height: parent.height; color: "red" }
        }
    }

    Component.onCompleted: {
        if (mapControl) {
            mapControl.addMapItem(targetMarker)
        }
    }
}
