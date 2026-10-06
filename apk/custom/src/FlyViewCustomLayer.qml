// VAMA GCS fly view overlay: Skydroid camera controls, laser range and the
// target position. Replaces QGC's empty FlyViewCustomLayer.qml (see custom.qrc).
//
// Layout follows the Skydroid app the client uses (photo, 2026-10-06): the
// video stays clear, with round icon buttons on both sides.
//  - Left: photo, record, laser.
//  - Right: yaw centre, gimbal centre, look down, zoom in, zoom out, zoom 1x.
//  - Top right: camera status and settings.
// The camera moves when a finger drags on the video (further = faster), or
// with an RC wheel set up in the camera settings.

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

    property var   _link:          SkydroidLink
    property real  _margin:        ScreenTools.defaultFontPixelWidth * 0.75
    property real  _buttonSize:    ScreenTools.defaultFontPixelHeight * 2.4
    property color _panelColor:    Qt.rgba(0, 0, 0, 0.4)
    property bool  _videoIsMain:   mapControl ? mapControl.pipState.state !== mapControl.pipState.fullState : false
    property bool  _laserBoxHidden: false
    property bool  _hintShown:     false
    property string _detectResult: ""

    readonly property string _iconPath: "/custom/img/"

    QGCPalette { id: qgcPal; colorGroupEnabled: true }

    // Our two button columns sit on the left and right edges; tell QGC so its
    // map tools keep clear of them.
    QGCToolInsets {
        id:                     _toolInsets
        leftEdgeTopInset:       parentToolInsets.leftEdgeTopInset
        leftEdgeCenterInset:    actionColumn.visible ? Math.max(parentToolInsets.leftEdgeCenterInset, actionColumn.x + actionColumn.width + _margin)
                                                     : parentToolInsets.leftEdgeCenterInset
        leftEdgeBottomInset:    parentToolInsets.leftEdgeBottomInset
        rightEdgeTopInset:      parentToolInsets.rightEdgeTopInset
        rightEdgeCenterInset:   gimbalPanel.visible ? Math.max(parentToolInsets.rightEdgeCenterInset, _root.width - gimbalPanel.x + _margin)
                                                    : parentToolInsets.rightEdgeCenterInset
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

    // Round icon button. "repeat" keeps clicking while held (zoom).
    component IconButton: Item {
        id: button

        property url   icon
        property color fill:   Qt.rgba(0, 0, 0, 0.55)
        property bool  repeat: false
        property real  size:   ScreenTools.defaultFontPixelHeight * 2.4

        signal clicked()

        implicitWidth:  size
        implicitHeight: size
        width:          size
        height:         size
        opacity:        enabled ? 1 : 0.45

        function flash() { flashAnimation.restart() }

        Rectangle {
            anchors.fill: parent
            radius:       width / 2
            color:        mouseArea.pressed ? Qt.rgba(1, 1, 1, 0.35) : button.fill
            border.color: Qt.rgba(1, 1, 1, 0.3)
            border.width: 1
        }
        Rectangle {
            id:           flashCircle
            anchors.fill: parent
            radius:       width / 2
            color:        "white"
            opacity:      0
            NumberAnimation on opacity { id: flashAnimation; running: false; from: 0.8; to: 0; duration: 400 }
        }
        QGCColoredImage {
            anchors.centerIn: parent
            width:            button.size * 0.52
            height:           width
            source:           button.icon
            color:            "white"
        }
        MouseArea {
            id:           mouseArea
            anchors.fill: parent
            onClicked:    { if (!button.repeat) button.clicked() }
            onPressed:    { if (button.repeat) { button.clicked(); repeatTimer.interval = 400; repeatTimer.start() } }
            onReleased:   repeatTimer.stop()
            onCanceled:   repeatTimer.stop()
        }
        Timer {
            id:          repeatTimer
            repeat:      true
            // Held: about 8 clicks a second (one C13 zoom step is small).
            // Stops by itself if the release was lost.
            onTriggered: {
                if (!mouseArea.pressed) {
                    stop()
                    return
                }
                interval = 120
                button.clicked()
            }
        }
    }

    // --- Status and settings, top right -----------------------------------------
    Row {
        id:                  topRightRow
        anchors.top:         parent.top
        anchors.right:       parent.right
        anchors.rightMargin: _margin
        spacing:             _margin

        Rectangle {
            id:                     statusChip
            anchors.verticalCenter: parent.verticalCenter
            width:                  statusRow.width + _margin * 3
            height:                 Math.max(_buttonSize * 0.7, statusRow.height + _margin)
            radius:                 height / 2
            color:                  _panelColor

            Row {
                id:               statusRow
                anchors.centerIn: parent
                spacing:          _margin

                Rectangle {
                    anchors.verticalCenter: parent.verticalCenter
                    width:  ScreenTools.defaultFontPixelHeight * 0.55
                    height: width
                    radius: width / 2
                    color:  !_link.enabled ? qgcPal.colorGrey
                                           : (!_link.answering ? qgcPal.colorRed
                                                               : (_link.attitudeValid ? qgcPal.colorGreen : qgcPal.colorOrange))
                }
                Column {
                    anchors.verticalCenter: parent.verticalCenter

                    QGCLabel {
                        color: "white"
                        text: {
                            if (!_link.enabled) {
                                return qsTr("Camera off")
                            }
                            if (!_link.answering) {
                                return qsTr("No camera reply")
                            }
                            var t = _link.modelName
                            if (!_link.attitudeValid) {
                                t += qsTr(", no angles")
                            }
                            return t
                        }
                    }
                    QGCLabel {
                        visible:        _link.enabled && _link.attitudeValid
                        color:          "white"
                        font.pointSize: ScreenTools.smallFontPointSize
                        text:           qsTr("Yaw %1°  Pitch %2°").arg(_fmt(_link.gimbalYaw, 1)).arg(_fmt(_link.gimbalPitch, 1))
                    }
                }
                QGCLabel {
                    anchors.verticalCenter: parent.verticalCenter
                    visible:   _link.enabled && _link.recording
                    color:     qgcPal.colorRed
                    font.bold: true
                    text:      qsTr("REC")
                }
            }

            MouseArea {
                anchors.fill: parent
                onClicked:    settingsPopup.open()
            }
        }

        IconButton {
            icon:      _iconPath + "vama_settings.svg"
            size:      _buttonSize * 0.85
            onClicked: settingsPopup.open()
        }
    }

    // --- Gimbal and zoom, right edge ----------------------------------------------
    Rectangle {
        id:                  gimbalPanel
        visible:             _link.enabled
        anchors.right:       parent.right
        anchors.rightMargin: _margin
        y:                   Math.max(topRightRow.y + topRightRow.height + _margin,
                                      (parent.height - parentToolInsets.bottomEdgeRightInset - height) / 2)
        width:               panelColumn.width + _margin * 2
        height:              panelColumn.height + _margin * 2
        radius:              _buttonSize / 2 + _margin
        color:               _panelColor

        // Taps between the buttons stop here instead of reaching the video.
        MouseArea { anchors.fill: parent }

        Column {
            id:               panelColumn
            anchors.centerIn: parent
            spacing:          _margin / 2

            GridLayout {
                id:            panelGrid
                columns:       2
                rowSpacing:    _margin
                columnSpacing: _margin

                IconButton { icon: _iconPath + "vama_yaw_center.svg"; size: _buttonSize; onClicked: _link.centerYaw() }
                IconButton { icon: _iconPath + "vama_zoom_in.svg";    size: _buttonSize; repeat: true; onClicked: _link.zoom(1) }
                IconButton { icon: _iconPath + "vama_center.svg";     size: _buttonSize; onClicked: _link.center() }
                IconButton { icon: _iconPath + "vama_zoom_out.svg";   size: _buttonSize; repeat: true; onClicked: _link.zoom(-1) }
                IconButton { icon: _iconPath + "vama_down.svg";       size: _buttonSize; onClicked: _link.pointDown() }
                IconButton { icon: _iconPath + "vama_zoom_reset.svg"; size: _buttonSize; onClicked: _link.zoomHome() }
            }

            // The zoom now, under the zoom column.
            QGCLabel {
                x:              panelGrid.width - _buttonSize + (_buttonSize - width) / 2
                visible:        text !== ""
                color:          "white"
                font.bold:      true
                text:           _link.zoomLabel
            }
        }
    }

    // --- Photo, record and laser, left edge ----------------------------------------
    Rectangle {
        id:      actionColumn
        visible: _link.enabled
        // Beside QGC's tool strip when it reaches this far down, else at the edge.
        y:       Math.max(_margin, (parent.height - parentToolInsets.bottomEdgeLeftInset - height) / 2)
        x:       y < parentToolInsets.topEdgeLeftInset ? parentToolInsets.leftEdgeTopInset + _margin : _margin
        width:   actionButtons.width + _margin * 2
        height:  actionButtons.height + _margin * 2
        radius:  _buttonSize / 2 + _margin
        color:   _panelColor

        // Taps between the buttons stop here instead of reaching the video.
        MouseArea { anchors.fill: parent }

        Column {
            id:               actionButtons
            anchors.centerIn: parent
            spacing:          _margin

            IconButton {
                id:        photoButton
                icon:      _iconPath + "vama_photo.svg"
                size:      _buttonSize
                onClicked: { _link.takePhoto(); photoButton.flash() }
            }
            IconButton {
                icon:      _iconPath + "vama_record.svg"
                size:      _buttonSize
                fill:      _link.recording ? Qt.rgba(0.85, 0.12, 0.12, 0.9) : Qt.rgba(0, 0, 0, 0.55)
                onClicked: _link.toggleRecord()
            }
            IconButton {
                icon:      _iconPath + "vama_laser.svg"
                size:      _buttonSize
                enabled:   !_link.laserBusy
                fill:      _link.laserBusy ? qgcPal.colorOrange : Qt.rgba(0, 0, 0, 0.55)
                onClicked: _link.fireLaser()
            }
        }
    }

    // --- Laser result, top centre ---------------------------------------------------
    Rectangle {
        id:                       laserBox
        visible:                  _link.enabled && !_laserBoxHidden &&
                                  (_link.laserBusy || _link.laserValid || _link.laserMessage !== "")
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top:              parent.top
        anchors.topMargin:        parentToolInsets.topEdgeCenterInset + _margin
        width:                    laserColumn.width + _margin * 3
        height:                   laserColumn.height + _margin * 2
        radius:                   _margin
        color:                    _panelColor

        Column {
            id:               laserColumn
            anchors.centerIn: parent
            width:            Math.min(ScreenTools.defaultFontPixelWidth * 34, _root.width * 0.45)
            spacing:          _margin / 3

            QGCLabel {
                width:               parent.width
                horizontalAlignment: Text.AlignHCenter
                color:               "white"
                font.pointSize:      ScreenTools.largeFontPointSize
                font.bold:           true
                text:                _link.laserBusy ? qsTr("Measuring...")
                                                     : (_link.laserValid ? qsTr("%1 m").arg(_fmt(_link.laserRangeM, 1)) : qsTr("No laser reading"))
            }
            QGCLabel {
                width:               parent.width
                visible:             !_link.laserBusy && !_link.laserValid && _link.laserMessage !== ""
                horizontalAlignment: Text.AlignHCenter
                wrapMode:            Text.WordWrap
                color:               qgcPal.colorOrange
                font.pointSize:      ScreenTools.smallFontPointSize
                text:                _link.laserMessage
            }
            QGCLabel {
                width:               parent.width
                visible:             !_link.laserBusy && _link.targetValid
                horizontalAlignment: Text.AlignHCenter
                color:               "white"
                text:                qsTr("Target %1, %2").arg(_fmt(_link.targetLat, 6)).arg(_fmt(_link.targetLon, 6))
            }
            QGCLabel {
                width:               parent.width
                visible:             !_link.laserBusy && _link.targetValid && _link.targetHasAlt
                horizontalAlignment: Text.AlignHCenter
                color:               "white"
                font.pointSize:      ScreenTools.smallFontPointSize
                text:                qsTr("Height %1 m above sea level, %2 m away on the ground")
                                         .arg(_fmt(_link.targetAltMsl, 0)).arg(_fmt(_link.targetHorizontalM, 0))
            }
            QGCLabel {
                width:               parent.width
                visible:             !_link.laserBusy && _link.laserValid && _link.targetMessage !== ""
                horizontalAlignment: Text.AlignHCenter
                wrapMode:            Text.WordWrap
                color:               qgcPal.colorOrange
                font.pointSize:      ScreenTools.smallFontPointSize
                text:                _link.targetMessage
            }
        }

        // Tap to hide; the next laser shot shows it again.
        MouseArea {
            anchors.fill: parent
            onClicked:    _laserBoxHidden = true
        }
    }

    // --- Drag on the video to move the camera ---------------------------------------
    // Lives in the fly view's map and video holder, just above the video and
    // below QGC's widgets and our buttons, like QGC's own video mouse area.
    MouseArea {
        id:              videoDrag
        parent:          _root.parent
        anchors.fill:    parent
        z:               1
        enabled:         _link.enabled && _videoIsMain
        visible:         enabled
        preventStealing: true

        property real _pressX:   0
        property real _pressY:   0
        property bool _dragging: false
        readonly property real _startDistance: ScreenTools.defaultFontPixelHeight * 0.6
        // Full speed when the finger is this far from where it went down.
        readonly property real _fullSpeedDistance: Math.min(width, height) * 0.2

        function _deflection() {
            var dx = (mouseX - _pressX) / _fullSpeedDistance
            var dy = (_pressY - mouseY) / _fullSpeedDistance
            return [Math.max(-1, Math.min(1, dx)), Math.max(-1, Math.min(1, dy))]
        }
        function endDrag() {
            touchRefresh.stop()
            if (_dragging) {
                _link.stopTouchMotion()
            }
            _dragging = false
        }

        onPressed: (mouse) => {
            _pressX = mouse.x
            _pressY = mouse.y
            _dragging = false
        }
        onPositionChanged: (mouse) => {
            if (!_dragging && (Math.abs(mouse.x - _pressX) > _startDistance || Math.abs(mouse.y - _pressY) > _startDistance)) {
                _dragging = true
                touchRefresh.start()
            }
            if (_dragging) {
                var d = _deflection()
                _link.setTouchMotion(d[0], d[1])
            }
        }
        onReleased:      endDrag()
        onCanceled:      endDrag()
        onEnabledChanged: if (!enabled) endDrag()
        onDoubleClicked: QGroundControl.videoManager.fullScreen = !QGroundControl.videoManager.fullScreen

        // The link stops on its own without updates, so keep sending while the finger is held still.
        Timer {
            id:          touchRefresh
            interval:    100
            repeat:      true
            onTriggered: {
                var d = videoDrag._deflection()
                _link.setTouchMotion(d[0], d[1])
            }
        }

        // Centre mark: the laser measures here.
        Item {
            anchors.centerIn: parent
            width:            ScreenTools.defaultFontPixelHeight * 2.2
            height:           width
            opacity:          0.85

            Rectangle { anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter; width: parent.width * 0.32; height: 2; color: "white" }
            Rectangle { anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter; width: parent.width * 0.32; height: 2; color: "white" }
            Rectangle { anchors.top: parent.top; anchors.horizontalCenter: parent.horizontalCenter; width: 2; height: parent.height * 0.32; color: "white" }
            Rectangle { anchors.bottom: parent.bottom; anchors.horizontalCenter: parent.horizontalCenter; width: 2; height: parent.height * 0.32; color: "white" }
            Rectangle { anchors.centerIn: parent; width: 4; height: 4; radius: 2; color: "white" }
        }

        // Where the finger went down, and how far it has moved.
        Rectangle {
            visible:      videoDrag._dragging
            x:            videoDrag._pressX - width / 2
            y:            videoDrag._pressY - height / 2
            width:        videoDrag._fullSpeedDistance * 2
            height:       width
            radius:       width / 2
            color:        Qt.rgba(0, 0, 0, 0.15)
            border.color: Qt.rgba(1, 1, 1, 0.6)
            border.width: 2
        }
        Rectangle {
            visible: videoDrag._dragging
            width:   ScreenTools.defaultFontPixelHeight * 1.6
            height:  width
            radius:  width / 2
            color:   Qt.rgba(1, 1, 1, 0.55)
            x:       videoDrag._pressX + Math.max(-1, Math.min(1, (videoDrag.mouseX - videoDrag._pressX) / videoDrag._fullSpeedDistance)) * videoDrag._fullSpeedDistance - width / 2
            y:       videoDrag._pressY + Math.max(-1, Math.min(1, (videoDrag.mouseY - videoDrag._pressY) / videoDrag._fullSpeedDistance)) * videoDrag._fullSpeedDistance - height / 2
        }
    }

    // One short hint the first time the video and the camera link are both on.
    Rectangle {
        id:                       dragHint
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom:           parent.bottom
        anchors.bottomMargin:     parentToolInsets.bottomEdgeCenterInset + _margin
        width:                    hintLabel.width + _margin * 3
        height:                   hintLabel.height + _margin * 1.5
        radius:                   height / 2
        color:                    _panelColor
        opacity:                  0
        visible:                  opacity > 0

        QGCLabel {
            id:               hintLabel
            anchors.centerIn: parent
            color:            "white"
            text:             qsTr("Drag on the video to move the camera")
        }

        SequentialAnimation {
            id: hintAnimation
            NumberAnimation { target: dragHint; property: "opacity"; to: 1; duration: 300 }
            PauseAnimation  { duration: 5000 }
            NumberAnimation { target: dragHint; property: "opacity"; to: 0; duration: 800 }
        }
    }

    property bool _dragAvailable: _link.enabled && _videoIsMain
    on_DragAvailableChanged: {
        if (_dragAvailable && !_hintShown) {
            _hintShown = true
            hintAnimation.restart()
        }
    }

    Connections {
        target: _link
        function onLaserBusyChanged() {
            if (_link.laserBusy) {
                _laserBoxHidden = false
            }
        }
        function onWheelDetected(axis, channel) {
            _detectResult = channel > 0 ? qsTr("Found the wheel on channel %1.").arg(channel)
                                        : qsTr("No wheel moved. Check that the drone is connected, then try again.")
        }
    }

    // Never leave the camera turning when the app goes to the background.
    Connections {
        target: Qt.application
        function onStateChanged() {
            if (Qt.application.state !== Qt.ApplicationActive) {
                videoDrag.endDrag()
            }
        }
    }

    // --- Camera settings -----------------------------------------------------
    Popup {
        id:               settingsPopup
        objectName:       "vamaCameraSettings"
        anchors.centerIn: parent
        modal:            true
        focus:            true
        padding:          _margin * 1.5
        width:            Math.min(_root.width * 0.92, ScreenTools.defaultFontPixelWidth * 64)
        height:           Math.min(_root.height * 0.96, settingsColumn.implicitHeight + padding * 2)

        onOpened: _detectResult = ""

        background: Rectangle {
            color:        qgcPal.window
            radius:       ScreenTools.defaultBorderRadius
            border.color: qgcPal.text
        }

        property var _channelNames: {
            var names = [qsTr("Off")]
            for (var i = 1; i <= 18; i++) {
                names.push(qsTr("Channel %1").arg(i))
            }
            return names
        }
        property var _speeds: [5, 10, 15, 20, 30, 45, 60]
        // One label column width, so the boxes line up across the sections.
        property real _labelWidth: ScreenTools.defaultFontPixelWidth * 14

        function _rcText(channel) {
            if (channel <= 0) {
                return ""
            }
            if (_link.rcChannels.length < channel) {
                return qsTr("No RC values from the drone yet.")
            }
            return qsTr("Channel %1 now: %2").arg(channel).arg(_link.rcChannels[channel - 1])
        }

        QGCFlickable {
            objectName:    "vamaCameraSettingsFlick"
            anchors.fill:  parent
            contentWidth:  width
            contentHeight: settingsColumn.implicitHeight
            clip:          true

            ColumnLayout {
                id:      settingsColumn
                width:   parent.width
                spacing: _margin

                QGCLabel { text: qsTr("Camera settings"); font.bold: true }

                QGCCheckBox {
                    text:      qsTr("Camera link on")
                    checked:   _link.enabled
                    onClicked: _link.enabled = checked
                }

                GridLayout {
                    Layout.fillWidth: true
                    columns:          2
                    columnSpacing:    _margin
                    rowSpacing:       _margin / 2

                    QGCLabel { text: qsTr("Camera model"); Layout.preferredWidth: settingsPopup._labelWidth }
                    QGCComboBox {
                        Layout.fillWidth: true
                        model:            _link.modelNames  // V12, V13, V14 Pro on screen
                        currentIndex:     _link.models.indexOf(_link.model)
                        onActivated:      (index) => { _link.model = _link.models[index] }
                    }

                    QGCLabel { text: qsTr("Camera IP address"); Layout.preferredWidth: settingsPopup._labelWidth }
                    QGCTextField {
                        id:                hostField
                        Layout.fillWidth:  true
                        text:              _link.host
                        onEditingFinished: _link.host = text
                    }

                    QGCLabel { text: qsTr("Control port"); Layout.preferredWidth: settingsPopup._labelWidth }
                    QGCTextField {
                        id:                portField
                        Layout.fillWidth:  true
                        text:              _link.port
                        inputMethodHints:  Qt.ImhDigitsOnly
                        onEditingFinished: _link.port = parseInt(text)
                    }
                }

                QGCLabel {
                    Layout.fillWidth: true
                    wrapMode:         Text.WordWrap
                    font.pointSize:   ScreenTools.smallFontPointSize
                    text: !_link.enabled ? qsTr("Usually 192.168.144.108 and port 5000.")
                                         : (_link.attitudeValid ? qsTr("The camera answers at %1.").arg(_link.activeEndpoint)
                                                                : qsTr("Waiting for the camera. The app also tries 192.168.144.12 and the other usual ports."))
                }

                // What the camera itself says about its zoom, to check the zoom number on a C13.
                QGCLabel {
                    Layout.fillWidth: true
                    visible:          _link.enabled && _link.zoomReport !== ""
                    wrapMode:         Text.WordWrap
                    font.pointSize:   ScreenTools.smallFontPointSize
                    text:             qsTr("Camera zoom report: %1").arg(_link.zoomReport)
                }

                Rectangle { Layout.fillWidth: true; height: 1; color: qgcPal.text; opacity: 0.3 }

                QGCLabel { text: qsTr("Moving the camera"); font.bold: true }
                QGCLabel {
                    Layout.fillWidth: true
                    wrapMode:         Text.WordWrap
                    font.pointSize:   ScreenTools.smallFontPointSize
                    text:             qsTr("Drag on the video: the further you drag, the faster it turns. Let go to stop.")
                }

                GridLayout {
                    Layout.fillWidth: true
                    columns:          2
                    columnSpacing:    _margin
                    rowSpacing:       _margin / 2

                    QGCLabel { text: qsTr("Top speed"); Layout.preferredWidth: settingsPopup._labelWidth }
                    QGCComboBox {
                        Layout.fillWidth: true
                        model:            settingsPopup._speeds.map(function(s) { return qsTr("%1° per second").arg(s) })
                        currentIndex:     {
                            var i = settingsPopup._speeds.indexOf(_link.maxSpeed)
                            return i >= 0 ? i : 3
                        }
                        onActivated:      (index) => { _link.maxSpeed = settingsPopup._speeds[index] }
                    }
                }

                QGCCheckBox {
                    text:      qsTr("Reverse left and right")
                    checked:   _link.reverseYaw
                    onClicked: _link.reverseYaw = checked
                }
                QGCCheckBox {
                    text:      qsTr("Reverse up and down")
                    checked:   _link.reversePitch
                    onClicked: _link.reversePitch = checked
                }

                Rectangle { Layout.fillWidth: true; height: 1; color: qgcPal.text; opacity: 0.3 }

                QGCLabel { text: qsTr("RC wheel"); font.bold: true }

                GridLayout {
                    Layout.fillWidth: true
                    columns:          3
                    columnSpacing:    _margin
                    rowSpacing:       _margin / 2

                    QGCLabel { text: qsTr("Up and down"); Layout.preferredWidth: settingsPopup._labelWidth }
                    QGCComboBox {
                        Layout.fillWidth: true
                        model:            settingsPopup._channelNames
                        currentIndex:     _link.wheelPitchChannel
                        onActivated:      (index) => { _link.wheelPitchChannel = index }
                    }
                    QGCButton {
                        text:      _link.detectingWheel === "pitch" ? qsTr("Turn it now...") : qsTr("Detect")
                        enabled:   _link.detectingWheel === ""
                        onClicked: { _detectResult = ""; _link.detectWheel("pitch") }
                    }

                    QGCLabel { text: qsTr("Left and right"); Layout.preferredWidth: settingsPopup._labelWidth }
                    QGCComboBox {
                        Layout.fillWidth: true
                        model:            settingsPopup._channelNames
                        currentIndex:     _link.wheelYawChannel
                        onActivated:      (index) => { _link.wheelYawChannel = index }
                    }
                    QGCButton {
                        text:      _link.detectingWheel === "yaw" ? qsTr("Turn it now...") : qsTr("Detect")
                        enabled:   _link.detectingWheel === ""
                        onClicked: { _detectResult = ""; _link.detectWheel("yaw") }
                    }

                    QGCLabel { text: qsTr("Wheel type"); Layout.preferredWidth: settingsPopup._labelWidth }
                    QGCComboBox {
                        Layout.fillWidth:  true
                        Layout.columnSpan: 2
                        model:             [qsTr("Springs back to the middle"), qsTr("Stays where you leave it")]
                        currentIndex:      _link.wheelHoldsPosition ? 1 : 0
                        onActivated:       (index) => { _link.wheelHoldsPosition = (index === 1) }
                    }
                }

                QGCCheckBox {
                    text:      qsTr("Reverse the wheels")
                    checked:   _link.wheelReverse
                    onClicked: _link.wheelReverse = checked
                }

                QGCLabel {
                    Layout.fillWidth: true
                    wrapMode:         Text.WordWrap
                    font.pointSize:   ScreenTools.smallFontPointSize
                    visible:          text !== ""
                    text: {
                        var lines = []
                        if (_link.detectingWheel !== "") {
                            lines.push(qsTr("Turn the wheel all the way, both ways."))
                        } else if (_detectResult !== "") {
                            lines.push(_detectResult)
                        }
                        var p = settingsPopup._rcText(_link.wheelPitchChannel)
                        if (p !== "") {
                            lines.push(p)
                        }
                        var y = settingsPopup._rcText(_link.wheelYawChannel)
                        if (y !== "" && _link.wheelYawChannel !== _link.wheelPitchChannel) {
                            lines.push(y)
                        }
                        return lines.join("\n")
                    }
                }

                Rectangle { Layout.fillWidth: true; height: 1; color: qgcPal.text; opacity: 0.3 }

                QGCLabel {
                    Layout.fillWidth: true
                    wrapMode:         Text.WordWrap
                    font.pointSize:   ScreenTools.smallFontPointSize
                    text:             qsTr("Video: set the camera RTSP URL in Application Settings, Video.")
                }

                QGCButton {
                    Layout.alignment: Qt.AlignRight
                    text:             qsTr("Close")
                    onClicked: {
                        _link.host = hostField.text
                        _link.port = parseInt(portField.text)
                        settingsPopup.close()
                    }
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
