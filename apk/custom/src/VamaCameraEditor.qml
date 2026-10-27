import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls

// VAMA (task AA, client 2026-10-16): the cameras the app knows, for the admin. The camera
// settings open this in Advanced Mode, so only after the admin password (task Z).
//  - A built-in camera (V12, V13, V14 Pro, V20, Viewpro A20KTR, SIYI ZR10) is copied, never changed.
//  - An added camera is a file in the app's folder "Cameras" (SkydroidLink.cameraFolder).
//    Copied into that folder on another remote control, it is in the list there too.
//  - A copy speaks the protocol of the camera it was copied from: Skydroid TOP, Viewpro or SIYI
//    (task AK, CameraProfile.h). The form shows what that protocol has.
// FlyViewCustomLayer.qml makes this window when it is first asked for.
Popup {
    id:          root
    objectName:  "vamaCameraEditor"
    modal:       true
    focus:       true
    closePolicy: Popup.CloseOnEscape
    padding:     ScreenTools.defaultFontPixelWidth * 1.5
    x:           parent ? Math.round((parent.width - width) / 2) : 0
    y:           parent ? Math.round((parent.height - height) / 2) : 0
    width:       Math.min(parent ? parent.width * 0.94 : 900, ScreenTools.defaultFontPixelWidth * 72)
    height:      Math.min(parent ? parent.height * 0.96 : 700, editorColumn.implicitHeight + padding * 2)

    property var    link:        SkydroidLink
    property var    _fields:     null  // the camera in the form, or null while the list shows
    property var    _lenses:     []    // its lenses (a camera that reports its zoom step)
    property string _problem:    ""
    property string _askDelete:  ""    // the id of the camera that Delete asks about
    property real   _labelWidth: ScreenTools.defaultFontPixelWidth * 20

    // The protocol of the camera in the form, and the zoom kinds it has (CameraProfile.h), in
    // the order of the zoom box.
    readonly property string _protocol: _fields && _fields.protocol ? _fields.protocol : "skydroid-top"
    readonly property bool   _top:      _protocol === "skydroid-top"
    readonly property var    _protocolZooms: ({ "skydroid-top": ["none", "counted", "reported", "pulse"],
                                                "viewpro":      ["none", "live"],
                                                "siyi":         ["none", "pulse"] })
    readonly property var    _zoomKinds: _protocolZooms[_protocol] || _protocolZooms["skydroid-top"]
    readonly property var    _zoomNames: ({ "none":     qsTr("No zoom"),
                                            "counted":  qsTr("Steps of 0.1x (as the V12 and V13)"),
                                            "reported": qsTr("The camera reports its zoom step (as the V14 Pro)"),
                                            "live":     qsTr("The camera says its zoom and its view (as the A20KTR)"),
                                            "pulse":    _top ? qsTr("Its lens zooms and does not say how far (as the V20)")
                                                             : qsTr("It zooms and does not say how far (as the ZR10)") })
    readonly property string _zoomKind:  _zoomKinds[zoomBox.currentIndex] || "none"
    readonly property bool   _thermalBySwitch: _protocol === "viewpro" && thermalBySwitchBox.checked
    // How the control commands reach the camera, in the order of the link box (the C20: TCP).
    // Only a TOP camera has the choice: Viewpro takes TCP, SIYI UDP.
    readonly property var _transportKinds: ["udp", "tcp"]

    onOpened: {
        link.reloadCameras()  // files copied in meanwhile
        _fields = null
        _problem = ""
        _askDelete = ""
    }

    // The form with a camera: an added camera's own fields, or a copy of any camera.
    function edit(id, asCopy) {
        var fields = link.cameraProfile(id)
        if (asCopy) {
            fields.id = ""
            fields.name = qsTr("%1 copy").arg(fields.name)
        }
        _problem = ""
        _askDelete = ""
        _fields = fields
        _fill()
        nameField.forceActiveFocus()
    }

    function _fill() {
        var f = _fields
        nameField.text = f.name
        hostField.text = f.host
        portField.text = String(f.port)
        transportBox.currentIndex = Math.max(0, _transportKinds.indexOf(f.transport))
        dayVideoField.text = f.dayVideoUrl
        thermalVideoField.text = f.thermalVideoUrl
        thermalBySwitchBox.checked = f.thermalBySwitch === true
        zoomBox.currentIndex = Math.max(0, _zoomKinds.indexOf(f.zoom))
        fovHField.text = String(f.dayFovH)
        fovVField.text = String(f.dayFovV)
        maxZoomField.text = String(f.maxZoom)
        _lenses = f.lenses.map(function(l) { return { fovH: l.fovH, fovV: l.fovV, firstStep: l.firstStep, lastStep: l.lastStep } })
        sensorWField.text = String(f.sensorW)
        sensorHField.text = String(f.sensorH)
        cropWField.text = String(f.cropStepW)
        cropHField.text = String(f.cropStepH)
        thermalFovHField.text = String(f.thermalFovH)
        thermalFovVField.text = String(f.thermalFovV)
        thermalSureBox.checked = f.thermalFovSure
        pitchMinField.text = String(f.pitchMin)
        pitchMaxField.text = String(f.pitchMax)
        yawLimitField.text = String(f.yawLimit)
        laserBox.checked = f.laser
        laserMaxField.text = String(f.laserMaxM)
        photoBox.checked = f.photo
        recordBox.checked = f.record
        trackerBox.checked = f.ownTracker
        upperHeaderBox.checked = f.upperGimbalHeader
    }

    // A number as typed: a comma counts as the decimal point. Anything else is not a number,
    // and the app says what is wrong.
    function _number(text) {
        var t = String(text).trim().replace(",", ".")
        return t === "" ? NaN : Number(t)
    }

    function save() {
        var f = _fields
        if (!f) {
            return
        }
        f.name = nameField.text
        f.host = hostField.text
        f.port = _number(portField.text)
        f.transport = _transportKinds[transportBox.currentIndex]
        f.dayVideoUrl = dayVideoField.text
        f.thermalBySwitch = _thermalBySwitch
        f.thermalVideoUrl = _thermalBySwitch ? "" : thermalVideoField.text  // the day video carries it
        f.zoom = _zoomKinds[zoomBox.currentIndex]
        f.dayFovH = _number(fovHField.text)
        f.dayFovV = _number(fovVField.text)
        f.maxZoom = _number(maxZoomField.text)
        f.lenses = _lenses
        f.sensorW = _number(sensorWField.text)
        f.sensorH = _number(sensorHField.text)
        f.cropStepW = _number(cropWField.text)
        f.cropStepH = _number(cropHField.text)
        f.thermalFovH = _number(thermalFovHField.text)
        f.thermalFovV = _number(thermalFovVField.text)
        f.thermalFovSure = thermalSureBox.checked
        f.pitchMin = _number(pitchMinField.text)
        f.pitchMax = _number(pitchMaxField.text)
        f.yawLimit = _number(yawLimitField.text)
        f.laser = laserBox.checked
        f.laserMaxM = _number(laserMaxField.text)
        f.photo = photoBox.checked
        f.record = recordBox.checked
        f.ownTracker = trackerBox.checked
        f.upperGimbalHeader = upperHeaderBox.checked
        var result = link.saveCamera(f)
        if (result.problem !== "") {
            _problem = result.problem
            return
        }
        _fields = null
        _problem = ""
    }

    function _addLens() {
        var lenses = _lenses.slice()
        var last = lenses.length > 0 ? lenses[lenses.length - 1] : null
        lenses.push({ fovH: last ? last.fovH / 2 : 60, fovV: last ? last.fovV / 2 : 40,
                      firstStep: last ? last.lastStep + 1 : 0, lastStep: last ? last.lastStep + 50 : 50 })
        _lenses = lenses
    }

    function _removeLens(index) {
        var lenses = _lenses.slice()
        lenses.splice(index, 1)
        _lenses = lenses
    }

    QGCPalette { id: qgcPal; colorGroupEnabled: true }

    background: Rectangle {
        color:        qgcPal.window
        radius:       ScreenTools.defaultBorderRadius
        border.color: qgcPal.text
    }

    // One label and a number field in a row of the form.
    component NumberField: QGCTextField {
        Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 10
        inputMethodHints:      Qt.ImhFormattedNumbersOnly
    }

    QGCFlickable {
        objectName:    "vamaCameraEditorFlick"
        anchors.fill:  parent
        contentWidth:  width
        contentHeight: editorColumn.implicitHeight
        clip:          true

        ColumnLayout {
            id:      editorColumn
            width:   parent.width
            spacing: ScreenTools.defaultFontPixelHeight / 2

            QGCLabel {
                text:      root._fields === null ? qsTr("Cameras")
                                                 : (root._fields.id === "" ? qsTr("New camera") : qsTr("Change camera"))
                font.bold: true
            }

            // --- The list ---------------------------------------------------------------
            QGCLabel {
                Layout.fillWidth: true
                visible:          root._fields === null
                wrapMode:         Text.WordWrap
                font.pointSize:   ScreenTools.smallFontPointSize
                text:             qsTr("Copy a camera to add a new one. The copy speaks the protocol of the camera it was copied from: Skydroid TOP, Viewpro or SIYI. A camera with another protocol needs an app update. Cameras you add are files in %1. Copy such a file into that folder on another remote control, and the camera is in its list too.").arg(root.link.cameraFolder)
            }

            QGCLabel {
                objectName:       "vamaCameraNotes"
                Layout.fillWidth: true
                visible:          root._fields === null && root.link.cameraNotes.length > 0
                wrapMode:         Text.WordWrap
                color:            qgcPal.warningText
                font.pointSize:   ScreenTools.smallFontPointSize
                text:             qsTr("Files that are not read:") + "\n" + root.link.cameraNotes.join("\n")
            }

            Repeater {
                model: root._fields === null ? root.link.cameras : []

                RowLayout {
                    Layout.fillWidth: true
                    spacing:          ScreenTools.defaultFontPixelWidth

                    QGCLabel {
                        Layout.fillWidth: true
                        elide:            Text.ElideRight
                        text:             modelData.name +
                                          (modelData.builtIn ? qsTr("  (built in)") : "") +
                                          (modelData.id === root.link.model ? qsTr("  (in use)") : "")
                    }
                    QGCButton {
                        objectName: "vamaCameraCopy_" + modelData.id
                        text:       qsTr("Copy")
                        onClicked:  root.edit(modelData.id, true)
                    }
                    QGCButton {
                        objectName: "vamaCameraEdit_" + modelData.id
                        visible:    !modelData.builtIn
                        text:       qsTr("Change")
                        onClicked:  root.edit(modelData.id, false)
                    }
                    QGCButton {
                        objectName: "vamaCameraDelete_" + modelData.id
                        visible:    !modelData.builtIn
                        text:       qsTr("Delete")
                        onClicked:  { root._problem = ""; root._askDelete = modelData.id }
                    }
                }
            }

            // Delete asks first.
            RowLayout {
                Layout.fillWidth: true
                visible:          root._fields === null && root._askDelete !== ""
                spacing:          ScreenTools.defaultFontPixelWidth

                QGCLabel {
                    Layout.fillWidth: true
                    wrapMode:         Text.WordWrap
                    text:             qsTr("Delete this camera? Its file is removed from this remote control.")
                }
                QGCButton {
                    objectName: "vamaCameraDeleteYes"
                    text:       qsTr("Delete")
                    onClicked: {
                        root._problem = root.link.deleteCamera(root._askDelete)
                        root._askDelete = ""
                    }
                }
                QGCButton {
                    text:      qsTr("Keep")
                    onClicked: root._askDelete = ""
                }
            }

            // --- The form ---------------------------------------------------------------
            GridLayout {
                Layout.fillWidth: true
                visible:          root._fields !== null
                columns:          2
                columnSpacing:    ScreenTools.defaultFontPixelWidth
                rowSpacing:       ScreenTools.defaultFontPixelHeight / 4

                QGCLabel { text: qsTr("Name"); Layout.preferredWidth: root._labelWidth }
                QGCTextField { id: nameField; objectName: "vamaCameraName"; Layout.fillWidth: true }

                QGCLabel { text: qsTr("Camera IP address"); Layout.preferredWidth: root._labelWidth }
                QGCTextField { id: hostField; objectName: "vamaCameraHost"; Layout.fillWidth: true }

                QGCLabel { text: qsTr("Control port"); Layout.preferredWidth: root._labelWidth }
                NumberField { id: portField; objectName: "vamaCameraPort" }

                QGCLabel { text: qsTr("Protocol"); Layout.preferredWidth: root._labelWidth }
                QGCLabel {
                    objectName:       "vamaCameraProtocol"
                    Layout.fillWidth: true
                    wrapMode:         Text.WordWrap
                    text:             root._protocol === "viewpro" ? qsTr("Viewpro, over TCP (as the A20KTR)")
                                    : root._protocol === "siyi"    ? qsTr("SIYI, over UDP (as the ZR10)")
                                                                   : qsTr("Skydroid TOP")
                }

                QGCLabel { text: qsTr("Control link"); Layout.preferredWidth: root._labelWidth; visible: root._top }
                QGCComboBox {
                    id:               transportBox
                    objectName:       "vamaCameraTransport"
                    Layout.fillWidth: true
                    visible:          root._top
                    model:            [qsTr("UDP (as the V12, V13 and V14 Pro)"), qsTr("TCP (as the V20)")]
                }

                QGCLabel { text: qsTr("Day video"); Layout.preferredWidth: root._labelWidth }
                QGCTextField { id: dayVideoField; objectName: "vamaCameraDayVideo"; Layout.fillWidth: true }

                QGCLabel { text: qsTr("Thermal video"); Layout.preferredWidth: root._labelWidth; visible: !root._thermalBySwitch }
                QGCTextField {
                    id:               thermalVideoField
                    objectName:       "vamaCameraThermalVideo"
                    Layout.fillWidth: true
                    visible:          !root._thermalBySwitch
                    placeholderText:  qsTr("Empty: no thermal picture")
                }

                // A Viewpro shows its thermal picture in its one video, after a sensor switch.
                Item { width: 1; height: 1; visible: root._protocol === "viewpro" }
                QGCCheckBox {
                    id:         thermalBySwitchBox
                    objectName: "vamaCameraThermalBySwitch"
                    visible:    root._protocol === "viewpro"
                    text:       qsTr("The thermal picture comes in the day video, by a switch (as the A20KTR)")
                }

                Item { width: 1; height: 1 }
                QGCButton {
                    objectName: "vamaCameraTakeAddresses"
                    text:       qsTr("Take the addresses in use now")
                    onClicked: {
                        hostField.text = root.link.host
                        portField.text = String(root.link.port)
                        dayVideoField.text = root.link.dayVideoUrl
                        thermalVideoField.text = root.link.thermalVideoUrl
                    }
                }

                QGCLabel { text: qsTr("Zoom"); Layout.preferredWidth: root._labelWidth }
                QGCComboBox {
                    id:               zoomBox
                    objectName:       "vamaCameraZoom"
                    Layout.fillWidth: true
                    model:            root._zoomKinds.map(function(kind) { return root._zoomNames[kind] })
                }

                QGCLabel {
                    text:                  root._zoomKind === "live" ? qsTr("Day lens at 1x, until the camera says its view (degrees)")
                                                                     : qsTr("Day lens, across and up-down (degrees)")
                    Layout.preferredWidth: root._labelWidth
                    wrapMode:              Text.WordWrap
                    visible:               root._zoomKind !== "reported"
                }
                RowLayout {
                    visible: root._zoomKind !== "reported"
                    NumberField { id: fovHField; objectName: "vamaCameraFovH" }
                    NumberField { id: fovVField; objectName: "vamaCameraFovV" }
                }

                QGCLabel {
                    text:                  qsTr("Highest zoom (times)")
                    Layout.preferredWidth: root._labelWidth
                    visible:               root._zoomKind === "counted"
                }
                NumberField { id: maxZoomField; objectName: "vamaCameraMaxZoom"; visible: root._zoomKind === "counted" }
            }

            // The lenses of a camera that reports its zoom step: across, up-down, first and last step.
            ColumnLayout {
                Layout.fillWidth: true
                visible:          root._fields !== null && root._zoomKind === "reported"
                spacing:          ScreenTools.defaultFontPixelHeight / 4

                QGCLabel {
                    Layout.fillWidth: true
                    wrapMode:         Text.WordWrap
                    font.pointSize:   ScreenTools.smallFontPointSize
                    text:             qsTr("Each lens: its view across and up-down (degrees) at its first zoom step, then its first and last step. Each step crops the sensor picture by the points below.")
                }
                Repeater {
                    model: root._lenses

                    RowLayout {
                        spacing: ScreenTools.defaultFontPixelWidth / 2

                        QGCLabel { text: qsTr("Lens %1").arg(index + 1) }
                        NumberField { text: String(modelData.fovH);      onTextChanged: root._lenses[index].fovH = root._number(text) }
                        NumberField { text: String(modelData.fovV);      onTextChanged: root._lenses[index].fovV = root._number(text) }
                        NumberField { text: String(modelData.firstStep); onTextChanged: root._lenses[index].firstStep = root._number(text) }
                        NumberField { text: String(modelData.lastStep);  onTextChanged: root._lenses[index].lastStep = root._number(text) }
                        QGCButton { text: qsTr("Remove"); onClicked: root._removeLens(index) }
                    }
                }
                QGCButton {
                    text:      qsTr("Add a lens")
                    enabled:   root._lenses.length < 4
                    onClicked: root._addLens()
                }
                RowLayout {
                    spacing: ScreenTools.defaultFontPixelWidth / 2
                    QGCLabel { text: qsTr("Sensor points"); Layout.preferredWidth: root._labelWidth }
                    NumberField { id: sensorWField }
                    NumberField { id: sensorHField }
                }
                RowLayout {
                    spacing: ScreenTools.defaultFontPixelWidth / 2
                    QGCLabel { text: qsTr("Crop of a step"); Layout.preferredWidth: root._labelWidth }
                    NumberField { id: cropWField }
                    NumberField { id: cropHField }
                }
            }

            GridLayout {
                Layout.fillWidth: true
                visible:          root._fields !== null
                columns:          2
                columnSpacing:    ScreenTools.defaultFontPixelWidth
                rowSpacing:       ScreenTools.defaultFontPixelHeight / 4

                QGCLabel {
                    text:                  qsTr("Thermal lens, across and up-down (degrees)")
                    Layout.preferredWidth: root._labelWidth
                    wrapMode:              Text.WordWrap
                }
                RowLayout {
                    NumberField { id: thermalFovHField; objectName: "vamaCameraThermalFovH" }
                    NumberField { id: thermalFovVField; objectName: "vamaCameraThermalFovV" }
                }

                Item { width: 1; height: 1 }
                QGCCheckBox {
                    id:   thermalSureBox
                    text: qsTr("The thermal lens is measured")
                }
                Item { width: 1; height: 1 }
                QGCLabel {
                    Layout.fillWidth: true
                    wrapMode:         Text.WordWrap
                    font.pointSize:   ScreenTools.smallFontPointSize
                    text:             qsTr("If it is not, a point on the thermal picture is placed only at its cross.")
                }

                QGCLabel { text: qsTr("Tilt from, to (degrees)"); Layout.preferredWidth: root._labelWidth }
                RowLayout {
                    NumberField { id: pitchMinField; objectName: "vamaCameraPitchMin" }
                    NumberField { id: pitchMaxField; objectName: "vamaCameraPitchMax" }
                }

                QGCLabel { text: qsTr("Turn either way (degrees)"); Layout.preferredWidth: root._labelWidth }
                NumberField { id: yawLimitField; objectName: "vamaCameraYawLimit" }

                // The app has no laser and no tracker of a Viewpro or SIYI camera (CameraProfile.h).
                QGCCheckBox { id: laserBox; objectName: "vamaCameraLaser"; text: qsTr("Laser"); visible: root._top }
                RowLayout {
                    visible: root._top
                    QGCLabel { text: qsTr("Longest reading (m)"); enabled: laserBox.checked }
                    NumberField { id: laserMaxField; objectName: "vamaCameraLaserMax"; enabled: laserBox.checked }
                }

                QGCCheckBox { id: photoBox; objectName: "vamaCameraPhoto"; text: qsTr("Photo") }
                QGCCheckBox { id: recordBox; objectName: "vamaCameraRecord"; text: qsTr("Video recording") }

                QGCCheckBox { id: trackerBox; objectName: "vamaCameraOwnTracker"; text: qsTr("Its own object tracker"); visible: root._top }
                QGCCheckBox { id: upperHeaderBox; text: qsTr("Upper-case gimbal frames only (as the V14 Pro)"); visible: root._top }
            }

            QGCLabel {
                objectName:       "vamaCameraProblem"
                Layout.fillWidth: true
                visible:          root._problem !== ""
                wrapMode:         Text.WordWrap
                color:            qgcPal.warningText
                text:             root._problem
            }

            RowLayout {
                Layout.alignment: Qt.AlignRight
                spacing:          ScreenTools.defaultFontPixelWidth

                QGCButton {
                    objectName: "vamaCameraCancel"
                    visible:    root._fields !== null
                    text:       qsTr("Cancel")
                    onClicked:  { root._fields = null; root._problem = "" }
                }
                QGCButton {
                    objectName: "vamaCameraSave"
                    visible:    root._fields !== null
                    text:       qsTr("Save")
                    onClicked:  root.save()
                }
                QGCButton {
                    objectName: "vamaCameraEditorClose"
                    visible:    root._fields === null
                    text:       qsTr("Close")
                    onClicked:  root.close()
                }
            }
        }
    }
}
