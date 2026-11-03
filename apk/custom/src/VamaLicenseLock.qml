import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls

// VAMA (row A9, the client 2026-11-03: "i dont want my Apk or exe to be copied and transferred to
// another RC without permission"): the app opens only on an RC with a license key made for it.
// VamaLicense (C++) shows this over the whole window at the start while the RC has no key, and the
// drone links wait until a key opens the app. It shows the RC's device code to send to VAMA and
// takes the key VAMA sends back. A key that does not fit is explained here, and the lock stays.
// A key that fits saves itself, and VamaLicense then removes this screen.
Popup {
    id:          root
    objectName:  "vamaLicenseLock"
    x:           0
    y:           0
    width:       parent ? parent.width : 0
    height:      parent ? parent.height : 0
    padding:     0
    modal:       true
    focus:       true
    closePolicy: Popup.NoAutoClose
    z:           100000  // above every popup of QGC

    required property var license  // VamaLicense

    property string _problem: license.savedKeyProblem  // why the last key does not open the app
    property bool   _copied:  false

    readonly property real _margin: ScreenTools.defaultFontPixelWidth * 2

    function activate() {
        _problem = license.activate(keyField.text)
    }

    QGCPalette { id: qgcPal; colorGroupEnabled: true }

    Timer {
        id:          copiedTimer
        interval:    1500
        onTriggered: root._copied = false
    }

    background: Rectangle { color: qgcPal.window }

    contentItem: QGCFlickable {
        id:            flick
        objectName:    "vamaLicenseLockContent"
        clip:          true
        focus:         true
        contentWidth:  width
        contentHeight: column.implicitHeight + root._margin * 2

        // The RC's back key does nothing here: QGC's own question to close the app would open
        // under this screen. "Close the app" is the way out. (A key the text field does not take
        // comes here too.)
        Keys.onBackPressed: (event) => { event.accepted = true }

        ColumnLayout {
            id:      column
            x:       Math.max(root._margin, (flick.width - width) / 2)
            y:       root._margin
            width:   Math.min(flick.width - root._margin * 2, ScreenTools.defaultFontPixelWidth * 64)
            spacing: ScreenTools.defaultFontPixelHeight / 2

            QGCLabel {
                Layout.fillWidth: true
                wrapMode:         Text.WordWrap
                font.bold:        true
                font.pointSize:   ScreenTools.largeFontPointSize
                text:             qsTr("This copy of Vama GCS needs a license key for this RC.")
            }

            QGCLabel {
                Layout.fillWidth: true
                wrapMode:         Text.WordWrap
                visible:          root.license.canActivate
                text:             qsTr("Send this device code to VAMA. You will get a license key back. Paste it below.")
            }

            RowLayout {
                spacing: ScreenTools.defaultFontPixelWidth
                visible: root.license.canActivate

                QGCLabel {
                    objectName:     "vamaLicenseDeviceCode"
                    font.bold:      true
                    font.family:    ScreenTools.fixedFontFamily
                    font.pointSize: ScreenTools.largeFontPointSize
                    text:           root.license.deviceCode
                }

                QGCButton {
                    objectName: "vamaLicenseCopyButton"
                    text:       root._copied ? qsTr("Copied") : qsTr("Copy")
                    onClicked: {
                        root.license.copyDeviceCode()
                        root._copied = true
                        copiedTimer.restart()
                    }
                }
            }

            TextArea {
                id:                     keyField
                objectName:             "vamaLicenseKeyField"
                Layout.fillWidth:       true
                Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 4
                visible:                root.license.canActivate
                wrapMode:               TextEdit.WrapAnywhere
                font.family:            ScreenTools.fixedFontFamily
                font.pointSize:         ScreenTools.defaultFontPointSize
                color:                  "#212529"
                placeholderText:        qsTr("License key")
                placeholderTextColor:   "#868e96"
                inputMethodHints:       Qt.ImhNoPredictiveText | Qt.ImhNoAutoUppercase
                background: Rectangle {
                    color:  "#ffffff"
                    radius: ScreenTools.defaultFontPixelHeight / 4
                }
                // Typing or pasting clears the words about the last key, as in VGCS's window.
                onTextChanged: root._problem = ""
            }

            QGCLabel {
                objectName:       "vamaLicenseProblem"
                Layout.fillWidth: true
                wrapMode:         Text.WordWrap
                visible:          text !== ""
                color:            qgcPal.warningText
                text:             root._problem
            }

            RowLayout {
                Layout.fillWidth: true
                spacing:          ScreenTools.defaultFontPixelWidth

                QGCButton {
                    objectName: "vamaLicensePasteButton"
                    visible:    root.license.canActivate
                    text:       qsTr("Paste")
                    onClicked:  keyField.text = root.license.clipboardText()
                }

                Item { Layout.fillWidth: true }

                QGCButton {
                    objectName: "vamaLicenseCloseButton"
                    text:       qsTr("Close the app")
                    onClicked:  Qt.quit()
                }

                QGCButton {
                    objectName: "vamaLicenseActivateButton"
                    visible:    root.license.canActivate
                    primary:    true
                    text:       qsTr("Activate")
                    onClicked:  root.activate()
                }
            }

            QGCLabel {
                Layout.fillWidth: true
                wrapMode:         Text.WordWrap
                visible:          root.license.canActivate
                color:            qgcPal.colorGrey
                font.pointSize:   ScreenTools.smallFontPointSize
                text:             qsTr("A key works only on the RC it was made for. A test build and a customer build need a key each, and after a factory reset the RC needs a new key.")
            }
        }
    }
}
