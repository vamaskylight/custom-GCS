import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls

// VAMA (task Z, client 2026-10-16): Advanced Mode, with the drone's parameters, asks for the
// admin password first (AdminPassword.h). The app menu (SelectViewDropdown.qml) shows this in
// place of its buttons, at the top of the screen, clear of the keyboard. It is not a window of
// its own: a window made from the menu stops working when the menu closes, as Qt gives it the
// menu's QML context (measured on Qt 6.11, 2026-10-19: its timer, handlers and functions all
// stopped). `admin` checks the password: QGC's core plugin in the app
// (CustomPlugin::unlockAdvanced), a stand-in in the screen preview. `finished` closes the menu.
ColumnLayout {
    id:      root
    spacing: ScreenTools.defaultFontPixelHeight / 2

    property var admin: QGroundControl.corePlugin

    property real _width: ScreenTools.defaultFontPixelWidth * 36
    property int  _wait:  0      // seconds until the next try
    property bool _wrong: false  // the last try was wrong

    signal finished()

    function tryUnlock() {
        if (_wait > 0 || passwordField.text === "") {
            return
        }
        var right = admin.unlockAdvanced(passwordField.text)
        passwordField.text = ""
        if (right) {
            root.finished()
            return
        }
        _wrong = true
        _wait = admin.adminWaitSeconds()
        waitTimer.restart()
    }

    // A wait from a wrong try goes on when the menu is opened again.
    Component.onCompleted: {
        _wait = admin.adminWaitSeconds()
        if (_wait > 0) {
            waitTimer.start()
        }
        passwordField.forceActiveFocus()
    }

    QGCPalette { id: qgcPal; colorGroupEnabled: true }

    Timer {
        id:       waitTimer
        interval: 250
        repeat:   true
        onTriggered: {
            root._wait = root.admin.adminWaitSeconds()
            if (root._wait <= 0) {
                stop()
            }
        }
    }

    QGCLabel { text: qsTr("Advanced Mode"); font.bold: true }

    QGCLabel {
        Layout.preferredWidth: root._width
        wrapMode:              Text.WordWrap
        text:                  qsTr("Advanced Mode shows the drone's parameters. Enter the admin password.")
    }

    QGCTextField {
        id:                    passwordField
        objectName:            "vamaAdminPasswordField"
        Layout.preferredWidth: root._width
        echoMode:              TextInput.Password
        inputMethodHints:      Qt.ImhSensitiveData | Qt.ImhNoPredictiveText | Qt.ImhNoAutoUppercase
        onAccepted:            root.tryUnlock()
    }

    QGCLabel {
        objectName:            "vamaAdminPasswordMessage"
        Layout.preferredWidth: root._width
        visible:               root._wrong || root._wait > 0
        wrapMode:              Text.WordWrap
        color:                 qgcPal.warningText
        text:                  root._wait <= 0 ? qsTr("Wrong password.")
                                               : (root._wrong ? qsTr("Wrong password. Try again in %1 s.").arg(root._wait)
                                                              : qsTr("Try again in %1 s.").arg(root._wait))
    }

    RowLayout {
        Layout.alignment: Qt.AlignRight
        spacing:          ScreenTools.defaultFontPixelWidth

        QGCButton {
            objectName: "vamaAdminPasswordCancel"
            text:       qsTr("Cancel")
            onClicked:  root.finished()
        }
        QGCButton {
            objectName: "vamaAdminPasswordOk"
            text:       qsTr("OK")
            enabled:    root._wait <= 0 && passwordField.text !== ""
            onClicked:  root.tryUnlock()
        }
    }
}
