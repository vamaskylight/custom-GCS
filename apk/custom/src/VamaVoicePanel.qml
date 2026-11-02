import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls

// VAMA (the client, Nish 2026-10-30: Indian English, female): the voice the app speaks its messages
// with (VamaVoice.h). The app menu (SelectViewDropdown.qml) shows this in place of its buttons, as
// it shows the admin password: a window made from the menu would stop working when the menu closes
// (VamaAdminPasswordPanel.qml). `voice` is the app's VamaVoice (in the screen preview too, with
// made-up voices). Test is QGC's own test of its speech, which speaks with the voice in use.
// `finished` closes the menu.
ColumnLayout {
    id:      root
    spacing: ScreenTools.defaultFontPixelHeight / 2

    property var voice: VamaVoice

    property real _width: ScreenTools.defaultFontPixelWidth * 44

    signal finished()

    Component.onCompleted: root.voice.panelOpened()

    QGCPalette { id: qgcPal; colorGroupEnabled: true }

    QGCLabel { text: qsTr("Voice"); font.bold: true }

    QGCLabel {
        Layout.preferredWidth: root._width
        wrapMode:              Text.WordWrap
        text:                  qsTr("The voice the app speaks its messages with. Automatic takes an Indian English voice, a female one when Android says which.")
    }

    QGCComboBox {
        id:                    voiceBox
        objectName:            "vamaVoiceBox"
        Layout.preferredWidth: root._width
        model:                 root.voice.choices
        // Follows the voice, also when the RC's voices change while this is open. A tap sets the
        // index from Qt's C++, which keeps this binding; a JavaScript assignment would remove it.
        currentIndex:          root.voice.choiceIndex
        onActivated:           (index) => root.voice.chooseIndex(index)
    }

    QGCLabel {
        objectName:            "vamaVoiceNote"
        Layout.preferredWidth: root._width
        visible:               text !== ""
        wrapMode:              Text.WordWrap
        color:                 qgcPal.warningText
        text:                  root.voice.note
    }

    RowLayout {
        Layout.alignment: Qt.AlignRight
        spacing:          ScreenTools.defaultFontPixelWidth

        QGCButton {
            objectName: "vamaVoiceTest"
            text:       qsTr("Test")
            onClicked:  QGroundControl.testAudioOutput()
        }
        QGCButton {
            objectName: "vamaVoiceDone"
            text:       qsTr("Done")
            onClicked:  root.finished()
        }
    }
}
