// Stand-in for QGC's main window under the license lock (row A9). VamaLicense::showLock puts the
// real VamaLicenseLock.qml over it, as CustomPlugin::createRootWindow does over QGC's window. The
// button under the lock counts its presses (1 each), and so does a modal dialog like the ones QGC
// opens at its start, after the lock (100 each): none may reach either while the lock is up.

import QtQuick
import QtQuick.Controls
import QtQuick.Window

Window {
    id:      window
    width:   1920
    height:  1080
    visible: true
    color:   "#212529"

    property int pressesUnder: 0

    function openQgcDialog()  { qgcDialog.open() }
    function closeQgcDialog() { qgcDialog.close() }

    Button {
        objectName:       "previewUnderTheLock"
        anchors.centerIn: parent
        width:            600
        height:           300
        text:             "QGC's screen"
        onClicked:        window.pressesUnder++
    }

    // As QGC's QGCPopupDialog: modal, no z of its own, and big enough to lie under the lock's buttons.
    Popup {
        id:          qgcDialog
        objectName:  "previewQgcDialog"
        x:           100
        y:           100
        width:       window.width - 200
        height:      window.height - 200
        modal:       true
        closePolicy: Popup.NoAutoClose

        contentItem: Button {
            objectName: "previewQgcDialogButton"
            text:       "QGC's dialog"
            onClicked:  window.pressesUnder += 100
        }
    }
}
