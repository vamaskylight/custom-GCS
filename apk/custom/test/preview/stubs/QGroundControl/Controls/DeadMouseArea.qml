// Copy of QGC 5.1.5's src/QmlControls/DeadMouseArea.qml for the preview.
// Stops clicks from reaching the screen below.

import QtQuick
import QtQuick.Controls

MouseArea {
    preventStealing:true
    hoverEnabled:   true
    onWheel:    (wheel) => { wheel.accepted = true; }
    onPressed:  (mouse) => { mouse.accepted = true; }
    onReleased: (mouse) => { mouse.accepted = true; }
}
