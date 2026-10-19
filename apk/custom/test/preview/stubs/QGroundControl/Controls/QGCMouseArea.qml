// Stand-in for QGC 5.1.5's src/QmlControls/QGCMouseArea.qml for the preview: a MouseArea.
// (QGC's can also show its touch area, and grows it on phones when it has a fillItem.)

import QtQuick

MouseArea {
    property var fillItem

    anchors.fill: fillItem ? fillItem : undefined
}
