import QtQuick
import QtQuick.Controls
import QtCore

import QGroundControl
import QGroundControl.Controls

// VAMA (B25, the client 2026-10-31: "Just like the mission planner proximity", "this proximity into hidden button so
// when we click on button then we can see the proximity and make this proximity dialogue box in adjustable dialogue
// box", and of B19's four bars: "the round view replace them"): the round proximity view, as Mission Planner shows it
// and as VGCS draws it (vgcs/map/proximity_window.py), in a box on the fly view. Its top drags it, its corner sizes it,
// and both are kept for the next time. FlyViewCustomLayer.qml opens it with its button and gives it the active drone's
// `sectors` (ProximityStore.forVehicle).
//
// The drone in the middle, its nose up: the readings are the drone's own directions, so the top is ahead, not north.
// Rings every tenth of the range (0.5 m at the 5 m it starts with), the degrees every 45, and an object as a red arc
// across its direction at its distance, with the distance. Only within the range set in the app menu ("Proximity
// range", VamaObstacleRangePanel.qml). Drawn only when a reading, the range or the size changes (the RC's lag, B23).
Item {
    id: root

    property var  sectors: null   // the active drone's ProximitySectors, or null
    property real range:   ProximityStore.barRange

    readonly property real _font:    ScreenTools.defaultFontPixelHeight
    readonly property real _titleH:  _font * 2
    readonly property real _minSize: _font * 9

    width:  _font * 15
    height: _font * 16.5

    // Where it is and how big, for the next time (QtCore Settings, as QGC's MainWindowSavedState.qml).
    Settings {
        id:       saved
        category: "VamaProximityWindow"
        property real x: -1
        property real y: -1
        property real w: 0
        property real h: 0
    }

    function _keepInside() {
        if (!parent) {
            return
        }
        width  = Math.max(_minSize, Math.min(width, parent.width))
        height = Math.max(_minSize, Math.min(height, parent.height))
        x = Math.max(0, Math.min(x, parent.width - width))
        y = Math.max(0, Math.min(y, parent.height - height))
    }

    function _save() {
        saved.x = x
        saved.y = y
        saved.w = width
        saved.h = height
    }

    // The words under the view, as VGCS's: an object within the range, or none.
    readonly property string status: {
        var values = sectors ? sectors.values : []
        for (var i = 0; i < values.length; ++i) {
            if (!isNaN(values[i]) && values[i] <= range) {
                return qsTr("Object within %1 m").arg(range)
            }
        }
        return sectors && sectors.telemetryAvailable ? qsTr("Clear within %1 m").arg(range) : qsTr("Waiting for the proximity sensor")
    }

    // Placed when it first opens, as it was left, else in the middle (the screen has its size by then).
    property bool _placed: false
    onVisibleChanged: {
        if (!visible || _placed || !parent) {
            return
        }
        _placed = true
        if (saved.w > 0 && saved.h > 0) {
            width  = saved.w
            height = saved.h
        }
        if (saved.x >= 0 && saved.y >= 0) {
            x = saved.x
            y = saved.y
        } else {
            x = (parent.width - width) / 2
            y = (parent.height - height) / 2
        }
        _keepInside()
    }
    Connections {
        target: root.parent
        function onWidthChanged()  { if (root._placed) root._keepInside() }
        function onHeightChanged() { if (root._placed) root._keepInside() }
    }

    // Taps on the box stop here instead of reaching the video or the map.
    MouseArea { anchors.fill: parent }

    Rectangle {
        anchors.fill: parent
        color:        "#25272b"
        radius:       root._font * 0.4
        border.color: Qt.rgba(1, 1, 1, 0.25)
        border.width: 1
    }

    // The top: dragged to move the box.
    Item {
        id:     titleBar
        width:  parent.width
        height: root._titleH

        QGCLabel {
            objectName:             "vamaProximityTitle"
            anchors.left:           parent.left
            anchors.leftMargin:     root._font * 0.6
            anchors.verticalCenter: parent.verticalCenter
            text:                   qsTr("Proximity, within %1 m").arg(root.range)
            color:                  "white"
            font.bold:              true
        }
        MouseArea {
            objectName:        "vamaProximityDrag"
            anchors.fill:      parent
            anchors.rightMargin: closeButton.width
            drag.target:       root
            drag.minimumX:     0
            drag.minimumY:     0
            drag.maximumX:     root.parent ? root.parent.width - root.width : 0
            drag.maximumY:     root.parent ? root.parent.height - root.height : 0
            onReleased:        root._save()
        }
        Rectangle {
            id:                     closeButton
            objectName:             "vamaProximityClose"
            anchors.right:          parent.right
            anchors.rightMargin:    root._font * 0.3
            anchors.verticalCenter: parent.verticalCenter
            width:                  root._titleH * 0.8
            height:                 width
            radius:                 width / 2
            color:                  closeArea.pressed ? Qt.rgba(1, 1, 1, 0.3) : Qt.rgba(1, 1, 1, 0.12)

            QGCLabel {
                anchors.centerIn: parent
                text:             "X"
                color:            "white"
                font.bold:        true
            }
            MouseArea {
                id:           closeArea
                anchors.fill: parent
                onClicked:    root.visible = false
            }
        }
    }

    Canvas {
        id:                  plot
        objectName:          "vamaProximityPlot"
        anchors.top:         titleBar.bottom
        anchors.left:        parent.left
        anchors.right:       parent.right
        anchors.bottom:      statusLine.top
        anchors.margins:     root._font * 0.2
        renderStrategy:      Canvas.Cooperative

        // A Canvas draws itself again at a new size (a breakage run showed handlers for it changed nothing).
        onPaint: {
            var ctx = getContext("2d")
            ctx.reset()
            ctx.fillStyle = "#25272b"
            ctx.fillRect(0, 0, width, height)
            var font = Math.max(9, Math.min(14, Math.round(Math.min(width, height) / 26)))
            var cx = width / 2
            var cy = height / 2
            var radius = Math.max(10, Math.min(width, height) / 2 - font * 1.9)
            var range = root.range > 0 ? root.range : 5
            ctx.font = font + "px sans-serif"

            // Rings every tenth of the range, and the degrees every 45.
            ctx.lineWidth = 1
            ctx.strokeStyle = "rgba(200, 200, 200, 0.43)"
            for (var i = 1; i <= 10; ++i) {
                ctx.beginPath()
                ctx.arc(cx, cy, radius * i / 10, 0, 2 * Math.PI, false)
                ctx.stroke()
            }
            ctx.textAlign = "center"
            ctx.textBaseline = "middle"
            for (var k = 0; k < 8; ++k) {
                var a = (k * 45 - 90) * Math.PI / 180
                ctx.strokeStyle = "rgba(200, 200, 200, 0.43)"
                ctx.beginPath()
                ctx.moveTo(cx, cy)
                ctx.lineTo(cx + radius * Math.cos(a), cy + radius * Math.sin(a))
                ctx.stroke()
                ctx.fillStyle = "#aaaeb4"
                ctx.fillText(String(k * 45), cx + (radius + font) * Math.cos(a), cy + (radius + font) * Math.sin(a))
            }
            // The rings' distances on the line ahead, every ring when there is room, else every second one.
            ctx.fillStyle = "#28aa46"
            ctx.textAlign = "left"
            ctx.textBaseline = "top"
            var every = radius / 10 >= font * 1.5 ? 1 : 2
            for (var j = every; j <= 10; j += every) {
                ctx.fillText((range * j / 10).toFixed(1) + "m", cx + 4, cy - radius * j / 10)
            }

            // The drone, its nose up, inside the first ring.
            var s = Math.max(4, radius * 0.035)
            ctx.strokeStyle = "#aacd32"
            ctx.lineWidth = Math.max(1.5, s * 0.22)
            var arms = [[0, -1], [1, 0], [0, 1], [-1, 0]]
            for (var n = 0; n < arms.length; ++n) {
                ctx.beginPath()
                ctx.moveTo(cx, cy)
                ctx.lineTo(cx + arms[n][0] * s, cy + arms[n][1] * s)
                ctx.stroke()
                ctx.beginPath()
                ctx.arc(cx + arms[n][0] * s * 1.4, cy + arms[n][1] * s * 1.4, s * 0.55, 0, 2 * Math.PI, false)
                ctx.stroke()
            }

            // The objects: a red arc across each one's direction, at its distance, and the distance.
            var values = root.sectors ? root.sectors.values : []
            ctx.textAlign = "center"
            ctx.textBaseline = "middle"
            for (var d = 0; d < 8 && d < values.length; ++d) {
                var metres = values[d]
                if (isNaN(metres) || metres > range) {
                    continue
                }
                var r = Math.max(2, radius * metres / range)
                var mid = (d * 45 - 90) * Math.PI / 180
                var half = 19.5 * Math.PI / 180
                ctx.strokeStyle = "#f02020"
                ctx.lineWidth = 5
                ctx.beginPath()
                ctx.arc(cx, cy, r, mid - half, mid + half, false)
                ctx.stroke()
                var tr = r + font + 4
                var tx = cx + tr * Math.cos(mid) - (d === 0 ? 34 : 0)  // ahead: left of the line, clear of the rings' distances
                ctx.fillStyle = "white"
                ctx.fillText(metres.toFixed(metres < 10 ? 1 : 0) + " m", tx, cy + tr * Math.sin(mid))
            }
        }

        Connections {
            target:                 root.sectors
            ignoreUnknownSignals:   true
            function onChanged()    { plot.requestPaint() }
        }
        Connections {
            target:                 root
            function onRangeChanged()   { plot.requestPaint() }
            function onSectorsChanged() { plot.requestPaint() }
        }
    }

    QGCLabel {
        id:                   statusLine
        objectName:           "vamaProximityStatus"
        anchors.left:         parent.left
        anchors.leftMargin:   root._font * 0.6
        anchors.bottom:       parent.bottom
        anchors.bottomMargin: root._font * 0.3
        text:                 root.status
        color:                "#d6dae2"
    }

    // The corner: dragged to size the box.
    Item {
        objectName:     "vamaProximityResize"
        anchors.right:  parent.right
        anchors.bottom: parent.bottom
        width:          root._font * 1.6
        height:         width

        Canvas {
            anchors.fill: parent
            onPaint: {
                var ctx = getContext("2d")
                ctx.reset()
                ctx.strokeStyle = "rgba(255, 255, 255, 0.55)"
                ctx.lineWidth = 2
                for (var i = 1; i <= 3; ++i) {
                    ctx.beginPath()
                    ctx.moveTo(width - i * width / 4, height - 2)
                    ctx.lineTo(width - 2, height - i * height / 4)
                    ctx.stroke()
                }
            }
        }
        MouseArea {
            anchors.fill: parent
            property real _startX: 0
            property real _startY: 0
            property real _startW: 0
            property real _startH: 0
            onPressed: (mouse) => {
                var p = mapToItem(root.parent, mouse.x, mouse.y)
                _startX = p.x
                _startY = p.y
                _startW = root.width
                _startH = root.height
            }
            onPositionChanged: (mouse) => {
                var p = mapToItem(root.parent, mouse.x, mouse.y)
                root.width  = Math.max(root._minSize, Math.min(_startW + p.x - _startX, root.parent.width - root.x))
                root.height = Math.max(root._minSize, Math.min(_startH + p.y - _startY, root.parent.height - root.y))
            }
            onReleased: root._save()
        }
    }
}
