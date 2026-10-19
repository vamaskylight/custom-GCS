// Stand-in for QGC 5.1.5's src/QmlControls/SubMenuButton.qml for the preview:
// a menu button with QGC's height and colours, without its icon.

import QtQuick
import QtQuick.Controls

Button {
    id:             control
    focusPolicy:    Qt.ClickFocus
    implicitHeight: ScreenTools.defaultFontPixelHeight * 2.5
    implicitWidth:  ScreenTools.defaultFontPixelWidth * 12
    font.pointSize: ScreenTools.defaultFontPointSize

    property bool   setupComplete: true
    property string imageResource: ""

    QGCPalette { id: qgcPal }

    background: Rectangle {
        color: control.pressed ? qgcPal.buttonHighlight : qgcPal.windowShade
    }

    contentItem: QGCLabel {
        leftPadding:       ScreenTools.defaultFontPixelWidth
        verticalAlignment: Text.AlignVCenter
        text:              control.text
    }
}
