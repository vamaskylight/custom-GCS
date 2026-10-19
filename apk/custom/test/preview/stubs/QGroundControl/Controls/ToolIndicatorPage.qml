// Stand-in for QGC 5.1.5's src/QmlControls/ToolIndicatorPage.qml for the preview:
// the normal view of a toolbar page (the app menu has no expanded view).

import QtQuick
import QtQuick.Layouts

RowLayout {
    id:      control
    spacing: ScreenTools.defaultFontPixelWidth

    property bool      showExpand:        false
    property Component contentComponent
    property Component expandedComponent
    property bool      expanded:          false
    property var       drawer

    Loader {
        id:               contentItemLoader
        Layout.alignment: Qt.AlignTop
        sourceComponent:  control.contentComponent
    }
}
