pragma Singleton
import QtQuick

// Sizes measured from the first field video: a 1920x1080 RC screen where
// QGC's default text line is about 40 px high.
QtObject {
    property real defaultFontPixelHeight: 40
    property real defaultFontPixelWidth:  22
    property real defaultFontPointSize:   24
    property real smallFontPointSize:     18
    property real mediumFontPointSize:    30
    property real largeFontPointSize:     36
    property real defaultBorderRadius:    8
    property bool isMobile:               true
}
