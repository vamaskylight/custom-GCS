import QtQuick

// The real one tints through QGC's "coloredsvg" image provider, which reads
// "/path" and "qrc:/path" from the app resources. The VAMA icons are white
// already, so this one only does the same path mapping.
Item {
    id: root
    property color color: "white"
    property url   source

    readonly property string _resource: {
        const s = source.toString()
        if (s.startsWith("qrc:/")) return s
        if (s.startsWith("/"))     return "qrc:" + s
        return s.length > 0 ? "qrc:/" + s : ""
    }

    Image {
        anchors.fill:      parent
        source:            root._resource
        sourceSize.height: height
        fillMode:          Image.PreserveAspectFit
        smooth:            true
    }
}
