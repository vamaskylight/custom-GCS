// Stand-in for QGC's Viewer3DManager (the C++ singleton QGCViewer3DManager of the
// module QGroundControl.Viewer3D): only which view the Fly view shows. QGC's enum
// is DisplayMode { Map, View3D }.
pragma Singleton

import QtQuick

QtObject {
    enum DisplayMode { Map, View3D }

    property int displayMode: 0  // Map
}
