// Stand-in for QGC's GeneralSettings.qml, the page Application Settings opens
// first. The page itself is not part of the preview.

import QtQuick

import QGroundControl.Controls

Item {
    property int sectionFilter: -1

    QGCLabel {
        text: "QGC's General page (not part of the preview)"
    }
}
