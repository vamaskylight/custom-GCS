// VAMA GCS: replaces QGC's Toolbar/VehicleGPSIndicator.qml (the GPS in the toolbar), see custom.qrc.
// QGC v5.1.5's file is QGC's GPSIndicator with three properties: kept here, around QGC's own
// GPSIndicator. Added, marked "VAMA" (A8 step 4, 2026-10-30): where the drone's position comes
// from, GPS 1 or the down camera as GPS 2 (the GPS-denied navigation, PositionSourceStore.h), as
// VGCS shows it:
//  - "CAM" beside the satellites while the camera gives the position ("GPS2" for a second real GPS).
//  - The GPS page has a "Position source" part above QGC's own page: VGCS's line in its colour, and
//    what GPS 2 is. QGC's page (GPSIndicatorPage.qml) is not copied: it is shown as it is under that
//    part, and its RTK settings stay its expanded part.
//  - The indicator also shows for a drone that reports only GPS 2.
// A drone without GPS 2 shows as before. When moving to a new QGC release, compare with QGC's
// VehicleGPSIndicator.qml and GPSIndicator.qml.

import QtQuick
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import QGroundControl.Toolbar   // VAMA: QGC's GPSIndicator and GPSIndicatorPage, which are not in this file's folder

Item {
    id:             control
    objectName:     "toolbar_gpsIndicator"
    width:          indicatorRow.width
    anchors.top:    parent.top
    anchors.bottom: parent.bottom

    property var  _activeVehicle: QGroundControl.multiVehicleManager.activeVehicle
    // VAMA: this drone's position source.
    property var  _source:        _activeVehicle ? PositionSourceStore.forVehicle(_activeVehicle.id) : null
    property bool showIndicator:  !!_activeVehicle && (_activeVehicle.gps.telemetryAvailable || (!!_source && _source.hasGps2))

    QGCPalette { id: qgcPal }

    Row {
        id:             indicatorRow
        anchors.top:    parent.top
        anchors.bottom: parent.bottom
        spacing:        ScreenTools.defaultFontPixelWidth / 2

        // QGC's own indicator: GPS 1's icon, satellites and HDOP.
        GPSIndicator { }

        // VAMA: "CAM" while the down camera gives the position.
        QGCLabel {
            objectName:             "vamaPositionSourceTag"
            anchors.verticalCenter: parent.verticalCenter
            visible:                text !== ""
            text:                   _source ? _source.tag : ""
            color:                  qgcPal.colorOrange
            font.bold:              true
        }
    }

    // VAMA: over QGC's own (in GPSIndicator), for the page with the position source.
    MouseArea {
        anchors.fill:   parent
        onClicked:      mainWindow.showIndicatorDrawer(gpsIndicatorPage, control)
    }

    Component {
        id: gpsIndicatorPage

        // QGC's page, with the position source above its own content.
        GPSIndicatorPage {
            contentComponent: Component {
                ColumnLayout {
                    spacing: ScreenTools.defaultFontPixelHeight / 2

                    SettingsGroupLayout {
                        objectName: "vamaPositionSourceGroup"
                        heading:    qsTr("Position source")
                        visible:    !!_source && _source.hasGps2

                        QGCLabel {
                            objectName:          "vamaPositionSourceText"
                            Layout.maximumWidth: ScreenTools.defaultFontPixelWidth * 40
                            wrapMode:            Text.WordWrap
                            text:                _source ? _source.text : ""
                            color:               !_source ? qgcPal.text
                                               : (_source.level === "bad" ? qgcPal.colorRed
                                               : (_source.level === "warn" ? qgcPal.colorOrange : qgcPal.text))
                        }

                        QGCLabel {
                            objectName:          "vamaPositionSourceDetails"
                            Layout.maximumWidth: ScreenTools.defaultFontPixelWidth * 40
                            wrapMode:            Text.WordWrap
                            font.pointSize:      ScreenTools.smallFontPointSize
                            text:                _source ? _source.details : ""
                        }
                    }

                    // QGC's page once more, for its content only: only the outer page is ever expanded.
                    GPSIndicatorPage { }
                }
            }
        }
    }
}
