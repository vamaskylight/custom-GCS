// VAMA copy of QGC 5.1.5's src/FlyView/ProximityRadarValues.qml (the 8 distances
// of the drone's obstacle sensor, for the radar on the map and on the video).
// custom.qrc serves it in place of QGC's file. Compare it with QGC's file again
// after every QGC update. The changes are marked "VAMA".
//
// QGC took these values from its DISTANCE_SENSOR facts (vehicle.distanceSensors),
// which keep each message by direction only and never clear a value. With the
// client's RPLidar A3 (LiDAR plan, 2026-10-27) that showed:
// - every direction with nothing in range as a "0.00" label in the middle of
//   the video (ArduPilot sends such a direction as the sensor's range, and the
//   A3's range is 0), all on top of each other;
// - the last arcs for ever once the sensor stopped (ArduPilot then sends no
//   direction at all);
// - a range circle that changed size with a downward rangefinder beside it.
// Here the values come from the app's own store (src/ProximityStore.h, fed by
// CustomPlugin::mavlinkMessage): only the proximity sensor's own messages, an
// empty direction or one not heard for 2 s has no value (NaN), so both views
// leave it out. Checked by rendering QGC's own views on Qt 6.11.1:
// test/proximity_view_test.py; the store by test/link/proximity_store_test.cc.

import QtQuick

import QGroundControl   // VAMA: ProximityStore

/// Object which exposes vehicle distanceSensors FactGroup information for use in UI
QtObject {
    property var    vehicle

    // VAMA: the store's directions of this drone, not QGC's distanceSensors facts
    property var    _sectors:           vehicle ? ProximityStore.forVehicle(vehicle.id) : null

    property bool   telemetryAvailable: _sectors ? _sectors.telemetryAvailable : false

    signal rotationValueChanged ///< Signalled when any available rotation value changes

    property real   rotationNoneValue:      _value(0)
    property real   rotationYaw45Value:     _value(1)
    property real   rotationYaw90Value:     _value(2)
    property real   rotationYaw135Value:    _value(3)
    property real   rotationYaw180Value:    _value(4)
    property real   rotationYaw225Value:    _value(5)
    property real   rotationYaw270Value:    _value(6)
    property real   rotationYaw315Value:    _value(7)
    property real   maxDistance:            _sectors ? _sectors.maxDistance : NaN

    property string rotationNoneValueString:    _text(rotationNoneValue)
    property string rotationYaw45ValueString:   _text(rotationYaw45Value)
    property string rotationYaw90ValueString:   _text(rotationYaw90Value)
    property string rotationYaw135ValueString:  _text(rotationYaw135Value)
    property string rotationYaw180ValueString:  _text(rotationYaw180Value)
    property string rotationYaw225ValueString:  _text(rotationYaw225Value)
    property string rotationYaw270ValueString:  _text(rotationYaw270Value)
    property string rotationYaw315ValueString:  _text(rotationYaw315Value)

    property var    rgRotationValues:           [ rotationNoneValue, rotationYaw45Value, rotationYaw90Value, rotationYaw135Value, rotationYaw180Value, rotationYaw225Value, rotationYaw270Value, rotationYaw315Value ]
    property var    rgRotationValueStrings:     [ rotationNoneValueString, rotationYaw45ValueString, rotationYaw90ValueString, rotationYaw135ValueString, rotationYaw180ValueString, rotationYaw225ValueString, rotationYaw270ValueString, rotationYaw315ValueString ]

    property string _noValueStr:            qsTr("–.––")

    // VAMA: a direction's distance in metres, NaN when it has nothing in range (the store decides)
    function _value(direction) {
        if (!_sectors) {
            return NaN
        }
        var distance = _sectors.values[direction]
        return (distance === undefined || distance === null) ? NaN : distance
    }

    // VAMA: as QGC's facts wrote their values (2 decimals)
    function _text(distance) {
        return isNaN(distance) ? _noValueStr : distance.toFixed(2)
    }

    onRotationNoneValueChanged:     rotationValueChanged()
    onRotationYaw45ValueChanged:    rotationValueChanged()
    onRotationYaw90ValueChanged:    rotationValueChanged()
    onRotationYaw135ValueChanged:   rotationValueChanged()
    onRotationYaw180ValueChanged:   rotationValueChanged()
    onRotationYaw225ValueChanged:   rotationValueChanged()
    onRotationYaw270ValueChanged:   rotationValueChanged()
    onRotationYaw315ValueChanged:   rotationValueChanged()
}
