// VAMA copy of QGC 5.1.5's src/FlyView/ProximityRadarValues.qml (the 8 distances
// of the drone's obstacle sensor, for the radar on the map and on the video).
// custom.qrc serves it in place of QGC's file. Compare it with QGC's file again
// after every QGC update. The changes are marked "VAMA".
//
// ArduPilot sends a direction with nothing in range as the sensor's maximum
// distance, and that maximum is 0 for a sensor whose range it does not know:
// the client's RPLidar A3 (LiDAR plan, 2026-10-27). QGC showed every such
// direction as a value: on the video a "0.00" label in the middle of the
// picture for each of them, all on top of each other, and with a known range a
// red arc on the range circle. Here a direction with nothing in range has no
// value (NaN), so both views leave it out. A real distance shows as before.
// Checked by rendering QGC's own views on Qt 6.11.1: test/proximity_view_test.py.

import QtQuick

/// Object which exposes vehicle distanceSensors FactGroup information for use in UI
QtObject {
    property var    vehicle

    property bool   telemetryAvailable: vehicle ? vehicle.distanceSensors.telemetryAvailable : false

    signal rotationValueChanged ///< Signalled when any available rotation value changes

    // VAMA: each through _reading, so that nothing in range is no value
    property real   rotationNoneValue:      _reading(_distanceSensors ? _distanceSensors.rotationNone : null)
    property real   rotationYaw45Value:     _reading(_distanceSensors ? _distanceSensors.rotationYaw45 : null)
    property real   rotationYaw90Value:     _reading(_distanceSensors ? _distanceSensors.rotationYaw90 : null)
    property real   rotationYaw135Value:    _reading(_distanceSensors ? _distanceSensors.rotationYaw135 : null)
    property real   rotationYaw180Value:    _reading(_distanceSensors ? _distanceSensors.rotationYaw180 : null)
    property real   rotationYaw225Value:    _reading(_distanceSensors ? _distanceSensors.rotationYaw225 : null)
    property real   rotationYaw270Value:    _reading(_distanceSensors ? _distanceSensors.rotationYaw270 : null)
    property real   rotationYaw315Value:    _reading(_distanceSensors ? _distanceSensors.rotationYaw315 : null)
    property real   maxDistance:            _distanceSensors ? _distanceSensors.maxDistance.value : NaN

    property string rotationNoneValueString:    _distanceSensors ? _distanceSensors.rotationNone.valueString : _noValueStr
    property string rotationYaw45ValueString:   _distanceSensors ? _distanceSensors.rotationYaw45.valueString : _noValueStr
    property string rotationYaw90ValueString:   _distanceSensors ? _distanceSensors.rotationYaw90.valueString : _noValueStr
    property string rotationYaw135ValueString:  _distanceSensors ? _distanceSensors.rotationYaw135.valueString : _noValueStr
    property string rotationYaw180ValueString:  _distanceSensors ? _distanceSensors.rotationYaw180.valueString : _noValueStr
    property string rotationYaw225ValueString:  _distanceSensors ? _distanceSensors.rotationYaw225.valueString : _noValueStr
    property string rotationYaw270ValueString:  _distanceSensors ? _distanceSensors.rotationYaw270.valueString : _noValueStr
    property string rotationYaw315ValueString:  _distanceSensors ? _distanceSensors.rotationYaw315.valueString : _noValueStr

    property var    rgRotationValues:           [ rotationNoneValue, rotationYaw45Value, rotationYaw90Value, rotationYaw135Value, rotationYaw180Value, rotationYaw225Value, rotationYaw270Value, rotationYaw315Value ]
    property var    rgRotationValueStrings:     [ rotationNoneValueString, rotationYaw45ValueString, rotationYaw90ValueString, rotationYaw135ValueString, rotationYaw180ValueString, rotationYaw225ValueString, rotationYaw270ValueString, rotationYaw315ValueString ]

    property var    _distanceSensors:       vehicle ? vehicle.distanceSensors : null
    property string _noValueStr:            qsTr("–.––")

    // VAMA: a distance, or NaN when the direction has nothing in range. That is 0 (or
    // less) for a sensor of unknown range, and the sensor's maximum otherwise. The
    // maximum is compared to within half a centimetre: both come in whole centimetres.
    // QGC keeps the maximum of the last DISTANCE_SENSOR of any sensor, so beside a
    // rangefinder it can be the rangefinder's: then only the exact value is left out,
    // never a nearer one.
    function _reading(fact) {
        var distance = fact ? fact.value : NaN
        if (!isFinite(distance) || distance <= 0) {
            return NaN
        }
        if (isFinite(maxDistance) && maxDistance > 0 && Math.abs(distance - maxDistance) < 0.005) {
            return NaN
        }
        return distance
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
