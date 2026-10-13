#include "VehicleFollowDrone.h"

#include <cmath>

#include "Fact.h"
#include "LinkInterface.h"
#include "MAVLinkProtocol.h"
#include "MultiVehicleManager.h"
#include "ParameterManager.h"
#include "Vehicle.h"
#include "VehicleLinkManager.h"

using skydroid::follow::Drone;
using skydroid::follow::Mode;

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

FollowDrone *vamaMakeFollowDrone(QObject *parent)
{
    return new VehicleFollowDrone(parent);
}

VehicleFollowDrone::VehicleFollowDrone(QObject *parent)
    : FollowDrone(parent)
{
    MultiVehicleManager *manager = MultiVehicleManager::instance();
    connect(manager, &MultiVehicleManager::activeVehicleChanged, this, &VehicleFollowDrone::_activeVehicleChanged);
    _activeVehicleChanged(manager->activeVehicle());
}

void VehicleFollowDrone::_activeVehicleChanged(Vehicle *vehicle)
{
    if (vehicle == _vehicle) {
        return;  // QGC names the active drone again, and it is the same one: nothing changes
    }
    if (_vehicle) {
        _vehicle->disconnect(this);
    }
    _vehicle = vehicle;
    // Nothing of the last drone counts for this one.
    _customMode = 0;
    _armed = false;
    _gps3d = false;
    if (_vehicle) {
        connect(_vehicle, &Vehicle::mavlinkMessageReceived, this, &VehicleFollowDrone::_message);
    }
    emit otherDrone();
}

bool VehicleFollowDrone::_isCopter() const
{
    return _vehicle && _vehicle->apmFirmware() && _vehicle->multiRotor();
}

Mode VehicleFollowDrone::_mode() const
{
    if (!_isCopter()) {
        return Mode::Other;
    }
    switch (_customMode) {
    case skydroid::follow::kCopterGuided:
        return Mode::Guided;
    case skydroid::follow::kCopterLoiter:
    case skydroid::follow::kCopterPosHold:
    case skydroid::follow::kCopterBrake:
        return Mode::Holding;
    default:
        return Mode::Other;
    }
}

void VehicleFollowDrone::_message(const mavlink_message_t &message)
{
    // The autopilot's own messages only: not a gimbal's or a companion computer's.
    if (!_vehicle || message.sysid != _vehicle->id() || message.compid != _vehicle->defaultComponentId()) {
        return;
    }
    switch (message.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t heartbeat;
        mavlink_msg_heartbeat_decode(&message, &heartbeat);
        _customMode = heartbeat.custom_mode;
        _armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t gps;
        mavlink_msg_gps_raw_int_decode(&message, &gps);
        _gps3d = gps.fix_type >= GPS_FIX_TYPE_3D_FIX;
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t position;
        mavlink_msg_global_position_int_decode(&message, &position);
        if (position.lat == 0 && position.lon == 0) {
            return;  // ArduPilot sends this without a position too, for the height alone
        }
        Drone drone;
        drone.latDeg = position.lat / 1e7;
        drone.lonDeg = position.lon / 1e7;
        drone.heightM = position.relative_alt / 1000.0;
        drone.northMps = position.vx / 100.0;
        drone.eastMps = position.vy / 100.0;
        if (position.hdg != UINT16_MAX) {
            drone.headingDeg = position.hdg / 100.0;
        }
        drone.armed = _armed;
        drone.flying = _vehicle->flying();
        drone.gps3d = _gps3d;
        drone.mode = _mode();
        emit telemetry(drone);
        break;
    }
    default:
        break;
    }
}

QString VehicleFollowDrone::whyNot() const
{
    if (!_vehicle) {
        return tr("Follow cannot start: no drone is connected.");
    }
    if (!_isCopter()) {
        return tr("Follow works with an ArduPilot multicopter only.");
    }
    return QString();
}

QString VehicleFollowDrone::modeName() const
{
    return _vehicle ? _vehicle->flightMode() : QString();
}

void VehicleFollowDrone::askGuided()
{
    if (!_isCopter()) {
        return;
    }
    // What QGC itself sends for a mode change, with ArduCopter's number for Guided. No error
    // box from QGC: Follow says itself when the drone did not change (Why::NotGuided).
    _vehicle->sendMavCommand(_vehicle->defaultComponentId(), MAV_CMD_DO_SET_MODE, false,
                             MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, skydroid::follow::kCopterGuided);
}

void VehicleFollowDrone::sendSpeed(double northMps, double eastMps, double headingDeg)
{
    if (!_isCopter()) {
        return;
    }
    const SharedLinkInterfacePtr link = _vehicle->vehicleLinkManager()->primaryLink().lock();
    if (!link) {
        return;
    }
    mavlink_set_position_target_local_ned_t target{};
    target.target_system = static_cast<uint8_t>(_vehicle->id());
    target.target_component = static_cast<uint8_t>(_vehicle->defaultComponentId());
    target.coordinate_frame = static_cast<uint8_t>(skydroid::follow::kSpeedFrame);
    // A speed that is not a number never goes out as one: the drone is told to stand.
    const bool sane = std::isfinite(northMps) && std::isfinite(eastMps);
    target.vx = sane ? static_cast<float>(northMps) : 0.0f;
    target.vy = sane ? static_cast<float>(eastMps) : 0.0f;
    target.vz = 0.0f;  // the height is kept
    if (std::isfinite(headingDeg)) {
        // The heading to keep, as a yaw angle from north, -pi to pi.
        target.type_mask = static_cast<uint16_t>(skydroid::follow::kSpeedTypeMask);
        target.yaw = static_cast<float>(std::remainder(headingDeg, 360.0) * kPi / 180.0);
    } else {
        target.type_mask = static_cast<uint16_t>(skydroid::follow::kSpeedTypeMaskNoHeading);
        target.yaw_rate = 0.0f;
    }
    mavlink_message_t message{};
    mavlink_msg_set_position_target_local_ned_encode_chan(static_cast<uint8_t>(MAVLinkProtocol::instance()->getSystemId()),
                                                         static_cast<uint8_t>(MAVLinkProtocol::getComponentId()),
                                                         link->mavlinkChannel(), &message, &target);
    (void) _vehicle->sendMessageOnLinkThreadSafe(link.get(), message);
}

double VehicleFollowDrone::fliesOnS() const
{
    // ArduCopter flies the last speed for GUID_TIMEOUT seconds, then stops.
    ParameterManager *parameters = _vehicle ? _vehicle->parameterManager() : nullptr;
    const QString name = QStringLiteral("GUID_TIMEOUT");
    if (!parameters || !parameters->parameterExists(ParameterManager::defaultComponentId, name)) {
        return -1.0;
    }
    bool ok = false;
    const double seconds = parameters->getParameter(ParameterManager::defaultComponentId, name)->rawValue().toDouble(&ok);
    return (ok && std::isfinite(seconds) && seconds >= 0.0) ? seconds : -1.0;
}
