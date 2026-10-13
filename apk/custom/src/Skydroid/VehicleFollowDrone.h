#pragma once

// The drone of Follow Me in the app: QGC's active vehicle, when it is an
// ArduPilot multicopter.
//
// Telemetry comes straight from the vehicle's MAVLink messages:
//  - GLOBAL_POSITION_INT: position, height above the take-off point, speed over
//    the ground and heading, in one message of one moment.
//  - HEARTBEAT: armed, and ArduCopter's own mode number. The number, not QGC's
//    mode name, which is translated with the app's language.
//  - GPS_RAW_INT: the kind of GPS fix.
// "Flying" is QGC's own (Vehicle::flying, from the drone's landed state).
//
// The speed goes out as SET_POSITION_TARGET_LOCAL_NED, as test/follow_sitl_test.py
// flies it in ArduCopter's simulator: see kSpeedTypeMask in FollowGuide.h.

#include <QtCore/QPointer>

#include "FollowDrone.h"
#include "MAVLinkLib.h"

class Vehicle;

class VehicleFollowDrone : public FollowDrone
{
    Q_OBJECT

public:
    explicit VehicleFollowDrone(QObject *parent = nullptr);

    QString whyNot() const override;
    QString modeName() const override;
    void askGuided() override;
    void sendSpeed(double northMps, double eastMps, double headingDeg) override;
    double fliesOnS() const override;

private:
    // Plain functions, not slots: a slot taking Vehicle* would need the full Vehicle class here.
    void _activeVehicleChanged(Vehicle *vehicle);
    void _message(const mavlink_message_t &message);
    bool _isCopter() const;
    skydroid::follow::Mode _mode() const;

    QPointer<Vehicle> _vehicle;
    quint32 _customMode = 0;  // 0 is Stabilize: no mode Follow starts from
    bool _armed = false;
    bool _gps3d = false;
};
