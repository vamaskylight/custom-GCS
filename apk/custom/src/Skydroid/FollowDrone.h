#pragma once

// The drone, as far as Follow Me needs it: telemetry in, a speed out.
// The app's is VehicleFollowDrone (QGC's active vehicle). The host tests give
// ObjectFollow a stand-in.

#include <QtCore/QObject>
#include <QtCore/QString>

#include "FollowGuide.h"

class FollowDrone : public QObject
{
    Q_OBJECT

public:
    using QObject::QObject;

    /// Why Follow cannot fly this drone at all (no drone, or not an ArduPilot multicopter). Empty: it can.
    virtual QString whyNot() const = 0;
    /// The name of its flight mode now, for a message.
    virtual QString modeName() const = 0;
    /// Asks the drone to change to Guided, the mode it takes speeds in.
    virtual void askGuided() = 0;
    /// A speed over the ground, north and east, m/s, and the heading to keep, degrees from north
    /// (not a number: the heading it has). The drone keeps its height.
    virtual void sendSpeed(double northMps, double eastMps, double headingDeg) = 0;
    /// How long the drone flies on after the last speed it got, seconds (GUID_TIMEOUT). Negative: not known.
    virtual double fliesOnS() const = 0;

signals:
    /// Where the drone is, each time it says so.
    void telemetry(const skydroid::follow::Drone &drone);
    /// This is another drone now, or none: nothing told before holds for it.
    void otherDrone();
};

/// The drone of the app: QGC's active vehicle (VehicleFollowDrone.cc). The host tests give their own.
FollowDrone *vamaMakeFollowDrone(QObject *parent);
