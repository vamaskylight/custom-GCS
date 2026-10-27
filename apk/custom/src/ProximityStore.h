#pragma once

#include <QtCore/QElapsedTimer>
#include <QtCore/QHash>
#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <QtCore/QVariantList>
#include <QtQml/qqmlregistration.h>

#include <functional>

class QJSEngine;
class QQmlEngine;

/// One drone's obstacle sensor as the radar on the map and on the video draw it (A7, 2026-10-27).
///
/// ArduPilot sends its proximity sensor (the client's RPLidar A3, a TF02 Pro looking ahead) as one
/// DISTANCE_SENSOR per direction: ids 10 to 17 (PROXIMITY_SENSOR_ID_START), orientation 0 to 7, only
/// while the sensor is good, an empty direction as the sensor's range, and that range is 0 for a
/// sensor ArduPilot knows no range for (the A3). QGC keeps DISTANCE_SENSOR by direction only
/// (VehicleDistanceSensorFactGroup) and never clears a value: when the LiDAR stops, the arcs froze
/// where they were; its range was that of the last DISTANCE_SENSOR of any sensor, so a downward
/// rangefinder beside the A3 changed the range circle; and an empty direction was a value ("0.00"
/// in the middle of the video for the A3).
///
/// Here only the proximity sensor's own messages count, each with its own range, an empty
/// direction has no value (NaN), and a direction not heard for kMaxAgeMs has none either.
class ProximitySectors : public QObject
{
    Q_OBJECT
    QML_ANONYMOUS

    /// True once a direction was heard: the radar is shown from then on.
    Q_PROPERTY(bool telemetryAvailable READ telemetryAvailable NOTIFY changed)
    /// The 8 directions in metres (0 ahead, then every 45 degrees clockwise). NaN: nothing in range.
    Q_PROPERTY(QVariantList values READ values NOTIFY changed)
    /// The sensor's range in metres, 0 when ArduPilot does not know it.
    Q_PROPERTY(double maxDistance READ maxDistance NOTIFY changed)

public:
    static constexpr int kDirections = 8;
    static constexpr int kFirstId = 10;
    static constexpr qint64 kMaxAgeMs = 2000;

    explicit ProximitySectors(QObject *parent = nullptr);

    bool telemetryAvailable() const { return _heard; }
    QVariantList values() const;
    double value(int direction) const;
    double maxDistance() const { return _maxM; }

    /// One DISTANCE_SENSOR. True when it was a direction of the proximity sensor (else nothing changes).
    bool hear(int sensorId, int orientation, int currentCm, int maxCm, qint64 nowMs);
    /// Directions not heard for kMaxAgeMs lose their value.
    void expire(qint64 nowMs);

signals:
    void changed();

private:
    double _metres[kDirections];
    qint64 _heardMs[kDirections];
    double _maxM = 0.0;
    bool _heard = false;
};

/// The app's obstacle sensors, one ProximitySectors for each drone, fed by CustomPlugin::mavlinkMessage.
/// QML: ProximityStore.forVehicle(vehicle.id) (src/ProximityRadarValues.qml).
class ProximityStore : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON

public:
    static constexpr int kExpiryCheckMs = 500;

    /// No default for the parent, on purpose: QML makes a singleton with a constructor that needs no
    /// argument when there is one, instead of using create() (see SkydroidLink.h, tasks X and V).
    explicit ProximityStore(QObject *parent);

    /// The app's one store. QML gets it too (create()).
    static ProximityStore *instance();
    static ProximityStore *create(QQmlEngine *qmlEngine, QJSEngine *jsEngine);

    /// The drone with this MAVLink system id (QGC's Vehicle::id()), made at the first ask.
    Q_INVOKABLE ProximitySectors *forVehicle(int vehicleId);

    /// One DISTANCE_SENSOR of that drone.
    void hear(int vehicleId, int sensorId, int orientation, int currentCm, int maxCm);
    /// Clears what was not heard lately. The store's timer calls it every kExpiryCheckMs.
    void expireAll();
    /// For tests: the clock, in milliseconds.
    void setClock(std::function<qint64()> clock) { _clock = std::move(clock); }

private:
    qint64 _now() const;

    QHash<int, ProximitySectors *> _vehicles;
    QTimer _expiry;
    QElapsedTimer _elapsed;
    std::function<qint64()> _clock;
};
