#pragma once

// Host-test stand-in for QGC's Vehicle: position, the "gps" and "vehicle"
// fact groups, and the RC channel signal, all set directly by the test.

#include <QtCore/QObject>
#include <QtCore/QVector>
#include <QtPositioning/QGeoCoordinate>

#include "FactGroup.h"

class Vehicle : public QObject
{
    Q_OBJECT

public:
    QGeoCoordinate coordinate() { return position; }
    FactGroup *gpsFactGroup() { return &gps; }
    FactGroup *vehicleFactGroup() { return &vehicle; }

    /// What QGC does when an RC_CHANNELS message arrives.
    void sendRc(const QVector<int> &values) { emit rcChannelsRawChanged(values); }

    QGeoCoordinate position;
    FactGroup gps;
    FactGroup vehicle;

signals:
    void rcChannelsRawChanged(QVector<int> channelValues);
};
