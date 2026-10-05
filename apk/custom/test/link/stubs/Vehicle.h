#pragma once

// Host-test stand-in for QGC's Vehicle: position plus the "gps" and
// "vehicle" fact groups, set directly by the test.

#include <QtPositioning/QGeoCoordinate>

#include "FactGroup.h"

class Vehicle
{
public:
    QGeoCoordinate coordinate() { return position; }
    FactGroup *gpsFactGroup() { return &gps; }
    FactGroup *vehicleFactGroup() { return &vehicle; }

    QGeoCoordinate position;
    FactGroup gps;
    FactGroup vehicle;
};
