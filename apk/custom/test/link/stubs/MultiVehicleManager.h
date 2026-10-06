#pragma once

// Host-test stand-in for QGC's MultiVehicleManager.

#include <QtCore/QObject>

class Vehicle;

class MultiVehicleManager : public QObject
{
    Q_OBJECT
    Q_MOC_INCLUDE("Vehicle.h")

public:
    static MultiVehicleManager *instance()
    {
        static MultiVehicleManager manager;
        return &manager;
    }
    Vehicle *activeVehicle() const { return _active; }
    void setActiveVehicle(Vehicle *vehicle)
    {
        _active = vehicle;
        emit activeVehicleChanged(vehicle);
    }

signals:
    void activeVehicleChanged(Vehicle *activeVehicle);

private:
    Vehicle *_active = nullptr;
};
