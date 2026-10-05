#pragma once

// Host-test stand-in for QGC's MultiVehicleManager.

class Vehicle;

class MultiVehicleManager
{
public:
    static MultiVehicleManager *instance()
    {
        static MultiVehicleManager manager;
        return &manager;
    }
    Vehicle *activeVehicle() const { return _active; }
    void setActiveVehicle(Vehicle *vehicle) { _active = vehicle; }

private:
    Vehicle *_active = nullptr;
};
