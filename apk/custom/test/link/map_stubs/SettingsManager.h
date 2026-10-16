#pragma once

// Stand-in for QGC's settings in map_pack_loader_test: the map choice only, as a fresh
// VAMA app has it (CustomPlugin: "Google", QGC's type "Hybrid").

#include <QtCore/QVariant>

class Fact
{
public:
    explicit Fact(const QVariant &value = {}) : _value(value) {}
    QVariant rawValue() const { return _value; }
    void setRawValue(const QVariant &value) { _value = value; }

private:
    QVariant _value;
};

class FlightMapSettings
{
public:
    Fact *mapProvider() { return &_provider; }
    Fact *mapType() { return &_type; }

private:
    Fact _provider{QStringLiteral("Google")};
    Fact _type{QStringLiteral("Hybrid")};
};

class SettingsManager
{
public:
    static SettingsManager *instance()
    {
        static SettingsManager manager;
        return &manager;
    }
    FlightMapSettings *flightMapSettings() { return &_flightMap; }

private:
    FlightMapSettings _flightMap;
};
