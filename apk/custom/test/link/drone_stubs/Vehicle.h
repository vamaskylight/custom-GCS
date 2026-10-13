#pragma once

// Host-test stand-in for QGC's Vehicle, as VehicleFollowDrone uses it: who the
// drone is, its MAVLink messages in, and what is sent to it kept for the test.

#include <QtCore/QList>
#include <QtCore/QObject>
#include <QtCore/QString>

#include "MAVLinkLib.h"
#include "ParameterManager.h"
#include "VehicleLinkManager.h"

class Vehicle : public QObject
{
    Q_OBJECT

public:
    struct Command
    {
        int component;
        MAV_CMD command;
        bool showError;
        float param1;
        float param2;
    };

    Vehicle() { links.link = std::make_shared<LinkInterface>(); }

    int id() const { return systemId; }
    int defaultComponentId() const { return componentId; }
    bool apmFirmware() const { return ardupilot; }
    bool multiRotor() const { return copter; }
    bool flying() const { return inTheAir; }
    QString flightMode() const { return mode; }
    VehicleLinkManager *vehicleLinkManager() { return &links; }
    ParameterManager *parameterManager() { return &parameters; }

    void sendMavCommand(int compId, MAV_CMD command, bool showError, float param1 = 0.0f, float param2 = 0.0f,
                        float param3 = 0.0f, float param4 = 0.0f, float param5 = 0.0f, float param6 = 0.0f,
                        float param7 = 0.0f)
    {
        Q_UNUSED(param3);
        Q_UNUSED(param4);
        Q_UNUSED(param5);
        Q_UNUSED(param6);
        Q_UNUSED(param7);
        commands.append({compId, command, showError, param1, param2});
    }
    bool sendMessageOnLinkThreadSafe(LinkInterface *link, mavlink_message_t message)
    {
        sentOn.append(link);
        sent.append(message);
        return true;
    }

    /// What QGC does with a message from this drone.
    void receive(const mavlink_message_t &message) { emit mavlinkMessageReceived(message); }

    int systemId = 1;
    int componentId = 1;
    bool ardupilot = true;
    bool copter = true;
    bool inTheAir = true;
    QString mode = QStringLiteral("Loiter");
    VehicleLinkManager links;
    ParameterManager parameters;
    QList<Command> commands;
    QList<mavlink_message_t> sent;
    QList<LinkInterface *> sentOn;

signals:
    void mavlinkMessageReceived(const mavlink_message_t &message);
};
