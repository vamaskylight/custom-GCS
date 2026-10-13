#pragma once

// Host-test stand-in for QGC's MAVLinkProtocol: who the ground station is in MAVLink.

#include "MAVLinkLib.h"

class MAVLinkProtocol
{
public:
    static MAVLinkProtocol *instance()
    {
        static MAVLinkProtocol protocol;
        return &protocol;
    }
    int getSystemId() const { return 255; }
    static int getComponentId() { return MAV_COMP_ID_MISSIONPLANNER; }
};
