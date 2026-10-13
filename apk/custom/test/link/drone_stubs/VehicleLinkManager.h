#pragma once

// Host-test stand-in for QGC's VehicleLinkManager: the link commands go out on.

#include "LinkInterface.h"

class VehicleLinkManager
{
public:
    WeakLinkInterfacePtr primaryLink() { return link; }

    SharedLinkInterfacePtr link;  // none: the drone's link is gone
};
