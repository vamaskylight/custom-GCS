#pragma once

// Host-test stand-in for QGC's LinkInterface: a link is its MAVLink channel.

#include <cstdint>
#include <memory>

class LinkInterface
{
public:
    uint8_t mavlinkChannel() const { return channel; }

    uint8_t channel = 0;
};

typedef std::shared_ptr<LinkInterface> SharedLinkInterfacePtr;
typedef std::weak_ptr<LinkInterface> WeakLinkInterfacePtr;
