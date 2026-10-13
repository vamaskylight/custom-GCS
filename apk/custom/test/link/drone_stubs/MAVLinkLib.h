#pragma once

// Host-test stand-in for QGC's MAVLinkLib.h: the real MAVLink headers of the
// "all" dialect, the ones the app is built with, as they come.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include <mavlink.h>
#pragma GCC diagnostic pop
