"""Our position into the flight controller as its second GPS (GPS-denied navigation, step 3a).

Ten times a second the program sends GPS_INPUT to ArduPilot (GPS 2, GPSn_TYPE 14). While the drone
is disarmed it passes the real GPS's own fix on, because ArduPilot arms only when every GPS it knows
has a 3D fix, and a camera on the ground matches nothing. From arming on, GPS 2 carries only the
camera's positions, carried forward to the moment of sending, or "no fix". ArduPilot uses GPS 2 only
while the real GPS has no 3D fix (GPS_AUTO_SWITCH 4).

The positions come from the picture matching (step 3, not written yet). For now the only source is
the ArduCopter simulator's true position with noise added (--sim-truth), to prove the way into
ArduPilot.
"""

__version__ = "1.0.0"
