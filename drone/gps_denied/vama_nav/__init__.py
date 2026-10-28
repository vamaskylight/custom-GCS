"""Our position into the flight controller as its second GPS (GPS-denied navigation, step 3).

The down camera's pictures are matched with a map on board (``--map``, matcher.py): each picture
turned north up with the drone's angles and height, its features paired with the map around where
the flight controller thinks the drone is, and the place of the point below the drone is the
position. Ten times a second the program sends GPS_INPUT to ArduPilot (GPS 2, GPSn_TYPE 14). While
the drone is disarmed it passes the real GPS's own fix on, because ArduPilot arms only when every
GPS it knows has a 3D fix, and a camera on the ground matches nothing. From arming on, GPS 2 carries
only the matched positions, carried forward to the moment of sending, or "no fix". ArduPilot uses
GPS 2 only while the real GPS has no 3D fix (GPS_AUTO_SWITCH 4).

In the ArduCopter simulator there is no camera: ``--sim-camera`` draws the pictures from a map of
another year at the simulator's true position, and ``--sim-truth`` (step 3a's stand-in) sends the
true position with noise. ``python -m vama_nav.replay`` is step 2: the matching on recorded flights.
"""

__version__ = "1.1.0"
