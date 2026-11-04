"""VAMA flight recorder: step 1 of the GPS-denied navigation work (item A8).

It runs on a Raspberry Pi on the drone. During normal day flights it saves the picture of a camera
that looks straight down, once a second, with the drone's GPS position, heights and angles at the
moment of each picture. These flights are the data to train and test the picture matching on.

Run it as ``python -m vama_recorder`` from the folder that holds this package. See RECORDER.md.
"""

__version__ = "1.2.0"  # 1.1.0 (2026-11-04): no GPS receiver (fix 0) is a PROBLEM in the bench check, and a warning in flight
# 1.2.0 (2026-11-04): without a heartbeat the bench check says what arrives on the port (nothing, GPS data, MAVLink
# at another speed, other data): on the client's board the Pi and the GPS shared one port
