"""VAMA flight recorder: step 1 of the GPS-denied navigation work (item A8).

It runs on a Raspberry Pi on the drone. During normal day flights it saves the picture of a camera
that looks straight down, once a second, with the drone's GPS position, heights and angles at the
moment of each picture. These flights are the data to train and test the picture matching on.

Run it as ``python -m vama_recorder`` from the folder that holds this package. See RECORDER.md.
"""

__version__ = "1.0.0"
