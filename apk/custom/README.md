# VAMA GCS custom build

This folder is our QGroundControl custom build.
It started from QGC's `custom-example` (v5.1.5) and was changed for VAMA:

- ArduPilot only. QGC's own ArduPilot support is on, PX4 is off.
- App name "VAMA GCS" (build name `VAMA-GCS`), Android package `com.vama.gcs`.
- Icons and logo come from `vgcs/assets/Vama Logo.png`. Regenerate them with `python apk/make_icons.py`.
- The example's PX4 plugins, PerimeterScan mission item, demo button and custom instrument panel were removed, so the normal QGC fly view shows.

Still from the example, for later use: the `Custom.Widgets` QML module in `res/Custom/Widgets` and the colour palette in `src/CustomPlugin.cc`.

Build steps and rules are in `apk/README.md`.
