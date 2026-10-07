# VAMA GCS custom build

This folder is our QGroundControl custom build.
It started from QGC's `custom-example` (v5.1.5) and was changed for VAMA:

- ArduPilot only. QGC's own ArduPilot support is on, PX4 is off.
- App name "VAMA GCS" (build name `VAMA-GCS`), Android package `com.vama.gcs`.
- Icons and logo come from `vgcs/assets/Vama Logo.png`. Regenerate them with `python apk/make_icons.py`.
- The example's PX4 plugins, PerimeterScan mission item, demo button and custom instrument panel were removed, so the normal QGC fly view shows.

VAMA additions:

- `src/Skydroid/`: the Skydroid camera link (protocol, UDP link, laser target maths). See the comment at the top of `SkydroidLink.h` for what the field taught.
- `src/FlyViewCustomLayer.qml`: the camera screen (replaces QGC's empty file of the same name). Icons are `res/Images/vama_*.svg`.
- `src/FlyViewTopRightColumnLayout.qml`: replaces QGC's file to remove QGC's own photo and video buttons, which send MAVLink camera commands the Skydroid camera does not use.
- `src/SelectViewDropdown.qml`: QGC's app menu with the VAMA mark on the Settings button and VAMA version text. QGC tints menu icons through its `coloredsvg` image provider, which reads `/res/QGCLogoWhite.svg` directly, so only a copy of this file can change that icon. Change the "Test build" line for every APK sent out.
- QGC's other logos are replaced by files with the same name under `/Custom/res` in `custom.qrc` (`QGCLogoFull.svg`, and `QGCLogoArrow.svg` for the phone position on the map). All logo files come from `apk/make_icons.py`.
- `src/AppSettings.qml`: a copy of QGC's Application Settings screen (client feedback 2026-10-07). The General page shows the VAMA mark (QGC tints it through `coloredsvg` too), and the Help page with QGC's links is left out.
- `src/FlightDisplayViewVideoOutput.qml`: a copy of QGC's video picture with the thermal colour modes of VGCS (client request 2026-10-07). While IR shows the thermal stream, the picture is drawn through a shader (`res/shaders/thermal_palette.frag`) that looks up each grey level in `res/Images/thermal_palettes.png`. The table, the compiled shader (`.qsb`, so the build needs no shader tools) and the test data come from `python apk/make_thermal_palettes.py`, which reads VGCS's own colour tables. The colour button beside IR and its list are in `src/FlyViewCustomLayer.qml`, the chosen mode in `SkydroidLink` (saved).
- Every copy of a QGC file marks its changes "VAMA". Compare each copy with QGC's file again after moving to a new QGC release.
- `src/CustomPlugin.cc`: RC channel stream default 10 Hz (QGC asks for 2), because the RC wheel moves the camera.

QML files replace QGC's through `custom.qrc` (prefix `/Custom/qml`, same path as QGC's file). They are not compiled at build time, so a mistake shows only when the app runs. Run `test/run_preview.ps1` after every QML change: it loads the real QML on this PC and prints every QML warning.

Still from the example, for later use: the `Custom.Widgets` QML module in `res/Custom/Widgets` and the colour palette in `src/CustomPlugin.cc`.

Build steps and rules are in `apk/README.md`.
