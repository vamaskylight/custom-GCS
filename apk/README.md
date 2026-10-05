# VAMA APK (white label QGroundControl)

This folder holds the Android app for customers who do not need DOOAF.
It is a custom build of QGroundControl (QGC) with VAMA branding and Skydroid camera controls.

VGCS (the Windows exe with DOOAF) is the rest of this repo, in `vgcs/` and `packaging/`.
The two apps share no code, because VGCS is Python and QGC is C++.

## Layout

```text
apk/
  README.md         this file
  qgc-version.txt   the one QGC release we build on
  fetch_qgc.ps1     downloads that release into apk/qgc-src and links apk/custom into it
  custom/           OUR CODE: branding, Skydroid controls, screens (in git)
  qgc-src/          QGC source, downloaded by fetch_qgc.ps1 (gitignored, never committed)
  build/            build output (gitignored)
```

Edit only `apk/custom`.
Never edit `apk/qgc-src`, because it is replaced when we move to a new QGC release.

## How it works

QGC builds a custom build when its source has a folder named `custom`.
`fetch_qgc.ps1` makes `apk/qgc-src/custom` a directory junction (a link) to `apk/custom`.
So QGC builds our code, and every change is saved in `apk/custom`, which is in git.

`apk/custom` was first created from QGC's own `custom-example` at the pinned release.

## Get the source

From the repo root, in PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File apk\fetch_qgc.ps1
```

It is safe to run again.
It never deletes anything, and it stops with a message if something does not match.

## Build tools

These versions come from `apk/qgc-src/.github/build-config.json` for QGC v5.1.5.
Check that file again whenever `qgc-version.txt` changes.

| Tool | Version |
|------|---------|
| Qt | 6.11.1 (6.11.0 minimum), Android arm64 kit plus the desktop host kit |
| Android SDK platform | 36 |
| Android build tools | 36.0.0 |
| Android NDK | r27c (27.2.12479018) |
| Java (JDK) | 21 |
| CMake | 3.25 or newer |
| GStreamer for Android | 1.28.4 (for video) |

The app needs **Android 9 or newer** (minimum SDK 28).
Check the Android version of the RC or tablet before promising it to a customer.

The exact build commands are in QGC's own CI files:

- `apk/qgc-src/.github/workflows/android.yml` (Android build)
- `apk/qgc-src/.github/workflows/custom-build.yml` (custom build)

QGC's custom build guide: https://dev.qgroundcontrol.com/en/custom_build/custom_build.html

## First tasks in apk/custom

The example we started from is not ours yet.

1. It is set up for PX4. Our drones run ArduPilot, so the firmware plugin must be changed.
2. It is still named "Custom-QGroundControl" with package `org.mavlink.customqgroundcontrol`.
   Set the VAMA name and package in `custom/cmake/CustomOverrides.cmake`, and replace the icons.
3. Remove the example parts we do not need (for example the PerimeterScan mission item).

## Moving to a new QGC release

1. Change the tag in `qgc-version.txt`.
2. Remove the link first: `cmd /c rmdir apk\qgc-src\custom`
3. Delete `apk\qgc-src`.
4. Run `fetch_qgc.ps1` again.
5. Compare the new `custom-example` with `apk/custom`, and bring over what changed.

**Always remove the link before deleting `apk\qgc-src`.**
Some delete tools follow the link and would also delete `apk\custom`.
Also do not use `git clean -x` in this repo.
It deletes every ignored folder, which includes `apk\qgc-src` and local folders such as `DOCS` and `tests`.

## Signing keys

The Android release key (`.jks` or `.keystore`) must never be in git.
`.gitignore` blocks these files, but keep the key outside the repo anyway, with a backup.
If the key is lost, installed apps cannot be updated.

## License

QGC is dual licensed under Apache 2.0 and GPLv3.
We use it under Apache 2.0, which allows a white label app.

- Keep QGC's license and notice files in the app.
- Do not call the app "QGroundControl". That name is protected.
- Qt is used under its open source license (LGPL). Check its terms for the Android build.
