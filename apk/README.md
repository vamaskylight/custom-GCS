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
  build_android.ps1 builds the signed APK (arm64-v8a and armeabi-v7a)
  build_ncnn.ps1    builds the ncnn library for the object finder (build_android.ps1 runs it)
  make_icons.py     makes every icon and logo from vgcs/assets/Vama Logo.png
  make_thermal_palettes.py  makes the thermal colour table and shader from VGCS's colour modes
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
| ncnn | release 20260526 (for the object finder; our choice, not QGC's) |

The app needs **Android 9 or newer** (minimum SDK 28).
Customer devices run Android 10 to 14 and newer, so this is fine.

## Build the APK

```powershell
powershell -ExecutionPolicy Bypass -File apk\build_android.ps1
```

It uses the same CMake settings as QGC's own Android CI on a Windows host
(`apk/qgc-src/.github/workflows/android.yml`).
The APK is built for both 64 bit (`arm64-v8a`) and 32 bit (`armeabi-v7a`) devices.
Use `-Abis arm64-v8a` for a faster test build.

The repo path has a space and Android builds make very deep paths, so the build runs in `C:\vama-apk`:

- `C:\vama-apk\src` is a link to `apk\qgc-src`.
- `C:\vama-apk\build-Release` holds the build output and the APK.

The app marks people and vehicles with a small neural network, which the ncnn library runs (see `custom/README.md`).
`build_android.ps1` builds that library first, with `apk\build_ncnn.ps1`:

- It downloads one fixed release of the library's source from GitHub (23 MB, once) and checks it against the hash its makers published.
- It builds the library for each ABI, with the same NDK, Android level and C++ library as the app (about 2 minutes, once).
- Everything is kept in `C:\vama-apk\ai`. A library that is there already is left alone.

A build stops when the library is missing, and the finished APK is checked for the finder in each ABI.
`-NoFinder` builds an app without it: such an app does not mark people and vehicles, and says so.

Test builds are signed with a development key, `%USERPROFILE%\.android\vama-dev.keystore`, created on the first build.
The release key for customers is a separate key (see Signing keys).

Tools on the dev PC: Qt in `C:\Qt\6.11.1`, JDK 21 in `%LOCALAPPDATA%\Programs\Eclipse Adoptium`, the Android SDK in `%LOCALAPPDATA%\Android\Sdk` with NDK 27.2.12479018, and CMake and Ninja from the Android SDK.

Qt was installed with the aqtinstall commit that QGC pins for Windows hosts (`apk/qgc-src/.github/scripts/android_matrix.py`), because older aqtinstall does not know the Qt 6.11 Windows folder layout.

QGC's custom build guide: https://dev.qgroundcontrol.com/en/custom_build/custom_build.html

### A build for customers

```powershell
powershell -ExecutionPolicy Bypass -File apk\make_release_key.ps1
powershell -ExecutionPolicy Bypass -File apk\build_android.ps1 -Customer -Version 1.0.0 -CheckOnly
powershell -ExecutionPolicy Bypass -File apk\build_android.ps1 -Customer -Version 1.0.0
```

1. Make the release key once (`make_release_key.ps1`), and back up its two files (see Signing keys).
2. Set the app menu text in `apk/custom/src/SelectViewDropdown.qml` to "Version 1.0.0", and commit everything under `apk/`.
3. `-CheckOnly` says what is still missing, and builds nothing.
4. The build runs in its own folder, `C:\vama-apk\build-Customer` (about 9 GB; the first build there takes longer than a test build).
5. The result is `apk\build\VAMA-GCS-1.0.0.apk`. The script proves that it is signed with the release key, and prints its SHA-256.

What is different from a test build:

- It is signed with the release key, not the development key. Android does not install it over a test build: the test build is uninstalled first, and its settings go with it.
- It is QGC's stable build (`QGC_STABLE_BUILD=ON`). The app's own name is "VAMA-GCS" without "Daily", so its settings and its folder on the device are new.
- It is made from committed files only, and only when the app menu names the version. A customer never gets an app that says "Test build".
- A version is built once: the script stops when `apk\build\VAMA-GCS-<version>.apk` is already there.
- Android's own app info still shows QGC's version (5.1.5), with the same version code as the test builds. The VAMA version is the text in the app menu. A later customer build installs over an earlier one, because both are signed with the release key.

The customer build was prepared on 2026-10-13 and its checks were tried with a throwaway key. A whole customer build has not been run yet.

## Tests on this PC (no camera or phone needed)

```powershell
powershell -ExecutionPolicy Bypass -File apk\custom\test\run_tests.ps1
powershell -ExecutionPolicy Bypass -File apk\custom\test\run_link_test.ps1
powershell -ExecutionPolicy Bypass -File apk\custom\test\run_preview.ps1
powershell -ExecutionPolicy Bypass -File apk\custom\test\run_android_link_check.ps1
```

- `run_tests.ps1` checks the camera protocol (`SkydroidTop`) and the laser target maths (`LaserGeo`) against values made by the VGCS Python code. After changing either side, regenerate them with `gen_skydroid_vectors.py` and `gen_laser_vectors.py` (run with `py -3.14`, which has the VGCS packages).
- `run_link_test.ps1` runs the real camera link (`SkydroidLink`) against fake cameras on 127.0.0.1, with stand-ins for QGC's vehicle classes (`test/link/stubs`): angles, address search, touch and RC wheel motion, laser and target rules.
- `run_preview.ps1` loads the real camera screen (`FlyViewCustomLayer.qml` from `custom.qrc`) with stand-ins for QGC's QML controls (`test/preview/stubs`) and a fake camera, prints every QML warning, and saves screenshots to `%TEMP%\vama-preview`. Run it after every QML change: the app only finds QML mistakes when it runs.
- The preview also opens our copies of QGC's files by QGC's own addresses, the way the app does: the video (thermal colours checked against VGCS's tables at 64 grey levels) and Application Settings. After changing VGCS's colour modes or the shader, run `python apk/make_thermal_palettes.py` (needs Qt's `qsb`, from the `msvc2022_64` kit).
- `run_link_test.ps1` also runs the object finder with its real network on three photos from the air (`marks_net_test`). That needs the ncnn library built for this PC, once: `powershell -ExecutionPolicy Bypass -File apk\build_ncnn.ps1 -ForHost -Abis ''`. Without it that test is left out, and the runner says so.
- `run_android_link_check.ps1` compiles the object finder with the NDK and links it with the ncnn library, for each ABI, with nothing left undefined. It proves that part of the APK without building one. It cannot prove that the code runs right on the phone's processor.
- They need MinGW 13.1 (`C:\Qt\Tools\mingw1310_64`) and, for the link test and the preview, the Qt 6.11.1 MinGW kit (`C:\Qt\6.11.1\mingw_64`).
- Tests and the preview use only 127.0.0.x addresses. `SkydroidLink::setProbeTargets` keeps the camera address search off the real network.

## What changed from the example

Done on 2026-10-06 (details in `apk/custom/README.md`):

1. ArduPilot only. QGC's own ArduPilot support is on, PX4 is off.
2. Name "VAMA GCS" (build name `VAMA-GCS`), Android package `com.vama.gcs`, VAMA icons.
3. Example parts removed: PX4 plugins, PerimeterScan, demo button, custom instrument panel.
4. Skydroid camera screen and link (`apk/custom/src`), and QGC's own photo and video buttons removed (see `apk/custom/README.md`).

The Android package id is permanent once customers install the app, so confirm it before the first release.

## Moving to a new QGC release

1. Change the tag in `qgc-version.txt`.
2. Remove the link first: `cmd /c rmdir apk\qgc-src\custom`
3. Delete `apk\qgc-src`.
4. Run `fetch_qgc.ps1` again.
5. Compare the new `custom-example` with `apk/custom`, and bring over what changed.
6. Compare our copies of QGC's QML files (`apk/custom/src`, listed in `apk/custom/README.md`) with the new release's files, and keep the parts marked "VAMA".

**Always remove the link before deleting `apk\qgc-src`.**
Some delete tools follow the link and would also delete `apk\custom`.
The same goes for `C:\vama-apk`: remove its link first with `cmd /c rmdir C:\vama-apk\src`.
Also do not use `git clean -x` in this repo.
It deletes every ignored folder, which includes `apk\qgc-src` and local folders such as `DOCS` and `tests`.

## Signing keys

Two keys, both outside the repo:

| Key | File | Used for |
|-----|------|----------|
| Development key | `%USERPROFILE%\.android\vama-dev.keystore`, made by the first test build | Test builds |
| Release key | `%USERPROFILE%\.vama\vama-release.keystore`, with its password in `vama-release.keystore.pass` beside it, made once by `apk\make_release_key.ps1` | Customer builds |

- Back up the release key and its password file as soon as they are made, in a place that is not this PC.
- If the release key is lost, no later version can be installed over a released app: every customer would have to uninstall first.
- Never put a key in git, and never send it to anyone. `.gitignore` blocks key files, and the keys stay outside the repo anyway.
- The release key is made once. `make_release_key.ps1` stops when its files are already there.

## License

QGC is dual licensed under Apache 2.0 and GPLv3.
We use it under Apache 2.0, which allows a white label app.

- Keep QGC's license and notice files in the app.
- Do not call the app "QGroundControl". That name is protected.
- Qt is used under its open source license (LGPL). Check its terms for the Android build.
- The object finder uses two more open source parts: the ncnn library (BSD 3-Clause, with parts under the zlib License) and the NanoDet-Plus-m network (Apache 2.0). Both allow use in a closed app, and both ask that their notices go with it. The notices are in `custom/res/ai/NOTICES.txt`, and the app shows them (camera settings, "Licences of the object finder"). Where the files come from: `custom/res/ai/MODEL_SOURCE.md`.
