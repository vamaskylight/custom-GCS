# ============================================================================
# VAMA GCS: branding and feature set
# Started from QGC's custom-example (v5.1.5).
# ============================================================================

# ----------------------------------------------------------------------------
# Application Branding
# ----------------------------------------------------------------------------
# QGC_APP_NAME becomes the CMake project and target name, so it must not
# contain spaces. The name shown on the phone is the label in
# android/AndroidManifest.xml ("VAMA GCS").
set(QGC_APP_NAME "VAMA-GCS" CACHE STRING "App Name" FORCE)
set(QGC_APP_DESCRIPTION "VAMA ground control station" CACHE STRING "Application description" FORCE)
set(QGC_ORG_NAME "VAMA" CACHE STRING "Organization name" FORCE)
set(QGC_ORG_DOMAIN "vama.local" CACHE STRING "Organization domain" FORCE)
# The Android package id is permanent once the app is installed on customer
# devices: changing it later makes it a different app. Confirm before release.
set(QGC_PACKAGE_NAME "com.vama.gcs" CACHE STRING "Package identifier" FORCE)
set(QGC_ANDROID_PACKAGE_NAME "com.vama.gcs" CACHE STRING "Android package identifier" FORCE)

# ----------------------------------------------------------------------------
# Custom Icons and Graphics
# ----------------------------------------------------------------------------

# macOS Icon
if(EXISTS "${CMAKE_SOURCE_DIR}/${QGC_CUSTOM_DIR}/res/icons/custom_qgroundcontrol.icns")
    set(QGC_MACOS_ICON_PATH "${CMAKE_SOURCE_DIR}/${QGC_CUSTOM_DIR}/res/icons/custom_qgroundcontrol.icns" CACHE FILEPATH "MacOS Icon Path" FORCE)
endif()

# Linux AppImage Icon
if(EXISTS "${CMAKE_SOURCE_DIR}/${QGC_CUSTOM_DIR}/res/icons/custom_qgroundcontrol.svg")
    set(QGC_APPIMAGE_ICON_SCALABLE_PATH "${CMAKE_SOURCE_DIR}/${QGC_CUSTOM_DIR}/res/icons/custom_qgroundcontrol.svg" CACHE FILEPATH "AppImage Icon SVG Path" FORCE)
endif()

# Windows Installer Header
if(EXISTS "${CMAKE_SOURCE_DIR}/${QGC_CUSTOM_DIR}/deploy/windows/installheader.bmp")
    set(QGC_WINDOWS_INSTALL_HEADER_PATH "${CMAKE_SOURCE_DIR}/${QGC_CUSTOM_DIR}/deploy/windows/installheader.bmp" CACHE FILEPATH "Windows Install Header Path" FORCE)
endif()

# Windows Application Icon
if(EXISTS "${CMAKE_SOURCE_DIR}/${QGC_CUSTOM_DIR}/deploy/windows/WindowsQGC.ico")
    set(QGC_WINDOWS_ICON_PATH "${CMAKE_SOURCE_DIR}/${QGC_CUSTOM_DIR}/deploy/windows/WindowsQGC.ico" CACHE FILEPATH "Windows Icon Path" FORCE)
endif()

# ----------------------------------------------------------------------------
# Feature Set Customization
# ----------------------------------------------------------------------------

# Our drones run ArduPilot. Keep QGC's own ArduPilot support (all its flight
# modes and setup pages) and turn PX4 off, so the app shows one flight stack.
# All firmware plugin code is still compiled; the UI adapts at runtime via
# FirmwarePluginManager::supportedFirmwareClasses().
set(QGC_DISABLE_APM_PLUGIN_FACTORY OFF CACHE BOOL "Disable APM Plugin Factory" FORCE)
set(QGC_DISABLE_PX4_PLUGIN_FACTORY ON CACHE BOOL "Disable PX4 Plugin Factory" FORCE)
