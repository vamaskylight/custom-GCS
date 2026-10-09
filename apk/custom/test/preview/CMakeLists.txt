# Preview of the VAMA camera screen on this PC: the real FlyViewCustomLayer.qml
# and SkydroidLink, with stand-ins for QGC (stubs/ and ../link/stubs) and a
# fake camera. Then Application Settings, with the VAMA copy of QGC's
# AppSettings.qml. Saves screenshots. Run with apk/custom/test/run_preview.ps1.

cmake_minimum_required(VERSION 3.25)
project(vama_preview LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Network Qml Quick QuickControls2 Positioning Svg)

set(CUSTOM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(QGC_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../../qgc-src/src)
set(LINK_STUBS ${CMAKE_CURRENT_SOURCE_DIR}/../link/stubs)

add_executable(vama_preview
    main.cpp
    ${CUSTOM_DIR}/src/Skydroid/SkydroidLink.cc
    ${CUSTOM_DIR}/src/Skydroid/SkydroidLink.h
    ${CUSTOM_DIR}/src/Skydroid/SkydroidTop.cc
    ${CUSTOM_DIR}/src/Skydroid/LaserGeo.cc
    ${CUSTOM_DIR}/src/Skydroid/ObjectTracker.cc
    ${LINK_STUBS}/MultiVehicleManager.h
    ${LINK_STUBS}/Vehicle.h
    # The app's own resources: the QML override path and the icons.
    ${CUSTOM_DIR}/custom.qrc
    # QGC's tinting for QGCColoredImage, as in the app.
    ${QGC_SRC}/QmlControls/ColoredSvgImageProvider.cc
    ${QGC_SRC}/QmlControls/ColoredSvgImageProvider.h
    # QGC's icons of Application Settings, and stand-ins for two of its pages.
    preview.qrc
)

target_include_directories(vama_preview PRIVATE ${LINK_STUBS} ${CUSTOM_DIR}/src/Skydroid ${QGC_SRC}/QmlControls)
target_compile_definitions(vama_preview PRIVATE
    VAMA_PREVIEW_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(vama_preview PRIVATE
    Qt6::Core Qt6::Gui Qt6::Network Qt6::Qml Qt6::Quick Qt6::QuickControls2 Qt6::Positioning Qt6::Svg
)
