# Host test for SkydroidLink: the real Qt code against an in-process fake
# camera, with stand-ins for QGC's vehicle classes (stubs/).
# Run with apk/custom/test/run_link_test.ps1.

cmake_minimum_required(VERSION 3.25)
project(vama_link_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Network Qml Positioning Test)

# Overridable so a mutation check can build against a modified copy.
set(SKYDROID_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src/Skydroid CACHE PATH "Folder with the Skydroid sources")

add_executable(skydroid_link_test
    skydroid_link_test.cc
    ${SKYDROID_SRC}/SkydroidLink.cc
    ${SKYDROID_SRC}/SkydroidLink.h
    ${SKYDROID_SRC}/SkydroidTop.cc
    ${SKYDROID_SRC}/LaserGeo.cc
    # Listed so AUTOMOC sees their signals.
    stubs/MultiVehicleManager.h
    stubs/Vehicle.h
)

# The stubs come first so they replace QGC's headers.
target_include_directories(skydroid_link_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    ${SKYDROID_SRC}
)

target_link_libraries(skydroid_link_test PRIVATE
    Qt6::Core Qt6::Network Qt6::Qml Qt6::Positioning Qt6::Test
)
