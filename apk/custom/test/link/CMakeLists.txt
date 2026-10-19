# Host test for SkydroidLink: the real Qt code against an in-process fake
# camera, with stand-ins for QGC's vehicle classes (stubs/).
# Run with apk/custom/test/run_link_test.ps1.

cmake_minimum_required(VERSION 3.25)
project(vama_link_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Network Qml Positioning Test)

# Overridable so a mutation check can build against a modified copy.
set(SKYDROID_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src/Skydroid CACHE PATH "Folder with the Skydroid sources")

add_executable(skydroid_link_test
    skydroid_link_test.cc
    ${SKYDROID_SRC}/SkydroidLink.cc
    ${SKYDROID_SRC}/SkydroidLink.h
    ${SKYDROID_SRC}/CameraProfile.cc
    ${SKYDROID_SRC}/SkydroidTop.cc
    ${SKYDROID_SRC}/LaserGeo.cc
    ${SKYDROID_SRC}/ObjectTracker.cc
    # Follow Me, with a stand-in for the drone in the test.
    ${SKYDROID_SRC}/FollowDrone.h
    ${SKYDROID_SRC}/FollowGuide.cc
    ${SKYDROID_SRC}/ObjectFollow.cc
    ${SKYDROID_SRC}/ObjectFollow.h
    # The marks on people and vehicles, with a stand-in for the network in the test.
    ${SKYDROID_SRC}/ObjectFinder.cc
    ${SKYDROID_SRC}/MarksWorker.cc
    ${SKYDROID_SRC}/MarksWorker.h
    ${SKYDROID_SRC}/ObjectMarks.cc
    ${SKYDROID_SRC}/ObjectMarks.h
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
    Qt6::Core Qt6::Gui Qt6::Network Qt6::Qml Qt6::Positioning Qt6::Test
)

# The object finder with its real network, on pictures from the air, through the app's own
# resource file. It needs the ncnn library built for this PC (apk/build_ncnn.ps1 -ForHost).
# Without it this test is left out.
set(VAMA_NCNN_HOST "C:/vama-apk/ai/ncnn-20260526/host-mingw" CACHE PATH "Folder with the ncnn library for this PC")
if(EXISTS "${VAMA_NCNN_HOST}/lib/libncnn.a")
    add_executable(marks_net_test
        marks_net_test.cc
        ${SKYDROID_SRC}/ObjectFinder.cc
        ${SKYDROID_SRC}/ObjectFinderNcnn.cc
        ${SKYDROID_SRC}/MarksWorker.cc
        ${SKYDROID_SRC}/MarksWorker.h
        ${SKYDROID_SRC}/ObjectMarksNet.cc
        # The app's own resources: the network's files under the names the app asks for.
        ${CMAKE_CURRENT_SOURCE_DIR}/../../custom.qrc
    )
    set_target_properties(marks_net_test PROPERTIES AUTORCC ON)
    target_include_directories(marks_net_test PRIVATE ${SKYDROID_SRC})
    target_include_directories(marks_net_test SYSTEM PRIVATE ${VAMA_NCNN_HOST}/include)
    target_compile_definitions(marks_net_test PRIVATE
        VAMA_TEST_PICTURES="${CMAKE_CURRENT_SOURCE_DIR}/../ai_pictures"
    )
    target_link_libraries(marks_net_test PRIVATE Qt6::Core Qt6::Gui Qt6::Test ${VAMA_NCNN_HOST}/lib/libncnn.a)
else()
    message(WARNING "marks_net_test is left out: no ncnn library in ${VAMA_NCNN_HOST} (apk/build_ncnn.ps1 -ForHost builds it)")
endif()

# The drone side of Follow Me, with the real MAVLink headers: those the last APK build
# fetched. Without them (no APK build on this computer yet) this test is left out.
set(MAVLINK_INCLUDE "C:/vama-apk/build-Release/_deps/mavlink-build/include/mavlink" CACHE PATH "Folder with the MAVLink C headers")
if(EXISTS "${MAVLINK_INCLUDE}/all/mavlink.h")
    add_executable(follow_drone_test
        follow_drone_test.cc
        ${SKYDROID_SRC}/VehicleFollowDrone.cc
        ${SKYDROID_SRC}/VehicleFollowDrone.h
        # Listed so AUTOMOC sees their signals.
        ${SKYDROID_SRC}/FollowDrone.h
        stubs/MultiVehicleManager.h
        drone_stubs/Vehicle.h
    )
    # drone_stubs first: its Vehicle.h is the one this test means.
    target_include_directories(follow_drone_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/drone_stubs
        ${CMAKE_CURRENT_SOURCE_DIR}/stubs
        ${SKYDROID_SRC}
        ${MAVLINK_INCLUDE}
        ${MAVLINK_INCLUDE}/all
    )
    target_link_libraries(follow_drone_test PRIVATE Qt6::Core Qt6::Test)
else()
    message(WARNING "follow_drone_test is left out: no MAVLink headers in ${MAVLINK_INCLUDE}")
endif()

# The map packs (task U): the app's pack code, with QGC's own tile store code (the code the
# app runs) from the QGC copy that apk/fetch_qgc.ps1 makes. Without that copy this test is
# left out. map_stubs stands in for QGC's logging and for the bodies of its UrlFactory.
set(QGC_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../../../qgc-src" CACHE PATH "QGC's source (apk/qgc-src)")
set(MAPS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src/Maps CACHE PATH "Folder with the map pack sources")
if(EXISTS "${QGC_SRC}/src/QtLocationPlugin/QGCTileCacheDatabase.cpp")
    find_package(Qt6 REQUIRED COMPONENTS Sql)
    add_executable(map_pack_test
        map_pack_test.cc
        ${MAPS_SRC}/MapPack.cc
        ${QGC_SRC}/src/QtLocationPlugin/QGCTileCacheDatabase.cpp
        ${QGC_SRC}/src/Utilities/Database/QGCSqlHelper.cc
        map_stubs/UrlFactory.cc
    )
    # map_stubs first: its QGCLoggingCategory.h is the one this test means.
    target_include_directories(map_pack_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/map_stubs
        ${MAPS_SRC}
        ${QGC_SRC}/src/QtLocationPlugin
        ${QGC_SRC}/src/Utilities/Database
    )
    target_compile_definitions(map_pack_test PRIVATE
        VAMA_MAP_PACKS="${CMAKE_CURRENT_SOURCE_DIR}/../map_packs"
        VAMA_QGC_SRC="${QGC_SRC}"
    )
    target_link_libraries(map_pack_test PRIVATE Qt6::Core Qt6::Sql Qt6::Test)

    # What the app does at its start with the packs it finds (MapPackLoader), with stand-ins for
    # the QGC parts it talks to: the map engine runs the tasks against QGC's real tile store.
    add_executable(map_pack_loader_test
        map_pack_loader_test.cc
        ${MAPS_SRC}/MapPack.cc
        ${MAPS_SRC}/MapPackLoader.cc
        ${MAPS_SRC}/MapPackLoader.h
        ${QGC_SRC}/src/QtLocationPlugin/QGCTileCacheDatabase.cpp
        ${QGC_SRC}/src/Utilities/Database/QGCSqlHelper.cc
        # QGC's map tasks, listed so AUTOMOC sees them. Their two destructors are in the stand-in.
        ${QGC_SRC}/src/QtLocationPlugin/QGCMapTaskBase.h
        ${QGC_SRC}/src/QtLocationPlugin/QGCMapTasks.h
        map_stubs/QGCMapTasksStub.cc
        map_stubs/QGCMapEngine.cc
        map_stubs/UrlFactory.cc
    )
    target_include_directories(map_pack_loader_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/map_stubs
        ${MAPS_SRC}
        ${QGC_SRC}/src/QtLocationPlugin
        ${QGC_SRC}/src/Utilities/Database
    )
    target_compile_definitions(map_pack_loader_test PRIVATE
        VAMA_MAP_PACKS="${CMAKE_CURRENT_SOURCE_DIR}/../map_packs"
    )
    target_link_libraries(map_pack_loader_test PRIVATE Qt6::Core Qt6::Sql Qt6::Test)
else()
    message(WARNING "map_pack_test and map_pack_loader_test are left out: no QGC source in ${QGC_SRC} (apk/fetch_qgc.ps1 makes it)")
endif()

# The camera profiles (task AA): the built-in cameras against the link's old formulas, the
# camera files and their folder.
add_executable(camera_profile_test
    camera_profile_test.cc
    ${SKYDROID_SRC}/CameraProfile.cc
    ${SKYDROID_SRC}/CameraProfile.h
)
target_include_directories(camera_profile_test PRIVATE ${SKYDROID_SRC})
target_link_libraries(camera_profile_test PRIVATE Qt6::Core Qt6::Network Qt6::Test)

# The admin password for Advanced Mode (task Z). admin_password_test is built as a test build
# (the test password), admin_password_team_test as a customer build: it reads the team's salt
# and hash from a file as apk/build_android.ps1 writes it, made here for the password
# Team-Pass-For-Tests-42.
# Overridable so a mutation check can build against a modified copy.
set(ADMIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src CACHE PATH "Folder with AdminPassword.cc")
foreach(test admin_password_test admin_password_team_test)
    add_executable(${test} admin_password_test.cc ${ADMIN_SRC}/AdminPassword.cc ${ADMIN_SRC}/AdminPassword.h)
    target_include_directories(${test} PRIVATE ${ADMIN_SRC})
    target_link_libraries(${test} PRIVATE Qt6::Core Qt6::Network Qt6::Test)
endforeach()
set(TEAM_PASSWORD_DIR ${CMAKE_CURRENT_BINARY_DIR}/team_password)
file(CONFIGURE OUTPUT ${TEAM_PASSWORD_DIR}/admin_password.inc @ONLY CONTENT
"// As apk/build_android.ps1 writes it, for the password Team-Pass-For-Tests-42.
#define VAMA_ADMIN_SALT_HEX \"5eed0f7e57c0ffee5eed0f7e57c0ffee\"
#define VAMA_ADMIN_HASH_HEX \"08d75cd548314a2c406e87ec97e20fae37a9cd0fb259188684ad06156d97436d\"
#define VAMA_ADMIN_ITERATIONS 100000
")
target_include_directories(admin_password_team_test PRIVATE ${TEAM_PASSWORD_DIR})
target_compile_definitions(admin_password_team_test PRIVATE VAMA_TEST_TEAM_PASSWORD="Team-Pass-For-Tests-42")
