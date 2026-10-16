# ============================================================================
# VAMA GCS custom build (started from QGroundControl's custom-example)
# ============================================================================

message(STATUS "QGC: Adding Custom Plugin")

# ============================================================================
# Custom Android Package Template
# ============================================================================

if(ANDROID)
    set(CUSTOM_ANDROID_DIR "${CMAKE_SOURCE_DIR}/custom/android")
    if(EXISTS "${CUSTOM_ANDROID_DIR}")
        file(GLOB CUSTOM_ANDROID_FILES "${CUSTOM_ANDROID_DIR}/*")
        if(CUSTOM_ANDROID_FILES)
            message(STATUS "QGC: Custom Android package template found. Overlaying custom files...")
            set(DEFAULT_ANDROID_DIR "${CMAKE_SOURCE_DIR}/android")
            set(FINAL_ANDROID_DIR "${CMAKE_BINARY_DIR}/custom/android")

            file(COPY "${DEFAULT_ANDROID_DIR}/." DESTINATION "${FINAL_ANDROID_DIR}")

            execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory "${CUSTOM_ANDROID_DIR}" "${FINAL_ANDROID_DIR}")

            # Track changes to Android directories
            set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
                "${DEFAULT_ANDROID_DIR}/"
                "${CUSTOM_ANDROID_DIR}/"
            )

            set(QGC_ANDROID_PACKAGE_SOURCE_DIR "${FINAL_ANDROID_DIR}" CACHE PATH "Path to a custom Android package template" FORCE)
            message(STATUS "QGC: Android package template path will be set to: ${QGC_ANDROID_PACKAGE_SOURCE_DIR}")
        else()
            message(STATUS "QGC: Custom Android package template empty. Using default.")
        endif()
    else()
        message(STATUS "QGC: No custom Android package template found. Using default.")
    endif()
endif()

# ============================================================================
# Custom Resources and Import Paths
# ============================================================================

list(APPEND CUSTOM_RESOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/custom.qrc
)
set(QGC_RESOURCES ${QGC_RESOURCES} ${CUSTOM_RESOURCES} CACHE STRING "Paths to .qrc Resources" FORCE)

set(QML_IMPORT_PATH ${QML_IMPORT_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/res" CACHE STRING "Extra qml import paths" FORCE)

# ============================================================================
# Custom QML Module
# ============================================================================

qt_add_library(CustomModule STATIC)

set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomArtificialHorizon.qml PROPERTIES QT_RESOURCE_ALIAS CustomArtificialHorizon.qml)
qgc_set_qt_resource_alias(
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomAttitudeWidget.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomIconButton.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomOnOffSwitch.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomQuickButton.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomSignalStrength.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomToolBarButton.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomVehicleButton.qml
)

qt_add_qml_module(CustomModule
    URI Custom.Widgets
    VERSION 1.0
    RESOURCE_PREFIX /qml
    QML_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomArtificialHorizon.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomAttitudeWidget.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomIconButton.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomOnOffSwitch.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomQuickButton.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomSignalStrength.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomToolBarButton.qml
        ${CMAKE_CURRENT_SOURCE_DIR}/res/Custom/Widgets/CustomVehicleButton.qml
    NO_PLUGIN
)

# ============================================================================
# The object finder's library (ncnn)
# ============================================================================
# The app marks people and vehicles in the video (src/Skydroid/ObjectFinder.h).
# Its neural network is run by the ncnn library. apk/build_ncnn.ps1 builds that
# for each Android ABI into the build's work folder, beside the link to this
# source: <work>/ai/ncnn-<release>/android-<abi>. apk/build_android.ps1 runs
# that script first. (Each ABI is configured by itself and finds its own.)
#
# A build without the finder has to be asked for: the environment variable
# VAMA_NO_FINDER=1 (build_android.ps1 -NoFinder). Such an app marks nothing and
# says so. Without that, a missing library stops the build here: an app that
# silently lacks the finder would look like a broken one.

set(VAMA_NCNN_RELEASE 20260526)
get_filename_component(VAMA_WORK_DIR "${CMAKE_SOURCE_DIR}/.." ABSOLUTE)
if(ANDROID)
    set(VAMA_NCNN_DIR "${VAMA_WORK_DIR}/ai/ncnn-${VAMA_NCNN_RELEASE}/android-${ANDROID_ABI}")
else()
    set(VAMA_NCNN_DIR "${VAMA_WORK_DIR}/ai/ncnn-${VAMA_NCNN_RELEASE}/host-mingw")
endif()
set(VAMA_FINDER_LIBRARY "")
if("$ENV{VAMA_NO_FINDER}" STREQUAL "1")
    message(WARNING "VAMA: this build has NO object finder (VAMA_NO_FINDER=1). The app will not mark people and vehicles.")
    set(VAMA_FINDER_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/ObjectFinderNone.cc)
elseif(EXISTS "${VAMA_NCNN_DIR}/lib/libncnn.a" AND EXISTS "${VAMA_NCNN_DIR}/include/ncnn/net.h")
    message(STATUS "VAMA: object finder with ncnn ${VAMA_NCNN_RELEASE} from ${VAMA_NCNN_DIR}")
    # An imported library: its headers count as system headers, so their warnings are not ours.
    add_library(VamaNcnn STATIC IMPORTED GLOBAL)
    set_target_properties(VamaNcnn PROPERTIES
        IMPORTED_LOCATION "${VAMA_NCNN_DIR}/lib/libncnn.a"
        INTERFACE_INCLUDE_DIRECTORIES "${VAMA_NCNN_DIR}/include"
    )
    set(VAMA_FINDER_LIBRARY VamaNcnn)
    set(VAMA_FINDER_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/ObjectFinderNcnn.cc)
else()
    message(FATAL_ERROR "VAMA: the ncnn library is missing at ${VAMA_NCNN_DIR}. "
        "Run apk/build_ncnn.ps1 (apk/build_android.ps1 does that by itself). "
        "For a build that does not mark people and vehicles, set the environment variable VAMA_NO_FINDER=1.")
endif()

# ============================================================================
# Custom Plugin Sources
# ============================================================================

set(CUSTOM_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CustomPlugin.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CustomPlugin.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/SkydroidTop.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/SkydroidTop.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/SkydroidLink.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/SkydroidLink.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/LaserGeo.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/LaserGeo.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/ObjectTracker.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/ObjectTracker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/FollowDrone.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/FollowGuide.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/FollowGuide.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/ObjectFollow.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/ObjectFollow.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/VehicleFollowDrone.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/VehicleFollowDrone.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/ObjectFinder.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/ObjectFinder.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/MarksWorker.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/MarksWorker.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/ObjectMarks.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/ObjectMarks.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid/ObjectMarksNet.cc
    ${VAMA_FINDER_SOURCE}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Maps/MapPack.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Maps/MapPack.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Maps/MapPackLoader.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Maps/MapPackLoader.h
    CACHE INTERNAL "" FORCE
)

# ============================================================================
# Custom Build Configuration
# ============================================================================

set(CUSTOM_LIBRARIES
    CustomModule
    ${VAMA_FINDER_LIBRARY}
    CACHE INTERNAL "" FORCE
)

set(CUSTOM_INCLUDE_DIRECTORIES
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Skydroid
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Maps
    CACHE INTERNAL "" FORCE
)

set(CUSTOM_DEFINITIONS
    QGC_CUSTOM_BUILD
    CUSTOMHEADER="CustomPlugin.h"
    CUSTOMCLASS=CustomPlugin
    CACHE INTERNAL "" FORCE
)

set(CUSTOM_QT_COMPONENTS
    Core
    Network
    CACHE INTERNAL "" FORCE
)
