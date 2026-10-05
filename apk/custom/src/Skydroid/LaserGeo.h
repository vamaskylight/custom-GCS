#pragma once

// Target position from a laser range: port of compute_lrf_slant_geo and
// _lrf_camera_dir_ned_unit in vgcs/observe/geo_reference.py.
//
// The look direction is built from the drone heading, roll and pitch and the
// gimbal yaw and pitch exactly as VGCS does, then the measured slant range is
// walked along it. Gimbal angles are the camera's raw reported values (GAC),
// as VGCS uses them. Plain C++17, tested on the host against the Python code.

#include <optional>
#include <string>

namespace skydroid::geo {

struct LaserInput
{
    double vehicleLatDeg = 0.0;
    double vehicleLonDeg = 0.0;
    double vehicleHeadingDeg = 0.0;
    double vehicleRollDeg = 0.0;
    double vehiclePitchDeg = 0.0;
    std::optional<double> vehicleAltMslM;
    double gimbalYawDeg = 0.0;
    double gimbalPitchDeg = 0.0;
    double slantRangeM = 0.0;
    // Where in the picture the laser points. 0.5, 0.5 is the centre.
    double videoXNorm = 0.5;
    double videoYNorm = 0.5;
    double cameraHfovDeg = 83.4;
    std::optional<double> cameraVfovDeg;
};

struct LaserResult
{
    bool ok = false;
    double targetLatDeg = 0.0;
    double targetLonDeg = 0.0;
    std::optional<double> targetAltMslM;
    double horizontalRangeM = 0.0;
    double depressionDeg = 0.0;  // positive = looking down
    double bearingDeg = 0.0;     // 0 = north, 90 = east
    bool nearHorizon = false;    // under 3 degrees down: less accurate on the ground
    std::string error;
};

constexpr double kEarthRadiusM = 6371000.0;

LaserResult computeLaserTarget(const LaserInput &input);

} // namespace skydroid::geo
