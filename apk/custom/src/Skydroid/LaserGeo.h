#pragma once

// Target position from a laser range: port of compute_lrf_slant_geo and
// _lrf_camera_dir_ned_unit in vgcs/observe/geo_reference.py.
//
// The look direction is built from the drone heading, roll and pitch and the
// gimbal yaw and pitch exactly as VGCS does, then the measured slant range is
// walked along it. Gimbal angles are the camera's raw reported values (GAC),
// as VGCS uses them, and gimbalYawLeftPositive says which way the yaw counts.
//
// And the position of a point in the picture WITHOUT the laser: port of the
// measured way of compute_geo_reference in the same Python file. The look
// direction to the point is followed down to the ground, which is taken as
// level at the drone's height under it. VGCS's rules decide whether that is a
// measurement: the drone at least 2.5 m up, and the point looked at steeply
// enough (8 degrees under the horizon, with VGCS's own small print).
//
// Plain C++17, tested on the host against the Python code.

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
    // Which way gimbalYawDeg counts. False: a turn to the right of the drone's
    // nose is positive. True: a turn to the LEFT is positive, which is how the
    // C12 and C13 report it (GAC). Until 2026-10-08 the C13's number was taken
    // as right-positive, and a target measured with the camera turned to a
    // side landed mirrored about the nose line.
    bool gimbalYawLeftPositive = false;
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

/// Why the picture gives no position.
enum class PictureWhy
{
    None,
    HeightUnknown,  // the drone's height above the ground is not known
    TooLow,         // the drone is under kPictureMinHeightM: no height to measure with
    AtTheHorizon,   // the point is looked at level or upwards: its ray never meets the ground
    TooFlat,        // looked at flatter than 8 degrees and further than 20 m: the point runs away with every part of a degree
};

struct PictureInput
{
    // The drone's pose and the camera's angles, as for the laser. videoXNorm and
    // videoYNorm are the point in the picture, cameraHfovDeg and cameraVfovDeg the
    // camera's lens at its zoom. slantRangeM is not used.
    LaserInput view;
    // The drone's height above the ground. Level ground is assumed: this is the
    // height above the take-off point.
    std::optional<double> heightAboveGroundM;
};

struct PictureResult
{
    LaserResult point;             // ok, lat long, height, range on the ground, look angle, bearing
    PictureWhy why = PictureWhy::None;
    double lookDownDeg = 0.0;      // how far under the horizon the point is looked at (also when there is no position)
    double slantRangeM = 0.0;      // from the drone to the point
    double metresPerDegree = 0.0;  // how far the point moves on the ground when the camera's angle is one degree off
};

constexpr double kPictureMinHeightM = 2.5;     // VGCS MIN_FACADE_AGL_M
constexpr double kPictureMinLookDownDeg = 8.0; // VGCS is_plausible_ground_range
constexpr double kPictureFlatLookReachM = 20.0; // the same: within this range a flatter look is still taken

PictureResult computePictureTarget(const PictureInput &input);

} // namespace skydroid::geo
