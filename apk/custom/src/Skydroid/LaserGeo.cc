// Port of vgcs/observe/geo_reference.py: compute_lrf_slant_geo, and the measured
// way of compute_geo_reference. Keep in step: apk/custom/test checks this
// against the Python code.

#include "LaserGeo.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace skydroid::geo {

namespace {

using Mat = std::array<std::array<double, 3>, 3>;
using Vec = std::array<double, 3>;

constexpr double kPi = 3.14159265358979323846;

double rad(double deg)
{
    return deg * kPi / 180.0;
}

double degrees(double radians)
{
    return radians * 180.0 / kPi;
}

Mat rotX(double r)
{
    const double c = std::cos(r), s = std::sin(r);
    return {{{1.0, 0.0, 0.0}, {0.0, c, -s}, {0.0, s, c}}};
}

Mat rotY(double r)
{
    const double c = std::cos(r), s = std::sin(r);
    return {{{c, 0.0, s}, {0.0, 1.0, 0.0}, {-s, 0.0, c}}};
}

Mat rotZ(double r)
{
    const double c = std::cos(r), s = std::sin(r);
    return {{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}};
}

Mat mul(const Mat &a, const Mat &b)
{
    Mat out{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    return out;
}

Vec mulVec(const Mat &m, const Vec &v)
{
    return {m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]};
}

/// Where the camera looks to a point of the picture: north, east, down (not of length one).
/// VGCS _lrf_camera_dir_ned_unit, and the same lines in compute_geo_reference.
Vec lookDirection(const LaserInput &in)
{
    const double hfov = std::max(5.0, std::min(120.0, in.cameraHfovDeg));
    double vfov = in.cameraVfovDeg ? *in.cameraVfovDeg : hfov * 0.5625;
    vfov = std::max(5.0, std::min(90.0, vfov));
    const double u = std::max(0.0, std::min(1.0, in.videoXNorm));
    const double v = std::max(0.0, std::min(1.0, in.videoYNorm));
    const double azOff = (u - 0.5) * hfov;
    // How far the point looks UP from the cross. The picture's v counts from
    // the top, so a point below the cross looks further down. Until
    // 2026-10-08 this had the other sign, here and in VGCS (_click_tilt_deg).
    const double elTilt = -(v - 0.5) * vfov;

    const Mat nedBody = mul(rotZ(rad(in.vehicleHeadingDeg)),
                            mul(rotY(rad(in.vehiclePitchDeg)), rotX(rad(in.vehicleRollDeg))));
    // The camera's own number, turned so that right of the nose is positive.
    const double gimbalYawRight = in.gimbalYawLeftPositive ? -in.gimbalYawDeg : in.gimbalYawDeg;
    const Mat bodyGimbal = mul(rotZ(rad(gimbalYawRight)), rotY(rad(in.gimbalPitchDeg)));
    const Mat gimbalCam = mul(rotY(rad(elTilt)), rotZ(rad(azOff)));
    const Mat nedCam = mul(nedBody, mul(bodyGimbal, gimbalCam));
    return mulVec(nedCam, {1.0, 0.0, 0.0});
}

} // namespace

LaserResult computeLaserTarget(const LaserInput &in)
{
    LaserResult result;
    if (!std::isfinite(in.slantRangeM)) {
        result.error = "invalid LRF range";
        return result;
    }
    if (in.slantRangeM < 0.5) {
        result.error = "LRF range too short";
        return result;
    }

    const Vec dir = lookDirection(in);
    const double mag = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    if (mag < 1e-9) {
        result.error = "invalid look direction";
        return result;
    }

    // compute_lrf_slant_geo
    const double north = in.slantRangeM * dir[0] / mag;
    const double east = in.slantRangeM * dir[1] / mag;
    const double down = in.slantRangeM * dir[2] / mag;
    const double horiz = std::hypot(north, east);
    result.depressionDeg = degrees(std::atan2(down, std::max(1e-6, horiz)));
    result.bearingDeg = std::fmod(degrees(std::atan2(east, north)) + 360.0, 360.0);
    result.horizontalRangeM = horiz;

    // _offset_lat_lon
    const double latRad = rad(in.vehicleLatDeg);
    result.targetLatDeg = in.vehicleLatDeg + degrees(north / kEarthRadiusM);
    result.targetLonDeg = in.vehicleLonDeg + degrees(east / (kEarthRadiusM * std::max(1e-6, std::cos(latRad))));
    if (in.vehicleAltMslM) {
        result.targetAltMslM = *in.vehicleAltMslM - down;
    }
    result.nearHorizon = result.depressionDeg < 3.0;
    result.ok = true;
    return result;
}

PictureResult computePictureTarget(const PictureInput &in)
{
    PictureResult result;
    // _why_not_measured: the height has to be a measured one, and that of a hover at least.
    if (!in.heightAboveGroundM || !std::isfinite(*in.heightAboveGroundM)) {
        result.why = PictureWhy::HeightUnknown;
        return result;
    }
    const double height = *in.heightAboveGroundM;
    if (height < kPictureMinHeightM) {
        result.why = PictureWhy::TooLow;
        return result;
    }

    // compute_geo_reference, solve(believe_camera=True), over level ground.
    const Vec dir = lookDirection(in.view);
    const double level = std::hypot(dir[0], dir[1]);
    const double down = dir[2];
    result.lookDownDeg = degrees(std::atan2(down, level));
    if (down <= 1e-4) {
        result.why = PictureWhy::AtTheHorizon;
        return result;
    }
    const double t = height / down;
    const double north = t * dir[0];
    const double east = t * dir[1];
    const double range = std::hypot(north, east);
    // VGCS is_plausible_ground_range. It has three rules, made for ground that
    // rises and falls (its terrain file). Over level ground, from 2.5 m up,
    // only this one can ever say no, so it is the one that is here: a look
    // flatter than 8 degrees gives no point, unless the point is within 20 m.
    if (result.lookDownDeg < kPictureMinLookDownDeg && range > kPictureFlatLookReachM) {
        result.why = PictureWhy::TooFlat;
        return result;
    }

    LaserResult &point = result.point;
    point.horizontalRangeM = range;
    point.depressionDeg = result.lookDownDeg;
    point.bearingDeg = std::fmod(degrees(std::atan2(east, north)) + 360.0, 360.0);
    const double latRad = rad(in.view.vehicleLatDeg);
    point.targetLatDeg = in.view.vehicleLatDeg + degrees(north / kEarthRadiusM);
    point.targetLonDeg = in.view.vehicleLonDeg + degrees(east / (kEarthRadiusM * std::max(1e-6, std::cos(latRad))));
    if (in.view.vehicleAltMslM) {
        point.targetAltMslM = *in.view.vehicleAltMslM - height;
    }
    point.ok = true;
    result.slantRangeM = std::hypot(range, height);
    // The range on the ground is height / tan(look down): one degree more or less moves it this far.
    const double sine = std::sin(rad(result.lookDownDeg));
    result.metresPerDegree = height / (sine * sine) * kPi / 180.0;
    return result;
}

} // namespace skydroid::geo
