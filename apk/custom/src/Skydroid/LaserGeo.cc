// Port of vgcs/observe/geo_reference.py compute_lrf_slant_geo. Keep in step:
// apk/custom/test checks this against the Python code.

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

    // _lrf_camera_dir_ned_unit
    const double hfov = std::max(5.0, std::min(120.0, in.cameraHfovDeg));
    double vfov = in.cameraVfovDeg ? *in.cameraVfovDeg : hfov * 0.5625;
    vfov = std::max(5.0, std::min(90.0, vfov));
    const double u = std::max(0.0, std::min(1.0, in.videoXNorm));
    const double v = std::max(0.0, std::min(1.0, in.videoYNorm));
    const double azOff = (u - 0.5) * hfov;
    const double elOff = (v - 0.5) * vfov;

    const Mat nedBody = mul(rotZ(rad(in.vehicleHeadingDeg)),
                            mul(rotY(rad(in.vehiclePitchDeg)), rotX(rad(in.vehicleRollDeg))));
    // The camera's own number, turned so that right of the nose is positive.
    const double gimbalYawRight = in.gimbalYawLeftPositive ? -in.gimbalYawDeg : in.gimbalYawDeg;
    const Mat bodyGimbal = mul(rotZ(rad(gimbalYawRight)), rotY(rad(in.gimbalPitchDeg)));
    const Mat gimbalCam = mul(rotY(rad(elOff)), rotZ(rad(azOff)));
    const Mat nedCam = mul(nedBody, mul(bodyGimbal, gimbalCam));
    const Vec dir = mulVec(nedCam, {1.0, 0.0, 0.0});
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

} // namespace skydroid::geo
