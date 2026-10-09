// Host unit test for LaserGeo: every case must match VGCS. A point by laser against
// compute_lrf_slant_geo, a point from the picture (without the laser) against the
// measured way of compute_geo_reference.
// Expected values come from laser_geo_vectors.inc and picture_geo_vectors.inc
// (gen_laser_vectors.py).

#include "LaserGeo.h"

#include <cmath>
#include <cstdio>
#include <optional>

using namespace skydroid::geo;

namespace {

int g_checks = 0;
int g_failures = 0;

bool near(double a, double b, double tol)
{
    return std::fabs(a - b) <= tol;
}

void checkLaser(int line, const LaserInput &in, bool ok, double lat, double lon, bool hasAlt, double alt,
                double horiz, double depression, double bearing)
{
    ++g_checks;
    const LaserResult r = computeLaserTarget(in);
    bool good = r.ok == ok;
    if (good && ok) {
        good = near(r.targetLatDeg, lat, 1e-9) && near(r.targetLonDeg, lon, 1e-9)
            && r.targetAltMslM.has_value() == hasAlt && (!hasAlt || near(*r.targetAltMslM, alt, 1e-6))
            && near(r.horizontalRangeM, horiz, 1e-6) && near(r.depressionDeg, depression, 1e-6)
            && near(std::fmod(r.bearingDeg - bearing + 540.0, 360.0) - 180.0, 0.0, 1e-6);
    }
    if (!good) {
        ++g_failures;
        std::printf("FAIL case at vectors line %d\n  got:  ok=%d lat=%.10f lon=%.10f horiz=%.6f dep=%.6f brg=%.6f\n"
                    "  want: ok=%d lat=%.10f lon=%.10f horiz=%.6f dep=%.6f brg=%.6f\n",
                    line, r.ok, r.targetLatDeg, r.targetLonDeg, r.horizontalRangeM, r.depressionDeg, r.bearingDeg,
                    ok, lat, lon, horiz, depression, bearing);
    }
}

LaserInput makeInput(double lat, double lon, double hdg, double roll, double pitch, std::optional<double> alt,
                     double gy, double gp, double range, double u, double v, double hfov, std::optional<double> vfov,
                     bool yawLeftPositive = false)
{
    LaserInput in;
    in.vehicleLatDeg = lat;
    in.vehicleLonDeg = lon;
    in.vehicleHeadingDeg = hdg;
    in.vehicleRollDeg = roll;
    in.vehiclePitchDeg = pitch;
    in.vehicleAltMslM = alt;
    in.gimbalYawDeg = gy;
    in.gimbalYawLeftPositive = yawLeftPositive;
    in.gimbalPitchDeg = gp;
    in.slantRangeM = range;
    in.videoXNorm = u;
    in.videoYNorm = v;
    in.cameraHfovDeg = hfov;
    in.cameraVfovDeg = vfov;
    return in;
}

PictureInput makePicture(double lat, double lon, double hdg, double roll, double pitch, std::optional<double> alt,
                         std::optional<double> height, double gy, double gp, double u, double v, double hfov,
                         std::optional<double> vfov, bool yawLeftPositive)
{
    PictureInput in;
    in.view = makeInput(lat, lon, hdg, roll, pitch, alt, gy, gp, 0.0, u, v, hfov, vfov, yawLeftPositive);
    in.heightAboveGroundM = height;
    return in;
}

void checkPicture(int line, const PictureInput &in, bool ok, double lat, double lon, bool hasAlt, double alt,
                  double horiz, double lookDown, double bearing)
{
    ++g_checks;
    const PictureResult r = computePictureTarget(in);
    const LaserResult &p = r.point;
    bool good = p.ok == ok && (r.why == PictureWhy::None) == ok;
    if (good && ok) {
        good = near(p.targetLatDeg, lat, 1e-9) && near(p.targetLonDeg, lon, 1e-9)
            && p.targetAltMslM.has_value() == hasAlt && (!hasAlt || near(*p.targetAltMslM, alt, 1e-6))
            && near(p.horizontalRangeM, horiz, 1e-6) && near(p.depressionDeg, lookDown, 1e-6)
            // Straight down there is no bearing to speak of.
            && (horiz < 1e-6 || near(std::fmod(p.bearingDeg - bearing + 540.0, 360.0) - 180.0, 0.0, 1e-6));
    }
    if (!good) {
        ++g_failures;
        std::printf("FAIL picture case at vectors line %d\n  got:  ok=%d why=%d lat=%.10f lon=%.10f horiz=%.6f down=%.6f brg=%.6f\n"
                    "  want: ok=%d lat=%.10f lon=%.10f horiz=%.6f down=%.6f brg=%.6f\n",
                    line, p.ok, static_cast<int>(r.why), p.targetLatDeg, p.targetLonDeg, p.horizontalRangeM, p.depressionDeg,
                    p.bearingDeg, ok, lat, lon, horiz, lookDown, bearing);
    }
}

void expect(bool ok, const char *what)
{
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("FAIL %s\n", what);
    }
}

} // namespace

int main()
{
#include "laser_geo_vectors.inc"

    // Sanity checks that do not depend on the Python vectors.
    {
        // The drone faces north. A C13 turned 30 degrees to the right reports
        // yaw -30. The target is to the north-east, on the side it looks at.
        // (Read as right-positive it landed to the north-west: mirrored.)
        LaserInput c13 = makeInput(20.0, 72.0, 0.0, 0.0, 0.0, 100.0, -30.0, -10.0, 500.0, 0.5, 0.5, 83.4, 46.9, true);
        const LaserResult right = computeLaserTarget(c13);
        ++g_checks;
        if (!right.ok || !near(right.bearingDeg, 30.0, 1e-6) || !(right.targetLonDeg > 72.0)) {
            ++g_failures;
            std::printf("FAIL a camera that counts left as positive: bearing %.3f, want 30\n", right.bearingDeg);
        }
        c13.gimbalYawLeftPositive = false;
        const LaserResult other = computeLaserTarget(c13);
        ++g_checks;
        if (!other.ok || !near(other.bearingDeg, 330.0, 1e-6)) {
            ++g_failures;
            std::printf("FAIL a camera that counts right as positive: bearing %.3f, want 330\n", other.bearingDeg);
        }
    }
    {
        // Straight down from 100 m: the target is under the drone, 100 m lower.
        LaserInput in = makeInput(20.0, 72.0, 0.0, 0.0, 0.0, 100.0, 0.0, -90.0, 100.0, 0.5, 0.5, 83.4, std::nullopt);
        const LaserResult r = computeLaserTarget(in);
        ++g_checks;
        if (!r.ok || !near(r.targetLatDeg, 20.0, 1e-9) || !near(r.targetLonDeg, 72.0, 1e-9)
            || !near(*r.targetAltMslM, 0.0, 1e-6) || !near(r.depressionDeg, 90.0, 1e-6)) {
            ++g_failures;
            std::printf("FAIL nadir sanity check\n");
        }
        // Heading east, level look: the target is due east (bearing 90).
        in = makeInput(20.0, 72.0, 90.0, 0.0, 0.0, 100.0, 0.0, 0.0, 1000.0, 0.5, 0.5, 83.4, std::nullopt);
        const LaserResult e = computeLaserTarget(in);
        ++g_checks;
        if (!e.ok || !near(e.bearingDeg, 90.0, 1e-6) || !e.nearHorizon || e.targetLonDeg <= 72.0) {
            ++g_failures;
            std::printf("FAIL east sanity check\n");
        }
    }

    // --- A point from the picture, without the laser -----------------------------
#include "picture_geo_vectors.inc"
    {
        // 100 m up, the camera 30 degrees down, the drone facing north: the point is
        // 173.2 m north on the ground and 200 m from the drone, at the take-off height.
        const PictureInput in = makePicture(20.0, 72.0, 0.0, 0.0, 0.0, 350.0, 100.0, 0.0, -30.0, 0.5, 0.5, 83.4, 46.9, true);
        const PictureResult r = computePictureTarget(in);
        expect(r.point.ok && r.why == PictureWhy::None, "picture: a plain case gives a point");
        expect(near(r.point.horizontalRangeM, 173.2051, 1e-3) && near(r.slantRangeM, 200.0, 1e-6),
               "picture: 100 m up and 30 degrees down is 173.2 m away on the ground, 200 m from the drone");
        expect(r.point.targetAltMslM && near(*r.point.targetAltMslM, 250.0, 1e-9), "picture: the point is at the height of the ground under the drone");
        expect(near(r.point.bearingDeg, 0.0, 1e-9) && r.point.targetLatDeg > 20.0 && near(r.point.targetLonDeg, 72.0, 1e-12),
               "picture: it lies where the camera looks, north of the drone");
        // One degree of camera angle: 100 / sin(30)^2 x pi / 180 = 6.98 m.
        expect(near(r.metresPerDegree, 6.9813, 1e-3), "picture: one degree of camera angle is 7 m on the ground here");
        // The same look one degree flatter lands that much further out (7.4 m: the slope grows).
        const PictureInput flatter = makePicture(20.0, 72.0, 0.0, 0.0, 0.0, 350.0, 100.0, 0.0, -29.0, 0.5, 0.5, 83.4, 46.9, true);
        const double moved = computePictureTarget(flatter).point.horizontalRangeM - r.point.horizontalRangeM;
        expect(moved > 6.9 && moved < 7.6, "picture: and a look one degree flatter lands about that much further out");
    }
    {
        // At 10 degrees down the same degree is 58 m: the number the operator is shown.
        const PictureResult r = computePictureTarget(makePicture(20.0, 72.0, 0.0, 0.0, 0.0, 350.0, 100.0, 0.0, -10.0, 0.5, 0.5, 83.4, 46.9, true));
        expect(r.point.ok && near(r.metresPerDegree, 57.88, 0.01), "picture: at 10 degrees down one degree is 58 m");
    }
    {
        // A camera that counts a turn to the left as positive (C13) and is turned 30 degrees
        // to the right says -30: the point is to the north-east, on the side it looks at.
        const PictureResult right = computePictureTarget(makePicture(20.0, 72.0, 0.0, 0.0, 0.0, 350.0, 100.0, -30.0, -30.0, 0.5, 0.5, 83.4, 46.9, true));
        expect(right.point.ok && near(right.point.bearingDeg, 30.0, 1e-6) && right.point.targetLonDeg > 72.0,
               "picture: a camera turned to the right puts the point on the right");
        // A tap right of the cross is further to the right, a tap below the cross is nearer.
        const PictureResult tapRight = computePictureTarget(makePicture(20.0, 72.0, 0.0, 0.0, 0.0, 350.0, 100.0, 0.0, -30.0, 0.75, 0.5, 83.4, 46.9, true));
        expect(tapRight.point.ok && tapRight.point.bearingDeg > 10.0 && tapRight.point.bearingDeg < 40.0,
               "picture: a tap right of the cross is a point to the right");
        const PictureResult tapBelow = computePictureTarget(makePicture(20.0, 72.0, 0.0, 0.0, 0.0, 350.0, 100.0, 0.0, -30.0, 0.5, 0.75, 83.4, 46.9, true));
        expect(tapBelow.point.ok && tapBelow.point.horizontalRangeM < 173.0 - 40.0, "picture: a tap below the cross is a nearer point");
    }
    {
        // Why there is no point.
        auto why = [](std::optional<double> height, double gimbalPitch) {
            return computePictureTarget(makePicture(20.0, 72.0, 0.0, 0.0, 0.0, 350.0, height, 0.0, gimbalPitch, 0.5, 0.5, 83.4, 46.9, true));
        };
        expect(why(std::nullopt, -30.0).why == PictureWhy::HeightUnknown && !why(std::nullopt, -30.0).point.ok,
               "picture: no height known, no point");
        expect(why(std::nan(""), -30.0).why == PictureWhy::HeightUnknown, "picture: a height that is not a number is no height");
        expect(why(2.4, -30.0).why == PictureWhy::TooLow && why(0.0, -30.0).why == PictureWhy::TooLow && why(-3.0, -30.0).why == PictureWhy::TooLow,
               "picture: under 2.5 m the drone is too low");
        expect(why(2.5, -30.0).point.ok, "picture: at 2.5 m it measures");
        const PictureResult level = why(100.0, 0.0);
        expect(level.why == PictureWhy::AtTheHorizon && near(level.lookDownDeg, 0.0, 1e-9), "picture: a level look meets no ground");
        expect(why(100.0, 12.0).why == PictureWhy::AtTheHorizon && why(100.0, 12.0).lookDownDeg < 0.0, "picture: a look upwards meets no ground");
        const PictureResult flat = why(82.7, -5.0);
        expect(flat.why == PictureWhy::TooFlat && near(flat.lookDownDeg, 5.0, 1e-6) && !flat.point.ok,
               "picture: 5 degrees down is too flat, and the angle is told");
        expect(why(100.0, -7.9).why == PictureWhy::TooFlat && why(100.0, -8.1).point.ok, "picture: the limit is 8 degrees");
    }

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
