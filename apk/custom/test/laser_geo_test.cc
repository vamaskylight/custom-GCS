// Host unit test for LaserGeo: every case must match VGCS compute_lrf_slant_geo.
// Expected values come from laser_geo_vectors.inc (gen_laser_vectors.py).

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

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
