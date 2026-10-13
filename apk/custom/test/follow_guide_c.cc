// FollowGuide behind plain C functions, built as a DLL, so that the simulator test
// (follow_sitl_test.py) flies the very code the app flies with.

#include "FollowGuide.h"

#include <limits>
#include <string>

using namespace skydroid::follow;

extern "C" {

__declspec(dllexport) void *fg_new()
{
    return new FollowGuide();
}

__declspec(dllexport) void fg_free(void *guide)
{
    delete static_cast<FollowGuide *>(guide);
}

__declspec(dllexport) void fg_set_limits(void *guide, double speedMps, double rangeM)
{
    static_cast<FollowGuide *>(guide)->setSpeedLimit(speedMps);
    static_cast<FollowGuide *>(guide)->setRangeLimit(rangeM);
}

/// mode: 0 Guided, 1 holding its place (Loiter, Position Hold, Brake), 2 anything else.
/// haveSpeed 0: the telemetry carries no speed. haveHeading 0: no heading.
__declspec(dllexport) void fg_drone(void *guide, double latDeg, double lonDeg, double heightM, int haveSpeed,
                                    double northMps, double eastMps, int haveHeading, double headingDeg, int armed,
                                    int flying, int gps3d, int mode, double atS)
{
    Drone d;
    d.latDeg = latDeg;
    d.lonDeg = lonDeg;
    d.heightM = heightM;
    if (haveSpeed) {
        d.northMps = northMps;
        d.eastMps = eastMps;
    }
    if (haveHeading) {
        d.headingDeg = headingDeg;
    }
    d.armed = armed != 0;
    d.flying = flying != 0;
    d.gps3d = gps3d != 0;
    d.mode = mode == 0 ? Mode::Guided : (mode == 1 ? Mode::Holding : Mode::Other);
    static_cast<FollowGuide *>(guide)->drone(d, atS);
}

/// haveBelow 0: the height above the object was not measured.
__declspec(dllexport) void fg_fix(void *guide, double latDeg, double lonDeg, double sigmaM, int haveBelow,
                                  double belowM, double atS)
{
    Fix f;
    f.latDeg = latDeg;
    f.lonDeg = lonDeg;
    f.sigmaM = sigmaM;
    if (haveBelow) {
        f.belowM = belowM;
    }
    static_cast<FollowGuide *>(guide)->fix(f, atS);
}

__declspec(dllexport) void fg_forget_object(void *guide)
{
    static_cast<FollowGuide *>(guide)->forgetObject();
}

/// The number of Why: 0 when it started.
__declspec(dllexport) int fg_start(void *guide, int lockSeen, double nowS)
{
    return static_cast<int>(static_cast<FollowGuide *>(guide)->start(lockSeen != 0, nowS));
}

/// Stopped by the operator.
__declspec(dllexport) void fg_stop(void *guide, double nowS)
{
    static_cast<FollowGuide *>(guide)->stop(Why::Stopped, nowS);
}

/// 1: send the speed in north and east, with the heading to keep (not a number: none). 0: send nothing.
__declspec(dllexport) int fg_tick(void *guide, int lockActive, int lockSeen, double nowS, double *northMps,
                                  double *eastMps, double *headingDeg)
{
    const Velocity v = static_cast<FollowGuide *>(guide)->tick(lockActive != 0, lockSeen != 0, nowS);
    *northMps = v.northMps;
    *eastMps = v.eastMps;
    *headingDeg = v.headingDeg;
    return v.send ? 1 : 0;
}

/// 0 off, 1 following, 2 waiting.
__declspec(dllexport) int fg_state(void *guide)
{
    return static_cast<int>(static_cast<FollowGuide *>(guide)->state());
}

__declspec(dllexport) int fg_why(void *guide)
{
    return static_cast<int>(static_cast<FollowGuide *>(guide)->why());
}

__declspec(dllexport) const char *fg_why_name(int why)
{
    switch (static_cast<Why>(why)) {
    case Why::None: return "None";
    case Why::NoLock: return "NoLock";
    case Why::DroneSilent: return "DroneSilent";
    case Why::NotFlying: return "NotFlying";
    case Why::NoGps: return "NoGps";
    case Why::WrongMode: return "WrongMode";
    case Why::TooLow: return "TooLow";
    case Why::NoFix: return "NoFix";
    case Why::TooClose: return "TooClose";
    case Why::Near: return "Near";
    case Why::DroneLate: return "DroneLate";
    case Why::NotSeen: return "NotSeen";
    case Why::FixOld: return "FixOld";
    case Why::Jumped: return "Jumped";
    case Why::Stopped: return "Stopped";
    case Why::LockEnded: return "LockEnded";
    case Why::NotGuided: return "NotGuided";
    case Why::PilotTookOver: return "PilotTookOver";
    case Why::TooFar: return "TooFar";
    case Why::FixLost: return "FixLost";
    case Why::NotObeyed: return "NotObeyed";
    case Why::DroneChanged: return "DroneChanged";
    }
    return "?";
}

__declspec(dllexport) int fg_object_known(void *guide, double nowS)
{
    return static_cast<FollowGuide *>(guide)->objectKnown(nowS) ? 1 : 0;
}

__declspec(dllexport) double fg_distance(void *guide, double nowS)
{
    return static_cast<FollowGuide *>(guide)->distanceM(nowS);
}

/// The object as the guide sees it: metres north and east of the drone. 0 when not known.
__declspec(dllexport) int fg_object_from_drone(void *guide, double nowS, double *north, double *east)
{
    return static_cast<FollowGuide *>(guide)->objectFromDrone(nowS, *north, *east) ? 1 : 0;
}

__declspec(dllexport) double fg_kept_distance(void *guide)
{
    return static_cast<FollowGuide *>(guide)->keptDistanceM();
}

__declspec(dllexport) double fg_off_asked(void *guide)
{
    return static_cast<FollowGuide *>(guide)->offAskedMps();
}

__declspec(dllexport) double fg_place_sigma(void *guide, double nowS)
{
    return static_cast<FollowGuide *>(guide)->placeSigmaM(nowS);
}

__declspec(dllexport) double fg_constant(const char *name)
{
    const std::string n = name ? name : "";
    if (n == "kMinDistanceM") return kMinDistanceM;
    if (n == "kStartDistanceM") return kStartDistanceM;
    if (n == "kMinHeightM") return kMinHeightM;
    if (n == "kEndHeightM") return kEndHeightM;
    if (n == "kTickS") return kTickS;
    if (n == "kBrakeS") return kBrakeS;
    if (n == "kNotObeyedS") return kNotObeyedS;
    if (n == "kGuidedGraceS") return kGuidedGraceS;
    if (n == "kDroneSilentS") return kDroneSilentS;
    if (n == "kWaitMaxS") return kWaitMaxS;
    if (n == "kSpeedTypeMask") return kSpeedTypeMask;
    if (n == "kSpeedTypeMaskNoHeading") return kSpeedTypeMaskNoHeading;
    if (n == "kSpeedFrame") return kSpeedFrame;
    if (n == "kCopterGuided") return kCopterGuided;
    if (n == "kCopterLoiter") return kCopterLoiter;
    if (n == "kCopterPosHold") return kCopterPosHold;
    if (n == "kCopterBrake") return kCopterBrake;
    return std::numeric_limits<double>::quiet_NaN();
}

}  // extern "C"
