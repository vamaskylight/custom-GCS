#include "FollowGuide.h"

#include <algorithm>
#include <cmath>

namespace skydroid::follow {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6371000.0;

// The numbers below were chosen on the test bench (test/follow_guide_test.cc), whose drone
// answers as ArduCopter did in its simulator. FOLLOW_TUNE_* lets the bench try others; the
// app is built without them.
#ifndef FOLLOW_TUNE_ACCEL
#define FOLLOW_TUNE_ACCEL 0.6
#endif
#ifndef FOLLOW_TUNE_GAIN
#define FOLLOW_TUNE_GAIN 0.3
#endif
#ifndef FOLLOW_TUNE_STILL_M
#define FOLLOW_TUNE_STILL_M 1.5
#endif
#ifndef FOLLOW_TUNE_STILL_MPS
#define FOLLOW_TUNE_STILL_MPS 0.4
#endif
#ifndef FOLLOW_TUNE_ASKED_ACCEL
#define FOLLOW_TUNE_ASKED_ACCEL 2.0
#endif
#ifndef FOLLOW_TUNE_SPEED_SHARE
#define FOLLOW_TUNE_SPEED_SHARE 1.0
#endif
#ifndef FOLLOW_TUNE_SMOOTH_S
#define FOLLOW_TUNE_SMOOTH_S 0.7
#endif

// The object's motion between two fixes: it may change its speed by about this much each second.
constexpr double kObjectAccelMps2 = FOLLOW_TUNE_ACCEL;
constexpr double kFirstSpeedSigmaMps = 3.0;  // at the first fix nothing is known of its speed
constexpr double kLeastSigmaM = 0.5;         // no fix is taken as more exact than this
constexpr double kGateSigmas = 4.0;          // a fix further from the estimate than this is not believed ...
constexpr int kGateResetAfter = 5;           // ... until this many in a row say so: then the object is there
// The object is known after this many fixes over this long. Sooner, its speed is a guess, and
// the place the drone is to keep would be set a few metres wrong.
constexpr int kFixesToKnow = 8;
constexpr double kKnownAfterS = 3.0;

// The speed asked from the drone.
constexpr double kPositionGain = FOLLOW_TUNE_GAIN;         // m/s for each metre the drone is away from its place
constexpr double kObjectSpeedShare = FOLLOW_TUNE_SPEED_SHARE;  // how much of the object's own speed is flown with
constexpr double kStillErrorM = FOLLOW_TUNE_STILL_M;       // the object stands and the drone is this near its place: no speed ...
constexpr double kStillObjectMps = FOLLOW_TUNE_STILL_MPS;
constexpr double kMoveErrorM = 2.0 * FOLLOW_TUNE_STILL_M;  // ... until it is this far off, or the object moves this fast
constexpr double kMoveObjectMps = 2.0 * FOLLOW_TUNE_STILL_MPS;
constexpr double kAskedAccelMps2 = FOLLOW_TUNE_ASKED_ACCEL;  // the speed asked changes no faster than this
constexpr double kAskedSmoothS = FOLLOW_TUNE_SMOOTH_S;       // and it is smoothed over about this long
constexpr double kDroneAheadMaxS = 0.6;      // the drone's place is carried forward by its speed at most this long
// Where a fast object is right now is less sure (the fixes are a little old): the least
// distance is taken this many seconds of the object's speed larger, and this many
// standard deviations of its place.
constexpr double kUnsureS = 0.5;
constexpr double kUnsureSigmas = 2.0;
// The object's place is known well enough to start when its standard deviation is under this.
constexpr double kKnownSigmaM = 2.5;
// The side the drone keeps is given up when the object is on the other side of it, or
// nearer than the least distance, and the direction to it is sure: the object further
// from the drone than its place is unsure, and this far at least.
constexpr double kSideSureM = 2.0;
// Speeds beside the object: none inside the least distance, all of them this much further out.
constexpr double kBesideBandM = 3.0;
// The drone needs about this long to take up a new speed. Flying this much (or this
// share) off the speed it should have by now is "another speed than asked".
constexpr double kObeyLagS = 1.0;
constexpr double kObeyMissMps = 2.0;
constexpr double kObeyMissShare = 0.6;

double clampOr(double value, double low, double high, double otherwise)
{
    return std::isfinite(value) ? std::clamp(value, low, high) : otherwise;
}

}  // namespace

void FollowGuide::setSpeedLimit(double mps)
{
    _speedLimit = clampOr(mps, kSpeedLimitLowMps, kSpeedLimitHighMps, kSpeedLimitDefaultMps);
}

void FollowGuide::setRangeLimit(double metres)
{
    _rangeLimit = clampOr(metres, kRangeLimitLowM, kRangeLimitHighM, kRangeLimitDefaultM);
}

void FollowGuide::_toLocal(double latDeg, double lonDeg, double &north, double &east) const
{
    north = (latDeg - _originLatDeg) * kPi / 180.0 * kEarthRadiusM;
    east = (lonDeg - _originLonDeg) * kPi / 180.0 * kEarthRadiusM * _cosLat;
}

void FollowGuide::_toGeo(double north, double east, double &latDeg, double &lonDeg) const
{
    latDeg = _originLatDeg + north / kEarthRadiusM * 180.0 / kPi;
    lonDeg = _originLonDeg + east / (kEarthRadiusM * _cosLat) * 180.0 / kPi;
}

void FollowGuide::drone(const Drone &d, double atS)
{
    if (!std::isfinite(d.latDeg) || !std::isfinite(d.lonDeg) || !std::isfinite(atS)) {
        return;
    }
    if (!_haveOrigin) {
        _haveOrigin = true;
        _originLatDeg = d.latDeg;
        _originLonDeg = d.lonDeg;
        _cosLat = std::max(0.01, std::cos(d.latDeg * kPi / 180.0));
    }
    double n = 0.0;
    double e = 0.0;
    _toLocal(d.latDeg, d.lonDeg, n, e);
    if (!_haveDrone) {
        _droneN = n;
        _droneE = e;
        _droneVn = 0.0;
        _droneVe = 0.0;
        _droneAtS = atS;
    } else if (std::isfinite(d.northMps) && std::isfinite(d.eastMps)) {
        // The drone's own speed estimate: better than two positions.
        _droneN = n;
        _droneE = e;
        _droneVn = d.northMps;
        _droneVe = d.eastMps;
        _droneAtS = atS;
    } else if (n != _droneN || e != _droneE) {
        // A new position. Telemetry repeats the last one until the next comes, so a
        // position that did not change is no news about the speed.
        const double dt = atS - _droneAtS;
        if (dt > 0.02) {
            const double keep = dt >= 1.0 ? 0.0 : 0.4;
            _droneVn = keep * _droneVn + (1.0 - keep) * (n - _droneN) / dt;
            _droneVe = keep * _droneVe + (1.0 - keep) * (e - _droneE) / dt;
        }
        _droneN = n;
        _droneE = e;
        _droneAtS = atS;
    } else if (atS - _droneAtS >= 1.0) {
        // The same place for a second: it stands.
        _droneVn = 0.0;
        _droneVe = 0.0;
        _droneAtS = atS;
    }
    _drone = d;
    _droneSeenAtS = atS;
    _haveDrone = true;
}

void FollowGuide::forgetObject()
{
    _haveObject = false;
    _fixes = 0;
    _rejected = 0;
    _jumps = 0;
    _haveBelow = false;
}

void FollowGuide::forgetDrone()
{
    _haveDrone = false;
}

void FollowGuide::_predict(double dt)
{
    const double q = kObjectAccelMps2 * kObjectAccelMps2;
    for (Axis *a : {&_north, &_east}) {
        a->p += a->v * dt;
        a->ppp += dt * (2.0 * a->ppv + dt * a->pvv) + q * dt * dt * dt * dt / 4.0;
        a->ppv += dt * a->pvv + q * dt * dt * dt / 2.0;
        a->pvv += q * dt * dt;
    }
}

void FollowGuide::fix(const Fix &f, double atS)
{
    if (!std::isfinite(f.latDeg) || !std::isfinite(f.lonDeg) || !std::isfinite(atS)) {
        return;
    }
    if (!_haveOrigin) {
        _haveOrigin = true;
        _originLatDeg = f.latDeg;
        _originLonDeg = f.lonDeg;
        _cosLat = std::max(0.01, std::cos(f.latDeg * kPi / 180.0));
    }
    double n = 0.0;
    double e = 0.0;
    _toLocal(f.latDeg, f.lonDeg, n, e);
    const double sigma = std::max(kLeastSigmaM, std::isfinite(f.sigmaM) ? f.sigmaM : 3.0);
    const double r = sigma * sigma;
    if (std::isfinite(f.belowM) && (!_haveBelow || atS >= _belowAtS)) {
        _haveBelow = true;
        _belowM = f.belowM;
        _belowAtS = atS;
    }
    auto begin = [&]() {
        for (Axis *a : {&_north, &_east}) {
            a->v = 0.0;
            a->ppp = r;
            a->ppv = 0.0;
            a->pvv = kFirstSpeedSigmaMps * kFirstSpeedSigmaMps;
        }
        _north.p = n;
        _east.p = e;
        _haveObject = true;
        _objectAtS = atS;
        _firstFixAtS = atS;
        _lastFixAtS = atS;
        _fixes = 1;
        _rejected = 0;
    };
    if (!_haveObject) {
        begin();
        return;
    }
    const double dt = atS - _objectAtS;
    if (dt < 0.0) {
        return;  // older than what is known already
    }
    _predict(dt);
    _objectAtS = atS;
    const double yn = n - _north.p;
    const double ye = e - _east.p;
    const double sn = _north.ppp + r;
    const double se = _east.ppp + r;
    if (yn * yn / sn + ye * ye / se > kGateSigmas * kGateSigmas) {
        // Far from where the object should be: one such fix is a wrong reading. Several in a
        // row mean the estimate is wrong, and it starts again from there.
        if (++_rejected >= kGateResetAfter) {
            begin();
            ++_jumps;
        }
        return;
    }
    auto update = [r](Axis &a, double y) {
        const double s = a.ppp + r;
        const double kp = a.ppp / s;
        const double kv = a.ppv / s;
        a.p += kp * y;
        a.v += kv * y;
        const double ppp = (1.0 - kp) * a.ppp;
        const double ppv = (1.0 - kp) * a.ppv;
        const double pvv = a.pvv - kv * a.ppv;
        a.ppp = ppp;
        a.ppv = ppv;
        a.pvv = pvv;
    };
    update(_north, yn);
    update(_east, ye);
    _rejected = 0;
    ++_fixes;
    _lastFixAtS = atS;
}

bool FollowGuide::objectKnown(double nowS) const
{
    return _haveObject && _fixes >= kFixesToKnow && _lastFixAtS - _firstFixAtS >= kKnownAfterS &&
           nowS - _lastFixAtS <= kFixFreshS && placeSigmaM(nowS) <= kKnownSigmaM;
}

double FollowGuide::placeSigmaM(double nowS) const
{
    if (!_haveObject) {
        return -1.0;
    }
    const double dt = std::clamp(nowS - _objectAtS, 0.0, kFixFreshS);
    auto variance = [dt](const Axis &a) { return a.ppp + dt * (2.0 * a.ppv + dt * a.pvv); };
    return std::sqrt(std::max(0.0, std::max(variance(_north), variance(_east))));
}

void FollowGuide::_objectAt(double atS, double &north, double &east) const
{
    const double dt = std::clamp(atS - _objectAtS, 0.0, kFixFreshS);
    north = _north.p + _north.v * dt;
    east = _east.p + _east.v * dt;
}

void FollowGuide::_droneAt(double atS, double &north, double &east) const
{
    const double dt = std::clamp(atS - _droneAtS, 0.0, kDroneAheadMaxS);
    north = _droneN + _droneVn * dt;
    east = _droneE + _droneVe * dt;
}

double FollowGuide::distanceM(double nowS) const
{
    if (!_haveObject || !_haveDrone) {
        return -1.0;
    }
    double on = 0.0;
    double oe = 0.0;
    double dn = 0.0;
    double de = 0.0;
    _objectAt(nowS, on, oe);
    _droneAt(nowS, dn, de);
    return std::hypot(on - dn, oe - de);
}

bool FollowGuide::objectFromDrone(double nowS, double &north, double &east) const
{
    if (!_haveObject || !_haveDrone) {
        return false;
    }
    double on = 0.0;
    double oe = 0.0;
    double dn = 0.0;
    double de = 0.0;
    _objectAt(nowS, on, oe);
    _droneAt(nowS, dn, de);
    north = on - dn;
    east = oe - de;
    return true;
}

bool FollowGuide::keptPlace(double &north, double &east) const
{
    if (_state == State::Off) {
        return false;
    }
    north = _offsetN;
    east = _offsetE;
    return true;
}

double FollowGuide::askedSpeedMps() const
{
    return std::hypot(_askedVn, _askedVe);
}

double FollowGuide::objectSpeedMps() const
{
    return _haveObject ? std::hypot(_north.v, _east.v) : 0.0;
}

double FollowGuide::fromStartM(double nowS) const
{
    if (_state == State::Off || !_haveDrone) {
        return 0.0;
    }
    double dn = 0.0;
    double de = 0.0;
    _droneAt(nowS, dn, de);
    return std::hypot(dn - _startN, de - _startE);
}

bool FollowGuide::objectPosition(double nowS, double &latDeg, double &lonDeg) const
{
    if (!_haveObject) {
        return false;
    }
    double on = 0.0;
    double oe = 0.0;
    _objectAt(nowS, on, oe);
    _toGeo(on, oe, latDeg, lonDeg);
    return true;
}

Why FollowGuide::wouldStart(bool lockSeen, double nowS) const
{
    if (!lockSeen) {
        return Why::NoLock;
    }
    if (!_haveDrone || nowS - _droneSeenAtS > kDroneSilentS) {
        return Why::DroneSilent;
    }
    if (!_drone.armed || !_drone.flying) {
        return Why::NotFlying;
    }
    if (!_drone.gps3d) {
        return Why::NoGps;
    }
    if (_drone.mode == Mode::Other) {
        return Why::WrongMode;
    }
    if (!(_drone.heightM >= kMinHeightM)) {
        return Why::TooLow;
    }
    if (!objectKnown(nowS)) {
        return Why::NoFix;
    }
    if (_haveBelow && nowS - _belowAtS <= kBelowFreshS && _belowM < kMinHeightM) {
        return Why::TooLow;  // the ground at the object is higher than the take-off point
    }
    if (!(distanceM(nowS) >= kStartDistanceM)) {
        return Why::TooClose;
    }
    return Why::None;
}

Why FollowGuide::start(bool lockSeen, double nowS)
{
    if (_state != State::Off) {
        return Why::None;  // it runs already
    }
    const Why why = wouldStart(lockSeen, nowS);
    if (why != Why::None) {
        _why = why;
        return why;
    }
    double on = 0.0;
    double oe = 0.0;
    double dn = 0.0;
    double de = 0.0;
    _objectAt(nowS, on, oe);
    _droneAt(nowS, dn, de);
    const double distance = std::hypot(dn - on, de - oe);
    // The drone stays where it is now, seen from the object.
    _offsetN = dn - on;
    _offsetE = de - oe;
    _keptDistanceM = distance;
    _startN = dn;
    _startE = de;
    _startedAtS = nowS;
    _headingDeg = _drone.headingDeg;
    _guidedSeen = _drone.mode == Mode::Guided;
    _askedVn = 0.0;
    _askedVe = 0.0;
    _still = false;
    _expectVn = _droneVn;
    _expectVe = _droneVe;
    _offAskedMps = 0.0;
    _missS = 0.0;
    _lastTickS = nowS;
    _brakeUntilS = 0.0;
    _state = State::Following;
    _why = Why::None;
    return Why::None;
}

void FollowGuide::_end(Why why, double nowS, bool brake)
{
    _state = State::Off;
    _why = why;
    _askedVn = 0.0;
    _askedVe = 0.0;
    _brakeUntilS = brake ? nowS + kBrakeS : 0.0;
}

void FollowGuide::stop(Why why, double nowS)
{
    if (_state != State::Off) {
        _end(why, nowS, true);
    }
}

void FollowGuide::drop(Why why)
{
    if (_state != State::Off) {
        _end(why, 0.0, false);
    }
}

bool FollowGuide::_notObeyed(double askedVn, double askedVe, double dt)
{
    // The speed the drone should have by now: what was asked, a second ago.
    const double share = std::min(1.0, dt / kObeyLagS);
    _expectVn += (askedVn - _expectVn) * share;
    _expectVe += (askedVe - _expectVe) * share;
    if (!_guidedSeen) {
        return false;  // not Follow's to fly yet
    }
    _offAskedMps = std::hypot(_droneVn - _expectVn, _droneVe - _expectVe);
    const double allowed = std::max(kObeyMissMps, kObeyMissShare * std::hypot(_expectVn, _expectVe));
    _missS = _offAskedMps > allowed ? _missS + dt : 0.0;
    return _missS > kNotObeyedS;
}

Velocity FollowGuide::tick(bool lockActive, bool lockSeen, double nowS)
{
    const double none = std::numeric_limits<double>::quiet_NaN();
    const Velocity nothing{false, 0.0, 0.0, none};
    const Velocity stand{true, 0.0, 0.0, _headingDeg};
    if (_state == State::Off) {
        // After the end the drone is told to stop for a moment. Then nothing more is said.
        return _brakeUntilS > nowS ? stand : nothing;
    }
    const double dt = std::clamp(nowS - _lastTickS, 0.0, 1.0);
    _lastTickS = nowS;

    // The drone itself first. When it is not Follow's to fly (the pilot has it, or it has
    // landed), nothing is sent to it.
    if (nowS - _droneSeenAtS > kDroneSilentS) {
        _end(Why::DroneSilent, nowS, true);
        return stand;
    }
    if (!_drone.armed || !_drone.flying) {
        _end(Why::NotFlying, nowS, false);
        return nothing;
    }
    if (_drone.mode == Mode::Guided) {
        _guidedSeen = true;
    } else if (_guidedSeen) {
        _end(Why::PilotTookOver, nowS, false);
        return nothing;
    } else if (nowS - _startedAtS > kGuidedGraceS) {
        _end(Why::NotGuided, nowS, false);
        return nothing;
    }
    if (!lockActive) {
        _end(Why::LockEnded, nowS, true);
        return stand;
    }
    // The drone holds its height by itself. Lower than it may be: the end.
    if (!(_drone.heightM >= kEndHeightM) || (_haveBelow && nowS - _belowAtS <= kBelowFreshS && _belowM < kEndHeightM)) {
        _end(Why::TooLow, nowS, true);
        return stand;
    }

    // Without the object the drone stands.
    Why wait = Why::None;
    if (nowS - _droneSeenAtS > kDroneLateS) {
        wait = Why::DroneLate;  // where the drone is now is not known: no speed is made up from an old place
    } else if (!lockSeen) {
        wait = Why::NotSeen;
    } else if (!_haveObject || nowS - _lastFixAtS > kFixFreshS) {
        wait = Why::FixOld;
    } else if (_fixes < kFixesToKnow || _lastFixAtS - _firstFixAtS < kKnownAfterS) {
        // The estimate was started anew, because the positions jumped. Where the object is
        // and how fast it moves has to hold for a few seconds again, as before the start.
        wait = Why::Jumped;
    }
    if (wait != Why::None) {
        if (_state != State::Waiting) {
            _state = State::Waiting;
            _waitSinceS = nowS;
        }
        _why = wait;
        _askedVn = 0.0;
        _askedVe = 0.0;
        if (nowS - _waitSinceS > kWaitMaxS) {
            _end(Why::FixLost, nowS, true);
        } else if (wait != Why::DroneLate && _notObeyed(0.0, 0.0, dt)) {
            _end(Why::NotObeyed, nowS, true);  // told to stand, and it flies on
        }
        return stand;
    }
    _state = State::Following;
    _why = Why::None;

    double on = 0.0;
    double oe = 0.0;
    double dn = 0.0;
    double de = 0.0;
    _objectAt(nowS, on, oe);
    _droneAt(nowS, dn, de);
    if (std::hypot(dn - _startN, de - _startE) > _rangeLimit) {
        _end(Why::TooFar, nowS, true);
        return stand;
    }

    // The object has come nearer than the least distance, or it has passed the drone (the
    // place the drone keeps is on the far side of the object now). The drone does not fly
    // round the object or over it to get to its place. It gives up its side and keeps its
    // distance where it is: straight out from the object.
    const double fromN = dn - on;
    const double fromE = de - oe;
    const double distance = std::hypot(fromN, fromE);
    // How far off the object's place may be: more for a fast object (the fixes are a little old).
    const double unsure = kUnsureS * std::hypot(_north.v, _east.v) + kUnsureSigmas * placeSigmaM(nowS);
    if ((distance < kMinDistanceM || fromN * _offsetN + fromE * _offsetE < 0.0) &&
        distance > std::max(kSideSureM, unsure)) {
        _offsetN = fromN / distance * _keptDistanceM;
        _offsetE = fromE / distance * _keptDistanceM;
    }

    // The drone's place is the object's place plus the offset of the start. It flies the
    // object's own speed, and a little more or less for what it is away from its place.
    const double en = on + _offsetN - dn;
    const double ee = oe + _offsetE - de;
    double vn = kObjectSpeedShare * _north.v + kPositionGain * en;
    double ve = kObjectSpeedShare * _east.v + kPositionGain * ee;
    const double error = std::hypot(en, ee);
    const double objectSpeed = std::hypot(_north.v, _east.v);
    if (_still) {
        _still = error < kMoveErrorM && objectSpeed < kMoveObjectMps;
    } else {
        _still = error < kStillErrorM && objectSpeed < kStillObjectMps;
    }
    if (_still) {
        vn = 0.0;
        ve = 0.0;
    }
    // Gentle changes of the speed asked: the fixes are noisy, and the drone should not twitch with them.
    if (kAskedSmoothS > 0.0) {
        const double share = std::min(1.0, std::max(dt, 0.05) / kAskedSmoothS);
        vn = _askedVn + (vn - _askedVn) * share;
        ve = _askedVe + (ve - _askedVe) * share;
    }
    const double step = kAskedAccelMps2 * std::max(dt, 0.05);
    const double change = std::hypot(vn - _askedVn, ve - _askedVe);
    if (change > step) {
        vn = _askedVn + (vn - _askedVn) * step / change;
        ve = _askedVe + (ve - _askedVe) * step / change;
    }
    // Never nearer than the least distance by the drone's own flying. The speed has a part
    // straight away from the object and a part beside it. Near the object, the part that
    // points at it is taken out. Inside the least distance the part beside it goes as well
    // (where the object is exactly is a few metres unsure, so "beside" is not sure either).
    // And with the object under the drone, nearer than its place is unsure, no way is "away":
    // the drone stands until the object is to one side of it.
    if (distance > 1e-6) {
        const double outN = fromN / distance;
        const double outE = fromE / distance;
        double away = vn * outN + ve * outE;
        const double besideN = vn - away * outN;
        const double besideE = ve - away * outE;
        if (distance < kMinDistanceM + unsure) {
            away = std::max(0.0, away);
        }
        if (unsure > 1e-6) {
            away *= std::clamp(distance / unsure - 1.0, 0.0, 1.0);
        }
        const double beside = std::clamp((distance - kMinDistanceM) / kBesideBandM, 0.0, 1.0);
        vn = away * outN + beside * besideN;
        ve = away * outE + beside * besideE;
    } else {
        vn = 0.0;
        ve = 0.0;
    }
    _why = distance < kMinDistanceM ? Why::Near : Why::None;
    // The speed limit.
    const double speed = std::hypot(vn, ve);
    if (speed > _speedLimit) {
        vn *= _speedLimit / speed;
        ve *= _speedLimit / speed;
    }
    _askedVn = vn;
    _askedVe = ve;
    if (_notObeyed(vn, ve, dt)) {
        _end(Why::NotObeyed, nowS, true);
        return stand;
    }
    return Velocity{true, vn, ve, _headingDeg};
}

}  // namespace skydroid::follow
