// Follow Me: the drone moves with an object that the app has locked, and keeps
// its distance to it. Plain C++, no Qt: the same code runs in the app, in the
// host tests and against the ArduCopter simulator.
//
// What it does:
//  - It takes fixes of the object's position (from the lock: the camera's own
//    angles with the laser's range or with the height above the ground) and
//    estimates where the object is and how fast it moves.
//  - It keeps the drone where it was relative to the object when Follow was
//    started: the same distance, on the same side, at the same height.
//  - It answers with a speed over the ground for the drone, five times a
//    second. The drone flies it in Guided mode (ArduCopter stops by itself
//    when the speeds stop coming, GUID_TIMEOUT).
//
// What it never does:
//  - It never flies the drone nearer to the object than kMinDistanceM. Inside
//    that distance the drone flies straight away from the object, or stands:
//    no speed towards the object, and none beside it. With the object under
//    the drone, where "away" is not sure, it stands. (The distance is the
//    app's own estimate, which is why Follow starts only kStartDistanceM
//    away: the estimate can be a few metres off. For a fast object and for an
//    unsure position, speeds towards the object stop a little further out.)
//  - It never flies the drone over the object, or round it, to get back to
//    its side. When the object has passed the drone, the drone follows on
//    from the side it is on then, at the same distance.
//  - It never turns the drone: every speed comes with the heading the drone
//    had at the start, to be kept.
//  - It never changes the drone's height, and it ends when the drone is lower
//    than kEndHeightM: above the take-off point, or above the object where
//    the laser measured that (rising ground).
//  - It never flies faster than the speed limit, and never further from where
//    Follow was started than the range limit.
//  - It never goes on without the object: when the lock does not see it, or
//    its position is old, the drone is told to stop.
//  - It never flies after a jump. An object is not somewhere else all at once:
//    when its position jumps (the lock took another object, say), the drone
//    stands until the new position has held for a few seconds.
//  - It never goes on with a drone that does not fly what it is asked, and
//    never with another drone than the one it was started on.

#pragma once

#include <limits>

namespace skydroid::follow {

constexpr double kMinDistanceM = 15.0;        // the drone never flies itself nearer to the object than this
constexpr double kStartDistanceM = 20.0;      // and Follow starts only this far from it, over the ground
constexpr double kMinHeightM = 10.0;          // Follow starts only this high (see Drone::heightM and Fix::belowM)
constexpr double kEndHeightM = 8.0;           // and ends when the drone is lower than this
constexpr double kSpeedLimitLowMps = 2.0;     // the speed limit setting: its range and its usual value
constexpr double kSpeedLimitHighMps = 15.0;
constexpr double kSpeedLimitDefaultMps = 8.0;
constexpr double kRangeLimitLowM = 50.0;      // the range limit setting, from where Follow was started
constexpr double kRangeLimitHighM = 2000.0;
constexpr double kRangeLimitDefaultM = 500.0;

constexpr double kTickS = 0.2;                // tick() is called this often
constexpr double kDroneLateS = 1.0;           // no telemetry this long: the drone is told to stand
constexpr double kDroneSilentS = 2.0;         // and this long: Follow ends
constexpr double kFixFreshS = 2.0;            // the object's position older than this: the drone waits
constexpr double kWaitMaxS = 10.0;            // waiting this long: Follow ends
constexpr double kGuidedGraceS = 4.0;         // the drone has this long to change to Guided after the start
constexpr double kBrakeS = 1.5;               // after the end, "stop" is sent this long, then nothing
constexpr double kNotObeyedS = 5.0;           // the drone flies another speed than asked this long: Follow ends
constexpr double kBelowFreshS = 15.0;         // a measured height above the object counts this long

// How the speed goes to ArduCopter: SET_POSITION_TARGET_LOCAL_NED, five times a second.
// Flown in ArduCopter's simulator (4.6.2 and 4.7.0, test/follow_sitl_test.py):
//  - With the speed alone (type mask 0x0DC7) ArduCopter turns its nose into the flight
//    direction (WP_YAW_BEHAVIOR), 45 degrees in two seconds. That would swing the camera
//    off the object.
//  - With a yaw rate of 0 in every message (0x05C7) the nose stayed on straight legs
//    (0.7 degrees), but drifted on a curve: 11.8 degrees in 110 s behind a runner on a
//    circle.
//  - With a yaw angle in every message (0x09C7) the drone keeps that heading. So the
//    heading of the start is sent with every speed.
//  - Outside Guided it does not fly the speed at all.
//  - After the last message it flies on for GUID_TIMEOUT (3 s as shipped), then it brakes
//    and holds its place, still in Guided.
constexpr unsigned kSpeedTypeMask = 0x09C7;           // used: the speed and the yaw angle. Not used: position, acceleration, yaw rate
constexpr unsigned kSpeedTypeMaskNoHeading = 0x05C7;  // the heading is not known: the speed and a yaw rate of 0
constexpr unsigned kSpeedFrame = 1;                   // MAV_FRAME_LOCAL_NED: north and east over the ground, yaw from north
constexpr unsigned kCopterGuided = 4;         // ArduCopter's own mode numbers (HEARTBEAT custom_mode)
constexpr unsigned kCopterLoiter = 5;
constexpr unsigned kCopterPosHold = 16;
constexpr unsigned kCopterBrake = 17;

/// The drone's flight mode, as far as Follow cares.
enum class Mode {
    Guided,   // the mode Follow flies in
    Holding,  // the drone holds its place by itself: Loiter, Position Hold, Brake. Follow may start from here.
    Other,    // anything else (a mission, RTL, Land, a pilot's mode): Follow does not start, and ends if it runs
};

struct Drone
{
    double latDeg = 0.0;
    double lonDeg = 0.0;
    double heightM = 0.0;  // above the take-off point
    // The drone's own speed over the ground, when it reports one. Not a number: not known,
    // and the speed is taken from the positions.
    double northMps = std::numeric_limits<double>::quiet_NaN();
    double eastMps = std::numeric_limits<double>::quiet_NaN();
    // Where its nose points, degrees from north. Not a number: not known.
    double headingDeg = std::numeric_limits<double>::quiet_NaN();
    bool armed = false;
    bool flying = false;
    bool gps3d = false;
    Mode mode = Mode::Other;
};

/// Where the object is, and how sure that is (one standard deviation, metres).
struct Fix
{
    double latDeg = 0.0;
    double lonDeg = 0.0;
    double sigmaM = 3.0;
    // How far the object is below the drone, when that was measured (the laser's range and
    // the camera's angle). Not a number: not measured, the ground is taken as level.
    double belowM = std::numeric_limits<double>::quiet_NaN();
};

/// What to send to the drone now.
struct Velocity
{
    bool send = false;  // false: say nothing
    double northMps = 0.0;
    double eastMps = 0.0;
    // The heading to keep, degrees from north: the drone's own at the start. Not a number:
    // it was not known then, and the speed goes with a yaw rate of 0.
    double headingDeg = std::numeric_limits<double>::quiet_NaN();
};

enum class State {
    Off,
    Following,
    Waiting,  // started, but the object is not seen or its position is old: the drone is told to stand still
};

/// Why Follow did not start, why it waits, or why it ended.
enum class Why {
    None,
    // It did not start:
    NoLock,          // no lock that sees the object
    DroneSilent,     // no telemetry from the drone (also an end)
    NotFlying,       // not armed or not in the air (also an end)
    NoGps,           // the drone has no 3D GPS position
    WrongMode,       // the drone is not holding its place (Loiter, Position Hold, Brake) or in Guided
    TooLow,          // under kMinHeightM (also an end: under kEndHeightM)
    NoFix,           // the object's position is not known yet, or not well enough
    TooClose,        // nearer to the object than kStartDistanceM
    // It follows, and:
    Near,            // the object is nearer than kMinDistanceM: the drone does not fly towards it
    // It waits:
    DroneLate,       // the drone's telemetry is late
    NotSeen,         // the lock does not see the object right now
    FixOld,          // no new position of the object
    Jumped,          // the object's position jumped: it is not known well enough again yet
    // It ended:
    Stopped,         // by the operator
    LockEnded,       // the lock ended
    NotGuided,       // the drone did not change to Guided after the start
    PilotTookOver,   // the flight mode was changed while following
    TooFar,          // the range limit
    FixLost,         // waited too long for the object
    NotObeyed,       // the drone does not fly the speeds it is asked
    DroneChanged,    // another drone is the active one now
};

class FollowGuide
{
public:
    /// The settings. Values outside their range are brought into it.
    void setSpeedLimit(double mps);
    void setRangeLimit(double metres);
    double speedLimit() const { return _speedLimit; }
    double rangeLimit() const { return _rangeLimit; }

    /// Telemetry: where the drone is at the time atS (seconds on any steady clock).
    void drone(const Drone &d, double atS);
    /// A fix of the object's position at the time atS. Fixes come while a lock runs, with or without Follow.
    void fix(const Fix &f, double atS);
    /// Forget the object (the lock ended, or a new object was locked).
    void forgetObject();
    /// Forget the drone: the telemetry so far was another drone's.
    void forgetDrone();

    /// Why Follow would not start now. Why::None: it would.
    Why wouldStart(bool lockSeen, double nowS) const;
    /// The operator starts Follow. Why::None when it started; the caller then asks the drone for Guided.
    Why start(bool lockSeen, double nowS);
    /// The end, by the operator or for a reason found outside (the lock ended).
    void stop(Why why, double nowS);
    /// The end without a word to the drone: it is not Follow's to fly any more.
    void drop(Why why);
    /// Every kTickS while state() is not Off, and for kBrakeS after the end. Gives the speed to send.
    Velocity tick(bool lockActive, bool lockSeen, double nowS);

    State state() const { return _state; }
    /// While following Why::None, or Why::Near; why it waits; after start() refused or after the end, the reason.
    Why why() const { return _why; }
    /// True while tick() still has "stop" to send after the end.
    bool braking(double nowS) const { return _state == State::Off && _brakeUntilS > nowS; }

    /// The object's position is known well enough to start (a fresh fix, a few of them, and they agree).
    bool objectKnown(double nowS) const;
    /// How sure the object's place is now: one standard deviation, metres. Negative when not known.
    double placeSigmaM(double nowS) const;
    /// How often the object's position has jumped (the estimate was started anew) since it was last forgotten.
    int jumps() const { return _jumps; }
    /// Drone to object over the ground, metres. Negative when not known.
    double distanceM(double nowS) const;
    /// The object as the drone sees it: metres north and east of the drone. False when not known.
    bool objectFromDrone(double nowS, double &north, double &east) const;
    /// The distance Follow keeps (set at the start).
    double keptDistanceM() const { return _keptDistanceM; }
    /// Where the drone stays, seen from the object: metres north and east. False when Follow is off.
    bool keptPlace(double &north, double &east) const;
    /// The drone as its last telemetry said, and how old that is. Negative age: none yet.
    const Drone &lastDrone() const { return _drone; }
    double droneAgeS(double nowS) const { return _haveDrone ? nowS - _droneSeenAtS : -1.0; }
    /// The height above the object as the laser last measured it. Negative: not measured, or too long ago.
    double belowM(double nowS) const { return (_haveBelow && nowS - _belowAtS <= kBelowFreshS) ? _belowM : -1.0; }
    /// The speed last asked from the drone, and the object's own estimated speed.
    double askedSpeedMps() const;
    double objectSpeedMps() const;
    /// How far the drone is from where Follow started.
    double fromStartM(double nowS) const;
    /// How far the drone's own speed is from the speed it should have by now (see kNotObeyedS).
    double offAskedMps() const { return _offAskedMps; }
    /// The object's estimated position now. False when not known.
    bool objectPosition(double nowS, double &latDeg, double &lonDeg) const;

private:
    struct Axis
    {
        // One direction of the object's motion: place and speed, with their covariance.
        double p = 0.0;
        double v = 0.0;
        double ppp = 0.0;
        double ppv = 0.0;
        double pvv = 0.0;
    };

    void _toLocal(double latDeg, double lonDeg, double &north, double &east) const;
    void _toGeo(double north, double east, double &latDeg, double &lonDeg) const;
    void _predict(double dt);
    void _objectAt(double atS, double &north, double &east) const;
    void _droneAt(double atS, double &north, double &east) const;
    void _end(Why why, double nowS, bool brake);
    /// True when the drone has not flown what it was asked for kNotObeyedS.
    bool _notObeyed(double askedVn, double askedVe, double dt);

    double _speedLimit = kSpeedLimitDefaultMps;
    double _rangeLimit = kRangeLimitDefaultM;

    // The local frame: metres north and east of the first position seen.
    bool _haveOrigin = false;
    double _originLatDeg = 0.0;
    double _originLonDeg = 0.0;
    double _cosLat = 1.0;

    // The drone.
    bool _haveDrone = false;
    Drone _drone;
    double _droneAtS = 0.0;      // the time its place holds for
    double _droneSeenAtS = 0.0;  // the last time telemetry came at all
    double _droneN = 0.0;
    double _droneE = 0.0;
    double _droneVn = 0.0;
    double _droneVe = 0.0;

    // The object.
    bool _haveObject = false;
    Axis _north;
    Axis _east;
    double _objectAtS = 0.0;   // the time the estimate holds for
    double _firstFixAtS = 0.0;
    double _lastFixAtS = 0.0;  // the last fix that was believed
    int _fixes = 0;
    int _rejected = 0;
    int _jumps = 0;
    bool _haveBelow = false;   // the height above the object was measured
    double _belowM = 0.0;
    double _belowAtS = 0.0;

    // Follow itself.
    State _state = State::Off;
    Why _why = Why::None;
    double _offsetN = 0.0;     // where the drone stays, seen from the object
    double _offsetE = 0.0;
    double _keptDistanceM = 0.0;
    double _startN = 0.0;      // where the drone was when Follow started
    double _startE = 0.0;
    double _startedAtS = 0.0;
    double _headingDeg = std::numeric_limits<double>::quiet_NaN();  // the heading the drone keeps
    bool _guidedSeen = false;
    double _waitSinceS = 0.0;
    double _askedVn = 0.0;
    double _askedVe = 0.0;
    bool _still = false;       // the object stands and the drone is at its place: no speed is asked
    double _expectVn = 0.0;    // the speed the drone should have by now
    double _expectVe = 0.0;
    double _offAskedMps = 0.0; // how far its speed is from that
    double _missS = 0.0;       // for how long it has flown another speed
    double _lastTickS = 0.0;
    double _brakeUntilS = 0.0;
};

}  // namespace skydroid::follow
