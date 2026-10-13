// Host test and test bench for FollowGuide (Follow Me): a made-up world with an
// object on the ground, a drone that flies the speeds it is sent (as ArduCopter
// does in Guided: gently, and it stops by itself when the speeds stop coming),
// late telemetry, and late and noisy fixes of the object's position.
//
// The drone answers as ArduCopter 4.6.2 and 4.7.0 did in their simulator on
// 2026-10-13 (follow_sitl_test.py flies the same guide there): a speed asked is
// taken up at 2.5 m/s2 at the most (WP_ACC), the acceleration changes by 5 m/s3
// at the most (PSC_JERK_NE), and it flies on for 3 s after the last command
// (GUID_TIMEOUT). A step to 5 m/s was at 1.4 m/s after 1 s, 4.0 after 2 s and
// 4.8 after 3 s there; this model is within 0.1 m/s of that.
//
// Every case knows where the object and the drone really are, so it can say how
// well the drone kept its place, and whether a rule held all the way.
//
// "-v" prints the numbers of every case.

#include "FollowGuide.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <string>
#include <vector>

using namespace skydroid::follow;

namespace {

int g_checks = 0;
int g_failures = 0;
bool g_verbose = false;
std::string g_trace;  // "-trace NAME": this case tick by tick

void check(bool ok, const std::string &what, const std::string &got = std::string())
{
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("FAIL %s%s%s\n", what.c_str(), got.empty() ? "" : ": ", got.c_str());
    } else if (g_verbose) {
        std::printf("ok   %s%s%s\n", what.c_str(), got.empty() ? "" : ": ", got.c_str());
    }
}

std::string num(double v, int digits = 1)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", digits, v);
    return buf;
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6371000.0;
constexpr double kHomeLat = 19.8816785;  // the client's flying field
constexpr double kHomeLon = 73.8199197;

struct Random
{
    std::uint32_t state;
    explicit Random(std::uint32_t seed) : state(seed) {}
    double uniform()  // 0 to 1
    {
        state = state * 1664525u + 1013904223u;
        return (state >> 8) / 16777216.0;
    }
    double normal()  // about a standard normal
    {
        return (uniform() + uniform() + uniform() + uniform() - 2.0) * std::sqrt(3.0);
    }
};

struct Point
{
    double n = 0.0;
    double e = 0.0;
};

void toGeo(const Point &p, double &lat, double &lon)
{
    lat = kHomeLat + p.n / kEarthRadiusM * 180.0 / kPi;
    lon = kHomeLon + p.e / (kEarthRadiusM * std::cos(kHomeLat * kPi / 180.0)) * 180.0 / kPi;
}

/// One case: what the object does, and what goes wrong on the way.
struct Case
{
    std::string name;
    std::function<Point(double)> object;        // where the object is at a time (seconds)
    Point droneStart{-30.0, 0.0};               // the drone hovers here, 30 m south of the origin
    double heightM = 30.0;
    double seconds = 90.0;
    double startFollowAtS = 6.0;
    double speedLimit = kSpeedLimitDefaultMps;
    double rangeLimit = kRangeLimitDefaultM;
    double fixSigmaM = 1.2;                     // noise of each fix, each direction
    double fixEveryS = 0.2;
    double videoLateS = 0.35;                   // how old a fix really is (the app takes 0.30)
    double wrongFixShare = 0.0;                 // this share of the fixes is 30 m off
    bool droneTellsSpeed = false;               // telemetry with the drone's own speed
    std::function<bool(double)> seen = [](double) { return true; };          // the lock sees the object
    std::function<bool(double)> lockActive = [](double) { return true; };
    std::function<bool(double)> fixesCome = [](double) { return true; };
    std::function<bool(double)> telemetryComes = [](double) { return true; };
    std::function<Mode(double, Mode)> pilot = [](double, Mode m) { return m; };  // the pilot's hand on the mode switch
    bool droneRefusesGuided = false;
    double stopByOperatorAtS = -1.0;
    std::function<double(double)> height;       // the drone's height at a time (not set: heightM all the way)
    std::function<double(double)> below;        // the measured height above the object (not set: not measured)
    // What the drone makes of the speed it is asked: (time, asked, what it flies now) gives what it goes for.
    std::function<Point(double, Point, Point)> droneWill = [](double, Point asked, Point) { return asked; };
};

struct Outcome
{
    Why startWhy = Why::None;
    Why endWhy = Why::None;
    double endedAtS = -1.0;
    bool followingAtEnd = false;
    double keptM = 0.0;            // the distance at the start
    double errorMean = 0.0;        // how far the drone was from its place (after the first 10 s)
    double error95 = 0.0;
    double errorMax = 0.0;
    double leastDistanceM = 1e9;   // true distance drone to object, while following
    double mostAskedMps = 0.0;
    double mostDroneMps = 0.0;
    // The largest speed asked towards the object while nearer than the least distance, by the
    // guide's own estimate: the rule itself.
    double mostClosingInsideMps = 0.0;
    // In truth: how far the drone's own flying brought it nearer to the object, while it was
    // truly nearer than 12 m and the distance was shrinking (metres).
    double ownClosingM = 0.0;
    int sideChanges = 0;           // how often the object went from one side of the drone to the other (north, south)
    double nearS = 0.0;            // for how long the guide said "the object is near"
    double leastDistanceLateM = 1e9;  // true distance in the last 10 s
    int jumps = 0;                 // how often the guide took the object's position as jumped
    double jumpWaitS = 0.0;        // and for how long it waited for that
    double askedChangeMps2 = 0.0;  // how much the speed asked changes each second, on average
    double rippleMps = 0.0;        // how far the drone's speed is from the object's own, on average (after the first 10 s)
    double errorLate95 = 0.0;      // off its place in the last 30 s
    double droneMovedAfterEndM = 0.0;
    double sentAfterEndS = 0.0;    // for how long speeds were still sent after the end
    int sendsAfterBrake = 0;       // speeds sent later than that
    double askedWhileNotSeen = 0.0;
    double stillAskedShare = 0.0;  // share of the ticks with a speed asked, in the last 20 s
    double heightChange = 0.0;
    Point droneEnd;
    Point objectEnd;
    double waitedS = 0.0;
    double fromStartAtEnd = 0.0;
};

Outcome run(const Case &c, std::uint32_t seed = 7)
{
    Outcome out;
    Random random(seed);
    FollowGuide guide;
    guide.setSpeedLimit(c.speedLimit);
    guide.setRangeLimit(c.rangeLimit);

    // The drone: it flies the speed it was last sent, as ArduCopter does in Guided.
    Point drone = c.droneStart;
    Point droneV;
    Point droneA;
    struct Command
    {
        double arrivesAtS;
        Point speed;
    };
    std::deque<Command> onTheWay;         // speeds sent, not yet at the drone
    Point command;                        // the speed the drone has
    double commandAtS = -100.0;
    Mode mode = Mode::Holding;
    double guidedAtS = -1.0;
    constexpr double kStep = 0.02;
    constexpr double kDroneAccel = 2.5;   // m/s2
    constexpr double kDroneJerk = 5.0;    // m/s3
    constexpr double kDroneGain = 2.0;    // 1/s: how hard it goes for the speed it is still short of
    constexpr double kCommandLate = 0.2;  // s: a speed sent is with the drone this much later
    constexpr double kGuidedTimeout = 3.0;
    constexpr double kTelemetryEvery = 0.25;
    constexpr double kTelemetryLate = 0.15;

    struct Late
    {
        double atS;
        Drone d;
    };
    std::deque<Late> telemetry;
    double nextTelemetry = 0.0;
    double nextFix = 0.05;
    double nextTick = 0.1;
    bool started = false;
    bool ended = false;
    Point trueOffset;
    std::vector<double> errors;
    Point droneAtEnd;
    int ticksLate = 0;
    int askedLate = 0;
    double lastSendS = -1.0;
    std::vector<double> errorsLate;
    Point askedBefore;
    double askedChangeSum = 0.0;
    int askedChanges = 0;
    double distanceBefore = -1.0;
    int sideBefore = 0;
    double rippleSum = 0.0;
    int ripples = 0;

    for (double t = 0.0; t < c.seconds; t += kStep) {
        const Point object = c.object(t);

        // The pilot, and the change to Guided that Follow asked for.
        mode = c.pilot(t, mode);
        if (guidedAtS >= 0.0 && t >= guidedAtS) {
            mode = Mode::Guided;
            guidedAtS = -1.0;
        }
        // The drone's flight.
        while (!onTheWay.empty() && onTheWay.front().arrivesAtS <= t) {
            command = onTheWay.front().speed;
            commandAtS = onTheWay.front().arrivesAtS;
            onTheWay.pop_front();
        }
        Point want;
        if (mode == Mode::Guided && t - commandAtS <= kGuidedTimeout) {
            want = command;
        }
        if (mode == Mode::Guided) {
            want = c.droneWill(t, want, droneV);
        }
        Point push{kDroneGain * (want.n - droneV.n), kDroneGain * (want.e - droneV.e)};
        const double pushSize = std::hypot(push.n, push.e);
        if (pushSize > kDroneAccel) {
            push.n *= kDroneAccel / pushSize;
            push.e *= kDroneAccel / pushSize;
        }
        Point change{push.n - droneA.n, push.e - droneA.e};
        const double changeSize = std::hypot(change.n, change.e);
        if (changeSize > kDroneJerk * kStep) {
            change.n *= kDroneJerk * kStep / changeSize;
            change.e *= kDroneJerk * kStep / changeSize;
        }
        droneA.n += change.n;
        droneA.e += change.e;
        droneV.n += droneA.n * kStep;
        droneV.e += droneA.e * kStep;
        drone.n += droneV.n * kStep;
        drone.e += droneV.e * kStep;

        if (t >= nextTelemetry) {
            nextTelemetry += kTelemetryEvery;
            if (c.telemetryComes(t)) {
                Drone d;
                toGeo(drone, d.latDeg, d.lonDeg);
                d.heightM = c.height ? c.height(t) : c.heightM;
                d.armed = true;
                d.flying = true;
                d.gps3d = true;
                d.mode = mode;
                if (c.droneTellsSpeed) {
                    d.northMps = droneV.n;
                    d.eastMps = droneV.e;
                }
                telemetry.push_back({t + kTelemetryLate, d});
            }
        }
        while (!telemetry.empty() && telemetry.front().atS <= t) {
            guide.drone(telemetry.front().d, t);
            telemetry.pop_front();
        }
        if (t >= nextFix) {
            nextFix += c.fixEveryS;
            if (c.lockActive(t) && c.seen(t) && c.fixesCome(t)) {
                Point p = c.object(std::max(0.0, t - c.videoLateS));
                p.n += random.normal() * c.fixSigmaM;
                p.e += random.normal() * c.fixSigmaM;
                if (random.uniform() < c.wrongFixShare) {
                    p.n += 30.0;
                }
                Fix f;
                toGeo(p, f.latDeg, f.lonDeg);
                f.sigmaM = std::max(1.5, c.fixSigmaM);
                if (c.below) {
                    f.belowM = c.below(t);
                }
                guide.fix(f, t - 0.30);
            }
        }
        if (!started && t >= c.startFollowAtS) {
            started = true;
            out.startWhy = guide.start(c.lockActive(t) && c.seen(t), t);
            if (out.startWhy == Why::None) {
                out.keptM = guide.keptDistanceM();
                trueOffset = Point{drone.n - object.n, drone.e - object.e};
                if (mode != Mode::Guided && !c.droneRefusesGuided) {
                    guidedAtS = t + 0.5;
                }
            }
        }
        if (started && c.stopByOperatorAtS >= 0.0 && t >= c.stopByOperatorAtS && guide.state() != State::Off) {
            guide.stop(Why::Stopped, t);
            ended = true;
            out.endWhy = guide.why();
            out.endedAtS = t;
            droneAtEnd = drone;
        }
        if (t >= nextTick) {
            nextTick += kTickS;
            const bool was = guide.state() != State::Off;
            const Velocity v = guide.tick(c.lockActive(t), c.seen(t), t);
            if (v.send) {
                onTheWay.push_back({t + kCommandLate, Point{v.northMps, v.eastMps}});
                lastSendS = t;
                out.mostAskedMps = std::max(out.mostAskedMps, std::hypot(v.northMps, v.eastMps));
                if (ended && t - out.endedAtS > kBrakeS + 0.3) {
                    ++out.sendsAfterBrake;
                }
            }
            if (was && guide.state() == State::Off && !ended) {
                ended = true;
                out.endWhy = guide.why();
                out.endedAtS = t;
                droneAtEnd = drone;
                out.fromStartAtEnd = std::hypot(drone.n - c.droneStart.n, drone.e - c.droneStart.e);
            }
            if (guide.state() == State::Waiting) {
                out.waitedS += kTickS;
                out.jumpWaitS += guide.why() == Why::Jumped ? kTickS : 0.0;
            }
            out.jumps = guide.jumps();
            if (!g_trace.empty() && c.name == g_trace) {
                double seenN = 0.0;
                double seenE = 0.0;
                guide.objectFromDrone(t, seenN, seenE);
                std::printf("TRACE %6.2f s  object %7.1f %7.1f  drone %7.1f %7.1f  true distance %6.1f  seen %6.1f (%6.1f %6.1f)  "
                            "asked %5.2f %5.2f%s  state %d why %d\n",
                            t, object.n, object.e, drone.n, drone.e, std::hypot(object.n - drone.n, object.e - drone.e),
                            guide.distanceM(t), seenN, seenE, v.northMps, v.eastMps, v.send ? "" : " (not sent)",
                            static_cast<int>(guide.state()), static_cast<int>(guide.why()));
            }
            if (guide.state() != State::Off) {
                const double distance = std::hypot(object.n - drone.n, object.e - drone.e);
                out.leastDistanceM = std::min(out.leastDistanceM, distance);
                if (t > c.seconds - 10.0) {
                    out.leastDistanceLateM = std::min(out.leastDistanceLateM, distance);
                }
                if (v.send && distance < kMinDistanceM - 3.0 && distance > 1e-6 && distanceBefore >= 0.0 &&
                    distance < distanceBefore) {
                    const double closing = (v.northMps * (object.n - drone.n) + v.eastMps * (object.e - drone.e)) / distance;
                    out.ownClosingM += std::max(0.0, closing) * kTickS;
                }
                distanceBefore = distance;
                const int side = drone.n - object.n > 1.0 ? 1 : (drone.n - object.n < -1.0 ? -1 : 0);
                if (side != 0) {
                    out.sideChanges += (sideBefore != 0 && side != sideBefore) ? 1 : 0;
                    sideBefore = side;
                }
                if (guide.why() == Why::Near) {
                    out.nearS += kTickS;
                }
                double seenN = 0.0;
                double seenE = 0.0;
                if (v.send && guide.objectFromDrone(t, seenN, seenE)) {
                    const double seenDistance = std::hypot(seenN, seenE);
                    if (seenDistance < kMinDistanceM && seenDistance > 1e-6) {
                        out.mostClosingInsideMps = std::max(out.mostClosingInsideMps,
                                                            (v.northMps * seenN + v.eastMps * seenE) / seenDistance);
                    }
                }
                if (guide.state() == State::Following && v.send) {
                    askedChangeSum += std::hypot(v.northMps - askedBefore.n, v.eastMps - askedBefore.e) / kTickS;
                    ++askedChanges;
                }
                askedBefore = v.send ? Point{v.northMps, v.eastMps} : Point{};
                if (!c.seen(t) && v.send) {
                    out.askedWhileNotSeen = std::max(out.askedWhileNotSeen, std::hypot(v.northMps, v.eastMps));
                }
                if (guide.state() == State::Following && t > c.startFollowAtS + 10.0) {
                    const Point ahead = c.object(t + 0.1);
                    const Point behind = c.object(t - 0.1);
                    const double dn = droneV.n - (ahead.n - behind.n) / 0.2;
                    const double de = droneV.e - (ahead.e - behind.e) / 0.2;
                    rippleSum += dn * dn + de * de;
                    ++ripples;
                    errors.push_back(std::hypot(drone.n - object.n - trueOffset.n, drone.e - object.e - trueOffset.e));
                    if (t > c.seconds - 30.0) {
                        errorsLate.push_back(errors.back());
                    }
                }
                if (t > c.seconds - 20.0) {
                    ++ticksLate;
                    askedLate += (v.send && std::hypot(v.northMps, v.eastMps) > 0.01) ? 1 : 0;
                }
            }
        }
        if (guide.state() != State::Off) {
            out.mostDroneMps = std::max(out.mostDroneMps, std::hypot(droneV.n, droneV.e));
        }
    }
    out.followingAtEnd = guide.state() != State::Off;
    out.droneEnd = drone;
    out.objectEnd = c.object(c.seconds);
    if (ended) {
        out.droneMovedAfterEndM = std::hypot(drone.n - droneAtEnd.n, drone.e - droneAtEnd.e);
        out.sentAfterEndS = std::max(0.0, lastSendS - out.endedAtS);
    }
    if (!errors.empty()) {
        std::sort(errors.begin(), errors.end());
        double sum = 0.0;
        for (double e : errors) {
            sum += e;
        }
        out.errorMean = sum / errors.size();
        out.error95 = errors[static_cast<size_t>(0.95 * (errors.size() - 1))];
        out.errorMax = errors.back();
    }
    if (!errorsLate.empty()) {
        std::sort(errorsLate.begin(), errorsLate.end());
        out.errorLate95 = errorsLate[static_cast<size_t>(0.95 * (errorsLate.size() - 1))];
    }
    out.askedChangeMps2 = askedChanges ? askedChangeSum / askedChanges : 0.0;
    out.rippleMps = ripples ? std::sqrt(rippleSum / ripples) : 0.0;
    out.stillAskedShare = ticksLate ? static_cast<double>(askedLate) / ticksLate : 0.0;
    if (g_verbose) {
        std::printf("     %-34s kept %5.1f m, off its place mean %4.1f, 95%% %4.1f, most %4.1f m; least distance %5.1f m; "
                    "asked up to %4.1f m/s, changing %4.2f m/s2, flew up to %4.1f m/s%s\n",
                    c.name.c_str(), out.keptM, out.errorMean, out.error95, out.errorMax,
                    out.leastDistanceM > 1e8 ? 0.0 : out.leastDistanceM, out.mostAskedMps, out.askedChangeMps2,
                    out.mostDroneMps, ended ? (", ended at " + num(out.endedAtS) + " s").c_str() : "");
    }
    return out;
}

// --- what the object does -----------------------------------------------------

Point stands(double) { return Point{0.0, 0.0}; }

std::function<Point(double)> walksEast(double mps, double fromS = 8.0)
{
    return [=](double t) { return Point{0.0, mps * std::max(0.0, t - fromS)}; };
}

/// Walks a square: each side for sideS seconds.
std::function<Point(double)> walksSquare(double mps, double sideS, double fromS = 8.0)
{
    return [=](double t) {
        double left = std::max(0.0, t - fromS);
        Point p;
        const double dn[4] = {0.0, 1.0, 0.0, -1.0};
        const double de[4] = {1.0, 0.0, -1.0, 0.0};
        for (int side = 0; left > 0.0; side = (side + 1) % 4) {
            const double s = std::min(left, sideS);
            p.n += dn[side] * mps * s;
            p.e += de[side] * mps * s;
            left -= s;
        }
        return p;
    };
}

/// Walks for walkS seconds, stands for standS, and again.
std::function<Point(double)> stopsAndGoes(double mps, double walkS, double standS, double fromS = 8.0)
{
    return [=](double t) {
        const double since = std::max(0.0, t - fromS);
        const double round = walkS + standS;
        const double rounds = std::floor(since / round);
        const double in = since - rounds * round;
        return Point{0.0, mps * (rounds * walkS + std::min(in, walkS))};
    };
}

/// Speeds up from standing to mps with accel, eastwards.
std::function<Point(double)> drivesOff(double mps, double accel, double fromS = 8.0)
{
    return [=](double t) {
        const double since = std::max(0.0, t - fromS);
        const double rampS = mps / accel;
        if (since < rampS) {
            return Point{0.0, 0.5 * accel * since * since};
        }
        return Point{0.0, 0.5 * accel * rampS * rampS + mps * (since - rampS)};
    };
}

/// Comes straight at the drone (which is south of it).
std::function<Point(double)> comesAtTheDrone(double mps, double fromS = 10.0)
{
    return [=](double t) { return Point{-mps * std::max(0.0, t - fromS), 0.0}; };
}

/// Comes at the drone fast, passes it, and goes on slowly: mps for fastS seconds, then slowMps.
std::function<Point(double)> passesTheDrone(double mps, double fastS, double slowMps, double fromS = 10.0)
{
    return [=](double t) {
        const double since = std::max(0.0, t - fromS);
        return Point{-(mps * std::min(since, fastS) + slowMps * std::max(0.0, since - fastS)), 0.0};
    };
}

/// A curve: a circle of this radius.
std::function<Point(double)> runsACircle(double mps, double radius, double fromS = 8.0)
{
    return [=](double t) {
        const double angle = mps * std::max(0.0, t - fromS) / radius;
        return Point{radius * std::sin(angle), radius * (1.0 - std::cos(angle))};
    };
}

const char *whyName(Why why)
{
    switch (why) {
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

// --- the cases ------------------------------------------------------------------

void followsWell()
{
    {
        Case c;
        c.name = "the object stands";
        c.object = stands;
        const Outcome o = run(c);
        check(o.startWhy == Why::None && o.followingAtEnd, "standing object: Follow starts and goes on", whyName(o.startWhy));
        check(std::abs(o.keptM - 30.0) < 3.0, "standing object: the distance kept is the distance at the start (30 m)", num(o.keptM));
        check(o.errorMax < 2.0, "standing object: the drone stays at its place", num(o.errorMax) + " m");
        check(o.stillAskedShare < 0.1, "standing object: the drone is left alone, no speeds back and forth",
              num(o.stillAskedShare * 100.0, 0) + " % of the time a speed was asked");
    }
    {
        Case c;
        c.name = "walks straight (1.4 m/s)";
        c.object = walksEast(1.4);
        c.seconds = 120.0;
        const Outcome o = run(c);
        check(o.followingAtEnd, "walk: Follow goes on for two minutes", whyName(o.endWhy));
        check(o.error95 < 4.0 && o.errorMax < 5.5, "walk: the drone keeps its place beside the walker",
              "95 % " + num(o.error95) + " m, most " + num(o.errorMax) + " m");
        check(std::abs(o.droneEnd.e - o.objectEnd.e) < 5.0 && std::abs(o.droneEnd.n - (o.objectEnd.n - 30.0)) < 5.0,
              "walk: at the end the drone is 30 m south of the walker, as at the start",
              num(o.droneEnd.n - o.objectEnd.n) + " m north, " + num(o.droneEnd.e - o.objectEnd.e) + " m east");
        check(o.mostAskedMps < 3.6, "walk: no speed far above the walker's own", num(o.mostAskedMps) + " m/s");
        check(o.askedChangeMps2 < 0.8, "walk: the speed asked is calm (it changes under 0.8 m/s each second)",
              num(o.askedChangeMps2, 2) + " m/s2");
    }
    {
        Case c;
        c.name = "walks a square";
        c.object = walksSquare(1.4, 20.0);
        c.seconds = 120.0;
        const Outcome o = run(c);
        check(o.followingAtEnd && o.error95 < 8.0 && o.errorMax < 10.0, "square: the drone stays with a walker who turns corners",
              "95 % " + num(o.error95) + " m, most " + num(o.errorMax) + " m");
    }
    {
        Case c;
        c.name = "walks and stands in turn";
        c.object = stopsAndGoes(1.4, 15.0, 12.0);
        c.seconds = 140.0;
        const Outcome o = run(c);
        check(o.followingAtEnd && o.error95 < 8.0 && o.errorMax < 10.0, "stop and go: the drone stays with it",
              "95 % " + num(o.error95) + " m, most " + num(o.errorMax) + " m");
    }
    {
        Case c;
        c.name = "runs a circle (4 m/s)";
        c.object = runsACircle(4.0, 40.0);
        c.seconds = 120.0;
        const Outcome o = run(c);
        check(o.followingAtEnd && o.error95 < 10.0 && o.errorMax < 11.0, "runner on a curve: the drone stays with it",
              "95 % " + num(o.error95) + " m, most " + num(o.errorMax) + " m");
    }
    {
        Case c;
        c.name = "drives off to 10 m/s, limit 12";
        c.object = drivesOff(10.0, 1.5);
        c.speedLimit = 12.0;
        c.rangeLimit = 2000.0;
        c.seconds = 100.0;
        const Outcome o = run(c);
        // While the car speeds up the drone falls behind (its own speed-up is gentle), then it comes back.
        check(o.followingAtEnd && o.errorMax < 35.0 && o.errorLate95 < 6.0,
              "car: the drone falls behind while a car speeds up to 10 m/s, and is back at its place afterwards",
              "most " + num(o.errorMax) + " m behind, " + num(o.errorLate95) + " m in the last 30 s");
        check(o.mostAskedMps <= 12.0 + 1e-9, "car: never above the speed limit", num(o.mostAskedMps) + " m/s");
    }
    {
        Case c;
        c.name = "walk, the drone tells its speed";
        c.object = walksSquare(1.4, 20.0);
        c.droneTellsSpeed = true;
        c.seconds = 120.0;
        const Outcome o = run(c);
        check(o.followingAtEnd && o.error95 < 8.0, "telemetry with the drone's own speed is used as well",
              "95 % " + num(o.error95) + " m");
    }
    {
        // Other noise: every run is another walk for the estimate.
        double worst95 = 0.0;
        for (std::uint32_t seed = 1; seed <= 12; ++seed) {
            Case c;
            c.name = "walks a square, noise " + std::to_string(seed);
            c.object = walksSquare(1.4, 20.0);
            c.fixSigmaM = 2.0;
            c.seconds = 120.0;
            const Outcome o = run(c, seed * 7919u);
            worst95 = std::max(worst95, o.error95);
        }
        check(worst95 < 10.0, "square with coarser fixes (2 m), 12 different noises: the drone stays with it", num(worst95) + " m");
    }
}

void keepsItsLimits()
{
    {
        Case c;
        c.name = "drives off to 12 m/s, limit 8";
        c.object = drivesOff(12.0, 1.5);
        c.seconds = 60.0;
        const Outcome o = run(c);
        check(o.mostAskedMps <= kSpeedLimitDefaultMps + 1e-9, "speed limit: never more is asked than the limit (8 m/s)",
              num(o.mostAskedMps, 2) + " m/s");
        check(o.mostAskedMps > 7.5, "speed limit: the drone does fly at the limit after a car", num(o.mostAskedMps, 2));
    }
    {
        Case c;
        c.name = "walks away, range limit 60 m";
        c.object = walksEast(1.4);
        c.rangeLimit = 60.0;
        c.seconds = 120.0;
        const Outcome o = run(c);
        check(o.endWhy == Why::TooFar, "range limit: Follow ends when the drone is that far from its start", whyName(o.endWhy));
        check(o.fromStartAtEnd > 58.0 && o.fromStartAtEnd < 66.0, "range limit: at 60 m", num(o.fromStartAtEnd) + " m");
        check(o.droneMovedAfterEndM < 6.0, "range limit: the drone stops there", num(o.droneMovedAfterEndM) + " m more");
    }
    {
        Case c;
        c.name = "walker comes at the drone";
        c.object = comesAtTheDrone(1.4);
        c.seconds = 90.0;
        const Outcome o = run(c);
        check(o.followingAtEnd && o.leastDistanceM > 23.0, "a walker coming at the drone: the drone backs away and keeps its distance",
              num(o.leastDistanceM) + " m at the least");
    }
    {
        Case c;
        c.name = "runner comes at a slow drone";
        c.object = comesAtTheDrone(6.0);
        c.speedLimit = 2.0;
        c.droneStart = Point{-24.0, 0.0};
        c.seconds = 40.0;
        const Outcome o = run(c);
        check(o.startWhy == Why::None && o.leastDistanceM < kMinDistanceM - 3.0,
              "(this runner does get much nearer than 15 m: the drone may fly 2 m/s only)", num(o.leastDistanceM) + " m");
        check(o.mostClosingInsideMps <= 1e-9, "inside 15 m, as the guide sees it, no speed towards the object is ever asked",
              num(o.mostClosingInsideMps, 3) + " m/s");
        check(o.ownClosingM <= 0.5, "and in truth the drone's own flying brings it no nearer, 12 m and nearer",
              num(o.ownClosingM, 2) + " m");
        check(o.nearS > 1.0, "the guide says so while the runner is nearer than 15 m", num(o.nearS) + " s");
        check(o.sideChanges == 1, "the runner passes under the drone once, and the drone does not fly over the runner afterwards",
              std::to_string(o.sideChanges) + " changes of side");
    }
    {
        // A runner comes at the drone at 7 m/s, passes under it (the drone may fly 5 m/s),
        // and walks on at 1.4 m/s. The drone was south of the runner. It ends north of the
        // walker, behind them, at the distance it kept before.
        Case c;
        c.name = "runner passes the drone, then walks";
        c.object = passesTheDrone(7.0, 12.0, 1.4);
        c.speedLimit = 5.0;
        c.droneStart = Point{-25.0, 0.0};
        c.seconds = 100.0;
        double mostOwn = 0.0;
        double mostRule = 0.0;
        int once = 0;
        int behind = 0;
        double nearestLate = 1e9;
        double farthestLate = 0.0;
        const int seeds = 8;
        for (std::uint32_t seed = 1; seed <= seeds; ++seed) {
            const Outcome o = run(c, seed * 15485863u);
            mostOwn = std::max(mostOwn, o.ownClosingM);
            mostRule = std::max(mostRule, o.mostClosingInsideMps);
            once += (o.followingAtEnd && o.sideChanges == 1) ? 1 : 0;
            const double northOf = o.droneEnd.n - o.objectEnd.n;
            behind += (northOf > 15.0 && std::abs(o.droneEnd.e - o.objectEnd.e) < 8.0) ? 1 : 0;
            nearestLate = std::min(nearestLate, o.leastDistanceLateM);
            farthestLate = std::max(farthestLate, std::hypot(northOf, o.droneEnd.e - o.objectEnd.e));
        }
        check(once == seeds, "an object that passes the drone: Follow goes on, and the drone never flies over the object to get back to its side",
              std::to_string(once) + " of " + std::to_string(seeds));
        check(behind == seeds, "it follows on from the side it is on then (behind the object now)",
              std::to_string(behind) + " of " + std::to_string(seeds));
        check(nearestLate > 20.0 && farthestLate < 31.0, "and is back at the distance it kept (25 m)",
              num(nearestLate) + " to " + num(farthestLate) + " m in the last 10 s");
        check(mostRule <= 1e-9 && mostOwn <= 0.8, "all the way no speed towards the object inside 15 m",
              num(mostRule, 3) + " m/s as the guide sees it, " + num(mostOwn, 2) + " m in truth");
    }
    {
        // The object walks to and fro near a drone that keeps only 21 m. The drone's place moves
        // with it, and the estimate is a few metres off at times.
        Case c;
        c.name = "walker to and fro, drone at 21 m";
        c.droneStart = Point{-21.0, 0.0};
        c.object = [](double t) {
            const double a = 0.12 * std::max(0.0, t - 9.0);
            return Point{-8.0 * (1.0 - std::cos(a)) * 0.6, 8.0 * std::sin(a)};
        };
        c.seconds = 100.0;
        double most = 0.0;
        double least = 1e9;
        int started = 0;
        for (std::uint32_t seed = 1; seed <= 8; ++seed) {
            const Outcome o = run(c, seed * 104729u);
            started += o.startWhy == Why::None ? 1 : 0;
            most = std::max(most, o.mostClosingInsideMps);
            least = std::min(least, o.leastDistanceM);
        }
        check(started >= 6, "at 21 m Follow starts (the estimate may say 20 m or a little less now and then)", std::to_string(started) + " of 8");
        check(most <= 1e-9, "at 21 m from a moving object, 8 noises: no speed towards it inside 15 m", num(most, 3) + " m/s");
        check(least > 13.0, "and the drone stays 13 m and more from it in truth", num(least) + " m");
    }
    {
        Case c;
        c.name = "the drone sinks";
        c.object = walksEast(1.4);
        c.height = [](double t) { return t < 30.0 ? 30.0 : std::max(4.0, 30.0 - (t - 30.0)); };
        c.seconds = 70.0;
        const Outcome o = run(c);
        // 8 m is reached at 52 s.
        check(o.endWhy == Why::TooLow && o.endedAtS > 51.5 && o.endedAtS < 53.5, "the drone sinks under 8 m: Follow ends",
              std::string(whyName(o.endWhy)) + " at " + num(o.endedAtS) + " s");
        check(o.droneMovedAfterEndM < 3.0 && o.sendsAfterBrake == 0, "and the drone is told to stop", num(o.droneMovedAfterEndM) + " m more");
    }
    {
        // The ground rises under the object: the drone holds its height above the take-off
        // point, and the laser finds the object less and less far below it.
        Case c;
        c.name = "the ground rises";
        c.object = walksEast(1.4);
        c.below = [](double t) { return t < 30.0 ? 30.0 : 30.0 - 0.5 * (t - 30.0); };
        c.seconds = 90.0;
        const Outcome o = run(c);
        // 8 m is reached at 74 s.
        check(o.endWhy == Why::TooLow && o.endedAtS > 73.5 && o.endedAtS < 75.5,
              "the ground rises to 8 m under the drone (the laser measures it): Follow ends",
              std::string(whyName(o.endWhy)) + " at " + num(o.endedAtS) + " s");
    }
}

void stopsWithoutTheObject()
{
    {
        Case c;
        c.name = "hidden for 1.5 s, twice";
        c.object = walksEast(1.4);
        c.seen = [](double t) { return !((t > 30.0 && t < 31.5) || (t > 60.0 && t < 61.5)); };
        c.seconds = 100.0;
        const Outcome o = run(c);
        check(o.followingAtEnd, "hidden for a moment: Follow goes on afterwards", whyName(o.endWhy));
        check(o.askedWhileNotSeen <= 1e-9, "hidden: no speed is asked while the lock does not see the object",
              num(o.askedWhileNotSeen, 2) + " m/s");
        check(o.waitedS > 2.0 && o.waitedS < 6.0, "hidden: it waits for about as long as the object is hidden", num(o.waitedS) + " s");
        check(o.error95 < 5.5, "hidden: and catches up", "95 % " + num(o.error95) + " m");
    }
    {
        Case c;
        c.name = "the lock ends";
        c.object = walksEast(1.4);
        c.lockActive = [](double t) { return t < 40.0; };
        c.seconds = 60.0;
        const Outcome o = run(c);
        check(o.endWhy == Why::LockEnded && std::abs(o.endedAtS - 40.0) < 0.5, "lock ended: Follow ends at once",
              std::string(whyName(o.endWhy)) + " at " + num(o.endedAtS) + " s");
        check(o.droneMovedAfterEndM < 3.0, "lock ended: the drone stops", num(o.droneMovedAfterEndM) + " m more");
        check(o.sentAfterEndS > 1.0 && o.sentAfterEndS < 2.0 && o.sendsAfterBrake == 0,
              "lock ended: \"stop\" is sent for a second and a half, then nothing more",
              num(o.sentAfterEndS) + " s, " + std::to_string(o.sendsAfterBrake) + " later");
    }
    {
        Case c;
        c.name = "no fixes for 4 s";
        c.object = walksEast(1.4);
        c.fixesCome = [](double t) { return !(t > 30.0 && t < 34.0); };
        c.seconds = 80.0;
        const Outcome o = run(c);
        check(o.followingAtEnd && o.waitedS > 1.0 && o.waitedS < 4.0,
              "no position for 4 s: the drone waits once the position is 2 s old, and goes on when it comes again",
              num(o.waitedS) + " s waited, " + whyName(o.endWhy));
    }
    {
        Case c;
        c.name = "no fixes any more";
        c.object = walksEast(1.4);
        c.fixesCome = [](double t) { return t < 30.0; };
        c.seconds = 60.0;
        const Outcome o = run(c);
        check(o.endWhy == Why::FixLost && o.endedAtS > 40.0 && o.endedAtS < 44.0,
              "no position any more: Follow ends after waiting 10 s", std::string(whyName(o.endWhy)) + " at " + num(o.endedAtS) + " s");
    }
    {
        Case c;
        c.name = "5 % of the fixes are 30 m off";
        c.object = walksSquare(1.4, 20.0);
        c.wrongFixShare = 0.05;
        c.seconds = 120.0;
        const Outcome o = run(c);
        check(o.followingAtEnd && o.error95 < 8.0 && o.errorMax < 10.0, "wrong fixes now and then are not believed",
              "95 % " + num(o.error95) + " m, most " + num(o.errorMax) + " m");
    }
    {
        Case c;
        c.name = "the operator stops it";
        c.object = walksEast(1.4);
        c.stopByOperatorAtS = 30.0;
        c.seconds = 45.0;
        const Outcome o = run(c);
        check(o.endWhy == Why::Stopped && o.droneMovedAfterEndM < 3.0 && o.sendsAfterBrake == 0,
              "stopped by the operator: the drone stops, and nothing more is sent after the stop",
              std::string(whyName(o.endWhy)) + ", " + num(o.droneMovedAfterEndM) + " m more");
    }
}

void needsADroneThatObeys()
{
    {
        Case c;
        c.name = "the drone takes no speeds";
        c.object = walksEast(3.0);
        c.droneWill = [](double, Point, Point) { return Point{}; };
        c.seconds = 40.0;
        const Outcome o = run(c);
        check(o.endWhy == Why::NotObeyed && o.endedAtS < 22.0, "a drone that stands whatever it is asked: Follow ends",
              std::string(whyName(o.endWhy)) + " at " + num(o.endedAtS) + " s");
    }
    {
        // As if north and east were mixed up between the app and the drone.
        Case c;
        c.name = "the drone flies the speeds turned by 90 degrees";
        c.object = walksEast(3.0);
        c.droneWill = [](double, Point asked, Point) { return Point{asked.e, -asked.n}; };
        c.seconds = 40.0;
        const Outcome o = run(c);
        check(o.endWhy == Why::NotObeyed && o.endedAtS < 22.0, "a drone that flies another way than asked: Follow ends",
              std::string(whyName(o.endWhy)) + " at " + num(o.endedAtS) + " s");
        check(o.fromStartAtEnd < 45.0, "before it has gone far", num(o.fromStartAtEnd) + " m from its start");
    }
    {
        // The object is hidden at 30 s, so the drone is told to stand. This one flies on.
        Case c;
        c.name = "told to stand, the drone flies on";
        c.object = walksEast(4.0);
        c.seen = [](double t) { return t < 30.0; };
        c.droneWill = [](double t, Point asked, Point flying) { return t < 29.0 ? asked : flying; };
        c.seconds = 50.0;
        const Outcome o = run(c);
        check(o.endWhy == Why::NotObeyed && o.endedAtS > 34.0 && o.endedAtS < 38.5,
              "a drone that flies on when it is told to stand: Follow ends after 5 s",
              std::string(whyName(o.endWhy)) + " at " + num(o.endedAtS) + " s");
    }
    {
        // A drone that obeys is never taken for one that does not: every walk of followsWell
        // ran to its end. Here the hardest for this rule: a car, and a drone that is slow to answer.
        Case c;
        c.name = "a car, and nothing wrong with the drone";
        c.object = drivesOff(10.0, 1.5);
        c.speedLimit = 12.0;
        c.rangeLimit = 2000.0;
        c.seconds = 100.0;
        const Outcome o = run(c);
        check(o.followingAtEnd, "a drone that obeys is not taken for one that does not, after a car either", whyName(o.endWhy));
    }
}

void leavesTheDroneToThePilot()
{
    {
        Case c;
        c.name = "the pilot switches to Loiter";
        c.object = walksEast(1.4);
        c.pilot = [](double t, Mode m) { return t >= 40.0 ? Mode::Holding : m; };
        c.seconds = 60.0;
        const Outcome o = run(c);
        check(o.endWhy == Why::PilotTookOver && o.endedAtS < 41.0, "the pilot changes the mode: Follow ends within a second",
              std::string(whyName(o.endWhy)) + " at " + num(o.endedAtS) + " s");
        check(o.sentAfterEndS <= 1e-9 && o.sendsAfterBrake == 0, "the pilot changes the mode: nothing at all is sent afterwards",
              num(o.sentAfterEndS) + " s");
    }
    {
        Case c;
        c.name = "a mission mode";
        c.object = walksEast(1.4);
        c.pilot = [](double t, Mode m) { return t >= 40.0 ? Mode::Other : m; };
        c.seconds = 60.0;
        const Outcome o = run(c);
        check(o.endWhy == Why::PilotTookOver && o.sentAfterEndS <= 1e-9, "RTL, Land or a mission: Follow ends and sends nothing",
              whyName(o.endWhy));
    }
    {
        Case c;
        c.name = "the drone does not go to Guided";
        c.object = walksEast(1.4);
        c.droneRefusesGuided = true;
        c.seconds = 30.0;
        const Outcome o = run(c);
        check(o.startWhy == Why::None && o.endWhy == Why::NotGuided && o.endedAtS > c.startFollowAtS + 3.5 &&
                  o.endedAtS < c.startFollowAtS + 5.0,
              "the drone does not change to Guided: Follow gives up after 4 s",
              std::string(whyName(o.endWhy)) + " at " + num(o.endedAtS) + " s");
        check(o.sentAfterEndS <= 1e-9, "and sends nothing afterwards", num(o.sentAfterEndS));
    }
    {
        Case c;
        c.name = "telemetry stops";
        c.object = walksEast(1.4);
        c.telemetryComes = [](double t) { return t < 40.0; };
        c.seconds = 60.0;
        const Outcome o = run(c);
        check(o.endWhy == Why::DroneSilent && o.endedAtS > 41.5 && o.endedAtS < 43.5,
              "no telemetry for 2 s: Follow ends", std::string(whyName(o.endWhy)) + " at " + num(o.endedAtS) + " s");
        // After one second without telemetry the drone is told to stand, so it stands before the end.
        check(o.waitedS > 0.5 && o.droneMovedAfterEndM < 1.5, "no telemetry for 1 s: the drone is told to stand, and stands",
              num(o.waitedS) + " s waited, " + num(o.droneMovedAfterEndM) + " m more after the end");
        check(o.sentAfterEndS > 1.0 && o.sentAfterEndS < 2.0 && o.sendsAfterBrake == 0,
              "no telemetry: \"stop\" is sent for a second and a half after the end, then nothing more",
              num(o.sentAfterEndS) + " s");
    }
}

/// One guide with a drone in the air and an object it knows: for the start rules.
struct Ready
{
    FollowGuide guide;
    Drone drone;
    double t = 10.0;

    explicit Ready(double distanceM = 30.0)
    {
        drone.heightM = 30.0;
        drone.armed = true;
        drone.flying = true;
        drone.gps3d = true;
        drone.mode = Mode::Holding;
        toGeo(Point{-distanceM, 0.0}, drone.latDeg, drone.lonDeg);
        tell();
        for (int i = 0; i <= 20; ++i) {
            Fix f;
            toGeo(Point{0.0, 0.0}, f.latDeg, f.lonDeg);
            f.sigmaM = 1.5;
            guide.fix(f, t - 4.0 + 0.2 * i);
        }
    }
    void tell() { guide.drone(drone, t); }
};

void startsOnlyWhenItCan()
{
    {
        Ready r;
        check(r.guide.objectKnown(r.t), "start: fixes for four seconds make the object known");
        check(r.guide.start(true, r.t) == Why::None && r.guide.state() == State::Following, "start: all is well, Follow starts");
        check(std::abs(r.guide.keptDistanceM() - 30.0) < 0.5, "start: it keeps the 30 m of the start", num(r.guide.keptDistanceM()));
        check(std::abs(r.guide.distanceM(r.t) - 30.0) < 0.5, "start: the distance to the object is known", num(r.guide.distanceM(r.t)));
    }
    {
        Ready r;
        check(r.guide.start(false, r.t) == Why::NoLock && r.guide.state() == State::Off, "start: not without a lock that sees the object");
    }
    {
        Ready r;
        check(r.guide.start(true, r.t + 5.0) == Why::DroneSilent, "start: not when the drone's telemetry is 5 s old");
    }
    {
        Ready r;
        r.drone.armed = false;
        r.tell();
        check(r.guide.start(true, r.t) == Why::NotFlying, "start: not when the drone is not armed");
        r.drone.armed = true;
        r.drone.flying = false;
        r.tell();
        check(r.guide.start(true, r.t) == Why::NotFlying, "start: not when the drone is on the ground");
    }
    {
        Ready r;
        r.drone.gps3d = false;
        r.tell();
        check(r.guide.start(true, r.t) == Why::NoGps, "start: not without a 3D GPS position");
    }
    {
        Ready r;
        r.drone.mode = Mode::Other;
        r.tell();
        check(r.guide.start(true, r.t) == Why::WrongMode, "start: not out of a mission, RTL, Land or a pilot's mode");
        r.drone.mode = Mode::Guided;
        r.tell();
        check(r.guide.start(true, r.t) == Why::None, "start: from Guided it starts");
    }
    {
        Ready r;
        r.drone.heightM = 9.0;
        r.tell();
        check(r.guide.start(true, r.t) == Why::TooLow, "start: not under 10 m above the take-off point");
        r.drone.heightM = 10.0;
        r.tell();
        check(r.guide.start(true, r.t) == Why::None, "start: at 10 m it starts");
    }
    {
        // The laser found the object 9 m below the drone: the ground there is higher than the take-off point.
        Ready r;
        Fix f;
        toGeo(Point{0.0, 0.0}, f.latDeg, f.lonDeg);
        f.sigmaM = 1.5;
        f.belowM = 9.0;
        r.guide.fix(f, r.t);
        check(r.guide.start(true, r.t) == Why::TooLow, "start: not when the laser finds the object less than 10 m below the drone");
        f.belowM = 12.0;
        r.guide.fix(f, r.t + 0.2);
        check(r.guide.start(true, r.t + 0.2) == Why::None, "start: with the object 12 m below it starts");
        Ready old;
        f.belowM = 9.0;
        old.guide.fix(f, old.t - 20.0);
        check(old.guide.start(true, old.t) == Why::None, "start: a height above the object measured 20 s ago does not count");
    }
    {
        Ready r(19.0);
        check(r.guide.start(true, r.t) == Why::TooClose, "start: not nearer to the object than 20 m", num(r.guide.distanceM(r.t)));
        Ready far(21.0);
        check(far.guide.start(true, far.t) == Why::None, "start: at 21 m it starts");
    }
    {
        FollowGuide guide;
        Ready r;
        guide.drone(r.drone, 10.0);
        check(guide.start(true, 10.0) == Why::NoFix, "start: not before the object's position is known");
        Fix f;
        toGeo(Point{0.0, 0.0}, f.latDeg, f.lonDeg);
        for (int i = 0; i < 12; ++i) {
            guide.fix(f, 10.0 + 0.2 * i);  // 12 fixes over 2.2 s
        }
        guide.drone(r.drone, 12.2);
        check(guide.start(true, 12.2) == Why::NoFix, "start: fixes for two seconds are not enough (the object's speed is a guess still)");
        for (int i = 12; i <= 16; ++i) {
            guide.fix(f, 10.0 + 0.2 * i);
        }
        guide.drone(r.drone, 13.2);
        check(guide.start(true, 13.2) == Why::None, "start: after three seconds of fixes it starts");
        FollowGuide few;
        few.drone(r.drone, 20.0);
        for (int i = 0; i < 6; ++i) {
            few.fix(f, 16.0 + 0.8 * i);  // 6 fixes over 4 s
        }
        check(few.start(true, 20.0) == Why::NoFix, "start: six fixes are not enough, however long they took");
    }
    {
        // Fixes that are 12 m unsure each (a flat look at a far object): after 4 s the object's
        // place is still 3 m and more unsure, and that is not good enough to keep a distance by.
        FollowGuide guide;
        Ready r;
        guide.drone(r.drone, 10.0);
        Fix f;
        toGeo(Point{0.0, 0.0}, f.latDeg, f.lonDeg);
        f.sigmaM = 12.0;
        for (int i = 0; i <= 20; ++i) {
            guide.fix(f, 6.0 + 0.2 * i);
        }
        check(!guide.objectKnown(10.0) && guide.start(true, 10.0) == Why::NoFix,
              "start: not while the object's place is unsure by more than 2.5 m", num(guide.placeSigmaM(10.0), 2) + " m");
        check(r.guide.placeSigmaM(r.t) < 1.2, "(fixes of 1.5 m give a place sure to a metre)", num(r.guide.placeSigmaM(r.t), 2) + " m");
    }
    {
        Ready r;
        check(r.guide.start(true, r.t + 1.5) == Why::None, "start: a position 1.5 s old is still good", "");
        Ready old;
        old.drone.heightM = 30.0;
        old.t += 3.0;
        old.tell();
        check(old.guide.start(true, old.t) == Why::NoFix, "start: not with a position of the object that is 3 s old");
    }
    {
        Ready r;
        r.guide.forgetObject();
        check(r.guide.start(true, r.t) == Why::NoFix && r.guide.distanceM(r.t) < 0.0, "start: not after the object was forgotten (a new lock)");
    }
    {
        Ready r;
        check(r.guide.start(true, r.t) == Why::None, "start: once");
        check(r.guide.start(true, r.t + 0.2) == Why::None && r.guide.state() == State::Following, "start: asking again changes nothing");
    }
}

/// A guide that follows already: the drone was 21 m south of an object that moves east at
/// `mps`. Each step() tells the guide where the drone is now, seen from the object, with a
/// new fix of the object, and gives the speed the guide asks then.
struct Following
{
    FollowGuide guide;
    Drone drone;
    double t = 0.0;
    double mps;
    double sigma;
    double jumpEast = 0.0;  // the positions that come are of something this far east of the object

    explicit Following(double objectMps = 0.0, double keptM = 21.0, double fixSigma = 1.5) : mps(objectMps), sigma(fixSigma)
    {
        drone.heightM = 30.0;
        drone.armed = true;
        drone.flying = true;
        drone.gps3d = true;
        drone.mode = Mode::Guided;
        for (int i = 0; i <= 30; ++i) {
            t = 0.2 * i;
            fix();
        }
        place(-keptM, 0.0);
        guide.start(true, t);
    }
    void fix()
    {
        Fix f;
        toGeo(Point{0.0, mps * t + jumpEast}, f.latDeg, f.lonDeg);
        f.sigmaM = sigma;
        guide.fix(f, t);
    }
    /// The drone, seen from the object: it flies along with it.
    void place(double north, double east)
    {
        toGeo(Point{north, mps * t + east}, drone.latDeg, drone.lonDeg);
        drone.northMps = 0.0;
        drone.eastMps = mps;
        guide.drone(drone, t);
    }
    Velocity step(double north, double east)
    {
        t += 0.2;
        fix();
        place(north, east);
        return guide.tick(true, true, t);
    }
    /// The speed asked after two seconds at that place.
    Velocity settle(double north, double east)
    {
        Velocity v;
        for (int i = 0; i < 10; ++i) {
            v = step(north, east);
        }
        return v;
    }
    Point kept()
    {
        Point p;
        guide.keptPlace(p.n, p.e);
        return p;
    }
    /// The part of a speed that points at the object, where the guide sees the object.
    double towards(const Velocity &v)
    {
        double n = 0.0;
        double e = 0.0;
        guide.objectFromDrone(t, n, e);
        const double d = std::hypot(n, e);
        return d > 1e-9 ? (v.northMps * n + v.eastMps * e) / d : 0.0;
    }
    /// And the part beside it.
    double beside(const Velocity &v)
    {
        const double along = towards(v);
        return std::sqrt(std::max(0.0, v.northMps * v.northMps + v.eastMps * v.eastMps - along * along));
    }
};

void keepsAwayFromTheObject()
{
    {
        // 30 m from a standing object, 9 m further than the drone keeps: it flies towards its place.
        Following f;
        const Velocity v = f.settle(-30.0, 0.0);
        check(f.guide.state() == State::Following && v.northMps > 0.5 && std::abs(v.eastMps) < 1e-9,
              "(far from the object the drone flies to its place: towards the object)", num(v.northMps, 2) + " m/s");
    }
    {
        // 12 m from an object that walks past: no speed towards it, and none beside it. Straight away only.
        Following f(1.4);
        double mostTowards = -1e9;
        double mostBeside = 0.0;
        for (int i = 0; i < 15; ++i) {
            const Velocity v = f.step(-12.0, 0.0);
            mostTowards = std::max(mostTowards, f.towards(v));
            mostBeside = std::max(mostBeside, f.beside(v));
        }
        const Velocity v = f.step(-12.0, 0.0);
        check(mostTowards <= 1e-9, "inside 15 m no speed towards the object", num(mostTowards, 3) + " m/s");
        check(mostBeside <= 1e-6, "inside 15 m no speed beside the object either (it walks at 1.4 m/s)", num(mostBeside, 3) + " m/s");
        check(f.towards(v) < -0.5, "inside 15 m the drone flies straight away from the object", num(-f.towards(v), 2) + " m/s away");
        check(f.guide.why() == Why::Near, "and the guide says that the object is near");
    }
    {
        // A little outside 15 m the speed beside the object comes back, bit by bit.
        Following f(1.4);
        const Velocity at16 = f.settle(-16.0, 0.0);
        Following g(1.4);
        const Velocity at19 = g.settle(-19.0, 0.0);
        check(at16.eastMps > 0.05 && at16.eastMps < 0.8 && at19.eastMps > 1.0,
              "from 15 to 18 m the speed beside the object comes back bit by bit",
              num(at16.eastMps, 2) + " m/s at 16 m, " + num(at19.eastMps, 2) + " m/s at 19 m");
        check(f.guide.why() == Why::None, "(at 16 m the object is not \"near\")");
    }
    {
        // A fast object: where it is now is less sure, so speeds towards it stop further out.
        // 22 m from an object at 12 m/s (the drone keeps 21 m): no speed towards it.
        Following fast(12.0);
        fast.guide.setSpeedLimit(15.0);
        double most = -1e9;
        for (int i = 0; i < 15; ++i) {
            most = std::max(most, fast.towards(fast.step(-22.0, 0.0)));
        }
        check(most <= 1e-9 && fast.guide.objectSpeedMps() > 11.0,
              "a fast object (12 m/s): no speed towards it at 22 m either", num(most, 3) + " m/s");
        Following slow(1.4);
        const Velocity v = slow.settle(-22.0, 0.0);
        check(slow.towards(v) > 0.1, "(a walker at 22 m: the drone does fly towards its place)", num(slow.towards(v), 2) + " m/s");
    }
    {
        // An unsure place: fixes that are 40 m unsure each. Speeds towards the object stop two
        // of its standard deviations further out than 15 m.
        Following f;
        f.sigma = 40.0;
        for (int i = 0; i < 150; ++i) {
            f.step(-21.0, 0.0);
        }
        const double unsure = f.guide.placeSigmaM(f.t);
        const double at = 15.0 + 2.0 * unsure - 0.5;
        double most = -1e9;
        for (int i = 0; i < 25; ++i) {
            most = std::max(most, f.towards(f.step(-at, 0.0)));
        }
        check(unsure > 4.9 && most <= 1e-9,
              "an unsure place: no speed towards the object as far out as it is unsure",
              "unsure by " + num(unsure, 1) + " m: " + num(most, 3) + " m/s at " + num(at) + " m");
        Following sure;
        const Velocity v = sure.settle(-at, 0.0);
        check(sure.towards(v) > 0.3, "(with a sure place the drone does fly towards its place from there)", num(sure.towards(v), 2) + " m/s");
    }
    {
        // The object under the drone: no way is "away", so the drone stands.
        Following f;
        double most = 0.0;
        for (int i = 0; i < 15; ++i) {
            const Velocity v = f.step(-0.5, 0.3);
            most = std::max(most, std::hypot(v.northMps, v.eastMps));
        }
        check(most <= 1e-9, "the object under the drone: the drone stands", num(most, 3) + " m/s");
        const Velocity v = f.settle(-5.0, 0.0);
        check(f.towards(v) < -0.5 && f.beside(v) < 1e-6, "5 m to one side of it: the drone flies straight away from it",
              num(-f.towards(v), 2) + " m/s away");
    }
    {
        // The object has passed the drone: the drone is north of it now. Its place is there too.
        Following f;
        f.step(30.0, 0.0);
        const Point kept = f.kept();
        check(std::abs(kept.n - 21.0) < 0.5 && std::abs(kept.e) < 1.0,
              "the object has passed the drone: the drone keeps its distance on the side it is on now",
              num(kept.n) + " m north, " + num(kept.e) + " m east of the object");
        check(std::abs(f.guide.keptDistanceM() - 21.0) < 0.5, "at the same distance as before", num(f.guide.keptDistanceM()));
    }
    {
        // Right over the object the side is not sure: the place stays where it was.
        Following f;
        f.step(1.0, 0.0);
        const Point kept = f.kept();
        check(std::abs(kept.n + 21.0) < 0.5, "1 m past the object the side is not sure yet: the place stays", num(kept.n) + " m north");
    }
    {
        // Nearer than 15 m the drone has no side: its place is straight out from the object.
        Following f;
        f.step(-7.07, 7.07);
        const Point kept = f.kept();
        check(std::abs(kept.n + 14.85) < 0.6 && std::abs(kept.e - 14.85) < 0.6,
              "inside 15 m the drone's place is straight out from the object", num(kept.n) + " m north, " + num(kept.e) + " m east");
        // Further out it has: beside its place, the place stays.
        Following g;
        g.step(-18.0, 10.0);
        const Point same = g.kept();
        check(std::abs(same.n + 21.0) < 0.5 && std::abs(same.e) < 0.5, "outside 15 m the drone keeps its side",
              num(same.n) + " m north, " + num(same.e) + " m east");
    }
}

void waitsWhenTheObjectJumps()
{
    {
        // The lock takes another object, 40 m east of the first. One such position is a wrong
        // reading, and four are. Five in a row are believed. But an object is not somewhere
        // else all at once: the drone stands until the new position has held for 3 s.
        Following f;
        f.settle(-21.0, 0.0);
        f.jumpEast = 40.0;
        double most = 0.0;
        for (int i = 0; i < 4; ++i) {
            const Velocity v = f.step(-21.0, 0.0);
            most = std::max(most, std::hypot(v.northMps, v.eastMps));
        }
        check(f.guide.state() == State::Following && f.guide.jumps() == 0 && most <= 1e-9,
              "four positions 40 m off change nothing: the drone stays at its place", num(most, 3) + " m/s");
        Velocity v = f.step(-21.0, 0.0);
        v = f.step(-21.0, 0.0);
        check(f.guide.jumps() == 1 && f.guide.state() == State::Waiting && f.guide.why() == Why::Jumped && v.send,
              "the fifth is believed, and Follow waits: the position jumped", whyName(f.guide.why()));
        for (int i = 0; i < 12; ++i) {
            v = f.step(-21.0, 0.0);
            most = std::max(most, std::hypot(v.northMps, v.eastMps));
        }
        check(f.guide.state() == State::Waiting && most <= 1e-9, "2.4 s later it still waits, and the drone stands",
              num(most, 3) + " m/s");
        for (int i = 0; i < 6; ++i) {
            v = f.step(-21.0, 0.0);
        }
        check(f.guide.state() == State::Following && f.guide.why() == Why::None,
              "when the new position has held for 3 s, Follow goes on", whyName(f.guide.why()));
        v = f.settle(-21.0, 0.0);
        check(v.eastMps > 1.0 && std::abs(v.northMps) < 0.3, "and the drone flies to its place by the object's new position (east)",
              num(v.northMps, 2) + " north, " + num(v.eastMps, 2) + " east");
        const Point kept = f.kept();
        check(std::abs(kept.n + 21.0) < 0.5 && std::abs(kept.e) < 0.5, "at the distance and on the side it kept before",
              num(kept.n) + " m north, " + num(kept.e) + " m east");
        f.guide.forgetObject();
        check(f.guide.jumps() == 0, "a new object starts with no jumps", std::to_string(f.guide.jumps()));
    }
    {
        // On the bench: a walker, and at 40 s the lock is on someone else 30 m further east.
        Case c;
        c.name = "the lock jumps to another walker";
        c.object = [](double t) { return Point{0.0, 1.4 * std::max(0.0, t - 8.0) + (t >= 40.0 ? 30.0 : 0.0)}; };
        c.seconds = 100.0;
        const Outcome o = run(c);
        check(o.followingAtEnd && o.jumps == 1 && o.jumpWaitS > 2.4 && o.jumpWaitS < 4.5,
              "a jump on the bench: Follow waits about 3 s and goes on", std::to_string(o.jumps) + " jump, waited " + num(o.jumpWaitS) + " s");
        check(o.errorLate95 < 5.0, "and the drone is at its place by the new walker afterwards", num(o.errorLate95) + " m");
    }
    {
        // What walkers, runners and cars do is no jump: no case of followsWell waited for one.
        int jumps = 0;
        for (const auto &path : {walksSquare(1.4, 20.0), stopsAndGoes(1.4, 15.0, 12.0), runsACircle(4.0, 40.0), drivesOff(10.0, 1.5)}) {
            for (std::uint32_t seed = 1; seed <= 4; ++seed) {
                Case c;
                c.name = "no jump";
                c.object = path;
                c.speedLimit = 12.0;
                c.rangeLimit = 2000.0;
                c.seconds = 100.0;
                jumps += run(c, seed * 2654435761u).jumps;
            }
        }
        check(jumps == 0, "corners, stops, curves and a car that speeds up are not taken for jumps", std::to_string(jumps) + " in 16 runs");
    }
}

void endsWhenTheDroneIsDown()
{
    {
        // Another drone becomes the active one: Follow ends, and says nothing to it.
        Following f(1.4);
        f.settle(-21.0, 0.0);
        f.guide.drop(Why::DroneChanged);
        const Velocity v = f.guide.tick(true, true, f.t + 0.2);
        check(f.guide.state() == State::Off && f.guide.why() == Why::DroneChanged && !v.send && !f.guide.braking(f.t + 0.2),
              "another drone is the active one: Follow ends, and nothing is sent, not even a stop", whyName(f.guide.why()));
        check(f.guide.wouldStart(true, f.t + 0.2) == Why::None, "(the guide still knows the drone: forgetting it is a call of its own)");
        f.guide.forgetDrone();
        check(f.guide.wouldStart(true, f.t + 0.2) == Why::DroneSilent, "a drone that was forgotten is one without telemetry",
              whyName(f.guide.wouldStart(true, f.t + 0.2)));
    }
    {
        Following f(1.4);
        f.settle(-21.0, 0.0);
        f.drone.flying = false;  // it has landed
        const Velocity v = f.step(-21.0, 0.0);
        check(f.guide.state() == State::Off && f.guide.why() == Why::NotFlying && !v.send,
              "the drone lands during Follow: Follow ends, and nothing is sent to it", whyName(f.guide.why()));
        check(!f.guide.braking(f.t) && !f.guide.tick(true, true, f.t + 0.2).send, "not even a stop");
    }
    {
        Following f(1.4);
        f.settle(-21.0, 0.0);
        f.drone.armed = false;
        const Velocity v = f.step(-21.0, 0.0);
        check(f.guide.state() == State::Off && f.guide.why() == Why::NotFlying && !v.send,
              "the drone is disarmed during Follow: the same", whyName(f.guide.why()));
    }
}

void asksGently()
{
    // The drone is suddenly 40 m from its place (the object's position jumped, say). The speed
    // asked goes up by 2 m/s each second, no faster.
    Following f;
    double before = 0.0;
    double mostStep = 0.0;
    double reached = 0.0;
    for (int i = 0; i < 20; ++i) {
        const Velocity v = f.step(-61.0, 0.0);
        const double speed = std::hypot(v.northMps, v.eastMps);
        mostStep = std::max(mostStep, speed - before);
        before = speed;
        reached = speed;
    }
    check(mostStep <= 2.0 * kTickS + 1e-9, "the speed asked rises by 2 m/s each second at the most", num(mostStep / kTickS, 2) + " m/s2");
    check(reached > 7.0, "and does get to the speed that is needed", num(reached, 1) + " m/s after 4 s");
}

void keepsItsHeading()
{
    {
        // The guide has no word for up and down at all: a Velocity is north, east and the heading to keep.
        // The heading is the drone's own at the start, whatever its telemetry says later.
        Ready r;
        r.drone.headingDeg = 135.0;
        r.tell();
        r.guide.start(true, r.t);
        r.drone.mode = Mode::Guided;
        r.drone.headingDeg = 170.0;  // blown round, say
        r.t += 0.2;
        r.tell();
        const Velocity v = r.guide.tick(true, true, r.t);
        check(v.send && v.headingDeg == 135.0, "every speed comes with the heading of the start, to be kept", num(v.headingDeg));
        r.guide.stop(Why::Stopped, r.t);
        const Velocity brake = r.guide.tick(true, true, r.t + 0.2);
        check(brake.send && brake.headingDeg == 135.0 && brake.northMps == 0.0 && brake.eastMps == 0.0,
              "the stop at the end as well", num(brake.headingDeg));
        Ready unknown;
        unknown.guide.start(true, unknown.t);
        const Velocity w = unknown.guide.tick(true, true, unknown.t + 0.2);
        check(w.send && std::isnan(w.headingDeg), "a drone that tells no heading gets none back (the speed then goes with a yaw rate of 0)");
    }
}

void settingsStayInTheirRange()
{
    FollowGuide guide;
    check(guide.speedLimit() == kSpeedLimitDefaultMps && guide.rangeLimit() == kRangeLimitDefaultM, "settings: 8 m/s and 500 m to begin with");
    guide.setSpeedLimit(100.0);
    check(guide.speedLimit() == kSpeedLimitHighMps, "settings: the speed limit is 15 m/s at the most", num(guide.speedLimit()));
    guide.setSpeedLimit(0.0);
    check(guide.speedLimit() == kSpeedLimitLowMps, "settings: and 2 m/s at the least", num(guide.speedLimit()));
    guide.setSpeedLimit(std::nan(""));
    check(guide.speedLimit() == kSpeedLimitDefaultMps, "settings: no number gives the usual 8 m/s", num(guide.speedLimit()));
    guide.setRangeLimit(1e9);
    check(guide.rangeLimit() == kRangeLimitHighM, "settings: the range limit is 2000 m at the most", num(guide.rangeLimit()));
    guide.setRangeLimit(-5.0);
    check(guide.rangeLimit() == kRangeLimitLowM, "settings: and 50 m at the least", num(guide.rangeLimit()));
    check(kMinDistanceM >= 15.0 && kStartDistanceM >= kMinDistanceM + 5.0 && kMinHeightM >= 10.0,
          "the least distance (15 m), the start distance (20 m) and the least height (10 m) are not settings");
}

void knowsWhereTheObjectIs()
{
    Ready r;
    double lat = 0.0;
    double lon = 0.0;
    check(r.guide.objectPosition(r.t, lat, lon), "the object's position can be read");
    double wantLat = 0.0;
    double wantLon = 0.0;
    toGeo(Point{0.0, 0.0}, wantLat, wantLon);
    check(std::abs(lat - wantLat) < 2e-5 && std::abs(lon - wantLon) < 2e-5, "and it is where the fixes were",
          num((lat - wantLat) * 111320.0, 2) + " m north off");
    // A walking object: its speed is found.
    FollowGuide guide;
    for (int i = 0; i <= 50; ++i) {
        Fix f;
        toGeo(Point{0.0, 1.4 * 0.2 * i}, f.latDeg, f.lonDeg);
        f.sigmaM = 1.5;
        guide.fix(f, 0.2 * i);
    }
    check(std::abs(guide.objectSpeedMps() - 1.4) < 0.15, "a walker's speed is found from the fixes (1.4 m/s)", num(guide.objectSpeedMps(), 2));
    // One fix 40 m away is not believed. Six in a row are.
    Fix away;
    toGeo(Point{40.0, 14.0}, away.latDeg, away.lonDeg);
    away.sigmaM = 1.5;
    guide.fix(away, 10.2);
    guide.objectPosition(10.2, lat, lon);
    toGeo(Point{0.0, 14.0}, wantLat, wantLon);
    check(std::abs(lat - wantLat) * 111320.0 < 3.0, "one fix 40 m away does not move the estimate",
          num((lat - wantLat) * 111320.0, 1) + " m");
    for (int i = 0; i < 6; ++i) {
        guide.fix(away, 10.4 + 0.2 * i);
    }
    guide.objectPosition(11.6, lat, lon);
    toGeo(Point{40.0, 14.0}, wantLat, wantLon);
    check(std::abs(lat - wantLat) * 111320.0 < 3.0, "six in a row do: the object is there",
          num((lat - wantLat) * 111320.0, 1) + " m");
}

/// For choosing the guide's numbers: the same walks with one line of numbers each.
/// "off": how far the drone is from its place (m); "changing": how restless the speed asked is.
void bench()
{
    struct Row
    {
        const char *name;
        Case c;
    };
    std::vector<Row> rows;
    auto add = [&rows](const char *name, std::function<Point(double)> object, double seconds, double sigma = 1.2,
                       double limit = kSpeedLimitDefaultMps) {
        Case c;
        c.name = name;
        c.object = std::move(object);
        c.seconds = seconds;
        c.fixSigmaM = sigma;
        c.speedLimit = limit;
        c.rangeLimit = kRangeLimitHighM;
        rows.push_back({name, c});
    };
    add("stands", stands, 90.0);
    add("walk", walksEast(1.4), 120.0);
    add("square", walksSquare(1.4, 20.0), 120.0);
    add("stopgo", stopsAndGoes(1.4, 15.0, 12.0), 140.0);
    add("circle4", runsACircle(4.0, 40.0), 120.0);
    add("car10", drivesOff(10.0, 1.5), 100.0, 1.2, 12.0);
    add("square_s2", walksSquare(1.4, 20.0), 120.0, 2.0);
    add("stands_s2", stands, 90.0, 2.0);
    for (const Row &row : rows) {
        double mean = 0.0;
        double p95 = 0.0;
        double most = 0.0;
        double change = 0.0;
        double late95 = 0.0;
        double still = 0.0;
        double ripple = 0.0;
        const int seeds = 8;
        for (int seed = 1; seed <= seeds; ++seed) {
            const Outcome o = run(row.c, static_cast<std::uint32_t>(seed) * 7919u);
            mean += o.errorMean / seeds;
            p95 = std::max(p95, o.error95);
            most = std::max(most, o.errorMax);
            change += o.askedChangeMps2 / seeds;
            late95 = std::max(late95, o.errorLate95);
            still += o.stillAskedShare / seeds;
            ripple += o.rippleMps / seeds;
        }
        std::printf("BENCH %-10s off mean %5.2f worst95 %5.2f most %5.2f late95 %5.2f changing %5.2f moving %4.2f ripple %4.2f\n",
                    row.name, mean, p95, most, late95, change, still, ripple);
    }
}

}  // namespace

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-v") {
            g_verbose = true;
        }
        if (std::string(argv[i]) == "-bench") {
            bench();
            return 0;
        }
        if (std::string(argv[i]) == "-trace" && i + 1 < argc) {
            g_trace = argv[++i];
        }
    }
    followsWell();
    keepsItsLimits();
    stopsWithoutTheObject();
    needsADroneThatObeys();
    leavesTheDroneToThePilot();
    startsOnlyWhenItCan();
    keepsAwayFromTheObject();
    waitsWhenTheObjectJumps();
    endsWhenTheDroneIsDown();
    asksGently();
    keepsItsHeading();
    settingsStayInTheirRange();
    knowsWhereTheObjectIs();
    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
