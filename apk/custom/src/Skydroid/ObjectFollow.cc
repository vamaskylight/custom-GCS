#include "ObjectFollow.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QSettings>
#include <QtQml/QJSEngine>

#include <cmath>

#include "SkydroidLink.h"

using skydroid::follow::Drone;
using skydroid::follow::Fix;
using skydroid::follow::State;
using skydroid::follow::Velocity;
using skydroid::follow::Why;

namespace {

constexpr const char *kSettingsGroup = "VamaFollow";
constexpr int kTickMs = 200;  // skydroid::follow::kTickS

}  // namespace

ObjectFollow::ObjectFollow(SkydroidLink *link, FollowDrone *drone, QObject *parent)
    : QObject(parent)
    , _link(link)
    , _drone(drone)
{
    _drone->setParent(this);
    _clock.start();
    _timer.setInterval(kTickMs);
    connect(&_timer, &QTimer::timeout, this, &ObjectFollow::_tick);
    connect(_drone, &FollowDrone::telemetry, this, &ObjectFollow::_telemetry);
    connect(_drone, &FollowDrone::otherDrone, this, &ObjectFollow::_otherDrone);
    connect(_link, &SkydroidLink::objectPlaced, this, &ObjectFollow::_placed);
    connect(_link, &SkydroidLink::lockChanged, this, &ObjectFollow::_lockChanged);

    QSettings settings;
    settings.beginGroup(kSettingsGroup);
    // A saved value that is no number gives the usual one (the guide does that for "not a
    // number"). A number outside the range is brought into it.
    const auto saved = [&settings](const QString &name) {
        bool ok = false;
        const double value = settings.value(name).toDouble(&ok);
        return ok ? value : std::nan("");
    };
    _guide.setSpeedLimit(saved(QStringLiteral("speedLimit")));
    _guide.setRangeLimit(saved(QStringLiteral("rangeLimit")));
    settings.endGroup();
    _lockChanged();
}

ObjectFollow::~ObjectFollow()
{
    // Never leave the drone flying a speed when this goes away.
    if (active()) {
        _drone->sendSpeed(0.0, 0.0, std::nan(""));
    }
}

ObjectFollow *ObjectFollow::instance()
{
    static ObjectFollow *follow = new ObjectFollow(SkydroidLink::instance(), vamaMakeFollowDrone(nullptr),
                                                   QCoreApplication::instance());
    return follow;
}

ObjectFollow *ObjectFollow::create(QQmlEngine *qmlEngine, QJSEngine *jsEngine)
{
    Q_UNUSED(qmlEngine);
    Q_UNUSED(jsEngine);
    ObjectFollow *follow = instance();
    QJSEngine::setObjectOwnership(follow, QJSEngine::CppOwnership);
    return follow;
}

double ObjectFollow::fliesOnS() const
{
    return _drone->fliesOnS();
}

void ObjectFollow::setSpeedLimit(double mps)
{
    const double before = _guide.speedLimit();
    _guide.setSpeedLimit(mps);
    if (_guide.speedLimit() == before) {
        return;
    }
    QSettings settings;
    settings.beginGroup(kSettingsGroup);
    settings.setValue(QStringLiteral("speedLimit"), _guide.speedLimit());
    settings.endGroup();
    emit settingsChanged();
}

void ObjectFollow::setRangeLimit(double metres)
{
    const double before = _guide.rangeLimit();
    _guide.setRangeLimit(metres);
    if (_guide.rangeLimit() == before) {
        return;
    }
    QSettings settings;
    settings.beginGroup(kSettingsGroup);
    settings.setValue(QStringLiteral("rangeLimit"), _guide.rangeLimit());
    settings.endGroup();
    emit settingsChanged();
}

void ObjectFollow::_telemetry(const Drone &drone)
{
    _guide.drone(drone, _now());
}

void ObjectFollow::_otherDrone()
{
    // Follow never goes on with another drone than the one it was started on. And it says
    // nothing to the new one, not even "stop": that drone may be doing something else.
    _guide.drop(Why::DroneChanged);
    _guide.forgetDrone();  // where the last drone was says nothing of this one
    _refresh();
    _keepTicking();
}

void ObjectFollow::_placed(double latDeg, double lonDeg, double sigmaM, double belowM, int ageMs)
{
    Fix fix;
    fix.latDeg = latDeg;
    fix.lonDeg = lonDeg;
    fix.sigmaM = sigmaM;
    fix.belowM = belowM;
    _guide.fix(fix, _now() - std::max(0, ageMs) / 1000.0);
    _byLaser = std::isfinite(belowM);
}

void ObjectFollow::_lockChanged()
{
    const bool lock = _link->lockActive() && _link->lockByApp();
    if (lock != _lockWasActive) {
        _lockWasActive = lock;
        if (!lock && active()) {
            _tick();  // the lock ended: the drone is told to stop now, not on the next tick
        }
        // The positions belong to the object that was locked, not to the next one.
        _guide.forgetObject();
        _byLaser = false;
    }
    _keepTicking();
    _refresh();
}

void ObjectFollow::_keepTicking()
{
    // While Follow runs, while the stop at its end goes out, and while a lock could be followed
    // (so the screen says whether Follow would start).
    const bool needed = active() || _guide.braking(_now()) || _lockWasActive;
    if (needed && !_timer.isActive()) {
        _timer.start();
    } else if (!needed && _timer.isActive()) {
        _timer.stop();
    }
}

void ObjectFollow::start()
{
    if (active()) {
        return;
    }
    const QString whyNot = _drone->whyNot();
    if (!whyNot.isEmpty()) {
        _setMessage(whyNot);
        return;
    }
    if (_link->lockActive() && !_link->lockByApp()) {
        _setMessage(tr("Follow needs the app's own lock. Choose \"The app follows the object\" in the camera settings."));
        return;
    }
    const bool lock = _link->lockActive() && _link->lockByApp();
    const Why why = _guide.start(lock && _link->lockSeen(), _now());
    if (why != Why::None) {
        _setMessage(text(why));
        return;
    }
    if (_guide.lastDrone().mode != skydroid::follow::Mode::Guided) {
        _drone->askGuided();
    }
    _wasActive = true;
    _keepTicking();
    _refresh();
}

void ObjectFollow::stop()
{
    if (!active()) {
        return;
    }
    _guide.stop(Why::Stopped, _now());
    _tick();  // the stop goes out now
}

void ObjectFollow::clearMessage()
{
    if (!active()) {
        _setMessage(QString());
    }
}

void ObjectFollow::_tick()
{
    const bool lock = _link->lockActive() && _link->lockByApp();
    const Velocity v = _guide.tick(lock, lock && _link->lockSeen(), _now());
    if (v.send) {
        _drone->sendSpeed(v.northMps, v.eastMps, v.headingDeg);
    }
    _refresh();
    _keepTicking();
}

void ObjectFollow::_setMessage(const QString &message)
{
    if (message != _message) {
        _message = message;
        _refresh();
    }
}

QString ObjectFollow::text(Why why) const
{
    const double now = _now();
    const Drone &drone = _guide.lastDrone();
    const QString hold = tr("The drone holds its place in Guided.");
    switch (why) {
    case Why::None:
        return active() ? tr("Following. The object is %1 m away.").arg(_guide.distanceM(now), 0, 'f', 0) : QString();
    // It does not start:
    case Why::NoLock:
        return tr("Follow needs a locked object that the app sees. Lock the object first.");
    case Why::DroneSilent:
        return active() || _wasActive ? tr("Follow ended: the drone sends no data. It stops by itself.")
                                      : tr("Follow cannot start: the drone sends no position.");
    case Why::NotFlying:
        return active() || _wasActive ? tr("Follow ended: the drone is not flying.")
                                      : tr("Follow cannot start: the drone is not flying.");
    case Why::NoGps:
        return tr("Follow cannot start: the drone has no 3D GPS position.");
    case Why::WrongMode:
        return tr("Follow starts from Loiter, Position Hold, Brake or Guided. The drone is in %1.").arg(_drone->modeName());
    case Why::TooLow: {
        if (_wasActive) {
            // Lower than it may be above the take-off point, or (the laser measured it) above the object.
            const QString low = drone.heightM >= skydroid::follow::kEndHeightM
                                    ? tr("Follow ended: the drone is less than %1 m above the object.")
                                    : tr("Follow ended: the drone is lower than %1 m.");
            return low.arg(skydroid::follow::kEndHeightM, 0, 'f', 0) + QLatin1Char(' ') + hold;
        }
        const double below = _guide.belowM(now);
        if (drone.heightM >= skydroid::follow::kMinHeightM && below >= 0.0) {
            // The ground at the object is higher than the take-off point.
            return tr("Follow cannot start: the drone is only %1 m above the object. It needs %2 m.")
                .arg(below, 0, 'f', 0).arg(skydroid::follow::kMinHeightM, 0, 'f', 0);
        }
        return tr("Follow cannot start: the drone is %1 m up. It needs %2 m.")
            .arg(drone.heightM, 0, 'f', 0).arg(skydroid::follow::kMinHeightM, 0, 'f', 0);
    }
    case Why::NoFix:
        return tr("Follow cannot start yet: the object's position is not known well enough. Keep it locked for a few "
                  "seconds. If this stays, tilt the camera further down or fly nearer.");
    case Why::TooClose:
        return tr("Follow cannot start: the object is %1 m away. It needs %2 m or more.")
            .arg(_guide.distanceM(now), 0, 'f', 0).arg(skydroid::follow::kStartDistanceM, 0, 'f', 0);
    // It follows, and:
    case Why::Near:
        return tr("The object is nearer than %1 m. The drone does not fly towards it.").arg(skydroid::follow::kMinDistanceM, 0, 'f', 0);
    // It waits:
    case Why::DroneLate:
        return tr("Follow waits: the drone's data is late. The drone stands.");
    case Why::NotSeen:
        return tr("Follow waits: the object is not seen. The drone stands.");
    case Why::FixOld:
        return tr("Follow waits: no new position of the object. The drone stands.");
    case Why::Jumped:
        return tr("Follow waits: the object's position jumped. The drone stands until it is sure again.");
    // It ended:
    case Why::Stopped:
        return tr("Follow stopped.") + QLatin1Char(' ') + hold;
    case Why::LockEnded:
        return tr("Follow ended: the lock ended.") + QLatin1Char(' ') + hold;
    case Why::NotGuided:
        return tr("Follow ended: the drone did not change to Guided. It is in %1.").arg(_drone->modeName());
    case Why::PilotTookOver:
        return tr("Follow ended: the flight mode was changed to %1.").arg(_drone->modeName());
    case Why::TooFar:
        return tr("Follow ended: the drone is %1 m from where Follow started, the limit.").arg(_guide.rangeLimit(), 0, 'f', 0) +
               QLatin1Char(' ') + hold;
    case Why::FixLost:
        return tr("Follow ended: the object was not found again.") + QLatin1Char(' ') + hold;
    case Why::NotObeyed:
        return tr("Follow ended: the drone did not fly the speeds it was asked. Take control of the drone.");
    case Why::DroneChanged:
        return tr("Follow ended: another drone is the active one now.");
    }
    return QString();
}

void ObjectFollow::_refresh()
{
    const double now = _now();
    const bool isActive = active();
    if (isActive) {
        // What it does now: one line that changes with the state.
        _message = text(_guide.why());
        _wasActive = true;
    } else if (_wasActive) {
        // It has just ended: why. The line stays until it is tapped away or Follow starts again.
        _message = text(_guide.why());
        _wasActive = false;
    }

    const bool lock = _link->lockActive() && _link->lockByApp();
    const double droneAge = _guide.droneAgeS(now);
    _heightM = (droneAge >= 0.0 && droneAge <= skydroid::follow::kDroneSilentS) ? _guide.lastDrone().heightM : -1.0;
    _distanceM = (lock && (isActive || _guide.objectKnown(now))) ? _guide.distanceM(now) : -1.0;

    QString notReady;
    if (!isActive) {
        notReady = _drone->whyNot();
        if (notReady.isEmpty() && _link->lockActive() && !_link->lockByApp()) {
            notReady = tr("Follow needs the app's own lock. Choose \"The app follows the object\" in the camera settings.");
        }
        if (notReady.isEmpty()) {
            const Why why = _guide.wouldStart(lock && _link->lockSeen(), now);
            notReady = why == Why::None ? QString() : text(why);
        }
    }
    _ready = !isActive && notReady.isEmpty();
    _notReadyText = notReady;

    // QML is told when something it shows has changed: whole metres, tenths of a metre a second.
    const QString told = QStringList{QString::number(isActive), QString::number(waiting()), _message,
                                     QString::number(_ready), _notReadyText, QString::number(qRound(_distanceM)),
                                     QString::number(qRound(keptDistanceM())), QString::number(qRound(_heightM)),
                                     QString::number(qRound(askedSpeedMps() * 10.0)), QString::number(_byLaser),
                                     QString::number(qRound(fliesOnS() * 10.0))}
                             .join(QLatin1Char('|'));
    if (told != _told) {
        _told = told;
        emit changed();
    }
}
