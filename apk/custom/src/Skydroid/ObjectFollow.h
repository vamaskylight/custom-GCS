#pragma once

// Follow Me in the app: the drone moves with the object that the app has
// locked (SkydroidLink), and keeps its distance to it.
//
// The pieces:
//  - SkydroidLink places the locked object on the ground, picture by picture
//    (objectPlaced), and says whether the lock still sees it.
//  - FollowGuide (plain C++, tested on the host and flown in ArduCopter's
//    simulator) turns those places and the drone's telemetry into a speed over
//    the ground, five times a second, and holds every rule of Follow.
//  - FollowDrone (FollowDrone.h) is the drone: telemetry in, speed out. The
//    app's is VehicleFollowDrone, on QGC's active vehicle.
//  - This class runs the clock, starts and stops Follow for the operator, and
//    says in plain words what is going on.
//
// QML uses it as the singleton "ObjectFollow" from the QGC module.

#include <QtCore/QElapsedTimer>
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QTimer>
#include <QtQmlIntegration/QtQmlIntegration>

#include "FollowDrone.h"
#include "FollowGuide.h"

class QJSEngine;
class QQmlEngine;
class SkydroidLink;

class ObjectFollow : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON

    /// Follow runs: the drone is flown by the app (following, or waiting for the object).
    Q_PROPERTY(bool active READ active NOTIFY changed)
    /// It runs, but the drone is told to stand: the object is not seen, or its position is old.
    Q_PROPERTY(bool waiting READ waiting NOTIFY changed)
    /// What Follow does, why it does not start, or why it ended. Empty: nothing to say.
    Q_PROPERTY(QString message READ message NOTIFY changed)
    /// Follow would start now.
    Q_PROPERTY(bool ready READ ready NOTIFY changed)
    /// Why it would not: one sentence for the operator. Empty when ready.
    Q_PROPERTY(QString notReadyText READ notReadyText NOTIFY changed)
    /// Drone to object over the ground, metres, as the app works it out. Negative: not known.
    Q_PROPERTY(double distanceM READ distanceM NOTIFY changed)
    /// The distance Follow keeps (the distance at the start).
    Q_PROPERTY(double keptDistanceM READ keptDistanceM NOTIFY changed)
    /// The drone's height above its take-off point, metres. Negative: not known.
    Q_PROPERTY(double heightM READ heightM NOTIFY changed)
    /// The speed asked from the drone now, m/s.
    Q_PROPERTY(double askedSpeedMps READ askedSpeedMps NOTIFY changed)
    /// The object's place comes with a height above it that the laser measured (else level ground is assumed).
    Q_PROPERTY(bool byLaser READ byLaser NOTIFY changed)
    /// How long the drone flies on if the app or the radio link stops, seconds. Negative: not known.
    Q_PROPERTY(double fliesOnS READ fliesOnS NOTIFY changed)

    // Settings, saved between runs.
    Q_PROPERTY(double speedLimit READ speedLimit WRITE setSpeedLimit NOTIFY settingsChanged)
    Q_PROPERTY(double rangeLimit READ rangeLimit WRITE setRangeLimit NOTIFY settingsChanged)

    // What does not change: for the texts on screen.
    Q_PROPERTY(double minDistanceM READ minDistanceM CONSTANT)
    Q_PROPERTY(double startDistanceM READ startDistanceM CONSTANT)
    Q_PROPERTY(double minHeightM READ minHeightM CONSTANT)

public:
    /// link: the camera link whose lock is followed. drone: the drone to fly (this object takes it over).
    ObjectFollow(SkydroidLink *link, FollowDrone *drone, QObject *parent = nullptr);
    ~ObjectFollow() override;

    static ObjectFollow *instance();
    static ObjectFollow *create(QQmlEngine *qmlEngine, QJSEngine *jsEngine);

    bool active() const { return _guide.state() != skydroid::follow::State::Off; }
    bool waiting() const { return _guide.state() == skydroid::follow::State::Waiting; }
    QString message() const { return _message; }
    bool ready() const { return _ready; }
    QString notReadyText() const { return _notReadyText; }
    double distanceM() const { return _distanceM; }
    double keptDistanceM() const { return active() ? _guide.keptDistanceM() : -1.0; }
    double heightM() const { return _heightM; }
    double askedSpeedMps() const { return active() ? _guide.askedSpeedMps() : 0.0; }
    bool byLaser() const { return _byLaser; }
    double fliesOnS() const;
    double speedLimit() const { return _guide.speedLimit(); }
    void setSpeedLimit(double mps);
    double rangeLimit() const { return _guide.rangeLimit(); }
    void setRangeLimit(double metres);
    double minDistanceM() const { return skydroid::follow::kMinDistanceM; }
    double startDistanceM() const { return skydroid::follow::kStartDistanceM; }
    double minHeightM() const { return skydroid::follow::kMinHeightM; }

    /// The operator starts Follow on the locked object. When it cannot start, message() says why.
    Q_INVOKABLE void start();
    /// The operator stops it. The drone is told to stop, and holds its place in Guided.
    Q_INVOKABLE void stop();
    /// The message was read (a tap on it).
    Q_INVOKABLE void clearMessage();

    /// One sentence for a reason. During Follow: what it does.
    QString text(skydroid::follow::Why why) const;

signals:
    void changed();
    void settingsChanged();

private slots:
    void _tick();

private:
    void _telemetry(const skydroid::follow::Drone &drone);
    void _otherDrone();
    void _placed(double latDeg, double lonDeg, double sigmaM, double belowM, int ageMs);
    void _lockChanged();
    double _now() const { return _clock.elapsed() / 1000.0; }
    /// Works out what the screen shows, and tells QML when any of it changed.
    void _refresh();
    void _setMessage(const QString &message);
    void _keepTicking();

    SkydroidLink *_link = nullptr;
    FollowDrone *_drone = nullptr;
    skydroid::follow::FollowGuide _guide;
    QElapsedTimer _clock;
    QTimer _timer;
    bool _lockWasActive = false;
    bool _wasActive = false;
    QString _message;
    bool _ready = false;
    QString _notReadyText;
    double _distanceM = -1.0;
    double _heightM = -1.0;
    bool _byLaser = false;
    // What QML was last told, so that it is told only of changes.
    QString _told;
};
