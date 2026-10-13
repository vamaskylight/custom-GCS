// Host test for SkydroidLink: the real Qt link code talks UDP to fake cameras
// in the same process. Checks gimbal angles (GAA push and GAC questions), the
// switch to another address when the configured one gives no angles, speed
// motion from touch and RC wheels, the frames each button sends, the laser
// sequence (laser module first, system address as fallback), the target
// position rules, the object lock (the app's own: pictures in, gimbal speeds
// out; and the camera's own tracker: turn, GOT, SUM confirm and stop), the
// thermal colour mode setting, where a locked object is on the ground,
// Follow Me (ObjectFollow with a stand-in drone: when it starts, what it sends,
// when it ends and what it says), and the marks on people and vehicles
// (ObjectMarks with a stand-in for the network: when it looks, what it shows,
// and the lock that a tap on a mark starts on the picture of a moment ago).
//
// Every address used here is on this computer (127.0.0.x).

#include <QtCore/QDir>
#include <QtCore/QElapsedTimer>
#include <QtCore/QMutex>
#include <QtCore/QSettings>
#include <QtCore/QThread>
#include <QtGui/QImage>
#include <QtNetwork/QNetworkDatagram>
#include <QtNetwork/QUdpSocket>
#include <QtTest/QSignalSpy>
#include <QtTest/QtTest>

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "FollowGuide.h"
#include "LaserGeo.h"
#include "MultiVehicleManager.h"
#include "ObjectFollow.h"
#include "ObjectMarks.h"
#include "SkydroidLink.h"
#include "SkydroidTop.h"
#include "Vehicle.h"

// VGCS's thermal colour modes (made by apk/make_thermal_palettes.py).
#include "../thermal_palette_vectors.inc"

namespace top = skydroid::top;

constexpr double kPi = 3.14159265358979323846;

class FakeCamera : public QObject
{
    Q_OBJECT

public:
    FakeCamera()
    {
        _socket.bind(QHostAddress::LocalHost, 0);
        connect(&_socket, &QUdpSocket::readyRead, this, &FakeCamera::_read);
        connect(&_pushTimer, &QTimer::timeout, this, &FakeCamera::_push);
        connect(&_trackTimer, &QTimer::timeout, this, [this]() { yaw += trackDriftDps * 0.1; });
        connect(&_speedTimer, &QTimer::timeout, this, [this]() {
            const double seconds = _speedClock.restart() / 1000.0;
            yaw -= yawDps * seconds;
            pitch = std::clamp(pitch + pitchDps * seconds, -90.0, 10.0);
        });
        _clock.start();
    }

    quint16 port() const { return _socket.localPort(); }

    bool answerGac = true;       // answer a GAC question
    bool pushAfterGaa = false;   // send angles on its own after GAA, like the camera's push
    bool followAngles = false;   // turn to GAY / GAP angle commands, like the real gimbal
    // Turn at the speeds of GSY, GSP and GSM, like the real gimbal. Right is + in the speed
    // commands and - in the C13's yaw.
    bool followSpeeds = false;
    double yawDps = 0.0;
    double pitchDps = 0.0;
    // Act only on long frames that start with an upper-case "#TP", like the
    // client's V13 (test build 3: angle commands sent as "#tp" did nothing).
    bool upperCaseOnly = false;
    double yaw = 0.0;
    double pitch = -30.0;
    std::optional<std::string> slrE;  // SLR data field from the laser module
    std::optional<std::string> slrD;  // SLR data field from the system address
    std::optional<int> dzmStep;
    int laserModuleDelayMs = 0;  // answer E reads late
    // After a SUM confirm the yaw drifts this fast, like a camera following an
    // object, until a SUM stop.
    double trackDriftDps = 0.0;
    // Half a second after the first SUM confirm the yaw jumps once by this much,
    // like the V13 in the field video of test build 4 (its tracker took over and
    // moved the aim).
    double lockJumpDeg = 0.0;
    bool tracking = false;
    QStringList received;
    QList<qint64> receivedAtMs;  // arrival time of each frame in received

    int countStartingWith(const QString &prefix) const
    {
        int n = 0;
        for (const QString &f : received) {
            if (f.startsWith(prefix)) {
                ++n;
            }
        }
        return n;
    }
    int countEqual(const std::string &frame) const { return received.count(QString::fromStdString(frame)); }
    int indexOf(const std::string &frame) const { return received.indexOf(QString::fromStdString(frame)); }
    int firstStartingWith(const QString &prefix) const
    {
        for (int i = 0; i < received.size(); ++i) {
            if (received.at(i).startsWith(prefix)) {
                return i;
            }
        }
        return -1;
    }
    void clear()
    {
        received.clear();
        receivedAtMs.clear();
    }

private slots:
    void _read()
    {
        while (_socket.hasPendingDatagrams()) {
            const QNetworkDatagram d = _socket.receiveDatagram();
            const std::string raw(d.data().constData(), static_cast<size_t>(d.data().size()));
            received << QString::fromStdString(raw);
            receivedAtMs << _clock.elapsed();
            const auto f = top::parseTpFrame(raw);
            if (!f || (upperCaseOnly && raw.rfind("#tp", 0) == 0)) {
                continue;
            }
            if ((f->tag == "GAY" || f->tag == "GAP") && f->ctrl == 'w' && followAngles && f->data.size() >= 4) {
                const auto angle = top::decodeAttitudeField4(f->data.substr(0, 4));
                if (angle) {
                    (f->tag == "GAY" ? yaw : pitch) = *angle;
                }
                continue;
            }
            if ((f->tag == "GSY" || f->tag == "GSP" || f->tag == "GSM") && f->ctrl == 'w' && followSpeeds) {
                auto speed = [](const std::string &hex) {
                    const int raw = std::stoi(hex, nullptr, 16);
                    return (raw > 127 ? raw - 256 : raw) * 0.5;
                };
                if (f->tag == "GSM" && f->data.size() >= 4) {
                    yawDps = speed(f->data.substr(0, 2));
                    pitchDps = speed(f->data.substr(2, 2));
                } else if (f->data.size() >= 2) {
                    (f->tag == "GSY" ? yawDps : pitchDps) = speed(f->data.substr(0, 2));
                }
                if (!_speedTimer.isActive()) {
                    _speedClock.start();
                    _speedTimer.start(20);
                }
                continue;
            }
            if (f->tag == "SUM" && f->ctrl == 'w') {
                const bool wasTracking = tracking;
                tracking = f->data == "01";
                if (tracking && !wasTracking && lockJumpDeg != 0.0) {
                    // Half a second later, as on the real camera.
                    QTimer::singleShot(500, this, [this]() { yaw += lockJumpDeg; });
                }
                if (!tracking) {
                    _trackTimer.stop();
                } else if (trackDriftDps != 0.0 && !_trackTimer.isActive()) {
                    _trackTimer.start(100);
                }
                continue;
            }
            if (f->tag == "GAA" && f->ctrl == 'w' && pushAfterGaa) {
                const int hz = std::stoi(f->data, nullptr, 16);
                _pushTo = d;
                if (hz > 0) {
                    _pushTimer.start(1000 / hz);
                } else {
                    _pushTimer.stop();
                }
                continue;
            }
            if (f->ctrl != 'r') {
                continue;
            }
            const char dest = f->address.size() == 2 ? f->address[1] : ' ';
            if (f->tag == "GAC" && answerGac) {
                _reply(d, _gacFrame());
            } else if (f->tag == "SLR" && dest == 'E' && slrE) {
                const std::string frame = top::buildTpFrame('U', 'r', "SLR", *slrE, 'E', 1);
                if (laserModuleDelayMs > 0) {
                    QTimer::singleShot(laserModuleDelayMs, this, [this, d, frame]() { _reply(d, frame); });
                } else {
                    _reply(d, frame);
                }
            } else if (f->tag == "SLR" && dest == 'D' && slrD) {
                _reply(d, top::buildTpFrame('U', 'r', "SLR", *slrD, 'D', 1));
            } else if (f->tag == "DZM" && dzmStep) {
                char buf[4];
                std::snprintf(buf, sizeof(buf), "%02X", *dzmStep);
                _reply(d, top::buildTpFrame('U', 'r', "DZM", buf, 'D', 0));
            }
        }
    }

    void _push() { _reply(_pushTo, _gacFrame()); }

private:
    std::string _gacFrame() const
    {
        return top::buildTpFrame('U', 'r', "GAC",
                                 top::encodeAttitudeField4(yaw) + top::encodeAttitudeField4(pitch) +
                                     top::encodeAttitudeField4(0.0),
                                 'G', 1);
    }

    void _reply(const QNetworkDatagram &request, const std::string &frame)
    {
        _socket.writeDatagram(frame.data(), static_cast<qint64>(frame.size()), request.senderAddress(),
                              request.senderPort());
    }

    QUdpSocket _socket;
    QTimer _pushTimer;
    QTimer _trackTimer;
    QTimer _speedTimer;
    QElapsedTimer _speedClock;
    QElapsedTimer _clock;
    QNetworkDatagram _pushTo;
};

/// A stand-in for the drone of Follow Me: the test says what its telemetry is, and sees what it is sent.
class FakeFollowDrone : public FollowDrone
{
    Q_OBJECT

public:
    explicit FakeFollowDrone(QObject *parent = nullptr) : FollowDrone(parent) { clock.start(); }

    struct Sent
    {
        double northMps;
        double eastMps;
        double headingDeg;
        qint64 atMs;
    };

    QString whyNot() const override { return whyNotText; }
    QString modeName() const override { return mode; }
    void askGuided() override { ++guidedAsked; }
    void sendSpeed(double northMps, double eastMps, double headingDeg) override
    {
        sent.append({northMps, eastMps, headingDeg, clock.elapsed()});
    }
    double fliesOnS() const override { return fliesOn; }

    void tell(const skydroid::follow::Drone &drone) { emit telemetry(drone); }

    QString whyNotText;
    QString mode = QStringLiteral("Loiter");
    int guidedAsked = 0;
    double fliesOn = 3.0;
    QList<Sent> sent;
    QElapsedTimer clock;
};

/// What ObjectFollow::instance() would fly in the app. No test uses that instance.
FollowDrone *vamaMakeFollowDrone(QObject *parent)
{
    return new FakeFollowDrone(parent);
}

/// A stand-in for the object finder's network. It "finds" what the test says is in the
/// picture, as far as a look can see it: only what lies in the part looked at (cut at its
/// border), and of small objects only those in a look at the picture's own size.
class FakePictureFinder : public PictureFinder
{
public:
    /// What the test and the finder share. The marks own the finder, the test keeps this.
    struct Seen
    {
        QMutex mutex;
        std::vector<skydroid::find::Found> objects;  // in points of the image handed in
        double smallBelow = 60.0;                    // objects lower than this are "small"
        int sleepMs = 0;                             // how long one look takes
        bool sleepBySize = false;                    // ... of 640 x 384 points: a larger look takes longer
        bool troubleCloseOnly = false;               // the trouble comes from looks at the picture's own size only
        double growWhenSmaller = 0.0;                // a box found in a picture made smaller is this much too large
        bool canLoad = true;
        QString whyNot;
        QString trouble;
        // What happened.
        std::vector<skydroid::find::Ask> asks;
        std::vector<QSize> sizes;
        std::vector<Qt::HANDLE> threads;
        int loads = 0;
        int rests = 0;
    };

    explicit FakePictureFinder(std::shared_ptr<Seen> seen) : _seen(std::move(seen)) {}

    bool load(QString &whyNot) override
    {
        QMutexLocker lock(&_seen->mutex);
        ++_seen->loads;
        whyNot = _seen->whyNot;
        return _seen->canLoad;
    }

    void rest() override
    {
        QMutexLocker lock(&_seen->mutex);
        ++_seen->rests;
    }

    std::vector<skydroid::find::Found> find(const skydroid::find::Picture &picture, const skydroid::find::Ask &ask,
                                            QString &whyNot) const override
    {
        std::vector<skydroid::find::Found> objects;
        double smallBelow = 0.0;
        int sleepMs = 0;
        bool sleepBySize = false;
        bool troubleCloseOnly = false;
        double growWhenSmaller = 0.0;
        QString trouble;
        {
            QMutexLocker lock(&_seen->mutex);
            _seen->asks.push_back(ask);
            _seen->sizes.push_back(QSize(picture.width, picture.height));
            _seen->threads.push_back(QThread::currentThreadId());
            objects = _seen->objects;
            smallBelow = _seen->smallBelow;
            sleepMs = _seen->sleepMs;
            sleepBySize = _seen->sleepBySize;
            troubleCloseOnly = _seen->troubleCloseOnly;
            growWhenSmaller = _seen->growWhenSmaller;
            trouble = _seen->trouble;
        }
        skydroid::find::Area area = ask.area;
        if (area.width <= 0 || area.height <= 0) {
            area = skydroid::find::Area{0, 0, picture.width, picture.height};
        }
        const skydroid::find::Fit fit = skydroid::find::fitArea(area.width, area.height, ask.inputW, ask.inputH, ask.keepShape);
        const bool ownSize = fit.scaleY >= 0.9;
        whyNot = (!troubleCloseOnly || ownSize) ? trouble : QString();
        if (sleepMs > 0) {
            const double units = sleepBySize ? fit.inputW * fit.inputH / (640.0 * 384.0) : 1.0;
            QThread::msleep(static_cast<unsigned long>(std::lround(sleepMs * units)));
        }
        std::vector<skydroid::find::Found> found;
        for (const skydroid::find::Found &object : objects) {
            if (fit.scaleY < 0.9 && object.h < smallBelow) {
                continue;  // too small to be seen in a picture made smaller
            }
            const double left = std::max(object.cx - object.w / 2.0, static_cast<double>(area.left));
            const double right = std::min(object.cx + object.w / 2.0, static_cast<double>(area.left + area.width));
            const double top = std::max(object.cy - object.h / 2.0, static_cast<double>(area.top));
            const double bottom = std::min(object.cy + object.h / 2.0, static_cast<double>(area.top + area.height));
            if (right - left < 4.0 || bottom - top < 4.0) {
                continue;  // not in this part of the picture
            }
            skydroid::find::Found f = object;
            f.cx = (left + right) / 2.0;
            f.cy = (top + bottom) / 2.0;
            f.w = right - left + (ownSize ? 0.0 : growWhenSmaller);
            f.h = bottom - top;
            found.push_back(f);
        }
        return found;
    }

private:
    std::shared_ptr<Seen> _seen;
};

/// What ObjectMarks::instance() would look with in the app. No test uses that instance.
PictureFinder *vamaMakePictureFinder()
{
    return new FakePictureFinder(std::make_shared<FakePictureFinder::Seen>());
}

PictureFinder *vamaMakePictureFinderFrom(const QString &, const QString &)
{
    return vamaMakePictureFinder();
}

QString vamaFinderNotices()
{
    return QStringLiteral("the notices of the stand-in finder");
}

QString vamaFinderName()
{
    return QStringLiteral("a stand-in finder");
}

class SkydroidLinkTest : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase()
    {
        // Keep test settings away from the real app settings.
        QSettings::setDefaultFormat(QSettings::IniFormat);
        QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, QDir::tempPath() + "/vama-link-test");
        // No probing of real network addresses from a test.
        SkydroidLink::setProbeTargets({}, {});
        MultiVehicleManager::instance()->setActiveVehicle(&_vehicle);
    }

    void init()
    {
        QSettings().remove("VamaSkydroid");
        QSettings().remove("VamaFollow");
        QSettings().remove("VamaMarks");
        SkydroidLink::setProbeTargets({}, {});
        _vehicle.position = QGeoCoordinate(20.0, 72.0);
        _vehicle.gps.set("lock", 3);
        _vehicle.vehicle.set("heading", 0.0);
        _vehicle.vehicle.set("roll", 0.0);
        _vehicle.vehicle.set("pitch", 0.0);
        _vehicle.vehicle.set("altitudeAMSL", 100.0);
        _vehicle.vehicle.remove("altitudeRelative");

        _camera = new FakeCamera;
        _link = new SkydroidLink;
        _link->setHost("127.0.0.1");
        _link->setPort(_camera->port());
        _link->setModel("C13");
        _link->setEnabled(true);
    }

    void cleanup()
    {
        delete _marks;  // with its finder
        _marks = nullptr;
        _seen.reset();
        delete _follow;  // with its drone
        _follow = nullptr;
        _drone = nullptr;
        delete _link;
        delete _camera;
        _link = nullptr;
        _camera = nullptr;
        SkydroidLink::setProbeTargets({}, {});
    }

    // --- Angles and addresses ------------------------------------------------

    void readsGimbalAngles()
    {
        _camera->yaw = 12.5;
        _camera->pitch = -45.0;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        QVERIFY(_link->answering());
        QCOMPARE(_link->gimbalYaw(), 12.5);
        QCOMPARE(_link->gimbalPitch(), -45.0);
    }

    void turnsTheAnglePushOnBeforeAsking()
    {
        // VGCS order: GAA first, then GAC.
        QTRY_VERIFY_WITH_TIMEOUT(_camera->received.size() >= 2, 2000);
        QCOMPARE(_camera->received.at(0), QString::fromStdString(top::buildGaaEnable(5)));
        QCOMPARE(_camera->received.at(1), QString::fromStdString(top::buildGacQuery()));
    }

    void readsAnglesFromAPushOnlyCamera()
    {
        // The field case: the camera answers the laser but never a GAC question.
        _camera->answerGac = false;
        _camera->pushAfterGaa = true;
        _camera->yaw = -20.0;
        _camera->pitch = -60.0;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        QCOMPARE(_link->gimbalYaw(), -20.0);
        QCOMPARE(_link->gimbalPitch(), -60.0);
    }

    void stopsTrustingAnglesWhenCameraGoesQuiet()
    {
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _camera->answerGac = false;
        QTRY_VERIFY_WITH_TIMEOUT(!_link->answering(), 6000);
        QVERIFY(!_link->attitudeValid());
    }

    void movesToTheAddressThatGivesAngles()
    {
        // The configured address answers the laser but not angles (like
        // 192.168.144.108:5000 in the field); a relay answers angles.
        _camera->answerGac = false;
        _camera->slrE = "03A0";  // 92.8 m
        FakeCamera relay;
        relay.yaw = 5.0;
        relay.pitch = -15.0;
        _restartWithProbeTargets({"127.0.0.1"}, {relay.port()});

        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 6000);
        QCOMPARE(_link->activeEndpoint(), QString("127.0.0.1:%1").arg(relay.port()));
        QCOMPARE(_link->gimbalPitch(), -15.0);
        // The relay only ever got questions before it answered.
        QVERIFY(relay.countStartingWith("#TPUG2wGAA") >= 1);

        // Motion now goes to the relay.
        _camera->clear();
        relay.clear();
        _link->setTouchMotion(1.0, 0.0);
        QTRY_VERIFY_WITH_TIMEOUT(relay.countStartingWith("#TPUG2wGSY") >= 1, 2000);
        QCOMPARE(_camera->countStartingWith("#TPUG2wGSY"), 0);
        _link->stopTouchMotion();

        // The laser still reaches the configured address, which answers it.
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(_link->laserValid());
        QCOMPARE(_link->laserRangeM(), 92.8);
        QVERIFY(_link->targetValid());
    }

    void noProbingWhileTheCameraAnswers()
    {
        FakeCamera other;
        _restartWithProbeTargets({"127.0.0.1"}, {other.port()});
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        other.clear();
        QTest::qWait(3500);
        QCOMPARE(other.received.size(), 0);
        QCOMPARE(_link->activeEndpoint(), QString("127.0.0.1:%1").arg(_camera->port()));
    }

    // --- Motion ----------------------------------------------------------------

    void speedCurve()
    {
        QCOMPARE(SkydroidLink::shapedSpeed(0.03, 20.0, 0.05), 0.0);
        QCOMPARE(SkydroidLink::shapedSpeed(1.0, 20.0, 0.05), 20.0);
        QCOMPARE(SkydroidLink::shapedSpeed(-1.0, 20.0, 0.05), -20.0);
        QCOMPARE(SkydroidLink::shapedSpeed(5.0, 20.0, 0.05), 20.0);
        QVERIFY(std::abs(SkydroidLink::shapedSpeed(0.525, 20.0, 0.05) - 5.0) < 1e-9);  // half way: a quarter
        QCOMPARE(SkydroidLink::shapedSpeed(std::nan(""), 20.0, 0.05), 0.0);
    }

    void touchSendsOneFramePerMovingAxisThenStops()
    {
        const std::string yawRight = top::buildGimbalSpeedAxis("GSY", 20.0);
        _link->setTouchMotion(1.0, 0.0);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(yawRight) >= 3, 2000);  // refreshed while held
        _link->setTouchMotion(1.0, 0.0);
        QCOMPARE(_camera->countStartingWith("#TPUG2wGSP"), 0);
        QCOMPARE(_camera->countStartingWith("#tpUG4wGSM"), 0);  // never the combined frame while moving
        _link->stopTouchMotion();
        // Stop: each axis zero, then GSM zero, sent twice.
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeed(0.0, 0.0)) >= 2, 2000);
        QVERIFY(_camera->countEqual(top::buildGimbalSpeedAxis("GSY", 0.0)) >= 2);
        QVERIFY(_camera->countEqual(top::buildGimbalSpeedAxis("GSP", 0.0)) >= 2);
        // Then quiet.
        const int count = _camera->countStartingWith("#TPUG2wGS");
        QTest::qWait(500);
        QCOMPARE(_camera->countStartingWith("#TPUG2wGS"), count);
    }

    void touchStopsWhenUpdatesStop()
    {
        // A lost "release" must not leave the gimbal turning.
        _link->setTouchMotion(0.0, 1.0);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeedAxis("GSP", 20.0)) >= 1, 2000);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeed(0.0, 0.0)) >= 1, 1500);
    }

    void axisThatStopsGetsItsZero()
    {
        _link->setTouchMotion(1.0, 1.0);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeedAxis("GSP", 20.0)) >= 1, 2000);
        QCOMPARE(_camera->countEqual(top::buildGimbalSpeedAxis("GSY", 0.0)), 0);
        for (int i = 0; i < 4; ++i) {
            _link->setTouchMotion(0.0, 1.0);
            QTest::qWait(100);
        }
        QCOMPARE(_camera->countEqual(top::buildGimbalSpeedAxis("GSY", 0.0)), 2);
        _link->stopTouchMotion();
    }

    void reverseAndMaxSpeedSettings()
    {
        _link->setReverseYaw(true);
        _link->setMaxSpeed(10);
        _link->setTouchMotion(1.0, -1.0);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeedAxis("GSY", -10.0)) >= 1, 2000);
        QVERIFY(_camera->countEqual(top::buildGimbalSpeedAxis("GSP", -10.0)) >= 1);
        _link->setReversePitch(true);
        _link->setTouchMotion(1.0, -1.0);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeedAxis("GSP", 10.0)) >= 1, 2000);
        _link->stopTouchMotion();
    }

    void buttonsSendTheirFrames()
    {
        // Centre and look down use angle commands: the PTZ codes did nothing
        // on the client's camera (test build 2), while yaw centre (GAY) worked.
        _link->center();
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalAngleAxis("GAY", 0.0, 30.0)), 1, 2000);
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalAngleAxis("GAP", 0.0, 30.0)), 1, 2000);
        _link->centerYaw();
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalAngleAxis("GAY", 0.0, 30.0)), 2, 2000);
        _link->pointDown();
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalAngleAxis("GAP", -90.0, 30.0)), 1, 2000);
        QCOMPARE(_camera->countStartingWith("#TPUG2wPTZ"), 0);
        _link->ptz("stop");
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildPtz("stop")), 3, 2000);
    }

    void gimbalFramesAlsoGoOutUpperCase()
    {
        // The V13 (C13) took no angle command in test build 3. Skydroid's TOP
        // documents start every gimbal frame with "#TP"; VGCS sends the long
        // ones as "#tp". So both go out, the documented one first.
        _camera->upperCaseOnly = true;
        _camera->followAngles = true;
        _camera->yaw = 20.0;
        _camera->pitch = -30.0;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        top::Options upper;
        upper.gClassUpperHeader = true;

        _link->pointDown();
        QTRY_VERIFY_WITH_TIMEOUT(std::abs(_link->gimbalPitch() + 90.0) < 0.01, 2000);
        const std::string downUpper = top::buildGimbalAngleAxis("GAP", -90.0, 30.0, upper);
        const std::string downLower = top::buildGimbalAngleAxis("GAP", -90.0, 30.0);
        QCOMPARE(downUpper.substr(0, 3), std::string("#TP"));
        QCOMPARE(_camera->countEqual(downUpper), 1);
        QCOMPARE(_camera->countEqual(downLower), 1);
        QVERIFY(_camera->indexOf(downUpper) < _camera->indexOf(downLower));

        _link->center();
        QTRY_VERIFY_WITH_TIMEOUT(std::abs(_link->gimbalYaw()) < 0.01 && std::abs(_link->gimbalPitch()) < 0.01, 2000);
        QCOMPARE(_camera->countEqual(top::buildGimbalAngleAxis("GAY", 0.0, 30.0, upper)), 1);
        QCOMPARE(_camera->countEqual(top::buildGimbalAngleAxis("GAP", 0.0, 30.0, upper)), 1);

        _link->setTouchMotion(0.8, 0.0);
        _link->stopTouchMotion();
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeed(0.0, 0.0, upper)) >= 1, 2000);
        QVERIFY(_camera->countEqual(top::buildGimbalSpeed(0.0, 0.0)) >= 1);

        // The C14 Pro is upper-case already: one frame, not two.
        _link->setModel("C14 Pro");
        _camera->clear();
        _link->pointDown();
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(downUpper), 1, 2000);
        QCOMPARE(_camera->countStartingWith("#tp"), 0);
    }

    void tapAndLockWorkOnACameraThatTakesOnlyUpperCase()
    {
        _camera->upperCaseOnly = true;
        _camera->followAngles = true;
        _camera->trackDriftDps = 5.0;
        _camera->slrE = "01F4";
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->aimAndMeasure(0.75, 0.4);
        QTRY_VERIFY_WITH_TIMEOUT(!_link->aimBusy() && !_link->laserBusy(), 5000);
        QVERIFY(_link->laserValid());
        QVERIFY(std::abs(_link->gimbalYaw() + 20.85) < 0.01);
        _link->setLockMode("camera");
        _link->lockAt(0.25, 0.5);
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockActive(), 5000);
        top::Options upper;
        upper.gClassUpperHeader = true;
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(_got(640, 360, upper)), 1, 1000);
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockFollowSeen(), 5000);
        _link->stopLock();
    }

    // --- Tap on an object: turn to it, then measure -----------------------------

    void fieldOfViewFollowsZoomAndLens()
    {
        double h = 0.0;
        double v = 0.0;
        _link->currentFov(h, v);
        QVERIFY(std::abs(h - 83.4) < 1e-9 && std::abs(v - 46.9) < 1e-9);  // VGCS's calibrated C13 view
        for (int i = 0; i < 10; ++i) {
            _link->zoom(1);  // 2.0x
        }
        _link->currentFov(h, v);
        const double expectedH = 2.0 * std::atan(std::tan(83.4 * kPi / 360.0) / 2.0) * 180.0 / kPi;
        QVERIFY(std::abs(h - expectedH) < 1e-9);
        _link->setModel("C14 Pro");
        _link->currentFov(h, v);
        QVERIFY(std::abs(h - 61.4) < 1e-9 && std::abs(v - 47.9) < 1e-9);  // short lens, step 0
        _camera->dzmStep = 86;
        _link->zoom(1);
        QTRY_COMPARE_WITH_TIMEOUT(_link->zoomStep(), 86, 3000);
        _link->currentFov(h, v);
        QVERIFY(std::abs(h - 14.7) < 1e-9 && std::abs(v - 11.1) < 1e-9);  // long lens, first step
    }

    void theLensOfThePictureIsTheThermalLensWhileTheThermalPictureShows()
    {
        double h = 0.0;
        double v = 0.0;
        QVERIFY(_link->pictureFov(h, v));  // the day picture: the day lens, known
        QVERIFY(std::abs(h - 83.4) < 1e-9 && std::abs(v - 46.9) < 1e-9);
        for (int i = 0; i < 10; ++i) {
            _link->zoom(1);  // 2.0x on the day camera
        }
        // The C13's thermal picture: its data sheet's 48.7 x 38.6, not narrowed by the day
        // zoom, and not known well enough to place a point.
        _link->setThermalPicture(true);
        QVERIFY(!_link->pictureFov(h, v));
        QVERIFY(std::abs(h - 48.7) < 1e-9 && std::abs(v - 38.6) < 1e-9);
        // C12: 36.4 x 27.7, from its 7 mm lens and 384 x 288 points of 12 um.
        _link->setModel("C12");
        QVERIFY(!_link->pictureFov(h, v));
        QVERIFY(std::abs(h - 36.4) < 1e-9 && std::abs(v - 27.7) < 1e-9);
        // C14 Pro: Skydroid's 32.84 x 26.35, known.
        _link->setModel("C14 Pro");
        QVERIFY(_link->pictureFov(h, v));
        QVERIFY(std::abs(h - 32.84) < 1e-9 && std::abs(v - 26.35) < 1e-9);
        _link->setThermalPicture(false);
        QVERIFY(_link->pictureFov(h, v));
        QVERIFY(std::abs(h - 61.4) < 1e-9 && std::abs(v - 47.9) < 1e-9);  // its day lens again
    }

    void aTapOnTheThermalPictureTurnsTheCameraByTheThermalLens()
    {
        _camera->yaw = 0.0;
        _camera->pitch = -30.0;
        _camera->followAngles = true;
        _camera->slrE = "01F4";  // 50 m
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->setThermalPicture(true);
        // 0.2 x 48.7 = 9.74 degrees right (C13 yaw -9.74), 0.1 x 38.6 = 3.86 up. By the
        // day lens the camera turned 16.68 right and 4.69 up, past the object.
        _link->aimAndMeasure(0.7, 0.4);
        QVERIFY(_link->aimBusy());
        const std::string gay = top::buildGimbalAngleAxis("GAY", -9.74, 30.0);
        const std::string gap = top::buildGimbalAngleAxis("GAP", -26.14, 30.0);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(gay) >= 1 && _camera->countEqual(gap) >= 1, 2000);
        QTRY_VERIFY_WITH_TIMEOUT(!_link->aimBusy() && !_link->laserBusy(), 5000);
        QVERIFY(_link->laserValid());
        QVERIFY(std::abs(_link->gimbalYaw() + 9.74) < 0.01);
    }

    void tapTurnsTheCameraThenMeasures()
    {
        _camera->yaw = 0.0;
        _camera->pitch = -30.0;
        _camera->followAngles = true;
        _camera->slrE = "01F4";  // 50 m
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        // Right of centre and a little up: 0.25 x 83.4 = 20.85 degrees right,
        // which is GAC yaw -20.85 on the C13; 0.1 x 46.9 = 4.69 degrees up.
        _link->aimAndMeasure(0.75, 0.4);
        QVERIFY(_link->aimBusy());
        const std::string gay = top::buildGimbalAngleAxis("GAY", -20.85, 30.0);
        const std::string gap = top::buildGimbalAngleAxis("GAP", -25.31, 30.0);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(gay) >= 1 && _camera->countEqual(gap) >= 1, 2000);
        QTRY_VERIFY_WITH_TIMEOUT(!_link->aimBusy() && !_link->laserBusy(), 5000);
        QVERIFY(_link->laserValid());
        QCOMPARE(_link->laserRangeM(), 50.0);
        // The laser fired only after the camera turned.
        const int turn = _camera->received.indexOf(QString::fromStdString(gay));
        const int shot = _camera->received.indexOf(QString::fromStdString(top::buildSlrTrigger('E')));
        QVERIFY(turn >= 0 && shot > turn);
        // The target uses the angles at the shot, so it lies off to that side:
        // the drone faces north and the camera turned right, so north-east.
        // (This used to check only that there was a target. It was on the
        // other side of the nose line, and nothing noticed.)
        QVERIFY(_link->targetValid());
        QVERIFY(std::abs(_link->gimbalYaw() + 20.85) < 0.01);
        QVERIFY2(std::abs(_link->targetBearingDeg() - 20.85) < 0.5,
                 qPrintable(QStringLiteral("target bearing %1, the camera looks 20.85 degrees right of north")
                                .arg(_link->targetBearingDeg())));
    }

    void tapNeverMeasuresIfTheCameraDoesNotTurn()
    {
        _camera->followAngles = false;  // the gimbal ignores the angles
        _camera->slrE = "01F4";
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->aimAndMeasure(0.9, 0.5);
        QTRY_VERIFY_WITH_TIMEOUT(!_link->aimBusy(), 10000);
        QVERIFY(_link->laserMessage().contains("did not turn"));
        QVERIFY(!_link->laserValid());
        QCOMPARE(_camera->countStartingWith("#TPUE2wSLR01"), 0);  // no shot at the wrong point
        // The angles were sent twice (UDP can drop one).
        QVERIFY(_camera->countStartingWith("#tpUG6wGAY") >= 2);
    }

    void tapWithoutGimbalAnglesSaysSo()
    {
        _camera->answerGac = false;
        _link->aimAndMeasure(0.5, 0.5);
        QVERIFY(!_link->aimBusy());
        QVERIFY(_link->laserMessage().contains("No gimbal angles"));
        QTest::qWait(300);
        QCOMPARE(_camera->countStartingWith("#tpUG6wGAY"), 0);
    }

    void tapYawCanBeReversed()
    {
        _camera->yaw = 0.0;
        _camera->pitch = -30.0;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->setReverseTapYaw(true);
        _link->aimAndMeasure(0.75, 0.4);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalAngleAxis("GAY", 20.85, 30.0)) >= 1, 2000);
    }

    // --- Object lock by the camera's own tracker (lock mode "camera"): turn to the object, GOT, SUM confirm ---

    void lockTurnsToTheObjectThenLocksIt()
    {
        _link->setLockMode("camera");
        _camera->yaw = 0.0;
        _camera->pitch = -30.0;
        _camera->followAngles = true;
        _camera->slrE = "01F4";  // 50 m
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.75, 0.4);  // same point as the tap test: 20.85 right, 4.69 up
        QVERIFY(_link->lockBusy());
        QVERIFY(!_link->lockActive());
        const std::string gay = top::buildGimbalAngleAxis("GAY", -20.85, 30.0);
        const std::string got = _got(640, 360);  // the object is under the cross now
        const std::string confirm = top::buildSumTrack(true);
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockActive(), 5000);
        QVERIFY(!_link->lockBusy());
        QVERIFY(_link->lockMessage().contains("Locked"));
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(confirm) >= 1, 1000);
        QVERIFY(_camera->tracking);
        // Turn first, then GOT, then the confirm.
        const int turn = _camera->received.indexOf(QString::fromStdString(gay));
        const int lock = _camera->received.indexOf(QString::fromStdString(got));
        const int conf = _camera->received.indexOf(QString::fromStdString(confirm));
        QVERIFY(turn >= 0 && lock > turn && conf > lock);
        // The laser measures the locked object, and the confirm is sent again.
        QTRY_VERIFY_WITH_TIMEOUT(_link->laserValid(), 2000);
        QCOMPARE(_link->laserRangeM(), 50.0);
        QVERIFY(_link->targetValid());
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(confirm) >= 2, 3000);
        // Stop: SUM stop, twice.
        _link->stopLock();
        QVERIFY(!_link->lockActive());
        QVERIFY(_link->lockMessage().contains("stopped"));
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildSumTrack(false)), 2, 1000);
        QVERIFY(!_camera->tracking);
        // Nothing more once stopped.
        const int confirms = _camera->countEqual(confirm);
        QTest::qWait(2500);
        QCOMPARE(_camera->countEqual(confirm), confirms);
    }

    void lockNearTheCentreLocksWithoutTurning()
    {
        _link->setLockMode("camera");
        _camera->yaw = 3.0;
        _camera->pitch = -30.0;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        // 0.01 of the view is under 1.5 degrees: lock where it was picked (1280 x 720 frame).
        _link->lockAt(0.51, 0.49);
        QVERIFY(_link->lockActive());
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(_got(653, 353)) == 1, 1000);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildSumTrack(true)) >= 1, 1000);
        QCOMPARE(_camera->countStartingWith("#tpUG6wGAY"), 0);
        QCOMPARE(_camera->countStartingWith("#tpUG6wGAP"), 0);
    }

    void lockAtAGimbalLimitPointsGotAtTheObject()
    {
        _link->setLockMode("camera");
        // The C13 tilts up to +10 only, and turns to +-90. An object past a
        // limit stays off the cross after the turn, so GOT goes where it is.
        _camera->yaw = -80.0;
        _camera->pitch = 5.0;
        _camera->followAngles = true;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        // 20.85 degrees right (C13 yaw -20.85): the object is at -100.85.
        // 14.07 degrees up: the object is at +19.07.
        _link->lockAt(0.75, 0.2);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalAngleAxis("GAY", -90.0, 30.0)) >= 1, 2000);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalAngleAxis("GAP", 10.0, 30.0)) >= 1, 2000);
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockActive(), 5000);
        // Still right of the cross by 10.85 degrees, and above it by 9.07.
        const double u = 0.5 + ((80.0 + 0.25 * 83.4) - 90.0) / 83.4;
        const double v = 0.5 - ((5.0 + 0.3 * 46.9) - 10.0) / 46.9;
        const int x = static_cast<int>(std::lround(u * 1280.0));
        const int y = static_cast<int>(std::lround(v * 720.0));
        QVERIFY(x > 760 && y < 300);
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(_got(x, y)), 1, 1000);
    }

    void aLockAtAGimbalLimitOnTheThermalPictureUsesTheThermalLens()
    {
        _link->setLockMode("camera");
        _link->setThermalPicture(true);
        _camera->yaw = -80.0;
        _camera->pitch = 5.0;
        _camera->followAngles = true;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        // On the C13's thermal lens (48.7 x 38.6) the object is 12.175 degrees right and
        // 11.58 up: past both limits.
        _link->lockAt(0.75, 0.2);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalAngleAxis("GAY", -90.0, 30.0)) >= 1, 2000);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalAngleAxis("GAP", 10.0, 30.0)) >= 1, 2000);
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockActive(), 5000);
        // Still right of the cross by 2.175 degrees, and above it by 6.58: there in the picture.
        // (The day lens put it at 807, 221. Which frame the camera's own tracker counts in on
        // the thermal picture is not known; the place in the picture is what this checks.)
        const double u = 0.5 + ((80.0 + 0.25 * 48.7) - 90.0) / 48.7;
        const double v = 0.5 - ((5.0 + 0.3 * 38.6) - 10.0) / 38.6;
        const int x = static_cast<int>(std::lround(u * 1280.0));
        const int y = static_cast<int>(std::lround(v * 720.0));
        QVERIFY(x < 720 && y > 230);
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(_got(x, y)), 1, 1000);
    }

    void lockNeverStartsIfTheCameraDoesNotTurn()
    {
        _link->setLockMode("camera");
        _camera->followAngles = false;  // the gimbal ignores the angles
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.9, 0.5);
        QVERIFY(_link->lockBusy());
        QTRY_VERIFY_WITH_TIMEOUT(!_link->lockBusy(), 10000);
        QVERIFY(!_link->lockActive());
        QVERIFY(_link->lockMessage().contains("did not turn"));
        QCOMPARE(_camera->countStartingWith("#tpUG8wGOT"), 0);
        QCOMPARE(_camera->countEqual(top::buildSumTrack(true)), 0);
    }

    void lockWithoutGimbalAnglesSaysSo()
    {
        _link->setLockMode("camera");
        _camera->answerGac = false;
        _link->lockAt(0.5, 0.5);
        QVERIFY(!_link->lockBusy() && !_link->lockActive());
        QVERIFY(_link->lockMessage().contains("No gimbal angles"));
        QTest::qWait(300);
        QCOMPARE(_camera->countStartingWith("#tpUG8wGOT"), 0);
    }

    // The camera turning by itself is all the app can see. It is told as a
    // number, and never as "is following": in the field video of test build 6
    // (2026-10-09) the V13 drifted 6 degrees in 17 s while the person it was
    // locked on walked 18 degrees, and the app said "is following".
    void lockSeesTheCameraTurnAndDoesNotCallItFollowing()
    {
        _link->setLockMode("camera");
        _camera->yaw = 0.0;
        _camera->pitch = -30.0;
        _camera->trackDriftDps = 5.0;  // the camera follows a moving object
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive());
        QVERIFY(!_link->lockFollowSeen());
        QVERIFY(_link->lockMessage().contains("cannot see what the camera follows"));
        // The first 2 s do not count: what the camera does then is its tracker
        // taking over. It has already turned about 7 degrees here.
        QTest::qWait(1500);
        QVERIFY(!_link->lockFollowSeen());
        QVERIFY(_link->lockTurnedDeg() < 0.1);
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockFollowSeen(), 3000);
        QVERIFY(_link->lockTurnedDeg() > 0.8);
        QVERIFY(_link->lockMessage().contains("camera's own tracker"));
        QVERIFY(!_link->lockMessage().contains("is following"));
        QVERIFY(!_link->lockByApp());
        QVERIFY(!_link->lockWantsPictures());
        _link->stopLock();
    }

    // Field video of test build 4 (2026-10-06 21:53): right after the lock the
    // V13 moved 2.6 degrees by itself beside a parked car and then stood still.
    // The app said "is following", and nothing was moving.
    void aJumpAtTheLockIsNotFollowing()
    {
        _link->setLockMode("camera");
        _camera->yaw = 0.0;
        _camera->pitch = -30.0;
        _camera->lockJumpDeg = 2.6;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive());
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockMessage().contains("moved 2.6 degrees at the lock"), 4000);
        QVERIFY(_link->lockMessage().contains("Check that the cross is on the object"));
        QVERIFY(!_link->lockFollowSeen());
        QVERIFY(std::abs(_link->lockJumpDeg() - 2.6) < 0.05);
        QVERIFY(_link->lockTurnedDeg() < 0.1);
        // Later the camera still stands still: the advice stays, and it never says "following".
        QTest::qWait(5000);
        QVERIFY(!_link->lockFollowSeen());
        QVERIFY(_link->lockMessage().contains("Check that the cross is on the object"));
        QVERIFY(!_link->lockMessage().contains("following"));
        QVERIFY(_link->lockActive());
        _link->stopLock();
    }

    void aTurnAfterAJumpIsStillSeen()
    {
        _link->setLockMode("camera");
        _camera->yaw = 0.0;
        _camera->pitch = -30.0;
        _camera->lockJumpDeg = 2.6;
        _camera->trackDriftDps = 5.0;  // and then it follows a moving object
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive());
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockFollowSeen(), 5000);
        QVERIFY(_link->lockJumpDeg() > 2.6);          // the jump, plus what it followed in the first 2 s
        QVERIFY(_link->lockTurnedDeg() > 0.8);        // counted from after those 2 s
        QVERIFY(_link->lockMessage().contains("Check that the cross is on the object"));  // the advice stays
        QVERIFY(!_link->lockMessage().contains("is following"));
        _link->stopLock();
    }

    void lockSaysSoWhenTheCameraDoesNotFollow()
    {
        _link->setLockMode("camera");
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);  // the camera never turns by itself
        QVERIFY(_link->lockActive());
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockMessage().contains("has not turned"), 8000);
        QVERIFY(!_link->lockFollowSeen());
        QVERIFY(_link->lockActive());  // only a warning: the operator decides
    }

    void lockSaysSoWhenTheAnglesStop()
    {
        _link->setLockMode("camera");
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive());
        _camera->answerGac = false;  // no more angles: the app cannot see the camera turn
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockMessage().contains("no angles"), 8000);
        QVERIFY(!_link->lockMessage().contains("not following"));
    }

    void movingTheCameraByHandEndsTheLock()
    {
        _link->setLockMode("camera");
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        const std::string stop = top::buildSumTrack(false);

        // A drag on the video.
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive());
        _camera->clear();
        _link->setTouchMotion(0.8, 0.0);
        QVERIFY(!_link->lockActive());
        QVERIFY(_link->lockMessage().contains("moved"));
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countStartingWith("#TPUG2wGSY") >= 1, 2000);
        // The camera hears the stop before the first speed command.
        QVERIFY(_camera->indexOf(stop) >= 0);
        QVERIFY(_camera->indexOf(stop) < _camera->firstStartingWith("#TPUG2wGSY"));
        _link->stopTouchMotion();

        // A gimbal button.
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive());
        _link->pointDown();
        QVERIFY(!_link->lockActive());

        // An RC wheel.
        _link->setWheelYawChannel(10);
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive());
        _vehicle.sendRc(_rc(10, 1900));
        QVERIFY(!_link->lockActive());
        _vehicle.sendRc(_rc(10, 1500));

        // A tap to measure another point.
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive());
        _link->aimAndMeasure(0.7, 0.5);
        QVERIFY(!_link->lockActive());
        QVERIFY(_link->aimBusy());
    }

    void turningTheLinkOffStopsTheLock()
    {
        _link->setLockMode("camera");
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive());
        _link->setEnabled(false);
        QVERIFY(!_link->lockActive());
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildSumTrack(false)) >= 1, 1000);
        QVERIFY(!_camera->tracking);
    }

    void aNewLockReplacesTheOldOne()
    {
        _link->setLockMode("camera");
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive());
        _camera->clear();
        _link->lockAt(0.505, 0.5);
        // The old lock stops first; the new GOT comes a moment later (VGCS waits 50 ms).
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockActive(), 1000);
        const std::string newGot = _got(646, 360);
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(newGot), 1, 1000);
        const int stopAt = _camera->indexOf(top::buildSumTrack(false));
        const int gotAt = _camera->indexOf(newGot);
        QVERIFY(stopAt >= 0 && stopAt < gotAt);
        QVERIFY(_camera->receivedAtMs.at(gotAt) - _camera->receivedAtMs.at(stopAt) >= 40);
    }

    void lockMeasuresAgainAndKeepsTheLastResultMeanwhile()
    {
        _link->setLockMode("camera");
        _camera->slrE = "01F4";
        _camera->laserModuleDelayMs = 600;  // a slow laser makes the refresh window wide
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);
        QTRY_VERIFY_WITH_TIMEOUT(_link->laserValid() && !_link->laserBusy(), 3000);
        QVERIFY(_link->targetValid());
        const std::string shot = top::buildSlrTrigger('E');
        const int shots = _camera->countEqual(shot);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(shot) > shots, 4000);
        // The second shot is out; the first result stays up until the new one lands.
        QVERIFY(_link->laserBusy());
        QVERIFY(_link->laserValid());
        QVERIFY(_link->targetValid());
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 2000);
        QVERIFY(_link->laserValid());
    }

    void c14ProLockUsesItsHeader()
    {
        _link->setLockMode("camera");
        _link->setModel("C14 Pro");
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);
        top::Options options;
        options.gClassUpperHeader = true;
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(_got(640, 360, options)), 1, 1000);
        QVERIFY(_camera->countEqual(_got(640, 360, options)) == 1);
        QCOMPARE(_camera->countStartingWith("#tpUG8wGOT"), 0);
        // Changing the camera stops the lock, in the old camera's format.
        _link->setModel("C13");
        QVERIFY(!_link->lockActive());
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildSumTrack(false, options)) >= 1, 1000);
    }

    // The lock point for the camera's own tracker is not the middle of 1280 x 720.
    // Both field videos (2026-10-06 and 2026-10-09) showed the V13 turn about
    // 2.4 degrees left and 1.2 up right after GOT at (640, 360) with the object
    // on the cross: 32 points across and 18 down. So the middle is sent as (672, 378).
    void theCamerasOwnTrackerGetsThePointInItsOwnCount()
    {
        _link->setLockMode("camera");
        _camera->yaw = 3.0;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive());
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildGotTarget(672, 378, 1344, 756)), 1, 1000);
        QCOMPARE(_camera->countEqual(top::buildGotTarget(640, 360)), 0);
        // The far corner of the picture stays inside the camera's frame.
        QCOMPARE(QString::fromStdString(_got(1280, 720)), QString::fromStdString(top::buildGotTarget(1312, 738, 1344, 756)));
    }

    // --- Object lock by the app (the usual lock mode): pictures in, gimbal speeds out -----

    void theAppsOwnLockIsTheUsualOne()
    {
        QCOMPARE(_link->lockMode(), QString("app"));
        QCOMPARE(SkydroidLink::lockModes(), QStringList({"app", "camera"}));
        QCOMPARE(SkydroidLink::lockModeNames().size(), 2);
        // An unknown mode is the app's own.
        _link->setLockMode("camera");
        _link->setLockMode("something else");
        QCOMPARE(_link->lockMode(), QString("app"));
        // The mode is kept for the next start.
        _link->setLockMode("camera");
        delete _link;
        _link = new SkydroidLink;
        QCOMPARE(_link->lockMode(), QString("camera"));
    }

    void followSpeedRule()
    {
        // Nothing inside half a degree, then 2.5 deg/s for each degree, in the direction of the object.
        QCOMPARE(SkydroidLink::followSpeed(0.0, 40.0), 0.0);
        QCOMPARE(SkydroidLink::followSpeed(0.5, 40.0), 0.0);
        QCOMPARE(SkydroidLink::followSpeed(-0.5, 40.0), 0.0);
        QCOMPARE(SkydroidLink::followSpeed(2.0, 40.0), 5.0);
        QCOMPARE(SkydroidLink::followSpeed(-2.0, 40.0), -5.0);
        QCOMPARE(SkydroidLink::followSpeed(10.0, 40.0), 25.0);
        // Never faster than the limit.
        QCOMPARE(SkydroidLink::followSpeed(30.0, 40.0), 40.0);
        QCOMPARE(SkydroidLink::followSpeed(-30.0, 40.0), -40.0);
        QCOMPARE(SkydroidLink::followSpeed(10.0, 12.0), 12.0);
        QCOMPARE(SkydroidLink::followSpeed(10.0, -12.0), 12.0);
        // Not a number: no motion.
        QCOMPARE(SkydroidLink::followSpeed(std::nan(""), 40.0), 0.0);
        QCOMPARE(SkydroidLink::followSpeed(5.0, std::nan("")), 0.0);
    }

    void appLockStartsOnTheFirstPictureAndNeedsNoAngles()
    {
        _camera->answerGac = false;  // the app's own lock does not use the gimbal angles
        QVERIFY(!_link->lockWantsPictures());
        _link->lockAtBox(0.7, 0.4, 0.06, 0.2);
        QVERIFY(_link->lockBusy());
        QVERIFY(!_link->lockActive());
        QVERIFY(_link->lockByApp());
        QVERIFY(_link->lockWantsPictures());
        QVERIFY(!_link->lockBoxValid());
        QCOMPARE(_link->lockMessage(), QString("Locking..."));
        _link->lockPicture(_picture(0.7, 0.4), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        QVERIFY(!_link->lockBusy());
        QVERIFY(_link->lockSeen());
        QVERIFY(_link->lockBoxValid());
        QVERIFY(std::abs(_link->lockBoxU() - 0.7) < 0.005 && std::abs(_link->lockBoxV() - 0.4) < 0.005);
        QVERIFY(std::abs(_link->lockBoxW() - 0.06) < 0.002 && std::abs(_link->lockBoxH() - 0.2) < 0.002);
        QVERIFY(_link->lockMessage().contains("Turning the camera to the object"));
        // The camera's own tracker is told to stop, and is never started.
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildSumTrack(false)) >= 1, 1000);
        QTest::qWait(300);
        QCOMPARE(_camera->countStartingWith("#TPUG8wGOT"), 0);
        QCOMPARE(_camera->countStartingWith("#tpUG8wGOT"), 0);
        QCOMPARE(_camera->countEqual(top::buildSumTrack(true)), 0);
        QCOMPARE(_camera->countStartingWith("#tpUG6wGAY"), 0);  // no angle commands either
        _link->stopLock();
        QVERIFY(!_link->lockActive() && !_link->lockWantsPictures() && !_link->lockBoxValid());
    }

    void appLockTurnsTheCameraTowardsTheObject_data()
    {
        QTest::addColumn<double>("u");
        QTest::addColumn<double>("v");
        QTest::addColumn<int>("yawSign");
        QTest::addColumn<int>("pitchSign");
        // The speed commands count right and up as +.
        QTest::newRow("object to the right") << 0.75 << 0.5 << 1 << 0;
        QTest::newRow("object to the left") << 0.25 << 0.5 << -1 << 0;
        QTest::newRow("object above") << 0.5 << 0.25 << 0 << 1;
        QTest::newRow("object below") << 0.5 << 0.75 << 0 << -1;
        QTest::newRow("right and below") << 0.7 << 0.7 << 1 << -1;
    }

    void appLockTurnsTheCameraTowardsTheObject()
    {
        QFETCH(double, u);
        QFETCH(double, v);
        QFETCH(int, yawSign);
        QFETCH(int, pitchSign);
        _link->lockAtBox(u, v, 0.08, 0.2);
        _camera->clear();
        _link->lockPicture(_picture(u, v), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        if (yawSign != 0) {
            QTRY_VERIFY_WITH_TIMEOUT(_camera->countStartingWith("#TPUG2wGSY") >= 1, 1000);
            const double yaw = _lastSpeed("GSY");
            QVERIFY2(yaw * yawSign >= 10.0 && std::abs(yaw) <= 40.0, qPrintable(QString::number(yaw)));
        }
        if (pitchSign != 0) {
            QTRY_VERIFY_WITH_TIMEOUT(_camera->countStartingWith("#TPUG2wGSP") >= 1, 1000);
            const double pitch = _lastSpeed("GSP");
            QVERIFY2(pitch * pitchSign >= 10.0 && std::abs(pitch) <= 40.0, qPrintable(QString::number(pitch)));
        }
        // An axis the object is centred on gets no speed.
        if (yawSign == 0) {
            QVERIFY(std::isnan(_lastSpeed("GSY")) || _lastSpeed("GSY") == 0.0);
        }
        if (pitchSign == 0) {
            QVERIFY(std::isnan(_lastSpeed("GSP")) || _lastSpeed("GSP") == 0.0);
        }
        QVERIFY(_link->lockOffDeg() > 10.0);
        _link->stopLock();
        // The gimbal is stopped at the end of the lock.
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeed(0.0, 0.0)) >= 1, 1000);
    }

    // Between two pictures the object must not leave the tracker's reach: about
    // half its own size. So a small object is turned to more slowly.
    void appLockTurnsMoreSlowlyToASmallObject()
    {
        _link->lockAtBox(0.75, 0.5, 0.03, 0.2);  // 2.5 degrees wide, 20.85 degrees right of the cross
        _camera->clear();
        _link->lockPicture(_picture(0.75, 0.5, true, 12, 44), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        QTRY_VERIFY_WITH_TIMEOUT(_lastSpeed("GSY") > 0.0, 1000);
        const double narrow = _lastSpeed("GSY");
        // Half of 2.5 degrees, 10 pictures a second: 12.5 deg/s, not the 40 of a wide object.
        QVERIFY2(narrow >= 10.0 && narrow <= 15.0, qPrintable(QString::number(narrow)));
        _link->stopLock();
        // The pitch limit comes from the box's height the same way.
        _link->lockAtBox(0.5, 0.2, 0.2, 0.05);  // 2.3 degrees high, 14 degrees above the cross
        _camera->clear();
        _link->lockPicture(_picture(0.5, 0.2, true, 76, 10), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        QTRY_VERIFY_WITH_TIMEOUT(_lastSpeed("GSP") > 0.0, 1000);  // the stop of the lock before is still on its way
        const double low = _lastSpeed("GSP");
        QVERIFY2(low >= 10.0 && low <= 15.0, qPrintable(QString::number(low)));
        _link->stopLock();
    }

    void appLockOnTheThermalPictureCountsDegreesByTheThermalLens()
    {
        _link->setThermalPicture(true);
        // 0.1 of the picture right of the cross is 4.87 degrees on the C13's thermal lens
        // (8.34 by the day lens), so the camera turns at about 2.5 x 4.87 = 12 deg/s, not 21.
        _link->lockAtBox(0.6, 0.5, 0.08, 0.2);
        _camera->clear();
        _link->lockPicture(_picture(0.6, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        QVERIFY2(std::abs(_link->lockOffDeg() - 4.87) < 0.3, qPrintable(QString::number(_link->lockOffDeg())));
        QTRY_VERIFY_WITH_TIMEOUT(_lastSpeed("GSY") > 0.0, 1000);
        const double speed = _lastSpeed("GSY");
        QVERIFY2(speed >= 9.0 && speed <= 13.5, qPrintable(QString::number(speed)));
        _link->stopLock();
    }

    void appLockNeverTurnsFasterThanFortyDegreesASecond()
    {
        // A wide box 25 degrees right of the cross: 2.5 x 25 would be 62 deg/s, and the box's
        // width allows more than that. The limit of 40 holds.
        _link->lockAtBox(0.8, 0.5, 0.3, 0.4);
        _camera->clear();
        _link->lockPicture(_picture(0.8, 0.5, true, 110, 80), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        QTRY_VERIFY_WITH_TIMEOUT(_lastSpeed("GSY") > 0.0, 1000);
        const double speed = _lastSpeed("GSY");
        QVERIFY2(std::abs(speed - 40.0) <= 0.5, qPrintable(QString::number(speed)));
        _link->stopLock();
    }

    void appLockObeysTheReverseSettings()
    {
        // A camera whose yaw runs the other way is set so for the finger, and the lock uses the same setting.
        _link->setReverseYaw(true);
        _link->lockAtBox(0.75, 0.5, 0.08, 0.2);
        _camera->clear();
        _link->lockPicture(_picture(0.75, 0.5), 0.0, 0.0, 1.0, 1.0);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countStartingWith("#TPUG2wGSY") >= 1, 1000);
        QVERIFY(_lastSpeed("GSY") <= -10.0);
        _link->stopLock();
    }

    void appLockFollowsTheObjectFromPictureToPicture()
    {
        _link->lockAtBox(0.3, 0.5, 0.06, 0.2);
        QSignalSpy boxes(_link, &SkydroidLink::lockBoxChanged);
        _link->lockPicture(_picture(0.3, 0.5), 0.0, 0.0, 1.0, 1.0);
        for (int i = 1; i <= 20; ++i) {
            QTest::qWait(40);
            _link->lockPicture(_picture(0.3 + 0.01 * i, 0.5 - 0.004 * i), 0.0, 0.0, 1.0, 1.0);
            QVERIFY2(_link->lockSeen(), qPrintable(QString("picture %1: %2").arg(i).arg(_link->lockNumbers())));
        }
        QVERIFY(std::abs(_link->lockBoxU() - 0.5) < 0.01);
        QVERIFY(std::abs(_link->lockBoxV() - 0.42) < 0.01);
        QVERIFY(boxes.count() >= 20);  // the box on the screen moves with every picture
        QVERIFY(_link->lockNumbers().contains("pictures/s"));
        _link->stopLock();
    }

    void appLockHoldsStillWhenTheObjectIsUnderTheCrossAndMeasuresIt()
    {
        _camera->slrE = "01F4";  // 50 m
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        _camera->clear();
        _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        QVERIFY(_link->lockMessage().contains("The camera follows the object"));
        QVERIFY(_link->lockOffDeg() < 0.5);
        for (int i = 0; i < 12; ++i) {
            QTest::qWait(90);
            _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
        }
        // No speed was ever asked for.
        QVERIFY(std::isnan(_lastSpeed("GSY")) || _lastSpeed("GSY") == 0.0);
        QVERIFY(std::isnan(_lastSpeed("GSP")) || _lastSpeed("GSP") == 0.0);
        // The laser measures what is under the cross: the object.
        QTRY_VERIFY_WITH_TIMEOUT(_link->laserValid(), 2000);
        QCOMPARE(_link->laserRangeM(), 50.0);
        QVERIFY(_link->targetValid());
        _link->stopLock();
    }

    void appLockDoesNotMeasureWhileTheObjectIsOffTheCross()
    {
        // The laser measures the cross. With the object 12 degrees away it would measure something else.
        _camera->slrE = "01F4";
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAtBox(0.65, 0.5, 0.06, 0.2);
        _camera->clear();
        for (int i = 0; i < 16; ++i) {
            _link->lockPicture(_picture(0.65, 0.5), 0.0, 0.0, 1.0, 1.0);
            QTest::qWait(90);
        }
        QVERIFY(_link->lockActive());
        QCOMPARE(_camera->countEqual(top::buildSlrTrigger('E')), 0);
        QVERIFY(!_link->laserValid());
        _link->stopLock();
    }

    // The picture is old when the app gets it. What the camera has turned since is
    // taken off, or it would run past the object: with the same picture coming
    // again and again (a slow video), the speed asked for must come down.
    void appLockTakesOffWhatTheCameraTurnedSinceThePicture()
    {
        _link->lockAtBox(0.56, 0.5, 0.2, 0.3);  // 5 degrees right; a wide box, so the speed limit is not in the way
        _camera->clear();
        _link->lockPicture(_picture(0.56, 0.5, true, 80, 70), 0.0, 0.0, 1.0, 1.0);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countStartingWith("#TPUG2wGSY") >= 1, 1000);
        const double first = _lastSpeed("GSY");
        QVERIFY2(std::abs(first - 12.5) <= 0.5, qPrintable(QString::number(first)));  // 5 degrees x 2.5
        double lowest = first;
        double highestLater = 0.0;
        for (int i = 0; i < 8; ++i) {
            QTest::qWait(100);
            _link->lockPicture(_picture(0.56, 0.5, true, 80, 70), 0.0, 0.0, 1.0, 1.0);
            QTest::qWait(20);
            const double now = _lastSpeed("GSY");
            lowest = std::min(lowest, now);
            if (i >= 2) {
                highestLater = std::max(highestLater, now);
            }
        }
        // 0.3 s at about 10 deg/s has been turned, 3 of the 5 degrees: the speed
        // falls to about half, and never comes back to the first. It does not
        // fall to nothing either: a picture that still shows the object 5
        // degrees off after that time means the camera is not there yet.
        const QString speeds = QString("first %1, lowest %2, highest later %3").arg(first).arg(lowest).arg(highestLater);
        QVERIFY2(lowest < 0.6 * first, qPrintable(speeds));
        QVERIFY2(lowest > 0.2 * first, qPrintable(speeds));
        QVERIFY2(highestLater < 0.85 * first, qPrintable(speeds));
        _link->stopLock();
    }

    void appLockSaysWhenTheObjectIsNotSeenAndEndsAfterTwoSeconds()
    {
        _link->lockAtBox(0.7, 0.5, 0.06, 0.2);
        _link->lockPicture(_picture(0.7, 0.5), 0.0, 0.0, 1.0, 1.0);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countStartingWith("#TPUG2wGSY") >= 1, 1000);
        QVERIFY(_lastSpeed("GSY") > 0.0);
        // The object is gone from the picture.
        _camera->clear();
        QElapsedTimer gone;
        gone.start();
        _link->lockPicture(_picture(0.7, 0.5, false), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        QVERIFY(!_link->lockSeen());
        QVERIFY(_link->lockMessage().contains("not seen right now"));
        // The camera waits: it does not keep turning after an object it does not see.
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeed(0.0, 0.0)) >= 1, 1000);
        while (_link->lockActive() && gone.elapsed() < 4000) {
            QTest::qWait(90);
            _link->lockPicture(_picture(0.7, 0.5, false), 0.0, 0.0, 1.0, 1.0);
        }
        QVERIFY(!_link->lockActive());
        QVERIFY2(gone.elapsed() >= 1900 && gone.elapsed() < 3000, qPrintable(QString::number(gone.elapsed())));
        QVERIFY(_link->lockMessage().contains("Lock lost"));
        QVERIFY(!_link->lockWantsPictures());
        QVERIFY(!_link->lockBoxValid());
        const double yaw = _lastSpeed("GSY");
        QVERIFY(std::isnan(yaw) || yaw == 0.0);
    }

    void appLockFindsTheObjectAgainWhenItComesBack()
    {
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
        for (int i = 0; i < 6; ++i) {
            QTest::qWait(60);
            _link->lockPicture(_picture(0.5, 0.5, false), 0.0, 0.0, 1.0, 1.0);  // hidden for a moment
        }
        QVERIFY(_link->lockActive() && !_link->lockSeen());
        QTest::qWait(60);
        _link->lockPicture(_picture(0.52, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockSeen());
        QVERIFY(_link->lockMessage().contains("Locked. "));
        QVERIFY(!_link->lockMessage().contains("not seen"));
        _link->stopLock();
    }

    void appLockStopsTheCameraWhenThePicturesStop()
    {
        _link->lockAtBox(0.7, 0.5, 0.06, 0.2);
        _link->lockPicture(_picture(0.7, 0.5), 0.0, 0.0, 1.0, 1.0);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countStartingWith("#TPUG2wGSY") >= 1, 1000);
        // No more pictures (the video froze): within half a second the camera is told to stop.
        _camera->clear();
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeed(0.0, 0.0)) >= 1, 900);
        QVERIFY(_link->lockActive());
        const int speedFrames = _camera->countStartingWith("#TPUG2wGSY2") + _camera->countStartingWith("#TPUG2wGSY1");
        QTest::qWait(600);
        QCOMPARE(_camera->countStartingWith("#TPUG2wGSY2") + _camera->countStartingWith("#TPUG2wGSY1"), speedFrames);
        // After three seconds without a picture the lock ends and says why.
        QTRY_VERIFY_WITH_TIMEOUT(!_link->lockActive(), 4000);
        QVERIFY(_link->lockMessage().contains("the video stopped"));
    }

    // A stopped video still gives pictures: the last one, again and again. The gimbal turns
    // (this fake one answers speed commands), the picture does not change: the camera stops,
    // as for an object not seen, and the lock ends when no new picture comes.
    void appLockStopsTheCameraWhenTheVideoStopsOnTheSamePicture()
    {
        _camera->followSpeeds = true;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAtBox(0.7, 0.5, 0.06, 0.2);
        const QImage still = _picture(0.7, 0.5);
        _link->lockPicture(still, 0.0, 0.0, 1.0, 1.0);
        QTRY_VERIFY_WITH_TIMEOUT(_lastSpeed("GSY") > 0.0, 1000);
        const double yawAtStart = _camera->yaw;
        QElapsedTimer since;
        since.start();
        qint64 noticedMs = -1;
        while (noticedMs < 0 && since.elapsed() < 1500) {
            QTest::qWait(90);
            _link->lockPicture(still, 0.0, 0.0, 1.0, 1.0);  // 11 pictures a second, all the same
            if (_link->lockMessage().contains("video has stopped")) {
                noticedMs = since.elapsed();
            }
        }
        QVERIFY2(noticedMs >= 400 && noticedMs < 1000, qPrintable(QString::number(noticedMs)));
        QVERIFY(_link->lockActive());
        QVERIFY(!_link->lockSeen());  // the box says the object is not seen
        // The camera is told to stop at once, not when the last speed runs out.
        QTRY_VERIFY_WITH_TIMEOUT(_lastSpeed("GSY") == 0.0, 250);
        // It turned for about half a second (at about 25 degrees a second here), not on and on.
        const double turned = std::abs(_camera->yaw - yawAtStart);
        QVERIFY2(turned > 3.0 && turned < 20.0, qPrintable(QString::number(turned)));
        QTest::qWait(300);
        QVERIFY2(std::abs(std::abs(_camera->yaw - yawAtStart) - turned) < 0.5,
                 qPrintable(QString("%1 then %2").arg(turned).arg(std::abs(_camera->yaw - yawAtStart))));
        // No new picture for three seconds: the lock ends and says why.
        while (_link->lockActive() && since.elapsed() < 5000) {
            QTest::qWait(90);
            _link->lockPicture(still, 0.0, 0.0, 1.0, 1.0);
        }
        QVERIFY(!_link->lockActive());
        QVERIFY2(since.elapsed() > 2800, qPrintable(QString::number(since.elapsed())));
        QVERIFY(_link->lockMessage().contains("the video stopped"));
    }

    // Without gimbal angles the same: the app asks the camera to turn, the picture stays.
    void appLockStopsTheCameraWhenTheVideoStopsAndNoAnglesCome()
    {
        _camera->answerGac = false;
        _link->lockAtBox(0.7, 0.5, 0.06, 0.2);
        const QImage still = _picture(0.7, 0.5);
        _link->lockPicture(still, 0.0, 0.0, 1.0, 1.0);
        QTRY_VERIFY_WITH_TIMEOUT(_lastSpeed("GSY") > 0.0, 1000);
        QElapsedTimer since;
        since.start();
        bool stoppedOnce = false;
        int turnsAfter = 0;
        while (since.elapsed() < 1500) {
            QTest::qWait(90);
            _link->lockPicture(still, 0.0, 0.0, 1.0, 1.0);
            QTest::qWait(30);
            const double yaw = _lastSpeed("GSY");
            if (yaw == 0.0) {
                stoppedOnce = true;
            } else if (stoppedOnce) {
                ++turnsAfter;  // once stopped, the same old picture must not start it again
            }
        }
        QVERIFY(stoppedOnce);
        QCOMPARE(turnsAfter, 0);
        QVERIFY2(_link->lockMessage().contains("video has stopped"), qPrintable(_link->lockMessage()));
        _link->stopLock();
    }

    // A slow video shows each picture for a while (here 0.3 s, three grabs each). That is not
    // a stopped video: the camera keeps following.
    void appLockKeepsFollowingASlowVideo()
    {
        _camera->followSpeeds = true;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAtBox(0.7, 0.5, 0.06, 0.2);
        _link->lockPicture(_picture(0.7, 0.5), 0.0, 0.0, 1.0, 1.0);
        for (int frame = 1; frame <= 6; ++frame) {
            const QImage picture = _picture(0.7 - 0.01 * frame, 0.5);  // the camera catches up a little
            for (int grab = 0; grab < 3; ++grab) {
                QTest::qWait(100);
                _link->lockPicture(picture, 0.0, 0.0, 1.0, 1.0);
                QVERIFY2(!_link->lockMessage().contains("video has stopped"),
                         qPrintable(QString("picture %1, grab %2: %3").arg(frame).arg(grab).arg(_link->lockMessage())));
            }
        }
        QVERIFY(_link->lockActive() && _link->lockSeen());
        QVERIFY2(_lastSpeed("GSY") > 0.0, qPrintable(QString::number(_lastSpeed("GSY"))));
        _link->stopLock();
    }

    void appLockRefusesAPlainBox()
    {
        // Ground all over, and a plain patch (sky, a white wall) where the box is drawn.
        QImage picture = _picture(0.5, 0.5, false);
        for (int y = 60; y < 156; ++y) {
            uchar *row = picture.scanLine(y);
            for (int x = 140; x < 244; ++x) {
                row[x] = 120;
            }
        }
        _link->lockAtBox(0.5, 0.5, 0.1, 0.2);
        _link->lockPicture(picture, 0.0, 0.0, 1.0, 1.0);
        QVERIFY(!_link->lockActive() && !_link->lockBusy());
        QVERIFY(_link->lockMessage().contains("plain area"));
        QVERIFY(!_link->lockWantsPictures());
    }

    // A phone that cannot copy its video picture gives the app black pictures. That is no
    // picture: the camera's own tracker is used and the text says so. (It used to say "draw
    // the box around the whole object", which sends the operator the wrong way.)
    void aBlackPictureIsNoPictureForTheLock()
    {
        _camera->followAngles = true;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        QImage black(384, 216, QImage::Format_RGB888);
        black.fill(Qt::black);
        _link->lockAtBox(0.75, 0.4, 0.06, 0.2);
        QElapsedTimer since;
        since.start();
        while (_link->lockByApp() && since.elapsed() < 2500) {
            _link->lockPicture(black, 0.0, 0.0, 1.0, 1.0);
            QVERIFY2(!_link->lockMessage().contains("plain area"), qPrintable(_link->lockMessage()));
            QVERIFY(!_link->lockActive());
            // Not locked yet, so nothing "has stopped" either: it is still waiting for a picture.
            QVERIFY2(!_link->lockByApp() || _link->lockMessage() == QString("Locking..."), qPrintable(_link->lockMessage()));
            QTest::qWait(90);
        }
        QVERIFY(!_link->lockByApp());
        QVERIFY2(_link->lockMessage().contains("No picture of the video reaches the app"), qPrintable(_link->lockMessage()));
        _link->stopLock();
    }

    // The video goes black during a lock (the stream starts again): the camera waits and the
    // lock says the video has stopped, not that the object went out of sight.
    void appLockSaysTheVideoStoppedWhenThePictureGoesBlack()
    {
        _link->lockAtBox(0.7, 0.5, 0.06, 0.2);
        _link->lockPicture(_picture(0.7, 0.5), 0.0, 0.0, 1.0, 1.0);
        QTRY_VERIFY_WITH_TIMEOUT(_lastSpeed("GSY") > 0.0, 1000);
        QImage black(384, 216, QImage::Format_Grayscale8);
        black.fill(0);
        _link->lockPicture(black, 0.0, 0.0, 1.0, 1.0);
        QVERIFY2(_link->lockMessage().contains("video has stopped"), qPrintable(_link->lockMessage()));
        QVERIFY(_link->lockActive() && !_link->lockSeen());
        QTRY_VERIFY_WITH_TIMEOUT(_lastSpeed("GSY") == 0.0, 250);
        // The video comes back: the lock goes on.
        _link->lockPicture(_picture(0.69, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockSeen());
        QVERIFY2(!_link->lockMessage().contains("video has stopped"), qPrintable(_link->lockMessage()));
        _link->stopLock();
    }

    void appLockTakesAColourPictureAndBlackBarsBesideIt()
    {
        // The thermal picture is 5:4 with black bars: the picture is the middle 0.6 of the image here.
        const QImage inner = _picture(0.7, 0.4, true, 22, 44, 320, 256);
        QImage wide(533, 256, QImage::Format_RGB32);
        wide.fill(Qt::black);
        const int left = (533 - 320) / 2;
        for (int y = 0; y < 256; ++y) {
            for (int x = 0; x < 320; ++x) {
                const int g = inner.constScanLine(y)[x];
                wide.setPixel(left + x, y, qRgb(g, g, g));
            }
        }
        _link->lockAtBox(0.7, 0.4, 0.07, 0.17);
        _link->lockPicture(wide, left / 533.0, 0.0, 320.0 / 533.0, 1.0);
        QVERIFY(_link->lockActive());
        QVERIFY2(std::abs(_link->lockBoxU() - 0.7) < 0.01 && std::abs(_link->lockBoxV() - 0.4) < 0.01,
                 qPrintable(QString("%1, %2").arg(_link->lockBoxU()).arg(_link->lockBoxV())));
        // 0.2 of the picture right of the cross, not 0.2 of the image.
        QVERIFY(std::abs(_link->lockOffDeg() - 0.2 * 83.4) < 1.0);
        _link->stopLock();
    }

    // What the app really gets: a colour image, four bytes a point, with black bars beside the
    // picture (the thermal picture is 5:4). The object must be followed in it, so the tracker
    // has to read the points with their real size and look where the picture is in the image.
    void appLockFollowsAColourObjectInAnImageWithBlackBars()
    {
        const int barLeft = (533 - 320) / 2;
        const auto framed = [barLeft](double u, double v) {
            const QImage inner = _picture(u, v, true, 22, 44, 320, 256);
            QImage wide(533, 256, QImage::Format_RGB32);
            wide.fill(Qt::black);
            for (int y = 0; y < 256; ++y) {
                const uchar *row = inner.constScanLine(y);
                QRgb *out = reinterpret_cast<QRgb *>(wide.scanLine(y));
                for (int x = 0; x < 320; ++x) {
                    // The object's light and dark blocks get colours of their own; the ground stays grey.
                    const int g = row[x];
                    out[barLeft + x] = g >= 225 ? qRgb(230, 60, 40) : (g <= 25 ? qRgb(30, 40, 150) : qRgb(g, g, g));
                }
            }
            return wide;
        };
        const double left = barLeft / 533.0;
        const double width = 320.0 / 533.0;
        _link->lockAtBox(0.3, 0.5, 0.09, 0.2);
        _link->lockPicture(framed(0.3, 0.5), left, 0.0, width, 1.0);
        QVERIFY2(_link->lockActive(), qPrintable(_link->lockMessage()));
        for (int i = 1; i <= 20; ++i) {
            QTest::qWait(40);
            _link->lockPicture(framed(0.3 + 0.01 * i, 0.5), left, 0.0, width, 1.0);
            QVERIFY2(_link->lockSeen(), qPrintable(QString("picture %1: %2").arg(i).arg(_link->lockNumbers())));
        }
        QVERIFY2(std::abs(_link->lockBoxU() - 0.5) < 0.015 && std::abs(_link->lockBoxV() - 0.5) < 0.015,
                 qPrintable(QString("%1, %2").arg(_link->lockBoxU()).arg(_link->lockBoxV())));
        _link->stopLock();
    }

    void appLockIgnoresPicturesItDidNotAskFor()
    {
        QSignalSpy boxes(_link, &SkydroidLink::lockBoxChanged);
        _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);  // no lock asked
        QVERIFY(!_link->lockActive() && !_link->lockBusy());
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        _link->lockPicture(QImage(), 0.0, 0.0, 1.0, 1.0);            // no picture at all
        _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 0.05, 0.05);  // a picture too small to see anything in
        QVERIFY(_link->lockBusy() && !_link->lockActive());
        _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 0.1, 1.0);    // too narrow: 38 points of the 64 it needs
        QVERIFY(_link->lockBusy() && !_link->lockActive());
        _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 0.1);    // too flat: 22 points of the 32 it needs
        QVERIFY(_link->lockBusy() && !_link->lockActive());
        _link->stopLock();
        QVERIFY(!_link->lockBusy());
        Q_UNUSED(boxes);
    }

    // On a device that gives the app no picture of the video, the lock must not
    // wait for ever: the camera's own tracker is used, and the text says so.
    void withNoPictureTheCamerasOwnTrackerIsUsed()
    {
        _camera->followAngles = true;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAtBox(0.75, 0.4, 0.06, 0.2);
        QVERIFY(_link->lockBusy() && _link->lockByApp());
        QTRY_VERIFY_WITH_TIMEOUT(!_link->lockByApp(), 2500);
        QVERIFY(_link->lockMessage().contains("No picture of the video reaches the app"));
        QVERIFY(_link->lockMessage().contains("camera's own tracker"));
        QVERIFY(!_link->lockWantsPictures());
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockActive(), 5000);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(_got(640, 360)) == 1, 1000);  // after the turn, under the cross
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildSumTrack(true)) >= 1, 1000);
        _link->stopLock();
    }

    void withNoPictureAndNoAnglesTheLockSaysItCannot()
    {
        _camera->answerGac = false;
        _link->lockAtBox(0.75, 0.4, 0.06, 0.2);
        QTRY_VERIFY_WITH_TIMEOUT(!_link->lockBusy(), 2500);
        QVERIFY(!_link->lockActive());
        QVERIFY(_link->lockMessage().contains("so it cannot lock"));
        QCOMPARE(_camera->countStartingWith("#tpUG8wGOT"), 0);
    }

    void movingTheCameraByHandEndsTheAppsLockToo()
    {
        const auto locked = [this]() {
            _link->lockAtBox(0.6, 0.5, 0.06, 0.2);
            _link->lockPicture(_picture(0.6, 0.5), 0.0, 0.0, 1.0, 1.0);
            return _link->lockActive();
        };
        // A finger on the video: the finger's speed goes out, not the lock's.
        QVERIFY(locked());
        _link->setTouchMotion(-0.8, 0.0);
        QVERIFY(!_link->lockActive());
        QVERIFY(_link->lockMessage().contains("moved"));
        _camera->clear();
        QTRY_VERIFY_WITH_TIMEOUT(_lastSpeed("GSY") < 0.0, 1000);  // to the left, as the finger asks
        _link->stopTouchMotion();
        // A gimbal button.
        QVERIFY(locked());
        _link->center();
        QVERIFY(!_link->lockActive());
        // A tap to measure another point.
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        QVERIFY(locked());
        _link->aimAndMeasure(0.7, 0.5);
        QVERIFY(!_link->lockActive());
        _link->stopGimbal();
        // An RC wheel.
        _link->setWheelYawChannel(10);
        QTRY_VERIFY_WITH_TIMEOUT(!_link->aimBusy(), 10000);
        QVERIFY(locked());
        _camera->clear();
        _vehicle.sendRc(_rc(10, 1900));
        QVERIFY(!_link->lockActive());
        // The wheel's speed goes out, not the lock's.
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeedAxis("GSY", 20.0)) >= 1, 2000);
        _vehicle.sendRc(_rc(10, 1500));
        // Turning the link off.
        QVERIFY(locked());
        _link->setEnabled(false);
        QVERIFY(!_link->lockActive() && !_link->lockWantsPictures());
    }

    void aZoomStepEndsTheAppsLock()
    {
        // The tracker knows the object at one size.
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        _link->zoom(1);
        QVERIFY(!_link->lockActive());
        QVERIFY(_link->lockMessage().contains("zoom was changed"));
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        _link->zoomHome();
        QVERIFY(!_link->lockActive());
        // The camera's own tracker is not ended by a zoom step, as before.
        _link->setLockMode("camera");
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive());
        _link->zoom(1);
        QVERIFY(_link->lockActive());
        _link->stopLock();
    }

    void aNewAppLockReplacesTheOldOne()
    {
        _link->lockAtBox(0.3, 0.5, 0.06, 0.2);
        _link->lockPicture(_picture(0.3, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        _link->lockAtBox(0.7, 0.3, 0.06, 0.2);
        QVERIFY(_link->lockBusy() && !_link->lockActive());
        _link->lockPicture(_picture(0.7, 0.3), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        QVERIFY(std::abs(_link->lockBoxU() - 0.7) < 0.01 && std::abs(_link->lockBoxV() - 0.3) < 0.01);
        _link->stopLock();
    }

    void aTapLocksWithABoxOfTheUsualSize()
    {
        _link->lockAt(0.5, 0.5);  // no box: a tap
        _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        // A square in points: 0.12 of the picture's height.
        QVERIFY(std::abs(_link->lockBoxH() - 0.12) < 0.005);
        QVERIFY(std::abs(_link->lockBoxW() * 384.0 - _link->lockBoxH() * 216.0) < 1.0);
        _link->stopLock();
    }

    // --- RC wheels ---------------------------------------------------------------

    void wheelThatSpringsBackSetsTheSpeed()
    {
        _link->setWheelPitchChannel(9);
        _vehicle.sendRc(_rc(9, 1900));
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeedAxis("GSP", 20.0)) >= 1, 2000);
        _vehicle.sendRc(_rc(9, 1500));
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeed(0.0, 0.0)) >= 1, 2000);
        _link->setWheelReverse(true);
        _vehicle.sendRc(_rc(9, 1900));
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeedAxis("GSP", -20.0)) >= 1, 2000);
        _vehicle.sendRc(_rc(9, 1500));
    }

    void wheelStopsWhenRcValuesStop()
    {
        _link->setWheelYawChannel(10);
        _vehicle.sendRc(_rc(10, 1100));
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeedAxis("GSY", -20.0)) >= 1, 2000);
        // No more RC_CHANNELS (RC link lost): the gimbal must stop.
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalSpeed(0.0, 0.0)) >= 1, 2500);
    }

    void wheelThatStaysSetsTheAngleOnlyAfterItIsTurned()
    {
        _link->setWheelHoldsPosition(true);
        _link->setWheelPitchChannel(9);
        _vehicle.sendRc(_rc(9, 1500));
        _vehicle.sendRc(_rc(9, 1505));
        QTest::qWait(300);
        QCOMPARE(_camera->countStartingWith("#tpUG6wGAP"), 0);  // the link starting never moves the gimbal
        _vehicle.sendRc(_rc(9, 1000));
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalAngleAxis("GAP", -90.0, 30.0)), 1, 2000);
        _vehicle.sendRc(_rc(9, 2000));
        // C13 pitch stops at +10 degrees (its limit), so the top of the wheel is +10.
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalAngleAxis("GAP", 10.0, 30.0)), 1, 2000);
        QCOMPARE(_camera->countStartingWith("#TPUG2wGS"), 0);  // no speed frames in this mode
    }

    void detectWheelPicksTheTurnedWheelNotAStick()
    {
        QSignalSpy detected(_link, &SkydroidLink::wheelDetected);
        _link->detectWheel("pitch");
        QCOMPARE(_link->detectingWheel(), QString("pitch"));
        for (int i = 0; i <= 10; ++i) {
            QVector<int> values(16, 1500);
            values[1] = 1100 + 80 * i;   // channel 2, a stick, moves even more than the wheel
            values[10] = 1300 + 40 * i;  // channel 11, the wheel
            _vehicle.sendRc(values);
            QTest::qWait(50);
        }
        QTRY_COMPARE_WITH_TIMEOUT(detected.count(), 1, 7000);
        QCOMPARE(detected.at(0).at(1).toInt(), 11);
        QCOMPARE(_link->wheelPitchChannel(), 11);
        QVERIFY(_link->detectingWheel().isEmpty());
    }

    // --- Camera ------------------------------------------------------------------

    void c13ZoomSendsLensAndEncoderSteps()
    {
        _link->zoom(1);
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countStartingWith("#tpPM2wZMC02"), 1, 2000);
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countStartingWith("#TPUD2wDZM0A"), 1, 2000);
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countStartingWith("#tpPM2wZMC00"), 1, 2000);
    }

    void c13ZoomShowsAValueAndGoesBackTo1x()
    {
        QCOMPARE(_link->zoomLabel(), QString("1.0x"));
        _link->zoom(1);
        _link->zoom(1);
        _link->zoom(1);
        QCOMPARE(_link->zoomLabel(), QString("1.3x"));
        _link->zoom(-1);
        QCOMPARE(_link->zoomLabel(), QString("1.2x"));
        _link->zoom(-1);
        _link->zoom(-1);
        _link->zoom(-1);  // never below 1x
        QCOMPARE(_link->zoomLabel(), QString("1.0x"));

        for (int i = 0; i < 4; ++i) {
            _link->zoom(1);
        }
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildDzmZoomStepV47("in")), 4 + 3, 2000);
        const int outBefore = _camera->countEqual(top::buildDzmZoomStepV47("out"));
        _link->zoomHome();
        QCOMPARE(_link->zoomLabel(), QString("1.0x"));
        // The absolute 1x and the 1x preset at once, then the steps back out
        // (4 counted plus 5 more), one every 60 ms.
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildDzmPreset(1)), 1, 2000);
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildDzmZoomStepV47("out")), outBefore + 9, 3000);
        QTest::qWait(300);
        QCOMPARE(_camera->countEqual(top::buildDzmZoomStepV47("out")), outBefore + 9);
    }

    void zoomTapStopsTheRunBackTo1x()
    {
        for (int i = 0; i < 20; ++i) {
            _link->zoom(1);
        }
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildDzmZoomStepV47("in")), 20, 2000);
        _link->zoomHome();
        QTest::qWait(150);
        _link->zoom(1);
        QCOMPARE(_link->zoomLabel(), QString("1.1x"));
        const int out = _camera->countEqual(top::buildDzmZoomStepV47("out"));
        QVERIFY(out < 25);
        QTest::qWait(400);
        QCOMPARE(_camera->countEqual(top::buildDzmZoomStepV47("out")), out);
    }

    void c13ZoomReportIsShownButNotTakenAsAStep()
    {
        _camera->dzmStep = 20;
        _link->zoom(1);
        QTRY_COMPARE_WITH_TIMEOUT(_link->zoomReport(), QString("14"), 3000);
        QCOMPARE(_link->zoomStep(), -1);
        QCOMPARE(_link->zoomLabel(), QString("1.1x"));
    }

    void c14ProZoomLabelFollowsTheCamera()
    {
        _link->setModel("C14 Pro");
        QCOMPARE(_link->zoomLabel(), QString());
        _camera->dzmStep = 0;
        _link->zoom(1);
        QTRY_COMPARE_WITH_TIMEOUT(_link->zoomLabel(), QString("W1.0x"), 3000);
        _camera->dzmStep = 70;   // short lens: 3840 / (3840 - 36 * 70) = 2.9
        _link->zoom(1);
        QTRY_COMPARE_WITH_TIMEOUT(_link->zoomLabel(), QString("W2.9x"), 3000);
        _camera->dzmStep = 100;  // long lens, 14 steps in: 3840 / 3336 * 4.60 = 5.3
        _link->zoom(1);
        QTRY_COMPARE_WITH_TIMEOUT(_link->zoomLabel(), QString("T5.3x"), 3000);
    }

    void c14ProZoomReadsTheStepBack()
    {
        _link->setModel("C14 Pro");
        _camera->dzmStep = 70;
        _link->zoom(1);
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countStartingWith("#TPUD2wDZM0A"), 1, 2000);
        QTRY_COMPARE_WITH_TIMEOUT(_link->zoomStep(), 70, 3000);
        // C14 Pro variable-length gimbal frames use the upper-case header.
        _link->stopGimbal();
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countStartingWith("#TPUG4wGSM0000") >= 1, 2000);
        QCOMPARE(_camera->countStartingWith("#tpUG4wGSM"), 0);
    }

    void photoAndRecord()
    {
        _link->takePhoto();
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countStartingWith("#TPUD2wCAP01"), 1, 2000);
        QVERIFY(!_link->recording());
        _link->toggleRecord();
        QVERIFY(_link->recording());
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countStartingWith("#TPUD2wREC01"), 1, 2000);
    }

    // --- Laser -------------------------------------------------------------------

    void laserUsesTheLaserModuleAndPlacesTheTarget()
    {
        _camera->yaw = 0.0;
        _camera->pitch = -30.0;
        _camera->slrE = "01F4";  // 500 dm = 50 m
        _camera->slrD = "0064";  // must be ignored when E answers
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(_link->laserValid());
        QCOMPARE(_link->laserRangeM(), 50.0);
        // The shot is fired at the laser module before it is read.
        QCOMPARE(_camera->countStartingWith("#TPUE2wSLR01"), 1);
        const int shot = _camera->received.indexOf(QString::fromStdString(top::buildSlrTrigger('E')));
        const int read = _camera->received.indexOf(QString::fromStdString(top::buildSlrQuery('E')));
        QVERIFY(shot >= 0 && read > shot);

        QVERIFY(_link->targetValid());
        QVERIFY(_link->targetMessage().isEmpty());
        QVERIFY(std::abs(_link->targetBearingDeg()) < 1e-6 || std::abs(_link->targetBearingDeg() - 360.0) < 1e-6);
        QVERIFY(std::abs(_link->targetHorizontalM() - 50.0 * std::cos(30.0 * kPi / 180.0)) < 1e-6);
        QVERIFY(_link->targetHasAlt());
        QVERIFY(std::abs(_link->targetAltMsl() - 75.0) < 1e-6);
        QVERIFY(_link->targetLat() > 20.0);
        QVERIFY(std::abs(_link->targetLon() - 72.0) < 1e-9);
    }

    void gimbalYawTurnsTheTargetDirection()
    {
        // The C13 counts a turn to the LEFT as positive yaw, and the tap
        // aiming has always known it. The drone faces north and the camera
        // reports +90: it looks west. Until 2026-10-08 this test expected
        // east, the mirrored side, and so did the code.
        _camera->yaw = 90.0;
        _camera->pitch = -30.0;
        _camera->slrE = "01F4";
        _vehicle.vehicle.set("heading", 0.0);
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid() && _link->gimbalYaw() == 90.0, 2000);
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(_link->targetValid());
        QVERIFY(std::abs(_link->targetBearingDeg() - 270.0) < 1e-6);
        QVERIFY(_link->targetLon() < 72.0);
    }

    void aCameraTurnedRightPutsTheTargetOnTheRight()
    {
        // Turned 30 degrees to the right, the C13 reports -30.
        _camera->yaw = -30.0;
        _camera->pitch = -30.0;
        _camera->slrE = "01F4";
        _vehicle.vehicle.set("heading", 0.0);
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid() && _link->gimbalYaw() == -30.0, 2000);
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(_link->targetValid());
        QVERIFY(std::abs(_link->targetBearingDeg() - 30.0) < 1e-6);
        QVERIFY(_link->targetLon() > 72.0);
    }

    void aCameraSetToCountTheOtherWayTurnsTheTargetWithIt()
    {
        // "Reverse left and right" for the tap says the camera counts the
        // other way round. It is the same camera, so the lat long follows.
        _link->setReverseTapYaw(true);
        _camera->yaw = 90.0;
        _camera->pitch = -30.0;
        _camera->slrE = "01F4";
        _vehicle.vehicle.set("heading", 0.0);
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid() && _link->gimbalYaw() == 90.0, 2000);
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(_link->targetValid());
        QVERIFY(std::abs(_link->targetBearingDeg() - 90.0) < 1e-6);
        QVERIFY(_link->targetLon() > 72.0);
    }

    void laserPrefersTheLaserModuleEvenWhenTheSystemAnswersFirst()
    {
        _camera->slrD = "0064";           // 10 m, answers at once
        _camera->slrE = "01F4";           // 50 m, answers late
        _camera->laserModuleDelayMs = 400;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(_link->laserValid());
        QCOMPARE(_link->laserRangeM(), 50.0);
    }

    void laserFallsBackToTheSystemAddress()
    {
        _camera->slrD = "0064";  // 10 m
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(_link->laserValid());
        QCOMPARE(_link->laserRangeM(), 10.0);
    }

    void noLaserReplyIsReportedNotInvented()
    {
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->fireLaser();
        QVERIFY(_link->laserBusy());
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(!_link->laserValid());
        QVERIFY(_link->laserMessage().contains("No laser reading"));
        QVERIFY(!_link->targetValid());
    }

    void noGpsFixMeansNoTarget()
    {
        _vehicle.gps.set("lock", 1);
        _camera->slrE = "01F4";
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(_link->laserValid());
        QVERIFY(!_link->targetValid());
        // The distance stands; the text says only the lat long is missing, and why.
        QCOMPARE(_link->laserRangeM(), 50.0);
        QVERIFY(_link->targetMessage().startsWith("No lat long"));
        QVERIFY(_link->targetMessage().contains("no GPS lock"));
        QVERIFY(_link->targetMessage().contains("distance is still valid"));

        _vehicle.gps.set("lock", 2);  // a 2D fix is not enough either
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(!_link->targetValid());
        QVERIFY(_link->targetMessage().contains("2D GPS fix"));

        _vehicle.gps.set("lock", 3);
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(_link->targetValid());
        QVERIFY(_link->targetMessage().isEmpty());
    }

    void noGimbalAnglesMeansNoTarget()
    {
        _camera->answerGac = false;
        _camera->slrE = "01F4";
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(_link->laserValid());
        QVERIFY(!_link->targetValid());
        QVERIFY(_link->targetMessage().startsWith("No lat long"));
        QVERIFY(_link->targetMessage().contains("gimbal"));
    }

    void noDroneMeansNoTarget()
    {
        MultiVehicleManager::instance()->setActiveVehicle(nullptr);
        _camera->slrE = "01F4";
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        MultiVehicleManager::instance()->setActiveVehicle(&_vehicle);
        QVERIFY(!_link->targetValid());
        QVERIFY(_link->targetMessage().startsWith("No lat long"));
        QVERIFY(_link->targetMessage().contains("no drone is connected"));
    }

    // --- A point from the picture, without the laser ------------------------------

    /// Frames that turn the camera to an angle, or fire or read the laser.
    int _turnsAndShots() const
    {
        int count = 0;
        for (const QString &frame : _camera->received) {
            count += (frame.contains("wGAY") || frame.contains("wGAP") || frame.contains("wSLR")) ? 1 : 0;
        }
        return count;
    }

    /// The drone 100 m up over level ground, 350 m above sea level, the camera 30 degrees down.
    void _flyAtAHundredMetres()
    {
        _vehicle.vehicle.set("altitudeRelative", 100.0);
        _vehicle.vehicle.set("altitudeAMSL", 350.0);
        _camera->yaw = 0.0;
        _camera->pitch = -30.0;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid() && _link->gimbalPitch() == -30.0, 2000);
    }

    void theLaserIsTheUsualWayToAPointAndTheChoiceIsKept()
    {
        QCOMPARE(_link->pointMode(), QString("laser"));
        QCOMPARE(SkydroidLink::pointModes(), QStringList({"laser", "picture"}));
        QCOMPARE(SkydroidLink::pointModeNames().size(), 2);
        QSignalSpy changed(_link, &SkydroidLink::settingsChanged);
        _link->setPointMode("picture");
        QCOMPARE(_link->pointMode(), QString("picture"));
        QCOMPARE(changed.count(), 1);
        _link->setPointMode("nonsense");  // an unknown choice is the laser
        QCOMPARE(_link->pointMode(), QString("laser"));
        _link->setPointMode("picture");
        delete _link;
        _link = new SkydroidLink;
        QCOMPARE(_link->pointMode(), QString("picture"));
    }

    void aTapInPictureModeGivesThePositionAtOnceWithoutTheLaser()
    {
        _link->setPointMode("picture");
        _flyAtAHundredMetres();
        _camera->slrE = "01F4";  // a laser that would answer 50 m: it is not asked
        _camera->clear();
        _link->aimAndMeasure(0.5, 0.5);
        // At once: nothing to wait for.
        QVERIFY(!_link->aimBusy() && !_link->laserBusy());
        QVERIFY(_link->targetValid() && _link->targetFromPicture() && _link->measuredByPicture());
        QVERIFY(!_link->laserValid());
        QVERIFY(_link->laserMessage().isEmpty());
        // 100 m up and 30 degrees down: 173.2 m north on the ground, 200 m from the drone, at the take-off height.
        QVERIFY2(std::abs(_link->targetHorizontalM() - 173.205) < 0.01, qPrintable(QString::number(_link->targetHorizontalM())));
        QVERIFY(std::abs(_link->targetSlantM() - 200.0) < 0.01);
        QVERIFY(_link->targetLat() > 20.0 && std::abs(_link->targetLon() - 72.0) < 1e-9);
        QVERIFY(_link->targetHasAlt() && std::abs(_link->targetAltMsl() - 250.0) < 1e-6);
        // The result says how it was made and how far to trust it.
        QVERIFY2(_link->targetMessage().contains("From the picture, not by laser"), qPrintable(_link->targetMessage()));
        QVERIFY2(_link->targetMessage().contains("is 7.0 m here"), qPrintable(_link->targetMessage()));
        // The camera was not turned and the laser was not fired.
        QTest::qWait(500);
        QCOMPARE(_turnsAndShots(), 0);
        QVERIFY(_link->targetValid());
    }

    void aTapAwayFromTheCrossInPictureModeIsPlacedThroughTheLens()
    {
        _link->setPointMode("picture");
        _flyAtAHundredMetres();
        // Right of the cross: to the right of north. Below the cross: nearer.
        _link->aimAndMeasure(0.75, 0.5);
        QVERIFY(_link->targetValid());
        const double wide = _link->targetBearingDeg();
        QVERIFY2(wide > 10.0 && wide < 40.0 && _link->targetLon() > 72.0, qPrintable(QString::number(wide)));
        _link->aimAndMeasure(0.5, 0.75);
        QVERIFY(_link->targetValid());
        QVERIFY2(_link->targetHorizontalM() < 173.2 - 40.0, qPrintable(QString::number(_link->targetHorizontalM())));
        // Zoomed in, the same place of the picture is nearer to the cross.
        _link->zoom(1);
        _link->zoom(1);
        _link->aimAndMeasure(0.75, 0.5);
        QVERIFY(_link->targetValid());
        QVERIFY2(_link->targetBearingDeg() < wide - 2.0, qPrintable(QString("%1 after %2").arg(_link->targetBearingDeg()).arg(wide)));
        // A camera turned to the right (the C13 says -30) puts the point on the right.
        _camera->yaw = -30.0;
        QTRY_VERIFY_WITH_TIMEOUT(_link->gimbalYaw() == -30.0, 2000);
        _link->aimAndMeasure(0.5, 0.5);
        QVERIFY2(std::abs(_link->targetBearingDeg() - 30.0) < 1e-6, qPrintable(QString::number(_link->targetBearingDeg())));
    }

    void pictureModeSaysWhyThereIsNoPosition()
    {
        _link->setPointMode("picture");
        _camera->yaw = 0.0;
        _camera->pitch = -30.0;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid() && _link->gimbalPitch() == -30.0, 2000);
        const auto tapSays = [this](const char *words) {
            _link->aimAndMeasure(0.5, 0.5);
            const bool ok = !_link->targetValid() && !_link->targetFromPicture() && _link->measuredByPicture() &&
                            !_link->laserValid() && _link->laserMessage().contains(words);
            if (!ok) {
                qWarning() << "wanted" << words << "got" << _link->laserMessage() << "target" << _link->targetValid();
            }
            return ok;
        };
        // The height of the drone is not known.
        QVERIFY(tapSays("the drone's height is not known"));
        // On the ground: no height to measure with.
        _vehicle.vehicle.set("altitudeRelative", 1.2);
        QVERIFY(tapSays("the drone is 1.2 m up, it needs 2.5 m"));
        // The flight of 2026-10-08: 83 m up, the camera 5 degrees under the horizon.
        _vehicle.vehicle.set("altitudeRelative", 82.7);
        _camera->pitch = -5.0;
        QTRY_VERIFY_WITH_TIMEOUT(_link->gimbalPitch() == -5.0, 2000);
        QVERIFY(tapSays("looked at 5 degrees down, it needs 8"));
        _camera->pitch = 4.0;
        QTRY_VERIFY_WITH_TIMEOUT(_link->gimbalPitch() == 4.0, 2000);
        QVERIFY(tapSays("at the horizon or above it"));
        // No GPS lock: no lat long, and nothing is said about a distance that was never measured.
        _camera->pitch = -30.0;
        QTRY_VERIFY_WITH_TIMEOUT(_link->gimbalPitch() == -30.0, 2000);
        _vehicle.gps.set("lock", 1);
        QVERIFY(tapSays("the drone has no GPS lock yet"));
        QVERIFY(!_link->laserMessage().contains("distance"));
        _vehicle.gps.set("lock", 3);
        // And with all of it there, the same tap gives the position.
        _link->aimAndMeasure(0.5, 0.5);
        QVERIFY(_link->targetValid() && _link->laserMessage().isEmpty());
    }

    void pictureModeNeedsTheCamerasOwnAngles()
    {
        // The camera said "30 degrees down" a while ago and has been silent since. Those old
        // angles would give a point that looks fine. None is placed.
        _link->setPointMode("picture");
        _flyAtAHundredMetres();
        _camera->answerGac = false;
        QTRY_VERIFY_WITH_TIMEOUT(!_link->attitudeValid(), 8000);
        _link->aimAndMeasure(0.5, 0.5);
        QVERIFY(!_link->targetValid());
        QVERIFY2(_link->laserMessage().contains("the camera sends no gimbal angles"), qPrintable(_link->laserMessage()));
    }

    void withNoLaserReadingThePositionComesFromThePicture()
    {
        // The usual way, by laser. The laser gives nothing (too far, or it did not answer).
        _flyAtAHundredMetres();
        _camera->clear();
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(_camera->countStartingWith("#TPUE2wSLR01") >= 1);  // the laser was asked
        QVERIFY(!_link->laserValid());
        QVERIFY(!_link->measuredByPicture());
        QCOMPARE(_link->laserMessage(), QString("No laser reading."));
        // Where the cross is on the ground, from the picture, and the result says so.
        QVERIFY(_link->targetValid() && _link->targetFromPicture());
        QVERIFY(std::abs(_link->targetHorizontalM() - 173.205) < 0.01);
        QVERIFY(_link->targetMessage().contains("From the picture, not by laser"));
    }

    void withNoLaserReadingThePictureStillNeedsTheCamerasAnglesAndGps()
    {
        // The camera said "30 degrees down" a while ago and has been silent since: a point
        // is never placed on angles as old as that.
        _flyAtAHundredMetres();
        _camera->answerGac = false;
        QTRY_VERIFY_WITH_TIMEOUT(!_link->attitudeValid(), 8000);
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(!_link->laserValid() && !_link->targetValid() && !_link->targetFromPicture());
        QVERIFY(_link->laserMessage().contains("No laser reading"));
        // The angles are there, the drone has no GPS lock: no position either.
        _camera->answerGac = true;
        _camera->pitch = -30.0;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid() && _link->gimbalPitch() == -30.0, 2000);
        _vehicle.gps.set("lock", 1);
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(!_link->laserValid() && !_link->targetValid() && !_link->targetFromPicture());
        // With GPS it is there.
        _vehicle.gps.set("lock", 3);
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(_link->targetValid() && _link->targetFromPicture());
    }

    void aLaserRangeIsNeverReplacedByThePicture()
    {
        _flyAtAHundredMetres();
        _camera->slrE = "01F4";  // 50 m: something nearer than the ground is under the cross
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(_link->laserValid());
        QVERIFY(_link->targetValid() && !_link->targetFromPicture() && !_link->measuredByPicture());
        QVERIFY(std::abs(_link->targetHorizontalM() - 50.0 * std::cos(30.0 * kPi / 180.0)) < 1e-6);
        QVERIFY(_link->targetMessage().isEmpty());
    }

    void onTheThermalPictureOnlyTheCrossIsPlacedUnlessTheLensIsKnown()
    {
        _link->setPointMode("picture");
        _flyAtAHundredMetres();
        QSignalSpy told(_link, &SkydroidLink::thermalPictureChanged);
        _link->setThermalPicture(true);
        QCOMPARE(told.count(), 1);
        // C13: its thermal lens is not known. A tap away from the cross is refused, and says what to do.
        _link->aimAndMeasure(0.8, 0.5);
        QVERIFY(!_link->targetValid());
        QVERIFY2(_link->laserMessage().contains("thermal") && _link->laserMessage().contains("tap the cross"),
                 qPrintable(_link->laserMessage()));
        // A tap on the cross (a finger is not exact) is the cross.
        _link->aimAndMeasure(0.53, 0.47);
        QVERIFY(_link->targetValid());
        QVERIFY(std::abs(_link->targetHorizontalM() - 173.205) < 0.01);
        const double bearing = _link->targetBearingDeg();
        QVERIFY2(bearing < 1e-6 || bearing > 360.0 - 1e-6, qPrintable(QString::number(bearing)));
        // The day picture again: the same tap away from the cross is placed.
        _link->setThermalPicture(false);
        _link->aimAndMeasure(0.8, 0.5);
        QVERIFY(_link->targetValid());
        // C14 Pro: its thermal lens is known (32.84 x 26.35 degrees; its day lens is 61.4 x 47.9).
        // So a tap on the thermal picture is placed, and nearer to the cross than the same tap by day.
        _link->setModel("C14 Pro");
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->aimAndMeasure(0.8, 0.5);
        QVERIFY(_link->targetValid());
        const double dayBearing = _link->targetBearingDeg();
        _link->aimAndMeasure(0.5, 0.8);
        QVERIFY(_link->targetValid());
        const double dayRange = _link->targetHorizontalM();
        _link->setThermalPicture(true);
        _link->aimAndMeasure(0.8, 0.5);
        QVERIFY2(_link->targetValid(), qPrintable(_link->laserMessage()));
        QVERIFY2(_link->targetBearingDeg() > 3.0 && _link->targetBearingDeg() < 0.7 * dayBearing,
                 qPrintable(QString("%1, by day %2").arg(_link->targetBearingDeg()).arg(dayBearing)));
        // Below the cross: by day 14 degrees further down, on the thermal picture 8.
        _link->aimAndMeasure(0.5, 0.8);
        QVERIFY(_link->targetValid());
        QVERIFY2(_link->targetHorizontalM() > dayRange + 15.0 && _link->targetHorizontalM() < 173.2,
                 qPrintable(QString("%1 m, by day %2 m").arg(_link->targetHorizontalM()).arg(dayRange)));
    }

    void aLockInPictureModeIsMeasuredFromThePictureWithoutTheLaser()
    {
        _link->setPointMode("picture");
        _flyAtAHundredMetres();
        _camera->slrE = "01F4";  // would answer 50 m: it is not asked
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        _camera->clear();
        _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        for (int i = 0; i < 12 && !_link->targetValid(); ++i) {
            QTest::qWait(90);
            _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
        }
        QVERIFY(_link->targetValid() && _link->targetFromPicture());
        QVERIFY(std::abs(_link->targetHorizontalM() - 173.205) < 0.01);
        QVERIFY(_link->lockActive());
        QCOMPARE(_camera->countStartingWith("#TPUE2wSLR"), 0);
        _link->stopLock();
    }

    // The video shows the scene a little late. Right after the operator moved the camera, the
    // picture tapped is from before the move, and the angles are from after it.
    void aTapInPictureModeWaitsUntilTheCameraIsStill()
    {
        _link->setPointMode("picture");
        _flyAtAHundredMetres();
        _link->setTouchMotion(0.8, 0.0);  // a finger drags the camera to the right
        QTRY_VERIFY_WITH_TIMEOUT(std::abs(_lastSpeed("GSY")) > 0.0, 1000);
        _link->stopTouchMotion();
        QTest::qWait(300);
        _link->aimAndMeasure(0.6, 0.6);
        QVERIFY(!_link->targetValid());
        QVERIFY2(_link->laserMessage().contains("still moving"), qPrintable(_link->laserMessage()));
        // A second after the camera stopped the same tap is placed.
        QTest::qWait(1000);
        _link->aimAndMeasure(0.6, 0.6);
        QVERIFY2(_link->targetValid() && _link->targetFromPicture(), qPrintable(_link->laserMessage()));
    }

    // A button turns the camera to an angle, and the turn goes on for seconds after the tap
    // on the button. A point from the picture waits until the gimbal's angles stand still.
    void aTapInPictureModeWaitsWhileAButtonTurnGoesOn()
    {
        _link->setPointMode("picture");
        _flyAtAHundredMetres();
        _link->pointDown();
        _link->aimAndMeasure(0.5, 0.5);  // at once, before the gimbal has even begun
        QVERIFY(!_link->targetValid());
        QVERIFY2(_link->laserMessage().contains("still moving"), qPrintable(_link->laserMessage()));
        // The gimbal takes two seconds to get there (30 degrees a second).
        for (int i = 1; i <= 20; ++i) {
            QTest::qWait(100);
            _camera->pitch = -30.0 - 3.0 * i;
        }
        QTRY_VERIFY_WITH_TIMEOUT(_link->gimbalPitch() == -90.0, 1000);
        _link->aimAndMeasure(0.5, 0.5);  // more than two seconds after the button, just arrived
        QVERIFY(!_link->targetValid());
        QVERIFY2(_link->laserMessage().contains("still moving"), qPrintable(_link->laserMessage()));
        QTest::qWait(1100);
        _link->aimAndMeasure(0.5, 0.5);
        QVERIFY2(_link->targetValid() && _link->targetFromPicture(), qPrintable(_link->laserMessage()));
    }

    // Angles that change long after the last command are not the app's doing (the drone turns
    // under a steady camera): they do not hold a tap back.
    void aTapInPictureModeDoesNotWaitForATurnTheAppDidNotAskFor()
    {
        _link->setPointMode("picture");
        _flyAtAHundredMetres();
        _link->centerYaw();  // the gimbal is at 0 already: nothing moves
        QTest::qWait(5300);
        _camera->yaw = 3.0;
        QTRY_VERIFY_WITH_TIMEOUT(_link->gimbalYaw() == 3.0, 1000);
        _link->aimAndMeasure(0.5, 0.5);
        QVERIFY2(_link->targetValid() && _link->targetFromPicture(), qPrintable(_link->laserMessage()));
    }

    void aTapInPictureModeEndsALockAndSaysWhy()
    {
        _link->setPointMode("picture");
        _flyAtAHundredMetres();
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        _link->aimAndMeasure(0.6, 0.6);
        QVERIFY(!_link->lockActive());
        QVERIFY2(_link->lockMessage().contains("another point was tapped"), qPrintable(_link->lockMessage()));
        QVERIFY(_link->targetValid() && _link->targetFromPicture());
    }

    void videoAddressesHaveDefaultsAndSurvive()
    {
        QCOMPARE(_link->dayVideoUrl(), QString("rtsp://192.168.144.108:554/stream=1"));
        QCOMPARE(_link->thermalVideoUrl(), QString("rtsp://192.168.144.108:555/stream=2"));
        _link->setDayVideoUrl("  rtsp://10.0.0.5:554/main  ");
        _link->setThermalVideoUrl("");  // empty = the camera's default
        delete _link;
        _link = new SkydroidLink;
        QCOMPARE(_link->dayVideoUrl(), QString("rtsp://10.0.0.5:554/main"));
        QCOMPARE(_link->thermalVideoUrl(), QString("rtsp://192.168.144.108:555/stream=2"));
    }

    void screenShowsVamaCameraNames()
    {
        // The screen shows VAMA's names; the code and settings keep Skydroid's.
        QCOMPARE(SkydroidLink::modelNames().size(), SkydroidLink::models().size());
        QCOMPARE(_link->model(), QString("C13"));
        QCOMPARE(_link->modelName(), QString("V13"));
        _link->setModel("C14 Pro");
        QCOMPARE(_link->modelName(), QString("V14 Pro"));
        _link->setModel("C12");
        QCOMPARE(_link->modelName(), QString("V12"));
        _link->setModel("V13");  // not a model id: ignored
        QCOMPARE(_link->model(), QString("C12"));
    }

    // --- Thermal colour modes ------------------------------------------------

    void thermalPalettesAreTheVgcsModes()
    {
        // The same ids, names and order as VGCS: the order is also the row of
        // each mode in thermal_palettes.png.
        QCOMPARE(SkydroidLink::thermalPalettes().size(), kThermalPaletteCount);
        QCOMPARE(SkydroidLink::thermalPaletteNames().size(), kThermalPaletteCount);
        for (int i = 0; i < kThermalPaletteCount; ++i) {
            QCOMPARE(SkydroidLink::thermalPalettes().at(i), QString::fromUtf8(kThermalPaletteIds[i]));
            QCOMPARE(SkydroidLink::thermalPaletteNames().at(i), QString::fromUtf8(kThermalPaletteNames[i]));
        }
    }

    void thermalPaletteStartsAsReceivedAndSurvives()
    {
        QCOMPARE(_link->thermalPalette(), QString("camera"));
        QCOMPARE(_link->thermalPaletteIndex(), 0);
        QSignalSpy changed(_link, &SkydroidLink::settingsChanged);
        _link->setThermalPalette("  Ironbow ");  // typed: any case, spaces
        QCOMPARE(_link->thermalPalette(), QString("ironbow"));
        QCOMPARE(_link->thermalPaletteIndex(), 2);
        QCOMPARE(changed.size(), 1);
        _link->setThermalPalette("ironbow");  // no change, no signal
        QCOMPARE(changed.size(), 1);
        delete _link;
        _link = new SkydroidLink;
        QCOMPARE(_link->thermalPalette(), QString("ironbow"));
        _link->setThermalPalette("plasma");  // unknown: as received, like VGCS
        QCOMPARE(_link->thermalPalette(), QString("camera"));
        QCOMPARE(_link->thermalPaletteIndex(), 0);
    }

    void unknownSavedThermalPaletteLoadsAsReceived()
    {
        delete _link;
        QSettings().setValue("VamaSkydroid/thermalPalette", "plasma");
        _link = new SkydroidLink;
        QCOMPARE(_link->thermalPalette(), QString("camera"));
        delete _link;
        QSettings().setValue("VamaSkydroid/thermalPalette", "SEPIA");
        _link = new SkydroidLink;
        QCOMPARE(_link->thermalPalette(), QString("sepia"));
        QCOMPARE(_link->thermalPaletteIndex(), kThermalPaletteCount - 1);
    }

    void settingsSurviveARestart()
    {
        _link->setModel("C14 Pro");
        _link->setHost("127.0.0.2");
        _link->setMaxSpeed(35);
        _link->setReverseYaw(true);
        _link->setWheelPitchChannel(9);
        _link->setWheelHoldsPosition(true);
        delete _link;
        _link = new SkydroidLink;
        QCOMPARE(_link->model(), QString("C14 Pro"));
        QCOMPARE(_link->host(), QString("127.0.0.2"));
        QCOMPARE(_link->maxSpeed(), 35);
        QVERIFY(_link->reverseYaw());
        QCOMPARE(_link->wheelPitchChannel(), 9);
        QVERIFY(_link->wheelHoldsPosition());
        QVERIFY(_link->enabled());
    }

    // --- Follow Me: where the locked object is on the ground ---------------------

    void aLockedObjectUnderTheCrossIsPlacedOnTheGround()
    {
        _flyAtAHundredMetres();
        QSignalSpy placed(_link, &SkydroidLink::objectPlaced);
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
        QCOMPARE(placed.count(), 1);
        const QList<QVariant> place = placed.constFirst();
        // 100 m up and 30 degrees down: 173.2 m north of the drone.
        const QGeoCoordinate object(place.at(0).toDouble(), place.at(1).toDouble());
        QVERIFY2(std::abs(_vehicle.position.distanceTo(object) - 173.2) < 0.5,
                 qPrintable(QString::number(_vehicle.position.distanceTo(object))));
        QVERIFY(object.latitude() > 20.0 && std::abs(object.longitude() - 72.0) < 1e-7);
        // Such a place is sure to about 5 m: half a degree of camera angle is 3.5 m here, and so are 2 m of height.
        QVERIFY2(std::abs(place.at(2).toDouble() - 4.92) < 0.1, qPrintable(QString::number(place.at(2).toDouble())));
        QVERIFY(std::isnan(place.at(3).toDouble()));  // no laser range: the ground is taken as level
        QCOMPARE(place.at(4).toInt(), 300);           // the picture is 0.3 s old
        _link->stopLock();
    }

    void aLockedObjectBesideTheCrossIsPlacedThroughTheLens()
    {
        _flyAtAHundredMetres();
        QSignalSpy placed(_link, &SkydroidLink::objectPlaced);
        _link->lockAtBox(0.62, 0.4, 0.06, 0.2);
        _link->lockPicture(_picture(0.62, 0.4), 0.0, 0.0, 1.0, 1.0);
        QCOMPARE(placed.count(), 1);
        // The same point as a tap on the box's middle gives without the laser.
        skydroid::geo::PictureInput input;
        input.view.vehicleLatDeg = 20.0;
        input.view.vehicleLonDeg = 72.0;
        input.view.gimbalYawLeftPositive = true;
        input.view.gimbalPitchDeg = -30.0;
        input.view.videoXNorm = _link->lockBoxU();
        input.view.videoYNorm = _link->lockBoxV();
        input.view.cameraHfovDeg = 83.4;
        input.view.cameraVfovDeg = 46.9;
        input.heightAboveGroundM = 100.0;
        const skydroid::geo::PictureResult want = skydroid::geo::computePictureTarget(input);
        QVERIFY(want.point.ok && want.point.targetLonDeg > 72.0001);  // right of the cross is east of north here
        QVERIFY(std::abs(placed.constFirst().at(0).toDouble() - want.point.targetLatDeg) < 1e-7);
        QVERIFY(std::abs(placed.constFirst().at(1).toDouble() - want.point.targetLonDeg) < 1e-7);
        _link->stopLock();
    }

    void aLockedObjectIsNotPlacedOnAGuess_data()
    {
        QTest::addColumn<QString>("missing");
        QTest::newRow("the drone's height is not known") << "height";
        QTest::newRow("the drone has no 3D GPS position") << "gps";
        QTest::newRow("the camera's angles are old") << "angles";
        QTest::newRow("the camera looks too flat") << "flat";
    }

    void aLockedObjectIsNotPlacedOnAGuess()
    {
        QFETCH(QString, missing);
        _flyAtAHundredMetres();
        if (missing == "angles") {
            // The camera has gone quiet: its last angles (30 degrees down) are 2 s old.
            _camera->answerGac = false;
            QTest::qWait(2000);
            QCOMPARE(_link->gimbalPitch(), -30.0);
        }
        if (missing == "height") {
            _vehicle.vehicle.remove("altitudeRelative");
        } else if (missing == "gps") {
            _vehicle.gps.set("lock", 2);
        } else if (missing == "flat") {
            _camera->pitch = -5.0;  // 5 degrees under the horizon: the point runs away with every part of a degree
            QTRY_VERIFY_WITH_TIMEOUT(_link->gimbalPitch() == -5.0, 2000);
        }
        QSignalSpy placed(_link, &SkydroidLink::objectPlaced);
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        _goOn(700);
        QVERIFY(_link->lockActive() && _link->lockSeen());  // the camera is locked on it all the same
        QCOMPARE(placed.count(), 0);
        _link->stopLock();
    }

    void aPlaceTakesOffWhatTheCameraHasTurnedSinceThePicture()
    {
        // The object is 10 degrees right of the cross, so the app turns the camera to it, at 25 degrees a
        // second. A picture is 0.3 s old: when the next one comes, the camera has turned part of the way
        // that picture still shows. (This camera's video never shows the turn: the same picture again.)
        _flyAtAHundredMetres();
        QSignalSpy placed(_link, &SkydroidLink::objectPlaced);
        _link->lockAtBox(0.62, 0.5, 0.06, 0.2);
        _link->lockPicture(_picture(0.62, 0.5), 0.0, 0.0, 1.0, 1.0);
        QCOMPARE(placed.count(), 1);
        QTest::qWait(250);
        _link->lockPicture(_picture(0.62, 0.5), 0.0, 0.0, 1.0, 1.0);
        QCOMPARE(placed.count(), 2);
        const double first = placed.at(0).at(1).toDouble();
        const double second = placed.at(1).at(1).toDouble();
        // Right of the cross is east here. 10 degrees are 35 m at this range; after 6 degrees of turn, 12 m.
        QVERIFY2(first > 72.00030 && first < 72.00037, qPrintable(QString::number(first, 'f', 6)));
        QVERIFY2(second > 72.00005 && second < 72.00018, qPrintable(QString::number(second, 'f', 6)));
        _link->stopLock();
    }

    void placesOfTheLockedObjectComeFiveTimesASecondAtTheMost()
    {
        _flyAtAHundredMetres();
        QSignalSpy placed(_link, &SkydroidLink::objectPlaced);
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        QElapsedTimer clock;
        clock.start();
        int pictures = 0;
        while (clock.elapsed() < 1000) {
            _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
            ++pictures;
            QTest::qWait(30);
        }
        QVERIFY(pictures >= 15);
        QVERIFY2(placed.count() >= 4 && placed.count() <= 6, qPrintable(QString::number(placed.count())));
        _link->stopLock();
    }

    void aLaserRangeOnTheLockedObjectTellsOfHigherGroundThere()
    {
        // 100 m up, 30 degrees down. Over level ground the laser would say 200 m. It says 150 m: the object
        // is 75 m below the drone, on ground 25 m higher than the take-off point, and so 129.9 m away, not 173.2.
        _flyAtAHundredMetres();
        _camera->slrE = "05DC";
        QSignalSpy placed(_link, &SkydroidLink::objectPlaced);
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        QVERIFY(_goOnUntil([this]() { return _link->laserValid(); }, 3000));
        QCOMPARE(_link->laserRangeM(), 150.0);
        placed.clear();
        _goOn(500);
        QVERIFY(placed.count() >= 1);
        const QList<QVariant> place = placed.constLast();
        const double away = _vehicle.position.distanceTo(QGeoCoordinate(place.at(0).toDouble(), place.at(1).toDouble()));
        QVERIFY2(std::abs(away - 129.9) < 0.5, qPrintable(QString::number(away)));
        QVERIFY2(std::abs(place.at(3).toDouble() - 75.0) < 0.2, qPrintable(QString::number(place.at(3).toDouble())));
        // And the place is surer than without the laser (4.9 m).
        QVERIFY2(place.at(2).toDouble() < 2.2, qPrintable(QString::number(place.at(2).toDouble())));
        _link->stopLock();
    }

    void aLaserRangeThatTellsOfLowerGroundDoesNotPutTheObjectFurtherAway()
    {
        // The laser says 250 m: ground 25 m lower than the take-off point, and the object 216.5 m away. But a
        // shot that passes the object and lands behind it says the same. An object taken as further away than
        // it is would have the drone flown nearer to it. So the object stays where level ground puts it.
        _flyAtAHundredMetres();
        _camera->slrE = "09C4";
        QSignalSpy placed(_link, &SkydroidLink::objectPlaced);
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        QVERIFY(_goOnUntil([this]() { return _link->laserValid(); }, 3000));
        QCOMPARE(_link->laserRangeM(), 250.0);
        placed.clear();
        _goOn(500);
        QVERIFY(placed.count() >= 1);
        const QList<QVariant> place = placed.constLast();
        const double away = _vehicle.position.distanceTo(QGeoCoordinate(place.at(0).toDouble(), place.at(1).toDouble()));
        QVERIFY2(std::abs(away - 173.2) < 0.5, qPrintable(QString::number(away)));
        // How far the object is below the drone is said as it was measured.
        QVERIFY2(std::abs(place.at(3).toDouble() - 125.0) < 0.2, qPrintable(QString::number(place.at(3).toDouble())));
        _link->stopLock();
    }

    void aLaserRangeOnTheLockedObjectCountsForFifteenSeconds()
    {
        // Later the object may stand on other ground, and the drone be elsewhere.
        _flyAtAHundredMetres();
        _camera->slrE = "05DC";  // 150 m: ground 25 m higher
        QSignalSpy placed(_link, &SkydroidLink::objectPlaced);
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        QVERIFY(_goOnUntil([this]() { return _link->laserValid(); }, 3000));
        _camera->slrE.reset();  // from now on the laser gives nothing
        const auto away = [this, &placed]() {
            const QList<QVariant> place = placed.constLast();
            return _vehicle.position.distanceTo(QGeoCoordinate(place.at(0).toDouble(), place.at(1).toDouble()));
        };
        _goOn(11500);
        placed.clear();
        _goOn(400);
        QVERIFY(placed.count() >= 1);
        QVERIFY2(std::abs(away() - 129.9) < 0.5, qPrintable(QString::number(away())));  // 12 s later it still counts
        _goOn(4000);
        placed.clear();
        _goOn(400);
        QVERIFY(placed.count() >= 1);
        QVERIFY2(std::abs(away() - 173.2) < 0.5, qPrintable(QString::number(away())));  // 16 s later: level ground again
        QVERIFY(std::isnan(placed.constLast().at(3).toDouble()));
        _link->stopLock();
    }

    void oneOddLaserShotDoesNotMoveTheLockedObject()
    {
        // Three shots of the lock, 3 s apart: 200 m (level ground), then twice 100 m (ground 50 m higher,
        // or something in the way). The first odd one changes nothing. Two of three are believed.
        _flyAtAHundredMetres();
        _camera->slrE = "07D0";
        QSignalSpy placed(_link, &SkydroidLink::objectPlaced);
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        QVERIFY(_goOnUntil([this]() { return _link->laserValid() && _link->laserRangeM() == 200.0; }, 3000));
        const auto away = [this, &placed]() {
            const QList<QVariant> place = placed.constLast();
            return _vehicle.position.distanceTo(QGeoCoordinate(place.at(0).toDouble(), place.at(1).toDouble()));
        };
        _camera->slrE = "03E8";
        QVERIFY(_goOnUntil([this]() { return _link->laserValid() && _link->laserRangeM() == 100.0 && !_link->laserBusy(); }, 5000));
        placed.clear();
        _goOn(500);
        QVERIFY(placed.count() >= 1);
        QVERIFY2(std::abs(away() - 173.2) < 0.5, qPrintable(QString::number(away())));
        // The next shot says 100 m again.
        QSignalSpy shots(_link, &SkydroidLink::laserBusyChanged);
        QVERIFY(_goOnUntil([&shots]() { return shots.count() >= 2; }, 5000));  // busy, and done
        placed.clear();
        _goOn(500);
        QVERIFY(placed.count() >= 1);
        QVERIFY2(std::abs(away() - 86.6) < 0.5, qPrintable(QString::number(away())));
        QVERIFY(std::abs(placed.constLast().at(3).toDouble() - 50.0) < 0.2);
        _link->stopLock();
    }

    void aNewLockForgetsTheGroundUnderTheLastObject()
    {
        _flyAtAHundredMetres();
        _camera->slrE = "05DC";  // 150 m: ground 25 m higher
        QSignalSpy placed(_link, &SkydroidLink::objectPlaced);
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        QVERIFY(_goOnUntil([this]() { return _link->laserValid(); }, 3000));
        _camera->slrE.reset();  // for the next object the laser gives nothing
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        placed.clear();
        _goOn(300);
        QVERIFY(placed.count() >= 1);
        const QList<QVariant> place = placed.constLast();
        const double away = _vehicle.position.distanceTo(QGeoCoordinate(place.at(0).toDouble(), place.at(1).toDouble()));
        QVERIFY2(std::abs(away - 173.2) < 0.5, qPrintable(QString::number(away)));
        QVERIFY(std::isnan(place.at(3).toDouble()));
        _link->stopLock();
    }

    void theLaserButtonDoesNotTellOfTheGroundUnderTheLockedObject()
    {
        // The operator's own shot can be at anything. (Point mode "picture": the lock fires no shots of its own.)
        _link->setPointMode("picture");
        _flyAtAHundredMetres();
        _camera->slrE = "05DC";
        QSignalSpy placed(_link, &SkydroidLink::objectPlaced);
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        _goOn(700);  // past the lock's first look at the object (from the picture, in this mode)
        _link->fireLaser();
        QVERIFY(_goOnUntil([this]() { return _link->laserValid(); }, 3000));
        QCOMPARE(_link->laserRangeM(), 150.0);
        placed.clear();
        _goOn(500);
        QVERIFY(placed.count() >= 1);
        const QList<QVariant> place = placed.constLast();
        const double away = _vehicle.position.distanceTo(QGeoCoordinate(place.at(0).toDouble(), place.at(1).toDouble()));
        QVERIFY2(std::abs(away - 173.2) < 0.5, qPrintable(QString::number(away)));
        QVERIFY(std::isnan(place.at(3).toDouble()));
        _link->stopLock();
    }

    // --- Follow Me: the drone flies after the locked object ----------------------

    void followStartsOnALockedObjectAndFliesTheDrone()
    {
        _withFollow();
        QVERIFY(!_follow->ready() && !_follow->active());
        QVERIFY2(_follow->notReadyText().contains("Lock the object first"), qPrintable(_follow->notReadyText()));
        QCOMPARE(_follow->fliesOnS(), 3.0);
        _lockAndKnowTheObject();
        QVERIFY(_follow->notReadyText().isEmpty());
        // 100 m up and 60 degrees down: the object is 57.7 m away.
        QVERIFY2(std::abs(_follow->distanceM() - 57.7) < 1.0, qPrintable(QString::number(_follow->distanceM())));
        QCOMPARE(_follow->heightM(), 100.0);
        QVERIFY(!_follow->byLaser());
        // Nothing goes to the drone before the operator starts Follow.
        QVERIFY(_drone->sent.isEmpty());
        QCOMPARE(_drone->guidedAsked, 0);

        QSignalSpy changed(_follow, &ObjectFollow::changed);
        _follow->start();
        QVERIFY(_follow->active() && !_follow->waiting() && !_follow->ready());
        QVERIFY(changed.count() >= 1);
        QCOMPARE(_drone->guidedAsked, 1);  // it was in Loiter
        QVERIFY2(_follow->message() == "Following. The object is 58 m away.", qPrintable(_follow->message()));
        _follow->clearMessage();  // a tap on it while the drone follows does not take the state away
        QVERIFY(_follow->message().startsWith("Following."));
        QVERIFY(std::abs(_follow->keptDistanceM() - 57.7) < 1.0);
        _nowGuided();
        _drone->sent.clear();
        _goOn(1000);
        // A speed five times a second, each with the heading to keep. The object stands, so the speed is none.
        QVERIFY2(_drone->sent.size() >= 4 && _drone->sent.size() <= 6, qPrintable(QString::number(_drone->sent.size())));
        for (const FakeFollowDrone::Sent &s : std::as_const(_drone->sent)) {
            QCOMPARE(s.headingDeg, 35.0);
            QCOMPARE(s.northMps, 0.0);
            QCOMPARE(s.eastMps, 0.0);
        }
        QVERIFY(_follow->active());
        QCOMPARE(_drone->guidedAsked, 1);
        _follow->stop();
    }

    void followDoesNotAskForGuidedWhenTheDroneIsInGuided()
    {
        _withFollow();
        _nowGuided();
        _lockAndKnowTheObject();
        _follow->start();
        QVERIFY(_follow->active());
        QCOMPARE(_drone->guidedAsked, 0);
        _follow->stop();
    }

    void followFliesAfterAnObjectThatMoves()
    {
        _withFollow();
        _lockAndKnowTheObject();
        _follow->start();
        _nowGuided();
        _follow->setSpeedLimit(6.0);
        // The camera follows the object to the right: 20 degrees in 4 s are 21 m at this range, 5 m/s to the east.
        QElapsedTimer clock;
        clock.start();
        int n = 0;
        while (clock.elapsed() < 4000) {
            _camera->yaw = -20.0 * clock.elapsed() / 4000.0;  // the C13 counts a turn to the right as minus
            // A live video: no two pictures are the same.
            _link->lockPicture(_picture(0.5 + 0.003 * (n % 2), 0.5), 0.0, 0.0, 1.0, 1.0);
            if ((n++ % 2) == 0) {
                _drone->tell(_telemetry);
            }
            QTest::qWait(90);
        }
        QVERIFY2(_follow->active(), qPrintable(_follow->message()));
        QVERIFY(!_drone->sent.isEmpty());
        const FakeFollowDrone::Sent last = _drone->sent.constLast();
        QVERIFY2(last.eastMps > 2.0 && std::abs(last.northMps) < 2.5,
                 qPrintable(QString("%1 north, %2 east").arg(last.northMps).arg(last.eastMps)));
        double most = 0.0;
        for (const FakeFollowDrone::Sent &s : std::as_const(_drone->sent)) {
            most = std::max(most, std::hypot(s.northMps, s.eastMps));
        }
        QVERIFY2(most <= 6.0 + 1e-9, qPrintable(QString::number(most)));  // the speed limit holds
        QVERIFY(_follow->askedSpeedMps() > 2.0);
        _follow->stop();
    }

    void aPlaceIsAsOldAsItsPicture()
    {
        // A place holds for the moment its picture was taken. An object at 10 m/s whose place is taken as a
        // second newer than it is would be put 10 m behind where it is.
        _withFollow();
        _hoverAt(100.0, -60.0);
        _vehicle.vehicle.remove("altitudeRelative");  // so that the link gives no places of its own here
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        QElapsedTimer clock;
        clock.start();
        int n = 0;
        while (clock.elapsed() < 4500) {
            // The object runs north, 100 m from the drone at first. Its picture is 1 s old: it shows it 10 m back.
            const double northNow = 100.0 + 10.0 * clock.elapsed() / 1000.0;
            const QGeoCoordinate shown = _vehicle.position.atDistanceAndAzimuth(northNow - 10.0, 0.0);
            emit _link->objectPlaced(shown.latitude(), shown.longitude(), 1.5, std::nan(""), 1000);
            _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
            if ((n++ % 2) == 0) {
                _drone->tell(_telemetry);
            }
            QTest::qWait(90);
        }
        const double northNow = 100.0 + 10.0 * clock.elapsed() / 1000.0;
        // Where the object is now (within what it runs between two looks at the clock), not where its picture showed it.
        QVERIFY2(std::abs(_follow->distanceM() - northNow) < 4.0,
                 qPrintable(QString("%1 m shown, the object is %2 m away").arg(_follow->distanceM()).arg(northNow)));
        _link->stopLock();
    }

    void followEndsAndStopsTheDroneWhenTheLockEnds()
    {
        _withFollow();
        _lockAndKnowTheObject();
        _follow->start();
        _nowGuided();
        _goOn(600);
        _drone->sent.clear();
        const qint64 endedAt = _drone->clock.elapsed();
        _link->stopLock();
        // At once, not on the next tick.
        QVERIFY(!_follow->active());
        QCOMPARE(_drone->sent.size(), 1);
        QVERIFY2(_follow->message() == "Follow ended: the lock ended. The drone holds its place in Guided.",
                 qPrintable(_follow->message()));
        // "Stop" goes on for a second and a half, then nothing more is sent.
        QTest::qWait(2400);
        const qsizetype stops = _drone->sent.size();
        QVERIFY2(stops >= 6 && stops <= 10, qPrintable(QString::number(stops)));
        for (const FakeFollowDrone::Sent &s : std::as_const(_drone->sent)) {
            QCOMPARE(s.northMps, 0.0);
            QCOMPARE(s.eastMps, 0.0);
            QCOMPARE(s.headingDeg, 35.0);
        }
        QVERIFY2(_drone->sent.constLast().atMs - endedAt < 1900, qPrintable(QString::number(_drone->sent.constLast().atMs - endedAt)));
        QTest::qWait(600);
        QCOMPARE(_drone->sent.size(), stops);
        // Its message stays until it is tapped away.
        QVERIFY(_follow->message().startsWith("Follow ended"));
        _follow->clearMessage();
        QVERIFY(_follow->message().isEmpty());
    }

    void followEndsWhenTheOperatorMovesTheCamera()
    {
        _withFollow();
        _lockAndKnowTheObject();
        _follow->start();
        _nowGuided();
        _goOn(600);
        _drone->sent.clear();
        _link->setTouchMotion(1.0, 0.0);  // a finger on the video: the lock ends
        QVERIFY(!_link->lockActive());
        QVERIFY(!_follow->active());
        QVERIFY(_follow->message().startsWith("Follow ended: the lock ended."));
        QCOMPARE(_drone->sent.size(), 1);
        QCOMPARE(std::hypot(_drone->sent.constLast().northMps, _drone->sent.constLast().eastMps), 0.0);
        _link->stopTouchMotion();
    }

    void aNewLockEndsFollow()
    {
        _withFollow();
        _lockAndKnowTheObject();
        _follow->start();
        _nowGuided();
        _goOn(600);
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);  // another object
        QVERIFY(!_follow->active());
        QVERIFY(_follow->message().startsWith("Follow ended: the lock ended."));
        // The new object's place is not known from the old one's.
        _goOn(600);
        QVERIFY(_link->lockActive());
        QVERIFY(!_follow->ready());
        QVERIFY2(_follow->notReadyText().contains("not known well enough"), qPrintable(_follow->notReadyText()));
        _link->stopLock();
    }

    void followStopsOnTheOperatorsWordAndTheCameraStaysLocked()
    {
        _withFollow();
        _lockAndKnowTheObject();
        _follow->start();
        _nowGuided();
        _goOn(600);
        _drone->sent.clear();
        _follow->stop();
        QVERIFY(!_follow->active());
        QCOMPARE(_drone->sent.size(), 1);
        QCOMPARE(std::hypot(_drone->sent.constLast().northMps, _drone->sent.constLast().eastMps), 0.0);
        QCOMPARE(_follow->message(), QString("Follow stopped. The drone holds its place in Guided."));
        QVERIFY(_link->lockActive());
        // It can be started again on the same object.
        _goOn(2000);
        QVERIFY2(_follow->ready(), qPrintable(_follow->notReadyText()));
        _follow->start();
        QVERIFY(_follow->active());
        QCOMPARE(_drone->guidedAsked, 1);  // only the first time: it is in Guided now
        _follow->stop();
        _link->stopLock();
    }

    void followDoesNotStartWithoutALockThatTheAppHolds()
    {
        _withFollow();
        _hoverAt(100.0, -60.0);
        _goOnTelemetry(500);
        _follow->start();
        QVERIFY(!_follow->active());
        QVERIFY2(_follow->message().contains("Lock the object first"), qPrintable(_follow->message()));
        // The camera's own tracker tells nothing of where the object is.
        _link->setLockMode("camera");
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive() && !_link->lockByApp());
        _goOnTelemetry(500);
        QVERIFY(!_follow->ready());
        _follow->start();
        QVERIFY(!_follow->active());
        QVERIFY2(_follow->message().contains("Follow needs the app's own lock"), qPrintable(_follow->message()));
        QVERIFY(_follow->notReadyText().contains("Follow needs the app's own lock"));
        QCOMPARE(_drone->guidedAsked, 0);
        QVERIFY(_drone->sent.isEmpty());
        _link->stopLock();
    }

    void followDoesNotStartWhenTheDroneIsNotReadyForIt()
    {
        _withFollow();
        _lockAndKnowTheObject();
        const auto refused = [this](const char *why) {
            _goOn(450);
            const bool notReady = !_follow->ready() && _follow->notReadyText().contains(why);
            _follow->start();
            return notReady && !_follow->active() && _follow->message().contains(why) && _drone->guidedAsked == 0 &&
                   _drone->sent.isEmpty();
        };
        _telemetry.heightM = 9.0;
        QVERIFY2(refused("the drone is 9 m up. It needs 10 m."), qPrintable(_follow->message()));
        _telemetry.heightM = 100.0;
        _telemetry.flying = false;
        QVERIFY2(refused("the drone is not flying"), qPrintable(_follow->message()));
        _telemetry.flying = true;
        _telemetry.armed = false;
        QVERIFY2(refused("the drone is not flying"), qPrintable(_follow->message()));
        _telemetry.armed = true;
        _telemetry.gps3d = false;
        QVERIFY2(refused("no 3D GPS position"), qPrintable(_follow->message()));
        _telemetry.gps3d = true;
        _telemetry.mode = skydroid::follow::Mode::Other;
        _drone->mode = "Auto";
        QVERIFY2(refused("The drone is in Auto."), qPrintable(_follow->message()));
        _telemetry.mode = skydroid::follow::Mode::Holding;
        _drone->mode = "Loiter";
        _drone->whyNotText = "Follow works with an ArduPilot multicopter only.";
        QVERIFY2(refused("ArduPilot multicopter only"), qPrintable(_follow->message()));
        _drone->whyNotText.clear();
        // No telemetry for a while: where the drone is, is not known.
        _tellTelemetry = false;
        _goOn(2300);
        QVERIFY2(refused("the drone sends no position"), qPrintable(_follow->message()));
        _tellTelemetry = true;
        // All is well again: it starts.
        _goOn(450);
        QVERIFY2(_follow->ready(), qPrintable(_follow->notReadyText()));
        _follow->start();
        QVERIFY(_follow->active());
        QCOMPARE(_drone->guidedAsked, 1);
        _follow->stop();
    }

    void followDoesNotStartNearerThanTwentyMetres()
    {
        _withFollow();
        // 100 m up and 80 degrees down: the object is 17.6 m away over the ground.
        _hoverAt(100.0, -80.0);
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        QVERIFY(_goOnUntil([this]() { return _follow->notReadyText().contains("m away"); }, 8000));
        _follow->start();
        QVERIFY(!_follow->active());
        QVERIFY2(_follow->message() == "Follow cannot start: the object is 18 m away. It needs 20 m or more.",
                 qPrintable(_follow->message()));
        QVERIFY(_drone->sent.isEmpty());
        _link->stopLock();
    }

    void followDoesNotStartBeforeTheObjectsPlaceIsKnown()
    {
        _withFollow();
        _hoverAt(100.0, -60.0);
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        _goOn(1000);
        _follow->start();
        QVERIFY(!_follow->active());
        QVERIFY2(_follow->message().contains("the object's position is not known well enough"), qPrintable(_follow->message()));
        QVERIFY(_follow->distanceM() < 0.0);  // and no distance is shown yet
        _link->stopLock();
    }

    void followDoesNotStartOverGroundThatIsTooHigh()
    {
        _withFollow();
        // 30 m up, 30 degrees down, and the laser says 16 m: the object is 8 m below the drone.
        _hoverAt(30.0, -30.0);
        _camera->slrE = "00A0";
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        QVERIFY(_goOnUntil([this]() { return _follow->byLaser() && _follow->notReadyText().contains("above the object"); }, 8000));
        _follow->start();
        QVERIFY(!_follow->active());
        QVERIFY2(_follow->message() == "Follow cannot start: the drone is only 8 m above the object. It needs 10 m.",
                 qPrintable(_follow->message()));
        _link->stopLock();
    }

    void followSaysWhenTheLaserGaveTheHeightAboveTheObject()
    {
        _withFollow();
        _hoverAt(100.0, -60.0);
        _camera->slrE = "0483";  // 115.5 m: level ground
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        _goOn(250);
        QVERIFY(!_follow->byLaser());
        QVERIFY(_goOnUntil([this]() { return _follow->byLaser(); }, 3000));
        _link->stopLock();
        QVERIFY(!_follow->byLaser());
    }

    void followEndsWhenThePilotChangesTheFlightMode()
    {
        _withFollow();
        _lockAndKnowTheObject();
        _follow->start();
        _nowGuided();
        _goOn(600);
        QVERIFY(_follow->active());
        _telemetry.mode = skydroid::follow::Mode::Holding;
        _drone->mode = "Loiter";
        _drone->tell(_telemetry);
        _drone->sent.clear();
        _goOn(700);
        QVERIFY(!_follow->active());
        QVERIFY2(_follow->message() == "Follow ended: the flight mode was changed to Loiter.", qPrintable(_follow->message()));
        // The drone is the pilot's: nothing at all is sent to it, not even a stop.
        QVERIFY2(_drone->sent.isEmpty(), qPrintable(QString::number(_drone->sent.size())));
        QVERIFY(_link->lockActive());  // the camera stays on the object
        _link->stopLock();
    }

    void followEndsWithoutAWordWhenAnotherDroneBecomesTheActiveOne()
    {
        _withFollow();
        _lockAndKnowTheObject();
        _follow->start();
        _nowGuided();
        _goOn(600);
        _drone->sent.clear();
        emit _drone->otherDrone();
        QVERIFY(!_follow->active());
        QVERIFY2(_follow->message() == "Follow ended: another drone is the active one now.", qPrintable(_follow->message()));
        // Where the drone before was says nothing of this one: Follow cannot start until it has said where it is.
        QVERIFY(!_follow->ready());
        QVERIFY2(_follow->notReadyText().contains("the drone sends no position"), qPrintable(_follow->notReadyText()));
        _tellTelemetry = false;
        _goOn(700);
        // Nothing is sent to it, not even a stop: it may be doing something else.
        QVERIFY2(_drone->sent.isEmpty(), qPrintable(QString::number(_drone->sent.size())));
        QVERIFY(!_follow->ready());
        _tellTelemetry = true;
        _goOn(700);
        QVERIFY2(_follow->ready(), qPrintable(_follow->notReadyText()));
        _link->stopLock();
    }

    void followEndsWhenTheDroneIsTooLow()
    {
        _withFollow();
        _lockAndKnowTheObject();
        _follow->start();
        _nowGuided();
        _goOn(600);
        // The drone sinks to 7 m.
        _telemetry.heightM = 7.0;
        _goOn(700);
        QVERIFY(!_follow->active());
        QVERIFY2(_follow->message() == "Follow ended: the drone is lower than 8 m. The drone holds its place in Guided.",
                 qPrintable(_follow->message()));
        QCOMPARE(std::hypot(_drone->sent.constLast().northMps, _drone->sent.constLast().eastMps), 0.0);
        _link->stopLock();
    }

    void followEndsWhenTheGroundUnderTheObjectIsTooNear()
    {
        _withFollow();
        _lockAndKnowTheObject();
        _follow->start();
        _nowGuided();
        _goOn(600);
        QVERIFY(_follow->active());
        // The laser's next shot finds the object 8 m away along the look, 60 degrees down: 6.9 m below
        // the drone, which is 100 m above its take-off point. The ground there has come up.
        _camera->slrE = "0050";
        QVERIFY(_goOnUntil([this]() { return !_follow->active(); }, 5000));
        QVERIFY2(_follow->message() == "Follow ended: the drone is less than 8 m above the object. The drone holds its place in Guided.",
                 qPrintable(_follow->message()));
        _link->stopLock();
    }

    void followGivesUpWhenTheDroneDoesNotChangeToGuided()
    {
        _withFollow();
        _lockAndKnowTheObject();
        _follow->start();
        QCOMPARE(_drone->guidedAsked, 1);
        _goOn(3000);
        QVERIFY(_follow->active());  // it has 4 s
        _goOn(1600);
        QVERIFY(!_follow->active());
        QVERIFY2(_follow->message() == "Follow ended: the drone did not change to Guided. It is in Loiter.",
                 qPrintable(_follow->message()));
        _link->stopLock();
    }

    void followWaitsWhileTheObjectIsNotSeenAndEndsWithTheLock()
    {
        _withFollow();
        _lockAndKnowTheObject();
        _follow->start();
        _nowGuided();
        _goOn(600);
        // The object is hidden.
        _drone->sent.clear();
        _goOn(900, 0.5, 0.5, false);
        QVERIFY(_follow->active() && _follow->waiting());
        QVERIFY2(_follow->message() == "Follow waits: the object is not seen. The drone stands.", qPrintable(_follow->message()));
        QVERIFY(_drone->sent.size() >= 3);
        for (const FakeFollowDrone::Sent &s : std::as_const(_drone->sent)) {
            QCOMPARE(std::hypot(s.northMps, s.eastMps), 0.0);
        }
        // It comes back: Follow goes on.
        _goOn(700);
        QVERIFY(_follow->active() && !_follow->waiting());
        QVERIFY(_follow->message().startsWith("Following."));
        // Hidden for good: the lock ends after 2 s, and Follow with it.
        _goOn(2600, 0.5, 0.5, false);
        QVERIFY(!_link->lockActive());
        QVERIFY(!_follow->active());
        QVERIFY(_follow->message().startsWith("Follow ended: the lock ended."));
    }

    void followEndsWhenTheDroneGoesSilent()
    {
        _withFollow();
        _lockAndKnowTheObject();
        _follow->start();
        _nowGuided();
        _goOn(600);
        _tellTelemetry = false;
        _goOn(1500);
        // After a second without telemetry the drone is told to stand.
        QVERIFY(_follow->active() && _follow->waiting());
        QVERIFY2(_follow->message() == "Follow waits: the drone's data is late. The drone stands.", qPrintable(_follow->message()));
        _goOn(1000);
        QVERIFY(!_follow->active());
        QVERIFY2(_follow->message() == "Follow ended: the drone sends no data. It stops by itself.", qPrintable(_follow->message()));
        _link->stopLock();
    }

    void followSettingsAreKeptAndStayInTheirRange()
    {
        _withFollow();
        QCOMPARE(_follow->speedLimit(), 8.0);
        QCOMPARE(_follow->rangeLimit(), 500.0);
        QCOMPARE(_follow->minDistanceM(), 15.0);
        QCOMPARE(_follow->startDistanceM(), 20.0);
        QCOMPARE(_follow->minHeightM(), 10.0);
        QSignalSpy changed(_follow, &ObjectFollow::settingsChanged);
        _follow->setSpeedLimit(12.0);
        _follow->setRangeLimit(200.0);
        QCOMPARE(changed.count(), 2);
        _follow->setSpeedLimit(12.0);  // no change, no signal
        QCOMPARE(changed.count(), 2);
        _follow->setSpeedLimit(99.0);
        QCOMPARE(_follow->speedLimit(), 15.0);
        _follow->setRangeLimit(1.0);
        QCOMPARE(_follow->rangeLimit(), 50.0);
        _follow->setSpeedLimit(12.0);
        _follow->setRangeLimit(200.0);
        // Another run of the app.
        delete _follow;
        _drone = new FakeFollowDrone;
        _follow = new ObjectFollow(_link, _drone);
        QCOMPARE(_follow->speedLimit(), 12.0);
        QCOMPARE(_follow->rangeLimit(), 200.0);
        // A settings file with nonsense in it gives the usual values.
        delete _follow;
        QSettings().setValue("VamaFollow/speedLimit", "fast");
        QSettings().setValue("VamaFollow/rangeLimit", -3);
        _drone = new FakeFollowDrone;
        _follow = new ObjectFollow(_link, _drone);
        QCOMPARE(_follow->speedLimit(), 8.0);  // no number at all: the usual one
        QCOMPARE(_follow->rangeLimit(), 50.0);  // a number out of the range: the nearest in it
    }

    void followNeverLeavesTheDroneFlyingWhenItGoesAway()
    {
        _withFollow();
        _lockAndKnowTheObject();
        _follow->start();
        _nowGuided();
        _goOn(600);
        // The app closes while Follow runs: a stop goes out. (ArduCopter would stop by itself as well.)
        QPointer<FakeFollowDrone> drone = _drone;
        QList<FakeFollowDrone::Sent> *sent = &_drone->sent;
        const qsizetype before = sent->size();
        _drone->setParent(nullptr);  // so that the test still has it afterwards
        delete _follow;
        _follow = nullptr;
        QVERIFY(drone);
        QCOMPARE(sent->size(), before + 1);
        QCOMPARE(std::hypot(sent->constLast().northMps, sent->constLast().eastMps), 0.0);
        delete _drone;
        _drone = nullptr;
        _link->stopLock();
    }

    // --- A lock on a marked object: the link's part ---------------------------------

    void picturesAreKeptOnlyWhenAskedFor()
    {
        QVERIFY(!_link->keepsPictures() && !_link->lockWantsPictures());
        QCOMPARE(_link->newestKeptPictureMs(), qint64(-1));
        QSignalSpy changed(_link, &SkydroidLink::lockChanged);
        _link->keepPictures(true);
        QVERIFY(_link->keepsPictures() && _link->lockWantsPictures());  // QML is to send pictures now
        QCOMPARE(changed.count(), 1);
        _link->keepPictures(true);
        QCOMPARE(changed.count(), 1);
        // Kept pictures lock nothing.
        _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(!_link->lockActive() && !_link->lockBusy() && !_link->lockBoxValid());
        const qint64 kept = _link->newestKeptPictureMs();
        QVERIFY(kept >= 0);
        _link->keepPictures(false);
        QVERIFY(!_link->keepsPictures() && !_link->lockWantsPictures());
        QCOMPARE(_link->newestKeptPictureMs(), qint64(-1));
        QCOMPARE(changed.count(), 2);
        // What was kept is gone: keeping again starts with nothing.
        _link->keepPictures(true);
        _link->lockAtFound(0.5, 0.5, 0.06, 0.2, kept);
        QVERIFY(_link->lockBusy() && !_link->lockActive());
        _link->stopLock();
    }

    void aBlackPictureIsNotKept()
    {
        // No picture of the scene (the video restarts): nothing a lock could start on.
        _link->keepPictures(true);
        QImage black(384, 216, QImage::Format_RGB888);
        black.fill(Qt::black);
        for (int i = 0; i < 3; ++i) {
            _link->lockPicture(black, 0.0, 0.0, 1.0, 1.0);
            QTest::qWait(30);
        }
        _link->lockAtFound(0.5, 0.5, 0.06, 0.2, _link->pictureClockMs() - 30);
        QVERIFY(_link->lockBusy() && !_link->lockActive());  // it waits for a picture, as for a drawn box
        _link->stopLock();
    }

    void theKeptPictureIsTheLinksOwnCopy()
    {
        // The screen may use the bytes of a picture again once it has handed it over.
        _link->keepPictures(true);
        const QImage first = _picture(0.3, 0.5);
        std::vector<uchar> bytes(first.constBits(), first.constBits() + first.sizeInBytes());
        const QImage handed(bytes.data(), first.width(), first.height(), static_cast<int>(first.bytesPerLine()),
                            QImage::Format_Grayscale8);
        _link->lockPicture(handed, 0.0, 0.0, 1.0, 1.0);
        const qint64 found = _link->newestKeptPictureMs();
        std::fill(bytes.begin(), bytes.end(), static_cast<uchar>(120));  // the bytes are used for something else
        QTest::qWait(30);
        _link->lockPicture(_picture(0.3, 0.5), 0.0, 0.0, 1.0, 1.0);
        _link->lockAtFound(0.3, 0.5, 0.06, 0.2, found);
        QVERIFY2(_link->lockActive() && _link->lockSeen(), qPrintable(_link->lockMessage()));
        QVERIFY(std::abs(_link->lockBoxU() - 0.3) < 0.015);
        _link->stopLock();
    }

    void aLockOnAFoundObjectStartsOnThePictureItWasFoundIn()
    {
        _link->keepPictures(true);
        // The object walks from 0.30 to 0.62 of the picture's width while the pictures come.
        qint64 foundAtMs = 0;
        for (int i = 0; i <= 16; ++i) {
            _link->lockPicture(_picture(0.30 + 0.02 * i, 0.5), 0.0, 0.0, 1.0, 1.0);
            if (i == 0) {
                foundAtMs = _link->newestKeptPictureMs();
                QVERIFY(foundAtMs >= 0 && foundAtMs <= _link->pictureClockMs());
            }
            QTest::qWait(30);
        }
        QVERIFY(!_link->lockActive() && !_link->lockBusy());
        QSignalSpy boxes(_link, &SkydroidLink::lockBoxChanged);
        // The object was found in the first picture, at 0.30. The operator taps its mark now.
        _link->lockAtFound(0.30, 0.5, 0.06, 0.2, foundAtMs);
        QVERIFY(_link->lockActive() && !_link->lockBusy() && _link->lockByApp());
        QVERIFY2(_link->lockSeen(), qPrintable(_link->lockNumbers()));
        QVERIFY2(std::abs(_link->lockBoxU() - 0.62) < 0.015 && std::abs(_link->lockBoxV() - 0.5) < 0.02,
                 qPrintable(QString("box at %1, %2").arg(_link->lockBoxU()).arg(_link->lockBoxV())));
        // The box on the screen goes straight to where the object is now, not through the old pictures.
        QVERIFY2(boxes.count() <= 2, qPrintable(QString::number(boxes.count())));
        // The camera is turned for the newest picture: the object is right of the cross.
        QTRY_VERIFY_WITH_TIMEOUT(_lastSpeed("GSY") > 0.0, 1000);
        // And the lock goes on with the pictures that come.
        _link->lockPicture(_picture(0.64, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockSeen() && std::abs(_link->lockBoxU() - 0.64) < 0.015);
        _link->stopLock();
        QVERIFY(_link->lockWantsPictures());  // still kept, for the next mark
    }

    void theSameBoxLockedAsADrawnOneMissesAnObjectThatWalkedOn()
    {
        // What lockAtFound is for. The box of a moment ago, on the newest picture, holds ground.
        _link->lockAtBox(0.30, 0.5, 0.06, 0.2);
        _link->lockPicture(_picture(0.62, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(!_link->lockActive() || std::abs(_link->lockBoxU() - 0.62) > 0.1);
    }

    void aFoundObjectWithNoPictureOfItsMomentIsLockedAsADrawnBox()
    {
        // Nothing kept: the lock waits for the next picture, as for a drawn box.
        _link->lockAtFound(0.4, 0.5, 0.06, 0.2, _link->pictureClockMs());
        QVERIFY(_link->lockBusy() && !_link->lockActive() && _link->lockByApp());
        _link->lockPicture(_picture(0.4, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive() && std::abs(_link->lockBoxU() - 0.4) < 0.01);
        _link->stopLock();
        // Pictures are kept, but none of that moment: it is 0.3 s before the first of them.
        _link->keepPictures(true);
        const qint64 before = _link->pictureClockMs();
        QTest::qWait(300);
        for (int i = 0; i < 4; ++i) {
            _link->lockPicture(_picture(0.4, 0.5), 0.0, 0.0, 1.0, 1.0);
            QTest::qWait(30);
        }
        _link->lockAtFound(0.4, 0.5, 0.06, 0.2, before);
        QVERIFY(_link->lockBusy() && !_link->lockActive());
        _link->stopLock();
        // A box that is no box is not locked at all.
        _link->lockAtFound(std::nan(""), 0.5, 0.06, 0.2, _link->pictureClockMs());
        QVERIFY(!_link->lockBusy() && !_link->lockActive());
    }

    void aFoundObjectIsLockedByTheCamerasOwnTrackerInThatMode()
    {
        _link->setLockMode("camera");
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->keepPictures(true);
        for (int i = 0; i < 3; ++i) {
            _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
            QTest::qWait(30);
        }
        _link->lockAtFound(0.5, 0.5, 0.06, 0.2, _link->pictureClockMs());
        QVERIFY(!_link->lockByApp());
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countStartingWith("#TPUG8wGOT") >= 1, 2000);
        _link->stopLock();
        // An object off the cross: the camera turns to it first. The app's own tracker stays out of it.
        QTest::qWait(100);
        _link->lockPicture(_picture(0.85, 0.5), 0.0, 0.0, 1.0, 1.0);
        _link->lockAtFound(0.85, 0.5, 0.06, 0.2, _link->pictureClockMs());
        QVERIFY2(_link->lockBusy() && !_link->lockActive() && !_link->lockByApp(), qPrintable(_link->lockMessage()));
        _link->stopLock();
    }

    void aZoomStepForgetsTheKeptPictures()
    {
        _link->keepPictures(true);
        QSignalSpy asked(_link, &SkydroidLink::zoomAsked);
        for (int i = 0; i < 3; ++i) {
            _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
            QTest::qWait(30);
        }
        const qint64 found = _link->pictureClockMs();
        _link->zoom(1);
        QCOMPARE(asked.count(), 1);
        // The pictures from before the zoom show the object at another size: none is used.
        _link->lockAtFound(0.5, 0.5, 0.06, 0.2, found);
        QVERIFY(_link->lockBusy() && !_link->lockActive());
        _link->stopLock();
        _link->zoomHome();
        QCOMPARE(asked.count(), 2);
        _link->zoom(0);  // no step
        QCOMPARE(asked.count(), 2);
    }

    void keptPicturesOfAnotherShapeAreDropped()
    {
        _link->keepPictures(true);
        _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
        const qint64 found = _link->pictureClockMs();
        QTest::qWait(200);
        // The thermal picture comes on: the image is as large, the picture inside it narrower.
        _link->lockPicture(_picture(0.5, 0.5), 0.1, 0.0, 0.8, 1.0);
        _link->lockAtFound(0.5, 0.5, 0.06, 0.2, found);
        QVERIFY(_link->lockBusy() && !_link->lockActive());
        _link->stopLock();
    }

    void noMoreThanFortyPicturesAreKept()
    {
        _link->keepPictures(true);
        _link->lockPicture(_picture(0.3, 0.5), 0.0, 0.0, 1.0, 1.0);
        const qint64 first = _link->pictureClockMs();
        for (int i = 0; i < 45; ++i) {
            QTest::qWait(30);
            _link->lockPicture(_picture(0.3, 0.5), 0.0, 0.0, 1.0, 1.0);
        }
        // 45 more pictures in less than the 2.5 s they are kept for: the count pushed the first ones out.
        QVERIFY2(_link->pictureClockMs() - first < 2400, qPrintable(QString::number(_link->pictureClockMs() - first)));
        _link->lockAtFound(0.3, 0.5, 0.06, 0.2, first);
        QVERIFY(_link->lockBusy() && !_link->lockActive());
        _link->stopLock();
    }

    void picturesOlderThanTwoAndAHalfSecondsAreNotKept()
    {
        _link->keepPictures(true);
        _link->lockPicture(_picture(0.3, 0.5), 0.0, 0.0, 1.0, 1.0);
        const qint64 old = _link->pictureClockMs();
        QTest::qWait(2400);
        // 2.4 s on, the first picture is still there.
        _link->lockAtFound(0.3, 0.5, 0.06, 0.2, old);
        QVERIFY(_link->lockActive());
        _link->stopLock();
        QTest::qWait(250);
        _link->lockPicture(_picture(0.3, 0.5), 0.0, 0.0, 1.0, 1.0);  // 2.65 s on: this one pushes it out
        _link->lockAtFound(0.3, 0.5, 0.06, 0.2, old);
        QVERIFY(_link->lockBusy() && !_link->lockActive());
        _link->stopLock();
    }

    void aFoundBoxOverPlainGroundEndsTheLockAndSaysWhy()
    {
        _link->keepPictures(true);
        QImage picture = _picture(0.5, 0.5, false);
        for (int y = 60; y < 156; ++y) {
            uchar *row = picture.scanLine(y);
            for (int x = 140; x < 244; ++x) {
                row[x] = 120;
            }
        }
        for (int i = 0; i < 3; ++i) {
            _link->lockPicture(picture, 0.0, 0.0, 1.0, 1.0);
            QTest::qWait(30);
        }
        _link->lockAtFound(0.5, 0.5, 0.1, 0.2, _link->pictureClockMs() - 60);
        QVERIFY(!_link->lockActive() && !_link->lockBusy());
        QVERIFY(_link->lockMessage().contains("plain area"));
    }

    void aSecondTapOnAMarkWhileTheFirstLockStartsIsNotTaken()
    {
        // Pictures are kept, and a lock waits for its first picture. A lock on a found object
        // asked for now must not start the tracker on a kept picture with the waiting lock's box.
        _link->keepPictures(true);
        for (int i = 0; i < 3; ++i) {
            _link->lockPicture(_picture(0.7, 0.3), 0.0, 0.0, 1.0, 1.0);
            QTest::qWait(30);
        }
        const qint64 found = _link->newestKeptPictureMs();
        _link->lockAtBox(0.3, 0.5, 0.06, 0.2);  // waits for the next picture
        QVERIFY(_link->lockBusy() && !_link->lockActive());
        _link->lockAtFound(0.7, 0.3, 0.06, 0.2, found);
        QVERIFY2(_link->lockBusy() && !_link->lockActive(), "the kept pictures started the lock that was waiting");
        _link->stopLock();
    }

    void aLockOnAFoundObjectEndsWhenNoPictureComesAfterIt()
    {
        // The video dies at the moment of the tap: the lock started on the kept pictures, and no
        // picture follows. It must end as any lock does, three seconds after the last picture.
        _link->keepPictures(true);
        for (int i = 0; i < 4; ++i) {
            _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
            QTest::qWait(30);
        }
        _link->lockAtFound(0.5, 0.5, 0.06, 0.2, _link->pictureClockMs() - 60);
        QVERIFY(_link->lockActive());
        QTest::qWait(2500);
        QVERIFY(_link->lockActive());
        QTRY_VERIFY_WITH_TIMEOUT(!_link->lockActive(), 1500);
        QVERIFY2(_link->lockMessage().contains("the video stopped"), qPrintable(_link->lockMessage()));
    }

    void aLockOnAFoundObjectReplacesTheLockThatIsOn()
    {
        _link->keepPictures(true);
        _link->lockAtBox(0.3, 0.5, 0.06, 0.2);
        _link->lockPicture(_picture(0.3, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive() && std::abs(_link->lockBoxU() - 0.3) < 0.01);
        QTest::qWait(30);
        // The same pictures are kept while a lock is on. Another object is found in the next one.
        _link->lockPicture(_picture(0.7, 0.3), 0.0, 0.0, 1.0, 1.0);
        const qint64 found = _link->newestKeptPictureMs();
        QTest::qWait(30);
        _link->lockPicture(_picture(0.72, 0.3), 0.0, 0.0, 1.0, 1.0);
        _link->lockAtFound(0.7, 0.3, 0.06, 0.2, found);
        QVERIFY(_link->lockActive() && _link->lockSeen());
        QVERIFY(std::abs(_link->lockBoxU() - 0.72) < 0.015 && std::abs(_link->lockBoxV() - 0.3) < 0.02);
        _link->stopLock();
    }

    // --- The marks on people and vehicles -------------------------------------------

    void marksRunWhileTheLockButtonWaitsAndTheSwitchIsOn()
    {
        _withMarks();
        QVERIFY(_marks->enabled() && _marks->usable());
        QVERIFY(!_marks->running() && !_marks->wantsPicture() && _marks->state().isEmpty());
        QVERIFY(!_link->keepsPictures());
        QSignalSpy running(_marks, &ObjectMarks::runningChanged);
        _marks->setLooking(true);
        QVERIFY(_marks->running() && _marks->wantsPicture());
        // The link keeps the pictures that a lock on a mark starts with.
        QVERIFY(_link->keepsPictures() && _link->lockWantsPictures());
        QCOMPARE(_marks->state(), QString("Looking for people and vehicles..."));
        QCOMPARE(running.count(), 1);
        _marks->setEnabled(false);
        QVERIFY(!_marks->running() && !_marks->wantsPicture() && !_link->keepsPictures());
        QVERIFY(_marks->state().isEmpty());
        _marks->setEnabled(true);
        QVERIFY(_marks->running());
        _marks->setLooking(false);
        QVERIFY(!_marks->running() && !_link->keepsPictures());
        QCOMPARE(running.count(), 4);
        // A picture that nobody asked for is not looked at, and the network is not even loaded.
        _marks->picture(_markPicture, 0.0, 0.0, 1.0, 1.0);
        QTest::qWait(60);
        QCOMPARE(_asks(), 0);
        QCOMPARE(_seen->loads, 0);
    }

    void theMarksSwitchIsKeptForTheNextStart()
    {
        _withMarks();
        QSignalSpy changed(_marks, &ObjectMarks::enabledChanged);
        _marks->setEnabled(false);
        _marks->setEnabled(false);
        QCOMPARE(changed.count(), 1);
        delete _marks;
        _withMarks();
        QVERIFY(!_marks->enabled());
        _marks->setLooking(true);
        QVERIFY(!_marks->running());  // off stays off
        _marks->setEnabled(true);
        QVERIFY(_marks->running());
        delete _marks;
        _withMarks();
        QVERIFY(_marks->enabled());
    }

    void whatTheFinderFindsIsMarked()
    {
        using skydroid::find::Kind;
        _withMarks();
        _see({_thing(Kind::Person, 640.0, 360.0, 32.0, 72.0, 0.9), _thing(Kind::Vehicle, 200.0, 600.0, 128.0, 72.0, 0.7)});
        _marks->setLooking(true);
        QSignalSpy changed(_marks, &ObjectMarks::marksChanged);
        QVERIFY(_look());
        QCOMPARE(_marks->count(), 2);
        QVERIFY(changed.count() >= 1);
        QCOMPARE(_seen->loads, 1);
        const QVariantList marks = _marks->marks();
        QCOMPARE(marks.size(), 2);
        // The surest first, in parts of the picture.
        const QVariantMap person = marks.at(0).toMap();
        QCOMPARE(person.value("kind").toString(), QString("person"));
        QVERIFY(_near(person.value("u").toDouble(), 0.5) && _near(person.value("v").toDouble(), 0.5));
        QVERIFY(_near(person.value("w").toDouble(), 32.0 / 1280.0) && _near(person.value("h").toDouble(), 0.1));
        const QVariantMap vehicle = marks.at(1).toMap();
        QCOMPARE(vehicle.value("kind").toString(), QString("vehicle"));
        QVERIFY(_near(vehicle.value("u").toDouble(), 200.0 / 1280.0) && _near(vehicle.value("v").toDouble(), 600.0 / 720.0));
        QVERIFY(_near(vehicle.value("w").toDouble(), 0.1) && _near(vehicle.value("h").toDouble(), 0.1));
        QVERIFY2(_marks->state().startsWith("Marked: 1 person, 1 vehicle. A look takes "), qPrintable(_marks->state()));
    }

    void theStateLineCountsWhatIsMarked()
    {
        using skydroid::find::Kind;
        _withMarks();
        _seen->sleepMs = 2;
        _marks->setLooking(true);
        QVERIFY(_look());
        QVERIFY2(_marks->state().startsWith("No person or vehicle found. A look takes "), qPrintable(_marks->state()));
        _see({_thing(Kind::Vehicle, 200.0, 600.0, 128.0, 72.0)});
        QVERIFY(_lookAgain());
        QVERIFY2(_marks->state().startsWith("Marked: 1 vehicle. "), qPrintable(_marks->state()));
        _see({_thing(Kind::Person, 200.0, 200.0, 32.0, 72.0), _thing(Kind::Person, 600.0, 200.0, 32.0, 72.0),
              _thing(Kind::Vehicle, 200.0, 600.0, 128.0, 72.0), _thing(Kind::Vehicle, 600.0, 600.0, 128.0, 72.0),
              _thing(Kind::Vehicle, 1000.0, 600.0, 128.0, 72.0)});
        QVERIFY(_lookAgain());
        QVERIFY2(_marks->state().startsWith("Marked: 2 people, 3 vehicles. "), qPrintable(_marks->state()));
        QCOMPARE(_marks->count(), 5);
        _see({_thing(Kind::Person, 200.0, 200.0, 32.0, 72.0)});
        QVERIFY(_lookAgain());
        QVERIFY(_lookAgain());  // the others were held for one look
        QVERIFY2(_marks->state().startsWith("Marked: 1 person. "), qPrintable(_marks->state()));
        QVERIFY(_marks->state().endsWith(" s."));
    }

    void aPictureGetsTwoLooksTheWholeOfItAndItsMiddle()
    {
        _withMarks();
        _seen->sleepMs = 2;
        _marks->setLooking(true);
        QVERIFY(_look());
        QCOMPARE(_asks(), 2);
        QCOMPARE(_seen->sizes.at(0), QSize(1280, 720));
        // All of the picture, in 640 x 384 points.
        QVERIFY(_isArea(_ask(0).area, 0, 0, 1280, 720) && _ask(0).inputW == 640 && _ask(0).inputH == 384 && _ask(0).keepShape);
        // And its middle 640 x 384 points at their own size: how fast the device is, is not known yet.
        QVERIFY(_isArea(_ask(1).area, 320, 168, 640, 384) && _ask(1).inputW == 640 && _ask(1).inputH == 384);
        QVERIFY(_near(_marks->closeLook().x(), 0.25) && _near(_marks->closeLook().width(), 0.5));
        QVERIFY(_near(_marks->closeLook().y(), 168.0 / 720.0) && _near(_marks->closeLook().height(), 384.0 / 720.0));
        // This finder answered in a few milliseconds: a fast device. From now on all of the
        // picture is looked at at its own size too, and no part is told to the screen as the close one.
        QVERIFY(_lookAgain());
        QCOMPARE(_asks(), 4);
        QVERIFY(_isArea(_ask(2).area, 0, 0, 1280, 720) && _ask(2).inputW == 640);
        QVERIFY(_isArea(_ask(3).area, 0, 0, 1280, 720) && _ask(3).inputW == 1280 && _ask(3).inputH == 736);
        QVERIFY(_marks->closeLook().isEmpty());
    }

    void aSlowDeviceKeepsToTheMiddle()
    {
        _withMarks();
        _seen->sleepMs = 300;  // 0.3 s for a look of 640 x 384 points
        _marks->setLooking(true);
        QVERIFY(_look());
        QVERIFY(_lookAgain());
        QCOMPARE(_asks(), 4);
        QVERIFY(_isArea(_ask(3).area, 320, 168, 640, 384) && _ask(3).inputW == 640 && _ask(3).inputH == 384);
        QVERIFY(_near(_marks->closeLook().width(), 0.5));
        QVERIFY(_marks->lookMs() >= 590.0);  // one look after the other in this test
        QVERIFY2(_marks->state().contains("A look takes 0.6 s.") || _marks->state().contains("A look takes 0.7 s."),
                 qPrintable(_marks->state()));
    }

    void theMiddleStaysTheSizeTheDeviceHasTimeFor()
    {
        _withMarks();
        _seen->sleepMs = 150;  // for a look of 640 x 384 points; a larger look takes longer in proportion
        _seen->sleepBySize = true;
        _marks->setLooking(true);
        QVERIFY(_look());
        // 0.15 s for 640 x 384 points: the middle 960 x 544 takes 0.32 s, all of the picture would take 0.57 s.
        for (int i = 1; i <= 5; ++i) {
            QVERIFY(_lookAgain());
            const skydroid::find::Ask middle = _ask(2 * i + 1);
            QVERIFY2(_isArea(middle.area, 160, 88, 960, 544),
                     qPrintable(QString("look %1: a middle of %2 x %3").arg(i).arg(middle.area.width).arg(middle.area.height)));
        }
        QVERIFY(_near(_marks->closeLook().width(), 0.75) && _near(_marks->closeLook().x(), 0.125));
    }

    void atThePicturesOwnSideTheCloseLooksBoxIsTheOneKept()
    {
        using skydroid::find::Kind;
        _withMarks();
        _seen->sleepMs = 2;
        _seen->growWhenSmaller = 6.0;  // a box found in the picture made smaller is 6 points too wide
        // The thermal picture starts 190 points into its image. A car stands at its left side.
        _see({_thing(Kind::Vehicle, 190.0 + 60.0, 360.0, 120.0, 70.0)});
        _marks->setLooking(true);
        const QRectF thermal(190.0 / 1280.0, 0.0, 900.0 / 1280.0, 1.0);
        QVERIFY(_look(thermal));
        QVERIFY(_lookAgain(thermal));  // a fast device: all of the picture at its own size now
        QVERIFY(_isArea(_ask(3).area, 190, 0, 900, 720));
        QCOMPARE(_marks->count(), 1);
        // The picture's own side cuts nothing: the close look's box is the one shown, not the wider one.
        QVERIFY2(_near(_marks->marksNow().at(0).box.w, 120.0 / 900.0),
                 qPrintable(QString::number(_marks->marksNow().at(0).box.w * 900.0)));
    }

    void aThermalPictureIsLookedAtInsideItsBars()
    {
        using skydroid::find::Kind;
        _withMarks();
        // The 5:4 thermal picture in the middle of a 16:9 image: 900 of its 1280 points.
        _see({_thing(Kind::Person, 640.0, 360.0, 36.0, 72.0), _thing(Kind::Person, 100.0, 360.0, 36.0, 72.0)});
        _marks->setLooking(true);
        QVERIFY(_look(QRectF(190.0 / 1280.0, 0.0, 900.0 / 1280.0, 1.0)));
        QCOMPARE(_asks(), 2);
        QVERIFY(_isArea(_ask(0).area, 190, 0, 900, 720) && _ask(0).inputW == 480 && _ask(0).inputH == 384);
        QVERIFY(_isArea(_ask(1).area, 320, 168, 640, 384));
        // The person in the bar beside the picture is not marked, the one in its middle is: at
        // half of the picture, not of the image.
        QCOMPARE(_marks->count(), 1);
        const skydroid::find::Found box = _marks->marksNow().at(0).box;
        QVERIFY(_near(box.cx, 0.5) && _near(box.cy, 0.5) && _near(box.w, 36.0 / 900.0) && _near(box.h, 0.1));
        QVERIFY(_near(_marks->closeLook().x(), 130.0 / 900.0) && _near(_marks->closeLook().width(), 640.0 / 900.0));
    }

    void aTapBesideASmallMarkCountsByThePicturesShape()
    {
        using skydroid::find::Kind;
        _withMarks();
        // A small person in the middle of the thermal picture (5 to 4): 9 x 27 of its 900 x 720 points.
        _see({_thing(Kind::Person, 640.0, 360.0, 9.0, 27.0)});
        _marks->setLooking(true);
        QVERIFY(_look(QRectF(190.0 / 1280.0, 0.0, 900.0 / 1280.0, 1.0)));
        QCOMPARE(_marks->count(), 1);
        const double below = 0.5 + 27.0 / 720.0 / 2.0;  // the mark's lower side, in parts of the picture
        // A finger's width is 0.012 of the picture's width: that is 0.015 of its height here
        // (it would be 0.021 in the camera's 16 to 9 picture).
        QVERIFY(!_marks->lockAt(0.5, below + 0.018));
        QVERIFY(!_link->lockBusy() && !_link->lockActive());
        QVERIFY(_marks->lockAt(0.5, below + 0.013));
        QVERIFY(_link->lockBusy() || _link->lockActive());
        _link->stopLock();
    }

    void anObjectInBothLooksIsOneMark()
    {
        using skydroid::find::Kind;
        _withMarks();
        // Large enough for the whole picture's look, and in the middle: both looks find it.
        _see({_thing(Kind::Vehicle, 640.0, 360.0, 150.0, 80.0)});
        _marks->setLooking(true);
        QVERIFY(_look());
        QCOMPARE(_marks->count(), 1);
        // A car across the middle's left border: the middle sees half of it, the whole picture all.
        _see({_thing(Kind::Vehicle, 320.0, 360.0, 150.0, 80.0)});
        delete _marks;
        _withMarks();
        _see({_thing(Kind::Vehicle, 320.0, 360.0, 150.0, 80.0)});
        _marks->setLooking(true);
        QVERIFY(_look());
        QCOMPARE(_marks->count(), 1);
        QVERIFY(_near(_marks->marksNow().at(0).box.w, 150.0 / 1280.0));
    }

    void aSmallObjectIsMarkedOnlyWhereTheAppLooksClosely()
    {
        using skydroid::find::Kind;
        _withMarks();
        _seen->sleepMs = 300;  // a slow device: only the middle is looked at at its own size
        // Two people 30 points tall: one in the middle of the picture, one near its corner.
        _see({_thing(Kind::Person, 640.0, 360.0, 13.0, 30.0), _thing(Kind::Person, 100.0, 100.0, 13.0, 30.0)});
        _marks->setLooking(true);
        QVERIFY(_look());
        QCOMPARE(_marks->count(), 1);
        QVERIFY(_near(_marks->marksNow().at(0).box.cx, 0.5));
        QVERIFY(_lookAgain());
        QCOMPARE(_marks->count(), 1);
    }

    void onAFastDeviceSmallObjectsAreMarkedEverywhere()
    {
        using skydroid::find::Kind;
        _withMarks();
        _seen->sleepMs = 2;
        _see({_thing(Kind::Person, 640.0, 360.0, 13.0, 30.0), _thing(Kind::Person, 100.0, 100.0, 13.0, 30.0)});
        _marks->setLooking(true);
        QVERIFY(_look());
        QCOMPARE(_marks->count(), 1);  // the first picture: the speed is not known yet
        QVERIFY(_lookAgain());
        QCOMPARE(_marks->count(), 2);
    }

    void aMarkStaysForOneMoreLookWhenItsObjectIsMissed()
    {
        using skydroid::find::Kind;
        _withMarks();
        _seen->sleepMs = 2;
        const skydroid::find::Found person = _thing(Kind::Person, 640.0, 360.0, 32.0, 72.0);
        const skydroid::find::Found car = _thing(Kind::Vehicle, 300.0, 600.0, 120.0, 70.0);
        _see({person, car});
        _marks->setLooking(true);
        QVERIFY(_look());
        QCOMPARE(_marks->count(), 2);
        const qint64 firstSeen = _marks->marksNow().at(0).atMs;
        // The next look misses the person: their mark stays, with the time of the picture they were found in.
        _see({car});
        QVERIFY(_lookAgain());
        QCOMPARE(_marked(Kind::Person), 1);
        QCOMPARE(_marked(Kind::Vehicle), 1);
        for (const ObjectMarks::Mark &mark : _marks->marksNow()) {
            if (mark.box.kind == Kind::Person) {
                QCOMPARE(mark.atMs, firstSeen);
                QCOMPARE(mark.missed, 1);
            } else {
                QVERIFY(mark.atMs > firstSeen);
                QCOMPARE(mark.missed, 0);
            }
        }
        // Missed again: the mark goes.
        QVERIFY(_lookAgain());
        QCOMPARE(_marked(Kind::Person), 0);
        QCOMPARE(_marked(Kind::Vehicle), 1);
        // Found again: marked again.
        _see({person, car});
        QVERIFY(_lookAgain());
        QCOMPARE(_marks->count(), 2);
    }

    void aMarkGoesWithItsObjectAndLeavesNoSecondOneBehind()
    {
        using skydroid::find::Kind;
        _withMarks();
        _seen->sleepMs = 2;
        _see({_thing(Kind::Person, 640.0, 360.0, 32.0, 72.0)});
        _marks->setLooking(true);
        QVERIFY(_look());
        QCOMPARE(_marks->count(), 1);
        // Between two looks the walker goes on by more than their own width: the boxes do not even touch.
        _see({_thing(Kind::Person, 690.0, 362.0, 32.0, 72.0)});
        QVERIFY(_lookAgain());
        QCOMPARE(_marks->count(), 1);
        QVERIFY(_near(_marks->marksNow().at(0).box.cx, 690.0 / 1280.0));
        QCOMPARE(_marks->marksNow().at(0).missed, 0);
        // Someone else, far from the first: the first one's mark is held as missed, not taken over.
        _see({_thing(Kind::Person, 200.0, 362.0, 32.0, 72.0)});
        QVERIFY(_lookAgain());
        QCOMPARE(_marks->count(), 2);
        // A vehicle where the person was is no person.
        delete _marks;
        _withMarks();
        _seen->sleepMs = 2;
        _see({_thing(Kind::Person, 640.0, 360.0, 32.0, 72.0)});
        _marks->setLooking(true);
        QVERIFY(_look());
        _see({_thing(Kind::Vehicle, 650.0, 360.0, 120.0, 72.0)});
        QVERIFY(_lookAgain());
        QCOMPARE(_marked(Kind::Person), 1);
        QCOMPARE(_marked(Kind::Vehicle), 1);
    }

    void whatTakesThePlaceOfAnOldMark()
    {
        using skydroid::find::Kind;
        const skydroid::find::Found old = _thing(Kind::Person, 0.5, 0.5, 0.02, 0.1);
        QVERIFY(ObjectMarks::takesThePlaceOf(_thing(Kind::Person, 0.5, 0.5, 0.02, 0.1), old));
        QVERIFY(ObjectMarks::takesThePlaceOf(_thing(Kind::Person, 0.539, 0.5, 0.02, 0.1), old));   // two widths on
        QVERIFY(!ObjectMarks::takesThePlaceOf(_thing(Kind::Person, 0.541, 0.5, 0.02, 0.1), old));  // a little more
        QVERIFY(ObjectMarks::takesThePlaceOf(_thing(Kind::Person, 0.5, 0.699, 0.02, 0.1), old));   // two heights down
        QVERIFY(!ObjectMarks::takesThePlaceOf(_thing(Kind::Person, 0.5, 0.701, 0.02, 0.1), old));
        QVERIFY(!ObjectMarks::takesThePlaceOf(_thing(Kind::Vehicle, 0.5, 0.5, 0.02, 0.1), old));   // another kind
    }

    void marksGoAwayWhenNoLookComesForTooLong()
    {
        using skydroid::find::Kind;
        _withMarks();
        _see({_thing(Kind::Person, 640.0, 360.0, 32.0, 72.0)});
        _marks->setLooking(true);
        QVERIFY(_look());
        QCOMPARE(_marks->count(), 1);
        QTest::qWait(1900);
        QCOMPARE(_marks->count(), 1);  // 1.9 s old: still shown
        QSignalSpy changed(_marks, &ObjectMarks::marksChanged);
        QTRY_COMPARE_WITH_TIMEOUT(_marks->count(), 0, 1000);  // 2.2 s: gone
        QVERIFY(changed.count() >= 1);
        QVERIFY2(_marks->state().startsWith("No person or vehicle found."), qPrintable(_marks->state()));
    }

    void marksGoAwayWhenTheLockButtonNoLongerWaits()
    {
        using skydroid::find::Kind;
        _withMarks();
        _see({_thing(Kind::Person, 640.0, 360.0, 32.0, 72.0)});
        _marks->setLooking(true);
        QVERIFY(_look());
        QCOMPARE(_marks->count(), 1);
        QSignalSpy changed(_marks, &ObjectMarks::marksChanged);
        _marks->setLooking(false);
        QCOMPARE(_marks->count(), 0);
        QVERIFY(_marks->marks().isEmpty());
        QCOMPARE(changed.count(), 1);
        QVERIFY(_marks->closeLook().isEmpty());
        QVERIFY(!_marks->lockAt(0.5, 0.5));  // and a tap is not theirs any more
    }

    void anAnswerFromBeforeTheButtonWasPressedIsNotShown()
    {
        using skydroid::find::Kind;
        _withMarks(true);  // on its own thread: the look goes on while the operator presses
        _seen->sleepMs = 100;
        _see({_thing(Kind::Person, 640.0, 360.0, 32.0, 72.0)});
        _marks->setLooking(true);
        QSignalSpy done(_marks, &ObjectMarks::lookDone);
        _marks->picture(_markPicture, 0.0, 0.0, 1.0, 1.0);  // a look is under way
        QTest::qWait(10);
        _marks->setLooking(false);  // the operator cancels
        QTest::qWait(10);
        _marks->setLooking(true);   // and presses again
        QVERIFY(!_marks->wantsPicture());  // the old look is not over
        QVERIFY(done.wait(3000));
        QCOMPARE(_marks->count(), 0);  // its picture is from before the button was pressed
        QCOMPARE(_marks->state(), QString("Looking for people and vehicles..."));
        QVERIFY(_lookAgain());
        QCOMPARE(_marks->count(), 1);
    }

    void anAnswerThatComesAfterMarkingStoppedIsDropped()
    {
        using skydroid::find::Kind;
        _withMarks(true);  // on its own thread: the look goes on while the operator presses
        _seen->sleepMs = 100;
        _see({_thing(Kind::Person, 640.0, 360.0, 32.0, 72.0)});
        _marks->setLooking(true);
        QSignalSpy done(_marks, &ObjectMarks::lookDone);
        _marks->picture(_markPicture, 0.0, 0.0, 1.0, 1.0);
        QTest::qWait(10);
        _marks->setLooking(false);  // the operator cancels while the finder looks
        QSignalSpy changed(_marks, &ObjectMarks::marksChanged);
        QVERIFY(done.wait(3000));
        QCOMPARE(_marks->count(), 0);
        QVERIFY(_marks->marks().isEmpty());
        QCOMPARE(changed.count(), 0);
        QVERIFY(!_marks->lockAt(0.5, 0.5));
        QVERIFY(!_link->lockBusy() && !_link->lockActive());
    }

    void aFinderThatWasNeverLoadedHasNothingToGiveBack()
    {
        _withMarks();
        _marks->setIdleRestMs(100);
        _marks->setLooking(true);
        _marks->setLooking(false);  // pressed and cancelled before a picture came
        QTest::qWait(350);
        QCOMPARE(_seen->loads, 0);
        QCOMPARE(_seen->rests, 0);
    }

    void aBlackPictureIsSaidToBeNoVideoAndIsNotLookedAt()
    {
        using skydroid::find::Kind;
        _withMarks();
        _seen->sleepMs = 2;
        _see({_thing(Kind::Person, 640.0, 360.0, 32.0, 72.0)});
        _marks->setLooking(true);
        QVERIFY(_look());
        QCOMPARE(_marks->count(), 1);
        const int asked = _asks();
        // The video goes black (it restarts, or this device cannot copy its video picture).
        QImage black(1280, 720, QImage::Format_ARGB32_Premultiplied);
        black.fill(Qt::black);
        QVERIFY(_lookAgain(QRectF(0.0, 0.0, 1.0, 1.0), &black));
        QCOMPARE(_asks(), asked);  // the finder is not asked about nothing
        QCOMPARE(_marks->count(), 0);  // and no mark is held over a black picture
        QCOMPARE(_marks->state(), QString("The video picture does not reach the app (it is black), so nothing can be marked."));
        QVERIFY(_marks->running() && _marks->usable());
        // The next picture is asked for half a second later, not at once: getting one costs the screen time.
        QTest::qWait(300);
        QVERIFY(!_marks->wantsPicture());
        QTRY_VERIFY_WITH_TIMEOUT(_marks->wantsPicture(), 1000);
        // One flat grey is no video either. A dark picture with something in it is one.
        QImage grey(1280, 720, QImage::Format_RGB888);
        grey.fill(QColor(40, 41, 42));
        QVERIFY(_look(QRectF(0.0, 0.0, 1.0, 1.0), &grey));
        QCOMPARE(_asks(), asked);
        QVERIFY(_marks->state().contains("does not reach the app"));
        QImage dark = grey;
        for (int x = 600; x < 680; ++x) {
            dark.setPixelColor(x, 360, QColor(48, 41, 42));
        }
        QVERIFY(_lookAgain(QRectF(0.0, 0.0, 1.0, 1.0), &dark));
        QVERIFY(_asks() > asked);
        QCOMPARE(_marks->count(), 1);
        QVERIFY2(_marks->state().startsWith("Marked: 1 person."), qPrintable(_marks->state()));
        // Only the video picture counts, not what is beside it in the image.
        QImage beside(1280, 720, QImage::Format_RGB888);
        beside.fill(Qt::black);
        for (int y = 0; y < 720; ++y) {
            uchar *row = beside.scanLine(y);
            for (int x = 0; x < 160; ++x) {
                row[3 * x] = static_cast<uchar>(120 + (x % 16) * 8);
                row[3 * x + 1] = 200;
            }
        }
        const int before = _asks();
        QVERIFY(_lookAgain(QRectF(190.0 / 1280.0, 0.0, 900.0 / 1280.0, 1.0), &beside));
        QCOMPARE(_asks(), before);
        QVERIFY(_marks->state().contains("does not reach the app"));
    }

    void theStateLineTellsHowLongAPictureTakesToGet()
    {
        _withMarks();
        _seen->sleepMs = 2;
        _marks->setLooking(true);
        QVERIFY(_look());
        QVERIFY2(_marks->state().endsWith(" s.") && !_marks->state().contains("Getting a picture"), qPrintable(_marks->state()));
        // The screen says how long its picture took: 48 ms.
        QTRY_VERIFY_WITH_TIMEOUT(_marks->wantsPicture(), 2000);
        QSignalSpy done(_marks, &ObjectMarks::lookDone);
        _marks->picture(_markPicture, 0.0, 0.0, 1.0, 1.0, 48.0);
        QVERIFY(done.wait(3000));
        QVERIFY2(_marks->state().endsWith("Getting a picture takes 0.05 s."), qPrintable(_marks->state()));
        // A time that is none is not told.
        QTRY_VERIFY_WITH_TIMEOUT(_marks->wantsPicture(), 2000);
        _marks->picture(_markPicture, 0.0, 0.0, 1.0, 1.0, std::numeric_limits<double>::infinity());
        QVERIFY(done.wait(3000));
        QVERIFY2(!_marks->state().contains("Getting a picture"), qPrintable(_marks->state()));
    }

    void aZoomStepOrTheOtherLensDropsTheMarks()
    {
        using skydroid::find::Kind;
        _withMarks();
        _seen->sleepMs = 2;
        _see({_thing(Kind::Person, 640.0, 360.0, 32.0, 72.0)});
        _marks->setLooking(true);
        QVERIFY(_look());
        QCOMPARE(_marks->count(), 1);
        _link->zoom(1);
        QCOMPARE(_marks->count(), 0);
        QVERIFY(_lookAgain());
        QCOMPARE(_marks->count(), 1);
        _link->setThermalPicture(true);
        QCOMPARE(_marks->count(), 0);
        QVERIFY(_marks->running());  // it goes on looking
    }

    void aTapOnAMarkLocksItsObjectWhereItIsNow()
    {
        using skydroid::find::Kind;
        _withMarks();
        _seen->sleepMs = 2;
        _marks->setLooking(true);
        // A person walks across the picture: at 0.30 of its width first, 0.02 further with every picture.
        const auto at = [](int i) { return 0.30 + 0.02 * i; };
        for (int i = 0; i <= 14; ++i) {
            _link->lockPicture(_picture(at(i), 0.5), 0.0, 0.0, 1.0, 1.0);
            if (i == 2) {
                // The finder looks at this moment's picture, and finds the walker there.
                _see({_thing(Kind::Person, at(2) * 1280.0, 360.0, 0.06 * 1280.0, 0.2 * 720.0)});
                QVERIFY(_look());
                QCOMPARE(_marks->count(), 1);
            }
            QTest::qWait(70);  // as the screen gives them: 14 pictures a second
        }
        // The mark is where the walker was. The operator taps it.
        QVERIFY(_near(_marks->marksNow().at(0).box.cx, at(2)));
        QVERIFY(_marks->lockAt(at(2), 0.5));
        QVERIFY(_link->lockActive() && _link->lockByApp());
        QVERIFY2(_link->lockSeen(), qPrintable(_link->lockNumbers()));
        // The lock is on the walker, twelve pictures on.
        QVERIFY2(std::abs(_link->lockBoxU() - at(14)) < 0.015,
                 qPrintable(QString("box at %1, walker at %2").arg(_link->lockBoxU()).arg(at(14))));
        _link->stopLock();
    }

    void aTapBesideAllMarksLocksNothing()
    {
        using skydroid::find::Kind;
        _withMarks();
        _see({_thing(Kind::Person, 640.0, 360.0, 32.0, 72.0)});
        _marks->setLooking(true);
        QVERIFY(_look());
        QVERIFY(!_marks->lockAt(0.9, 0.9));
        QVERIFY(!_marks->lockAt(std::nan(""), 0.5));
        QVERIFY(!_link->lockBusy() && !_link->lockActive());
        // On the mark: the lock is asked for (no picture was kept here, so it waits for one).
        QVERIFY(_marks->lockAt(0.5, 0.5));
        QVERIFY(_link->lockBusy() || _link->lockActive());
        _link->stopLock();
    }

    void whichMarkATapMeans()
    {
        using skydroid::find::Kind;
        const double shape = 16.0 / 9.0;
        const std::vector<ObjectMarks::Mark> marks = {_mark(Kind::Vehicle, 0.5, 0.5, 0.2, 0.2),
                                                      _mark(Kind::Person, 0.52, 0.5, 0.02, 0.08),
                                                      _mark(Kind::Person, 0.8, 0.2, 0.01, 0.04)};
        QCOMPARE(ObjectMarks::markAt(marks, 0.45, 0.45, shape), 0);  // inside the vehicle
        QCOMPARE(ObjectMarks::markAt(marks, 0.52, 0.5, shape), 1);   // inside both: the smaller, the rider
        // Beside a small mark: a finger's width still means it (0.012 of the picture's width).
        QCOMPARE(ObjectMarks::markAt(marks, 0.8 + 0.005 + 0.010, 0.2, shape), 2);
        QCOMPARE(ObjectMarks::markAt(marks, 0.8 + 0.005 + 0.014, 0.2, shape), -1);
        QCOMPARE(ObjectMarks::markAt(marks, 0.8, 0.2 + 0.02 + 0.020, shape), 2);  // the same on the screen, downwards
        QCOMPARE(ObjectMarks::markAt(marks, 0.8, 0.2 + 0.02 + 0.023, shape), -1);
        // Beside a large mark: three tenths of its size.
        QCOMPARE(ObjectMarks::markAt(marks, 0.35, 0.5, shape), 0);
        QCOMPARE(ObjectMarks::markAt(marks, 0.33, 0.5, shape), -1);
        // Between two marks: the nearer.
        const std::vector<ObjectMarks::Mark> two = {_mark(Kind::Person, 0.40, 0.5, 0.02, 0.08),
                                                    _mark(Kind::Person, 0.44, 0.5, 0.02, 0.08)};
        QCOMPARE(ObjectMarks::markAt(two, 0.418, 0.5, shape), 0);
        QCOMPARE(ObjectMarks::markAt(two, 0.424, 0.5, shape), 1);
        QCOMPARE(ObjectMarks::markAt(two, 0.4205, 0.5, shape), 1);  // a finger's width from both: the second is nearer
        QCOMPARE(ObjectMarks::markAt(two, 0.4195, 0.5, shape), 0);
        QCOMPARE(ObjectMarks::markAt({}, 0.5, 0.5, shape), -1);
        QCOMPARE(ObjectMarks::markAt(marks, std::nan(""), 0.5, shape), -1);
        QCOMPARE(ObjectMarks::markAt(marks, 0.45, 0.45, std::nan("")), 0);  // a shape that is none: 16 to 9
    }

    void howLongTheFinderRests()
    {
        QCOMPARE(ObjectMarks::restAfter(0.0), 60);
        QCOMPARE(ObjectMarks::restAfter(100.0), 60);
        QCOMPARE(ObjectMarks::restAfter(200.0), 60);
        QCOMPARE(ObjectMarks::restAfter(300.0), 90);
        QCOMPARE(ObjectMarks::restAfter(1000.0), 300);
        QCOMPARE(ObjectMarks::restAfter(-5.0), 60);
        QCOMPARE(ObjectMarks::restAfter(std::nan("")), 60);
        QCOMPARE(ObjectMarks::restAfter(1.0e12), 18000);
        QCOMPARE(ObjectMarks::restAfter(std::numeric_limits<double>::infinity()), 60);  // no number: the least rest
    }

    void theFinderRestsBetweenItsLooks()
    {
        _withMarks();
        _seen->sleepMs = 100;
        _marks->setLooking(true);
        QVERIFY(_look());
        QVERIFY(!_marks->wantsPicture());  // resting
        QVERIFY2(_marks->lookMs() >= 195.0 && _marks->lookMs() < 400.0, qPrintable(QString::number(_marks->lookMs())));
        const int rest = ObjectMarks::restAfter(_marks->lookMs());
        QVERIFY(rest >= 60);
        QElapsedTimer clock;
        clock.start();
        QTRY_VERIFY_WITH_TIMEOUT(_marks->wantsPicture(), 2000);
        QVERIFY2(clock.elapsed() >= rest - 30 && clock.elapsed() <= rest + 400, qPrintable(QString::number(clock.elapsed())));
    }

    void oneLookAtATime()
    {
        _withMarks();
        _seen->sleepMs = 80;
        _marks->setLooking(true);
        QSignalSpy done(_marks, &ObjectMarks::lookDone);
        _marks->picture(_markPicture, 0.0, 0.0, 1.0, 1.0);
        QVERIFY(!_marks->wantsPicture());
        _marks->picture(_markPicture, 0.0, 0.0, 1.0, 1.0);  // pictures that come anyway are not looked at
        _marks->picture(_markPicture, 0.0, 0.0, 1.0, 1.0);
        QVERIFY(done.wait(3000));
        QTest::qWait(250);
        QCOMPARE(done.count(), 1);
        QCOMPARE(_asks(), 2);  // one picture: its two looks
        // A picture that is none is not looked at either.
        QTRY_VERIFY_WITH_TIMEOUT(_marks->wantsPicture(), 2000);
        _marks->picture(QImage(), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_marks->wantsPicture());
        // And one with no room for a picture in it gives no marks, and no trouble.
        QVERIFY(_look(QRectF(0.0, 0.0, 0.01, 0.01)));
        QCOMPARE(_asks(), 2);
        QCOMPARE(_marks->count(), 0);
        // (It is not taken for a black video: that message would send the operator the wrong way.)
        QVERIFY2(_marks->state().startsWith("No person or vehicle found."), qPrintable(_marks->state()));
        QVERIFY(_lookAgain(QRectF(0.0, 0.0, std::nan(""), 1.0)));  // nor one whose place is no number
        QCOMPARE(_asks(), 2);
        QVERIFY(_lookAgain(QRectF(std::nan(""), 0.0, 1.0, 1.0)));
        QCOMPARE(_asks(), 2);
        QVERIFY(_lookAgain(QRectF(0.0, std::numeric_limits<double>::infinity(), 1.0, 1.0)));
        QCOMPARE(_asks(), 2);
        QVERIFY2(_marks->state().startsWith("No person or vehicle found.") && !_marks->state().contains("trouble"),
                 qPrintable(_marks->state()));
    }

    void aFinderThatCannotWorkSaysSoAndIsLeftAlone()
    {
        _withMarks();
        _seen->canLoad = false;
        _seen->whyNot = "this build has no object finding in it";
        _marks->setLooking(true);
        QSignalSpy state(_marks, &ObjectMarks::stateChanged);
        QVERIFY(_look());
        QVERIFY(!_marks->usable() && !_marks->running() && !_marks->wantsPicture());
        QCOMPARE(_marks->state(), QString("People and vehicles cannot be marked: this build has no object finding in it."));
        QVERIFY(state.count() >= 1);
        QVERIFY(!_link->keepsPictures());
        QCOMPARE(_seen->loads, 1);
        QCOMPARE(_asks(), 0);
        // It is not tried again, whatever the operator presses.
        _marks->setLooking(false);
        _marks->setLooking(true);
        QVERIFY(!_marks->running());
        _marks->picture(_markPicture, 0.0, 0.0, 1.0, 1.0);
        QTest::qWait(60);
        QCOMPARE(_seen->loads, 1);
        QVERIFY(!_marks->lockAt(0.5, 0.5));
    }

    void aLookThatFailsIsToldInTheStateLine()
    {
        _withMarks();
        _seen->sleepMs = 2;
        _seen->trouble = "no memory for the picture";
        _marks->setLooking(true);
        QVERIFY(_look());
        QVERIFY(_marks->usable() && _marks->running());
        QVERIFY2(_marks->state().contains("The finder has trouble: no memory for the picture."), qPrintable(_marks->state()));
        // A look that failed says nothing of how fast the device is: the next one is as careful
        // as the first (the whole picture, and the smallest middle).
        QVERIFY(_lookAgain());
        QCOMPARE(_asks(), 4);
        QVERIFY(_isArea(_ask(3).area, 320, 168, 640, 384));
        {
            QMutexLocker lock(&_seen->mutex);
            _seen->trouble.clear();
        }
        QVERIFY(_lookAgain());
        QVERIFY(!_marks->state().contains("trouble"));
        // Trouble with the close look alone is told too.
        {
            QMutexLocker lock(&_seen->mutex);
            _seen->trouble = "no memory for the picture";
            _seen->troubleCloseOnly = true;
        }
        QVERIFY(_lookAgain());
        QVERIFY2(_marks->state().contains("The finder has trouble: no memory for the picture."), qPrintable(_marks->state()));
    }

    void theFinderGivesItsMemoryBackWhenItIsNotUsed()
    {
        _withMarks();
        _marks->setIdleRestMs(150);
        _marks->setLooking(true);
        QVERIFY(_look());
        QCOMPARE(_seen->loads, 1);
        QTest::qWait(300);
        QCOMPARE(_seen->rests, 0);  // in use: nothing is given back
        _marks->setLooking(false);
        QTRY_COMPARE_WITH_TIMEOUT(_seen->rests, 1, 2000);
        QTest::qWait(300);
        QCOMPARE(_seen->rests, 1);  // once
        // The next look loads it again.
        _marks->setLooking(true);
        QVERIFY(_lookAgain());
        QCOMPARE(_seen->loads, 2);
        // Pressed again before the time was up: it stays loaded.
        _marks->setLooking(false);
        QTest::qWait(60);
        _marks->setLooking(true);
        QTest::qWait(300);
        QCOMPARE(_seen->rests, 1);
    }

    void theNoticesComeWithTheFinder()
    {
        _withMarks();
        QCOMPARE(_marks->notices(), QString("the notices of the stand-in finder"));
        QCOMPARE(_marks->finderName(), QString("a stand-in finder"));
        QCOMPARE(_marks->pictureWidth(), 1280);
    }

    void theLooksRunOnAThreadOfTheirOwn()
    {
        using skydroid::find::Kind;
        _withMarks(true);
        _seen->sleepMs = 120;
        _see({_thing(Kind::Person, 640.0, 360.0, 32.0, 72.0), _thing(Kind::Vehicle, 200.0, 600.0, 128.0, 72.0)});
        _marks->setLooking(true);
        QElapsedTimer clock;
        clock.start();
        while (_marks->count() < 2 && clock.elapsed() < 5000) {
            if (_marks->wantsPicture()) {
                _marks->picture(_markPicture, 0.0, 0.0, 1.0, 1.0);
            }
            QTest::qWait(10);
        }
        QCOMPARE(_marks->count(), 2);
        QMutexLocker lock(&_seen->mutex);
        QVERIFY(_seen->threads.size() >= 2);
        for (const Qt::HANDLE thread : _seen->threads) {
            QVERIFY(thread != QThread::currentThreadId());
        }
        if (QThread::idealThreadCount() >= 4) {
            // Side by side: the two looks at a picture on two threads, and done in the time of one
            // (0.12 s each: one after the other they would take 0.24 s).
            QVERIFY(_seen->threads.at(0) != _seen->threads.at(1));
            QVERIFY2(_marks->lookMs() < 215.0, qPrintable(QString::number(_marks->lookMs())));
        }
    }

private:

    /// The marks on this link, with a stand-in for the network: it finds what the test puts into
    /// _seen. ownThread: the looks run on a thread of their own, as in the app.
    void _withMarks(bool ownThread = false)
    {
        _seen = std::make_shared<FakePictureFinder::Seen>();
        _marks = new ObjectMarks(_link, new FakePictureFinder(_seen), ownThread);
        if (_markPicture.isNull()) {
            // Ground with a pattern. (A picture of one flat colour counts as no video.)
            _markPicture = QImage(1280, 720, QImage::Format_RGB888);
            for (int y = 0; y < 720; ++y) {
                uchar *row = _markPicture.scanLine(y);
                for (int x = 0; x < 1280; ++x) {
                    const uchar grey = static_cast<uchar>(70 + ((x / 16) * 37 + (y / 16) * 91) % 90);
                    row[3 * x] = grey;
                    row[3 * x + 1] = static_cast<uchar>(grey + 8);
                    row[3 * x + 2] = static_cast<uchar>(grey / 2);
                }
            }
        }
    }

    /// What is in the picture from now on, in points of the 1280 x 720 image the finder gets.
    void _see(const std::vector<skydroid::find::Found> &objects)
    {
        QMutexLocker lock(&_seen->mutex);
        _seen->objects = objects;
    }

    static skydroid::find::Found _thing(skydroid::find::Kind kind, double cx, double cy, double w, double h,
                                        double score = 0.8)
    {
        skydroid::find::Found f;
        f.kind = kind;
        f.label = kind == skydroid::find::Kind::Person ? 0 : 2;
        f.score = score;
        f.cx = cx;
        f.cy = cy;
        f.w = w;
        f.h = h;
        return f;
    }

    static ObjectMarks::Mark _mark(skydroid::find::Kind kind, double u, double v, double w, double h)
    {
        ObjectMarks::Mark mark;
        mark.box = _thing(kind, u, v, w, h);
        return mark;
    }

    /// Gives the marks one picture for the finder (1280 x 720 points; pictureRect: where the video
    /// picture is inside it) and waits for the answer. False when no picture was wanted, or no
    /// answer came.
    bool _look(const QRectF &pictureRect = QRectF(0.0, 0.0, 1.0, 1.0), const QImage *image = nullptr)
    {
        if (!_marks->wantsPicture()) {
            return false;
        }
        QSignalSpy done(_marks, &ObjectMarks::lookDone);
        _marks->picture(image ? *image : _markPicture, pictureRect.x(), pictureRect.y(), pictureRect.width(),
                        pictureRect.height());
        return done.count() > 0 || done.wait(5000);
    }

    /// Waits until the finder has rested, then looks.
    bool _lookAgain(const QRectF &pictureRect = QRectF(0.0, 0.0, 1.0, 1.0), const QImage *image = nullptr)
    {
        QElapsedTimer clock;
        clock.start();
        while (!_marks->wantsPicture() && clock.elapsed() < 5000) {
            QTest::qWait(5);
        }
        return _look(pictureRect, image);
    }

    int _asks() const
    {
        QMutexLocker lock(&_seen->mutex);
        return static_cast<int>(_seen->asks.size());
    }

    skydroid::find::Ask _ask(int i) const
    {
        QMutexLocker lock(&_seen->mutex);
        return _seen->asks.at(static_cast<std::size_t>(i));
    }

    static bool _isArea(const skydroid::find::Area &area, int left, int top, int width, int height)
    {
        return area.left == left && area.top == top && area.width == width && area.height == height;
    }

    static bool _near(double a, double b, double by = 1e-6)
    {
        return std::abs(a - b) <= by;
    }

    /// How many marks of a kind are shown.
    int _marked(skydroid::find::Kind kind) const
    {
        int n = 0;
        for (const ObjectMarks::Mark &mark : _marks->marksNow()) {
            n += mark.box.kind == kind ? 1 : 0;
        }
        return n;
    }

    /// Follow Me on this link, with a stand-in for the drone: it hovers where the test's vehicle is,
    /// in Loiter, its nose to 35 degrees.
    void _withFollow()
    {
        _drone = new FakeFollowDrone;
        _follow = new ObjectFollow(_link, _drone);
        _telemetry = skydroid::follow::Drone{};
        _telemetry.latDeg = _vehicle.position.latitude();
        _telemetry.lonDeg = _vehicle.position.longitude();
        _telemetry.heightM = 100.0;
        _telemetry.northMps = 0.0;
        _telemetry.eastMps = 0.0;
        _telemetry.headingDeg = 35.0;
        _telemetry.armed = true;
        _telemetry.flying = true;
        _telemetry.gps3d = true;
        _telemetry.mode = skydroid::follow::Mode::Holding;
        _tellTelemetry = true;
    }

    /// The drone has changed to Guided, as Follow asked.
    void _nowGuided()
    {
        _telemetry.mode = skydroid::follow::Mode::Guided;
        _drone->mode = "Guided";
        _drone->tell(_telemetry);
    }

    /// The drone hovers this high over level ground, the camera this far down, straight ahead (north).
    void _hoverAt(double heightM, double pitchDeg)
    {
        _vehicle.vehicle.set("altitudeRelative", heightM);
        _vehicle.vehicle.set("altitudeAMSL", 250.0 + heightM);
        _telemetry.heightM = heightM;
        _camera->yaw = 0.0;
        _camera->pitch = pitchDeg;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid() && _link->gimbalPitch() == pitchDeg, 2000);
    }

    /// The video goes on until `done`, or for this long: a picture every 90 ms with the object at u, v (or
    /// without it), and the drone's telemetry with every other picture.
    template <typename Done>
    bool _goOnUntil(Done done, int timeoutMs, double u = 0.5, double v = 0.5, bool objectThere = true)
    {
        QElapsedTimer clock;
        clock.start();
        int n = 0;
        while (clock.elapsed() < timeoutMs) {
            if (done()) {
                return true;
            }
            _link->lockPicture(_picture(u, v, objectThere), 0.0, 0.0, 1.0, 1.0);
            if (_drone && _tellTelemetry && (n++ % 2) == 0) {
                _drone->tell(_telemetry);
            }
            QTest::qWait(90);
        }
        return done();
    }

    void _goOn(int ms, double u = 0.5, double v = 0.5, bool objectThere = true)
    {
        _goOnUntil([]() { return false; }, ms, u, v, objectThere);
    }

    /// Telemetry only, with no lock to give pictures to.
    void _goOnTelemetry(int ms)
    {
        QElapsedTimer clock;
        clock.start();
        while (clock.elapsed() < ms) {
            _drone->tell(_telemetry);
            QTest::qWait(180);
        }
    }

    /// A lock on an object under the cross, 57.7 m north of a drone that hovers 100 m up (the camera 60
    /// degrees down), held until Follow would start.
    void _lockAndKnowTheObject()
    {
        _hoverAt(100.0, -60.0);
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        QVERIFY2(_goOnUntil([this]() { return _follow->ready(); }, 8000), qPrintable(_follow->notReadyText()));
    }

    /// A new link with the same saved settings, trying these addresses when quiet.
    void _restartWithProbeTargets(const QStringList &hosts, const QList<int> &ports)
    {
        delete _link;
        SkydroidLink::setProbeTargets(hosts, ports);
        _link = new SkydroidLink;
        QVERIFY(_link->enabled());
    }

    /// The lock frame for the camera's own tracker for a point of the 1280 x 720
    /// picture: the camera counts 32 points further across and 18 further down.
    static std::string _got(int x, int y, const top::Options &options = {})
    {
        return top::buildGotTarget(x + 32, y + 18, 1344, 756, options);
    }

    /// The speed of the last GSY or GSP frame the camera got, in degrees per second. Not a number when none came.
    double _lastSpeed(const char *tag) const
    {
        const QString prefix = QStringLiteral("#TPUG2w") + QLatin1String(tag);
        for (int i = _camera->received.size() - 1; i >= 0; --i) {
            const QString &frame = _camera->received.at(i);
            if (frame.startsWith(prefix) && frame.size() >= prefix.size() + 2) {
                bool ok = false;
                const int raw = frame.mid(prefix.size(), 2).toInt(&ok, 16);
                if (ok) {
                    return (raw > 127 ? raw - 256 : raw) * 0.5;
                }
            }
        }
        return std::nan("");
    }

    /// A picture of the video for the app's lock: ground with a pattern, and an
    /// object (light and dark blocks) with its centre at u, v of the picture.
    static QImage _picture(double u, double v, bool objectThere = true, int objectW = 22, int objectH = 44,
                           int width = 384, int height = 216)
    {
        QImage image(width, height, QImage::Format_Grayscale8);
        quint32 state = 12345;
        const auto next = [&state]() {
            state = state * 1664525u + 1013904223u;
            return static_cast<int>((state >> 16) & 0xff);
        };
        const int cell = 12;
        const int columns = width / cell + 1;
        std::vector<int> ground(columns * (height / cell + 1));
        for (int &g : ground) {
            g = 70 + next() % 90;
        }
        for (int y = 0; y < height; ++y) {
            uchar *row = image.scanLine(y);
            for (int x = 0; x < width; ++x) {
                row[x] = static_cast<uchar>(ground[(y / cell) * columns + x / cell]);
            }
        }
        if (objectThere) {
            const int block = 4;
            const int blockColumns = objectW / block + 1;
            std::vector<int> blocks(blockColumns * (objectH / block + 1));
            quint32 objectState = 777;
            for (int &b : blocks) {
                objectState = objectState * 1664525u + 1013904223u;
                b = ((objectState >> 16) & 1) ? 230 : 20;
            }
            const int left = static_cast<int>(std::lround(u * width - objectW / 2.0));
            const int top = static_cast<int>(std::lround(v * height - objectH / 2.0));
            for (int y = 0; y < objectH; ++y) {
                if (top + y < 0 || top + y >= height) {
                    continue;
                }
                uchar *row = image.scanLine(top + y);
                for (int x = 0; x < objectW; ++x) {
                    if (left + x >= 0 && left + x < width) {
                        row[left + x] = static_cast<uchar>(blocks[(y / block) * blockColumns + x / block]);
                    }
                }
            }
        }
        return image;
    }

    static QVector<int> _rc(int channel, int value)
    {
        QVector<int> values(16, 1500);
        values[channel - 1] = value;
        return values;
    }

    Vehicle _vehicle;
    FakeCamera *_camera = nullptr;
    SkydroidLink *_link = nullptr;
    // Follow Me, in the tests that call _withFollow().
    FakeFollowDrone *_drone = nullptr;
    ObjectFollow *_follow = nullptr;
    skydroid::follow::Drone _telemetry;
    bool _tellTelemetry = true;
    // The marks, in the tests that call _withMarks().
    ObjectMarks *_marks = nullptr;
    std::shared_ptr<FakePictureFinder::Seen> _seen;
    QImage _markPicture;
};

QTEST_GUILESS_MAIN(SkydroidLinkTest)
#include "skydroid_link_test.moc"
