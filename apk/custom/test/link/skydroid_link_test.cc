// Host test for SkydroidLink: the real Qt link code talks UDP to fake cameras
// in the same process. Checks gimbal angles (GAA push and GAC questions), the
// switch to another address when the configured one gives no angles, speed
// motion from touch and RC wheels, the frames each button sends, the laser
// sequence (laser module first, system address as fallback), the target
// position rules, the object lock (the app's own: pictures in, gimbal speeds
// out; and the camera's own tracker: turn, GOT, SUM confirm and stop), and the
// thermal colour mode setting.
//
// Every address used here is on this computer (127.0.0.x).

#include <QtCore/QDir>
#include <QtCore/QElapsedTimer>
#include <QtCore/QSettings>
#include <QtGui/QImage>
#include <QtNetwork/QNetworkDatagram>
#include <QtNetwork/QUdpSocket>
#include <QtTest/QSignalSpy>
#include <QtTest/QtTest>

#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "LaserGeo.h"
#include "MultiVehicleManager.h"
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
        _clock.start();
    }

    quint16 port() const { return _socket.localPort(); }

    bool answerGac = true;       // answer a GAC question
    bool pushAfterGaa = false;   // send angles on its own after GAA, like the camera's push
    bool followAngles = false;   // turn to GAY / GAP angle commands, like the real gimbal
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
    QElapsedTimer _clock;
    QNetworkDatagram _pushTo;
};

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

    void appLockRefusesAPlainBox()
    {
        QImage plain(384, 216, QImage::Format_Grayscale8);
        plain.fill(120);
        _link->lockAtBox(0.5, 0.5, 0.1, 0.2);
        _link->lockPicture(plain, 0.0, 0.0, 1.0, 1.0);
        QVERIFY(!_link->lockActive() && !_link->lockBusy());
        QVERIFY(_link->lockMessage().contains("plain area"));
        QVERIFY(!_link->lockWantsPictures());
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

    void aTapInPictureModeEndsALockAndSaysWhy()
    {
        _link->setPointMode("picture");
        _flyAtAHundredMetres();
        _link->lockAtBox(0.5, 0.5, 0.06, 0.2);
        _link->lockPicture(_picture(0.5, 0.5), 0.0, 0.0, 1.0, 1.0);
        QVERIFY(_link->lockActive());
        _link->aimAndMeasure(0.6, 0.6);
        QVERIFY(!_link->lockActive());
        QVERIFY2(_link->lockMessage().contains("another point was measured"), qPrintable(_link->lockMessage()));
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

private:
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
};

QTEST_GUILESS_MAIN(SkydroidLinkTest)
#include "skydroid_link_test.moc"
