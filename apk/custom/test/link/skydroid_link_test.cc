// Host test for SkydroidLink: the real Qt link code talks UDP to fake cameras
// in the same process. Checks gimbal angles (GAA push and GAC questions), the
// switch to another address when the configured one gives no angles, speed
// motion from touch and RC wheels, the frames each button sends, the laser
// sequence (laser module first, system address as fallback), the target
// position rules, the object lock (turn, GOT, SUM confirm and stop), and the
// thermal colour mode setting.
//
// Every address used here is on this computer (127.0.0.x).

#include <QtCore/QDir>
#include <QtCore/QElapsedTimer>
#include <QtCore/QSettings>
#include <QtNetwork/QNetworkDatagram>
#include <QtNetwork/QUdpSocket>
#include <QtTest/QSignalSpy>
#include <QtTest/QtTest>

#include <cmath>
#include <optional>
#include <string>

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
        _link->lockAt(0.25, 0.5);
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockActive(), 5000);
        top::Options upper;
        upper.gClassUpperHeader = true;
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildGotTarget(640, 360, 1280, 720, upper)), 1, 1000);
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
        // The target uses the angles at the shot, so it lies off to that side.
        QVERIFY(_link->targetValid());
        QVERIFY(std::abs(_link->gimbalYaw() + 20.85) < 0.01);
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

    // --- Object lock: turn to the object, GOT, SUM confirm -------------------------

    void lockTurnsToTheObjectThenLocksIt()
    {
        _camera->yaw = 0.0;
        _camera->pitch = -30.0;
        _camera->followAngles = true;
        _camera->slrE = "01F4";  // 50 m
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.75, 0.4);  // same point as the tap test: 20.85 right, 4.69 up
        QVERIFY(_link->lockBusy());
        QVERIFY(!_link->lockActive());
        const std::string gay = top::buildGimbalAngleAxis("GAY", -20.85, 30.0);
        const std::string got = top::buildGotTarget(640, 360);  // the object is under the cross now
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
        _camera->yaw = 3.0;
        _camera->pitch = -30.0;
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        // 0.01 of the view is under 1.5 degrees: lock where it was picked (1280 x 720 frame).
        _link->lockAt(0.51, 0.49);
        QVERIFY(_link->lockActive());
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildGotTarget(653, 353)) == 1, 1000);
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildSumTrack(true)) >= 1, 1000);
        QCOMPARE(_camera->countStartingWith("#tpUG6wGAY"), 0);
        QCOMPARE(_camera->countStartingWith("#tpUG6wGAP"), 0);
    }

    void lockAtAGimbalLimitPointsGotAtTheObject()
    {
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
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildGotTarget(x, y)), 1, 1000);
    }

    void lockNeverStartsIfTheCameraDoesNotTurn()
    {
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
        _camera->answerGac = false;
        _link->lockAt(0.5, 0.5);
        QVERIFY(!_link->lockBusy() && !_link->lockActive());
        QVERIFY(_link->lockMessage().contains("No gimbal angles"));
        QTest::qWait(300);
        QCOMPARE(_camera->countStartingWith("#tpUG8wGOT"), 0);
    }

    void lockSeesTheCameraFollow()
    {
        _camera->yaw = 0.0;
        _camera->pitch = -30.0;
        _camera->trackDriftDps = 5.0;  // the camera follows a moving object
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive());
        QVERIFY(!_link->lockFollowSeen());
        // The first 2 s do not count: what the camera does then is its tracker
        // taking over. It has already turned about 7 degrees here.
        QTest::qWait(1500);
        QVERIFY(!_link->lockFollowSeen());
        QVERIFY(_link->lockTurnedDeg() < 0.1);
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockFollowSeen(), 3000);
        QVERIFY(_link->lockTurnedDeg() > 0.8);
        QVERIFY(_link->lockMessage().contains("following"));
        _link->stopLock();
    }

    // Field video of test build 4 (2026-10-06 21:53): right after the lock the
    // V13 moved 2.6 degrees by itself beside a parked car and then stood still.
    // The app said "is following", and nothing was moving.
    void aJumpAtTheLockIsNotFollowing()
    {
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

    void followingAfterAJumpIsStillSeen()
    {
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
        QVERIFY(_link->lockMessage().contains("following"));
        _link->stopLock();
    }

    void lockSaysSoWhenTheCameraDoesNotFollow()
    {
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);  // the camera never turns by itself
        QVERIFY(_link->lockActive());
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockMessage().contains("has not turned"), 8000);
        QVERIFY(!_link->lockFollowSeen());
        QVERIFY(_link->lockActive());  // only a warning: the operator decides
    }

    void lockSaysSoWhenTheAnglesStop()
    {
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive());
        _camera->answerGac = false;  // no more angles: the app cannot see the camera turn
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockMessage().contains("no angles"), 8000);
        QVERIFY(!_link->lockMessage().contains("not following"));
    }

    void movingTheCameraByHandEndsTheLock()
    {
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
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);
        QVERIFY(_link->lockActive());
        _camera->clear();
        _link->lockAt(0.505, 0.5);
        // The old lock stops first; the new GOT comes a moment later (VGCS waits 50 ms).
        QTRY_VERIFY_WITH_TIMEOUT(_link->lockActive(), 1000);
        const std::string newGot = top::buildGotTarget(646, 360);
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(newGot), 1, 1000);
        const int stopAt = _camera->indexOf(top::buildSumTrack(false));
        const int gotAt = _camera->indexOf(newGot);
        QVERIFY(stopAt >= 0 && stopAt < gotAt);
        QVERIFY(_camera->receivedAtMs.at(gotAt) - _camera->receivedAtMs.at(stopAt) >= 40);
    }

    void lockMeasuresAgainAndKeepsTheLastResultMeanwhile()
    {
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
        _link->setModel("C14 Pro");
        QTRY_VERIFY_WITH_TIMEOUT(_link->attitudeValid(), 2000);
        _link->lockAt(0.5, 0.5);
        top::Options options;
        options.gClassUpperHeader = true;
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildGotTarget(640, 360, 1280, 720, options)), 1, 1000);
        QVERIFY(_camera->countEqual(top::buildGotTarget(640, 360, 1280, 720, options)) == 1);
        QCOMPARE(_camera->countStartingWith("#tpUG8wGOT"), 0);
        // Changing the camera stops the lock, in the old camera's format.
        _link->setModel("C13");
        QVERIFY(!_link->lockActive());
        QTRY_VERIFY_WITH_TIMEOUT(_camera->countEqual(top::buildSumTrack(false, options)) >= 1, 1000);
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
