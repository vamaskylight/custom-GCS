// Host test for SkydroidLink: the real Qt link code talks UDP to fake cameras
// in the same process. Checks gimbal angles (GAA push and GAC questions), the
// switch to another address when the configured one gives no angles, speed
// motion from touch and RC wheels, the frames each button sends, the laser
// sequence (laser module first, system address as fallback), and the target
// position rules.
//
// Every address used here is on this computer (127.0.0.x).

#include <QtCore/QDir>
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
    }

    quint16 port() const { return _socket.localPort(); }

    bool answerGac = true;       // answer a GAC question
    bool pushAfterGaa = false;   // send angles on its own after GAA, like the camera's push
    double yaw = 0.0;
    double pitch = -30.0;
    std::optional<std::string> slrE;  // SLR data field from the laser module
    std::optional<std::string> slrD;  // SLR data field from the system address
    std::optional<int> dzmStep;
    int laserModuleDelayMs = 0;  // answer E reads late
    QStringList received;

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
    void clear() { received.clear(); }

private slots:
    void _read()
    {
        while (_socket.hasPendingDatagrams()) {
            const QNetworkDatagram d = _socket.receiveDatagram();
            const std::string raw(d.data().constData(), static_cast<size_t>(d.data().size()));
            received << QString::fromStdString(raw);
            const auto f = top::parseTpFrame(raw);
            if (!f) {
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
        _link->center();
        _link->centerYaw();
        _link->pointDown();
        _link->ptz("stop");
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildPtz("center")), 1, 2000);
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildGimbalAngleAxis("GAY", 0.0, 30.0)), 1, 2000);
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildPtz("nadir")), 1, 2000);
        QTRY_COMPARE_WITH_TIMEOUT(_camera->countEqual(top::buildPtz("stop")), 3, 2000);
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
        QVERIFY(_link->targetMessage().contains("GPS"));
    }

    void noGimbalAnglesMeansNoTarget()
    {
        _camera->answerGac = false;
        _camera->slrE = "01F4";
        _link->fireLaser();
        QTRY_VERIFY_WITH_TIMEOUT(!_link->laserBusy(), 3000);
        QVERIFY(_link->laserValid());
        QVERIFY(!_link->targetValid());
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
        QVERIFY(_link->targetMessage().contains("No drone"));
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
