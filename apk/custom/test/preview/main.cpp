// Preview of the VAMA camera screen without a phone or a QGC build.
//
// Loads the real FlyViewCustomLayer.qml from custom.qrc (as the app does),
// with stand-in QML modules for QGC (stubs/), the real SkydroidLink talking
// to a fake camera on 127.0.0.1, and a fake drone with a GPS fix. Then it
// walks through the main states and saves a screenshot of each:
//   1_main.png      camera answering, video as the main view
//   2_laser.png     after a laser shot: distance and target position
//   3_drag.png      a finger dragging on the video (floating stick)
//   4_settings.png  the camera settings window
//   5_settings_wheel.png  the same window scrolled to the RC wheel part
//   6_camera_off.png      the camera link turned off
//   7_ir_on.png           after the IR button: thermal stream, button lit
//   8_thermal_colours_menu.png  the list of thermal colour modes (button under IR)
//   8_thermal_ironbow.png       the thermal picture in Ironbow
//   9_tap_aiming.png, 10_tap_result.png   a tap on an object: turn, then measure
//   11_lock_pick.png      after the lock button: the hint to pick the object
//   12_lock_drawing.png   a box being drawn around the walker
//   13_locked.png         locked: the app turned the camera to the walker, the box is on it
//   14_lock_following.png the walker walks and the app turns the camera after it
//   14b_lock_on_the_cross.png  the walker stands: under the cross, measured by the laser
//   14c_lock_not_seen.png      the walker is gone: the box turns red, the camera waits
//   15_lock_lost.png      two seconds later the lock has ended and says why
//   15b_camera_tracker.png     the same with the camera's own tracker (lock mode "camera")
//   15c_lock_stopped.png  after the lock button again
//   16_no_gps_lock.png    a laser shot while the drone has no GPS lock: distance, and why no lat long
//   20_follow_ask.png     Follow Me: the follow button beside the lock button, and the window that asks first
//   21_following.png      the drone follows the walking walker: the green button, the state in the result box
//   22_follow_stopped.png after the green button: why Follow ended
//   23_follow_not_ready.png    the window when Follow cannot start (the drone too low): why, and no start
//   24_marks.png          after the lock button: the walker and a car are marked, and corners show where the app looks closely
//   25_mark_locked.png    after a tap on the walker's mark: locked, the box on the walker
//   26_marks_setting.png  the switch for the marks in the camera settings
//   27_notices.png        the licences of the object finder, as the app shows them
//   28_picking_stopped.png the lock button pressed again while it waits: the lock box says nothing was locked, and why
//   17_app_settings.png   Application Settings: the VAMA mark on General, no Help page
//   18_warnings_small.png QGC's "No GPS Lock" and pre-arm texts, small at the bottom while the video is the main view
//   19_warnings_map.png   the same texts as QGC shows them, with the map as the main view
//   29_admin_password.png the app menu after the hidden Advanced Mode tap: it asks for the admin password (task Z)
//   30_admin_wrong.png    after a wrong password: the wait before the next try
//   31_cameras.png        the camera editor (task AA): the built-in cameras, to copy
//   32_camera_form.png    a copy of the V13 being made into a camera without laser, photo, recording, zoom or IR
//   33_plain_camera.png   that camera in use: its buttons only
//   34_map_pack_loading.png  a map pack loading: the fly view says so (task AB)
// It also taps the IR button twice and checks QGC's video address each time,
// and checks the frames the camera got for the tap and the lock.
// The lock is tried for real: the day picture is a look into a world that
// stands still (stubs/QtMultimedia/VideoOutput.qml), the fake camera turns at
// the speeds it is told, and a walker walks through the world. The app has to
// find the walker in the pictures it grabs and keep it under the cross.
// With IR on it picks Ironbow in the thermal colour list, then checks every
// mode against VGCS's colour table at 64 grey levels of the thermal picture,
// and that the black bars beside the picture stay black. The video is QGC's
// own address, answered by the VAMA copy (FlightDisplayViewVideoOutput.qml).
// Application Settings is opened by QGC's own address, through the app's
// override rule, and checked for the client feedback of 2026-10-07 (tasks E
// and F in DOCS/APK-V1-DEV-PLAN.md).
// Every QML warning or error is printed; the exit code is 1 when there was one.

#include <QtCore/QDir>
#include <QtCore/QElapsedTimer>
#include <QtCore/QFile>
#include <QtCore/QLoggingCategory>
#include <QtCore/QMutex>
#include <QtCore/QRegularExpression>
#include <QtCore/QSettings>
#include <QtCore/QThread>
#include <QtCore/QEventLoop>
#include <QtCore/QTimer>
#include <QtGui/QGuiApplication>
#include <QtGui/QImage>
#include <QtGui/QMouseEvent>
#include <QtNetwork/QNetworkDatagram>
#include <QtNetwork/QUdpSocket>
#include <QtQml/QQmlAbstractUrlInterceptor>
#include <QtQml/QQmlApplicationEngine>
#include <QtQml/QQmlContext>
#include <QtQuick/QQuickItem>
#include <QtQuick/QQuickWindow>
#include <QtTest/QTest>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "AdminPassword.h"
#include "ColoredSvgImageProvider.h"
#include "MultiVehicleManager.h"
#include "FollowGuide.h"
#include "ObjectFollow.h"
#include "ObjectMarks.h"
#include "SkydroidLink.h"
#include "SkydroidTop.h"
#include "Vehicle.h"

// VGCS's thermal colour modes (made by apk/make_thermal_palettes.py).
#include "../thermal_palette_vectors.inc"

namespace top = skydroid::top;

namespace {

int g_problems = 0;

void messageHandler(QtMsgType type, const QMessageLogContext &, const QString &message)
{
    std::fprintf(stderr, "%s\n", qPrintable(message));
    // Windows says this when another program holds the clipboard at that moment: about the PC,
    // not the app (it cost one run of six a problem, 2026-10-15).
    if (message == QLatin1String("Retrying to obtain clipboard.")) {
        return;
    }
    if (type != QtDebugMsg && type != QtInfoMsg) {
        ++g_problems;
    }
}

} // namespace

class FakeVideoManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool fullScreen MEMBER fullScreen NOTIFY fullScreenChanged)
    Q_PROPERTY(double aspectRatio READ aspectRatio CONSTANT)

public:
    bool fullScreen = false;
    double aspectRatio() const { return 16.0 / 9.0; }  // the C13 stream, 1280 x 720

signals:
    void fullScreenChanged();
    void imageFileChanged(const QString &filename);  // QGC's video snapshot
};

/// Stand-in for a QGC Fact: only rawValue.
class FakeFact : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVariant rawValue READ rawValue WRITE setRawValue NOTIFY rawValueChanged)

public:
    explicit FakeFact(const QVariant &value) : _value(value) {}
    QVariant rawValue() const { return _value; }
    void setRawValue(const QVariant &value)
    {
        if (value != _value) {
            _value = value;
            emit rawValueChanged();
        }
    }

signals:
    void rawValueChanged();

private:
    QVariant _value;
};

/// QGC's VideoSettings: what the IR button reads and writes.
class FakeVideoSettings : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QObject *rtspUrl READ rtspUrl CONSTANT)
    Q_PROPERTY(QObject *videoSource READ videoSource CONSTANT)
    Q_PROPERTY(QObject *videoFit READ videoFit CONSTANT)
    Q_PROPERTY(QString rtspVideoSource READ rtspVideoSource CONSTANT)

public:
    // A day address the user set by hand in QGC (not the app's default).
    FakeFact rtspUrlFact{QStringLiteral("rtsp://192.168.144.108:554/main")};
    FakeFact videoSourceFact{QStringLiteral("RTSP Video Stream")};
    FakeFact videoFitFact{1};  // QGC's default, "Fit Height"
    QObject *rtspUrl() { return &rtspUrlFact; }
    QObject *videoSource() { return &videoSourceFact; }
    QObject *videoFit() { return &videoFitFact; }
    QString rtspVideoSource() const { return QStringLiteral("RTSP Video Stream"); }
};

class FakeSettingsManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QObject *videoSettings READ videoSettings CONSTANT)

public:
    QObject *videoSettings() { return &video; }
    FakeVideoSettings video;
};

/// One value that QML reads by name ("isValid", "supported").
class FakeFlag : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool isValid READ value NOTIFY changed)
    Q_PROPERTY(bool supported READ value NOTIFY changed)

public:
    explicit FakeFlag(bool value) : _value(value) {}
    bool value() const { return _value; }
    void set(bool value)
    {
        if (value != _value) {
            _value = value;
            emit changed();
        }
    }

signals:
    void changed();

private:
    bool _value;
};

/// What QGC's VehicleWarnings.qml reads of the active vehicle.
class FakeQmlVehicle : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool requiresGpsFix READ requiresGpsFix CONSTANT)
    Q_PROPERTY(bool armed READ armed CONSTANT)
    Q_PROPERTY(QObject *coordinate READ coordinate CONSTANT)
    Q_PROPERTY(QString prearmError READ prearmError NOTIFY prearmErrorChanged)
    Q_PROPERTY(QObject *healthAndArmingCheckReport READ report CONSTANT)

public:
    bool requiresGpsFix() const { return true; }
    bool armed() const { return false; }
    QObject *coordinate() { return &position; }
    QObject *report() { return &_report; }
    QString prearmError() const { return _prearmError; }
    void setPrearmError(const QString &text)
    {
        _prearmError = text;
        emit prearmErrorChanged();
    }
    FakeFlag position{true};  // a GPS position: no warning

signals:
    void prearmErrorChanged();

private:
    FakeFlag _report{false};  // an ArduPilot that sends pre-arm texts, not the newer report
    QString _prearmError;
};

class FakeQmlVehicleManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QObject *activeVehicle READ activeVehicle CONSTANT)

public:
    QObject *activeVehicle() { return &vehicle; }
    FakeQmlVehicle vehicle;
};

/// What the app menu reads of QGC's options.
class FakeCoreOptions : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool combineSettingsAndSetup READ combineSettingsAndSetup CONSTANT)

public:
    bool combineSettingsAndSetup() const { return false; }  // QGC's default
};

/// QGC's core plugin as the app menu and the admin password panel use it (task Z). The password
/// is checked as CustomPlugin does it, by the app's own AdminPassword with the test build's
/// password, on a clock that the preview moves (nowMs).
class FakeCorePlugin : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool showAdvancedUI READ showAdvancedUI WRITE setShowAdvancedUI NOTIFY showAdvancedUIChanged)
    Q_PROPERTY(bool showTouchAreas MEMBER showTouchAreas NOTIFY showTouchAreasChanged)
    Q_PROPERTY(QObject *options READ options CONSTANT)

public:
    bool showAdvancedUI() const { return _advanced; }
    void setShowAdvancedUI(bool on)
    {
        if (on != _advanced) {
            _advanced = on;
            emit showAdvancedUIChanged(on);
        }
    }
    QObject *options() { return &_options; }

    Q_INVOKABLE bool unlockAdvanced(const QString &password)
    {
        ++tries;
        if (_password.check(password, nowMs) != vama::AdminPassword::Result::Right) {
            return false;
        }
        setShowAdvancedUI(true);
        return true;
    }
    Q_INVOKABLE int adminWaitSeconds() const { return _password.waitSeconds(nowMs); }

    // CustomPlugin's map pack line and signal (task AB).
    Q_PROPERTY(QString mapPackLoading MEMBER mapPackLoading NOTIFY mapPackLoadingChanged)
    void setMapPackLoading(const QString &line)
    {
        mapPackLoading = line;
        emit mapPackLoadingChanged();
    }

    bool showTouchAreas = false;
    qint64 nowMs = 0;
    int tries = 0;  // passwords checked
    QString mapPackLoading;

signals:
    void showAdvancedUIChanged(bool on);
    void showTouchAreasChanged();
    void mapPackLoadingChanged();
    void mapPackLoaded(double latitude, double longitude);

private:
    FakeCoreOptions _options;
    vama::AdminPassword _password{vama::builtInAdminSecret()};
    bool _advanced = false;
};

class FakeGlobal : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QObject *videoManager READ videoManager CONSTANT)
    Q_PROPERTY(QObject *settingsManager READ settingsManager CONSTANT)
    Q_PROPERTY(QObject *multiVehicleManager READ multiVehicleManager CONSTANT)
    Q_PROPERTY(QObject *corePlugin READ corePlugin CONSTANT)
    Q_PROPERTY(QString qgcVersion READ qgcVersion CONSTANT)
    Q_PROPERTY(qreal zOrderTopMost READ zOrderTopMost CONSTANT)
    // QGC's saved map position and zoom (QGroundControlQmlGlobal).
    Q_PROPERTY(QGeoCoordinate flightMapPosition MEMBER flightMapPosition NOTIFY flightMapChanged)
    Q_PROPERTY(double flightMapZoom MEMBER flightMapZoom NOTIFY flightMapChanged)

public:
    QObject *videoManager() { return &_videoManager; }
    qreal zOrderTopMost() const { return 1000; }  // QGC's value
    QObject *settingsManager() { return &settings; }
    QObject *multiVehicleManager() { return &vehicles; }
    QObject *corePlugin() { return &core; }
    QString qgcVersion() const { return QStringLiteral("v5.1.5"); }
    FakeSettingsManager settings;
    FakeQmlVehicleManager vehicles;
    FakeCorePlugin core;
    QGeoCoordinate flightMapPosition;
    double flightMapZoom = 16.0;

signals:
    void flightMapChanged();

private:
    FakeVideoManager _videoManager;
};

/// Answers angle questions and laser reads like the client's V13 (C13). Like it,
/// it ignores long gimbal frames that start with a lower-case "#tp" (test build 3).
/// It turns at the speeds it is told (GSY, GSP) and to the angles it is told
/// (GAY, GAP), and the stand-in video shows what it looks at. Its yaw counts to
/// the left, as the C13 reports it.
class FakeCamera : public QObject
{
    Q_OBJECT
    Q_PROPERTY(double yaw READ yaw NOTIFY lookChanged)
    Q_PROPERTY(double pitch READ pitch NOTIFY lookChanged)
    // The walker in the stand-in video: where it is in the world, in degrees
    // right of straight ahead and above the level line.
    Q_PROPERTY(double walkerRight READ walkerRight NOTIFY walkerChanged)
    Q_PROPERTY(double walkerUp READ walkerUp NOTIFY walkerChanged)
    Q_PROPERTY(bool walkerVisible READ walkerVisible NOTIFY walkerChanged)
    // A car, for the marks: where it stands in the world, as the walker.
    Q_PROPERTY(double carRight READ carRight NOTIFY carChanged)
    Q_PROPERTY(double carUp READ carUp NOTIFY carChanged)
    Q_PROPERTY(bool carVisible READ carVisible NOTIFY carChanged)

public:
    FakeCamera()
    {
        _socket.bind(QHostAddress::LocalHost, 0);
        connect(&_socket, &QUdpSocket::readyRead, this, &FakeCamera::_read);
        connect(&_follow, &QTimer::timeout, this, [this]() { setYaw(_yaw + 0.3); });
        connect(&_motion, &QTimer::timeout, this, &FakeCamera::_move);
        _clock.start();
        _motion.start(20);
    }
    quint16 port() const { return _socket.localPort(); }

    double yaw() const { return _yaw; }
    double pitch() const { return _pitch; }
    void setYaw(double degrees)
    {
        _yaw = std::clamp(degrees, -90.0, 90.0);
        emit lookChanged();
    }
    void setPitch(double degrees)
    {
        _pitch = std::clamp(degrees, -90.0, 10.0);  // the C13 tilts from -90 to +10
        emit lookChanged();
    }
    /// Where the camera looks, in degrees right of straight ahead.
    double lookRight() const { return -_yaw; }

    double walkerRight() const { return _walkerRight; }
    double walkerUp() const { return _walkerUp; }
    bool walkerVisible() const { return _walkerVisible; }
    void placeWalker(double right, double up)
    {
        _walkerRight = right;
        _walkerUp = up;
        _walkerVisible = true;
        emit walkerChanged();
    }
    void setWalkerVisible(bool visible)
    {
        _walkerVisible = visible;
        emit walkerChanged();
    }
    double walkerSpeed = 0.0;  // degrees a second to the right

    double carRight() const { return _carRight; }
    double carUp() const { return _carUp; }
    bool carVisible() const { return _carVisible; }
    void placeCar(double right, double up)
    {
        _carRight = right;
        _carUp = up;
        _carVisible = true;
        emit carChanged();
    }
    void setCarVisible(bool visible)
    {
        _carVisible = visible;
        emit carChanged();
    }

    /// Degrees a second the camera turns now: to the right, and up.
    double yawRate = 0.0;
    double pitchRate = 0.0;
    int angleCommands = 0;  // GAY and GAP frames received (tap aiming)
    int laserFrames = 0;    // SLR frames received: shots and reads
    bool laserAnswers = true;  // false: too far, nothing comes back
    int speedCommands = 0;  // GSY and GSP frames with a speed in them
    int gotFrames = 0;      // the camera's own tracker: GOT
    QString lastGot;        // its data: where the point is, in the camera's count
    int sumConfirms = 0;    // the camera's own tracker: SUM 01
    int sumStops = 0;       // the camera's own tracker: SUM 00

signals:
    void lookChanged();
    void walkerChanged();
    void carChanged();

private slots:
    void _move()
    {
        // At most 0.2 s a tick: on a busy PC this stand-in world slows down with the app, so the
        // preview checks what the app does, not how it fares when it gets too little of the CPU.
        // (Counted in full, 2026-10-15: with 12 busy processes the stand-in camera swung at full
        // speed while the app got few pictures, and mark locks were lost in 4 of 6 runs.)
        const double seconds = std::min(0.2, _clock.restart() / 1000.0);
        if (yawRate != 0.0 || pitchRate != 0.0) {
            _yaw = std::clamp(_yaw - yawRate * seconds, -90.0, 90.0);  // right is a smaller yaw
            _pitch = std::clamp(_pitch + pitchRate * seconds, -90.0, 10.0);
            emit lookChanged();
        }
        if (walkerSpeed != 0.0) {
            _walkerRight += walkerSpeed * seconds;
            emit walkerChanged();
        }
    }

    void _read()
    {
        while (_socket.hasPendingDatagrams()) {
            const QNetworkDatagram d = _socket.receiveDatagram();
            const std::string raw(d.data().constData(), static_cast<size_t>(d.data().size()));
            if (raw.rfind("#tp", 0) == 0) {
                continue;
            }
            const auto f = top::parseTpFrame(raw);
            // The gimbal turns to angle commands at once.
            if (f && (f->tag == "GAY" || f->tag == "GAP") && f->ctrl == 'w' && f->data.size() >= 4) {
                if (const auto angle = top::decodeAttitudeField4(f->data.substr(0, 4))) {
                    if (f->tag == "GAY") {
                        setYaw(*angle);
                    } else {
                        setPitch(*angle);
                    }
                }
                ++angleCommands;
                continue;
            }
            // Speed commands: one signed byte for each axis, in half degrees a second.
            if (f && (f->tag == "GSY" || f->tag == "GSP") && f->ctrl == 'w' && f->data.size() >= 2) {
                const double speed = _speed(f->data.substr(0, 2));
                (f->tag == "GSY" ? yawRate : pitchRate) = speed;
                if (speed != 0.0) {
                    ++speedCommands;
                }
                continue;
            }
            if (f && f->tag == "GSM" && f->ctrl == 'w' && f->data.size() >= 4) {
                yawRate = _speed(f->data.substr(0, 2));
                pitchRate = _speed(f->data.substr(2, 2));
                continue;
            }
            if (f && f->tag == "GOT" && f->ctrl == 'w') {
                ++gotFrames;
                lastGot = QString::fromStdString(f->data);
                continue;
            }
            // After a SUM confirm the camera turns slowly by itself (3 deg/s).
            if (f && f->tag == "SUM" && f->ctrl == 'w') {
                if (f->data == "01") {
                    ++sumConfirms;
                    if (!_follow.isActive()) {
                        _follow.start(100);
                    }
                } else {
                    ++sumStops;
                    _follow.stop();
                }
                continue;
            }
            if (!f || f->ctrl != 'r') {
                continue;
            }
            std::string reply;
            if (f->tag == "GAC") {
                reply = top::buildTpFrame('U', 'r', "GAC",
                                          top::encodeAttitudeField4(_yaw) + top::encodeAttitudeField4(_pitch) +
                                              top::encodeAttitudeField4(0.0),
                                          'G', 1);
            } else if (f->tag == "SLR") {
                ++laserFrames;
                if (laserAnswers && f->address.size() == 2 && f->address[1] == 'E') {
                    reply = top::buildTpFrame('U', 'r', "SLR", "03A0", 'E', 1);  // 92.8 m
                }
            }
            if (!reply.empty()) {
                _socket.writeDatagram(reply.data(), static_cast<qint64>(reply.size()), d.senderAddress(),
                                      d.senderPort());
            }
        }
    }

private:
    static double _speed(const std::string &twoHex)
    {
        int value = 0;
        try {
            value = std::stoi(twoHex, nullptr, 16);
        } catch (...) {
            return 0.0;
        }
        return (value > 127 ? value - 256 : value) * 0.5;
    }

    QUdpSocket _socket;
    QTimer _follow;
    QTimer _motion;
    QElapsedTimer _clock;
    double _yaw = -12.4;
    double _pitch = -31.5;
    double _walkerRight = 0.0;
    double _walkerUp = 0.0;
    bool _walkerVisible = false;
    double _carRight = 0.0;
    double _carUp = 0.0;
    bool _carVisible = false;
};

/// The app's override rule (CustomOverrideInterceptor in CustomPlugin.cc): a
/// qrc address is answered by its copy under /Custom when custom.qrc has one.
/// A stand-in for the drone of Follow Me: the steps say what its telemetry is, and see what it is sent.
class PreviewDrone : public FollowDrone
{
    Q_OBJECT

public:
    using FollowDrone::FollowDrone;

    struct Sent
    {
        double northMps = 0.0;
        double eastMps = 0.0;
        double headingDeg = 0.0;
    };

    QString whyNot() const override { return QString(); }
    QString modeName() const override { return mode; }
    void askGuided() override { ++guidedAsked; }
    void sendSpeed(double northMps, double eastMps, double headingDeg) override { sent.append({northMps, eastMps, headingDeg}); }
    double fliesOnS() const override { return 3.0; }

    void tell(const skydroid::follow::Drone &drone) { emit telemetry(drone); }

    QString mode = QStringLiteral("Loiter");
    int guidedAsked = 0;
    QList<Sent> sent;
};

/// What ObjectFollow::instance() would fly in the app. The preview makes its own ObjectFollow.
FollowDrone *vamaMakeFollowDrone(QObject *parent)
{
    return new PreviewDrone(parent);
}

/// A stand-in for the object finder's network: it "finds" the walker and the car of the
/// stand-in video where they are on the screen (the preview notes that 30 times a second),
/// and takes 0.12 s for a look of 640 x 384 points, longer for a larger one: a device of
/// middle speed.
class PreviewFinder : public PictureFinder
{
public:
    struct Seen
    {
        QMutex mutex;
        std::vector<skydroid::find::Found> things;  // in parts of the picture
        int looks = 0;
    };

    explicit PreviewFinder(std::shared_ptr<Seen> seen) : _seen(std::move(seen)) {}

    bool load(QString &) override { return true; }

    std::vector<skydroid::find::Found> find(const skydroid::find::Picture &picture, const skydroid::find::Ask &ask,
                                            QString &) const override
    {
        std::vector<skydroid::find::Found> things;
        {
            QMutexLocker lock(&_seen->mutex);
            things = _seen->things;
            ++_seen->looks;
        }
        skydroid::find::Area area = ask.area;
        if (area.width <= 0 || area.height <= 0) {
            area = skydroid::find::Area{0, 0, picture.width, picture.height};
        }
        const skydroid::find::Fit fit = skydroid::find::fitArea(area.width, area.height, ask.inputW, ask.inputH, ask.keepShape);
        QThread::msleep(static_cast<unsigned long>(120.0 * fit.inputW * fit.inputH / (640.0 * 384.0)));
        std::vector<skydroid::find::Found> found;
        for (skydroid::find::Found thing : things) {
            // In points of the picture, and what of it lies in the part looked at.
            const double left = std::max((thing.cx - thing.w / 2.0) * picture.width, static_cast<double>(area.left));
            const double right = std::min((thing.cx + thing.w / 2.0) * picture.width, static_cast<double>(area.left + area.width));
            const double top = std::max((thing.cy - thing.h / 2.0) * picture.height, static_cast<double>(area.top));
            const double bottom = std::min((thing.cy + thing.h / 2.0) * picture.height, static_cast<double>(area.top + area.height));
            if (right - left < 8.0 || bottom - top < 8.0) {
                continue;
            }
            thing.cx = (left + right) / 2.0;
            thing.cy = (top + bottom) / 2.0;
            thing.w = right - left;
            thing.h = bottom - top;
            found.push_back(thing);
        }
        return found;
    }

private:
    std::shared_ptr<Seen> _seen;
};

/// What ObjectMarks::instance() would look with in the app. The preview makes its own ObjectMarks.
PictureFinder *vamaMakePictureFinder()
{
    return new PreviewFinder(std::make_shared<PreviewFinder::Seen>());
}

QString vamaFinderName()
{
    return QStringLiteral("a stand-in for the preview");
}

/// The notices as the app reads them: from the app's own resources (custom.qrc is in the preview).
QString vamaFinderNotices()
{
    QFile file(QStringLiteral(":/custom/ai/NOTICES.txt"));
    return file.open(QIODevice::ReadOnly) ? QString::fromUtf8(file.readAll()) : QString();
}

class OverrideInterceptor : public QQmlAbstractUrlInterceptor
{
public:
    QUrl intercept(const QUrl &url, QQmlAbstractUrlInterceptor::DataType type) override
    {
        switch (type) {
        case QQmlAbstractUrlInterceptor::QmlFile:
        case QQmlAbstractUrlInterceptor::UrlString:
            if (url.scheme() == QStringLiteral("qrc")) {
                const QString origPath = url.path();
                const QString overrideRes = QStringLiteral(":/Custom%1").arg(origPath);
                if (QFile::exists(overrideRes)) {
                    const QString relPath = overrideRes.mid(2);
                    QUrl result;
                    result.setScheme(QStringLiteral("qrc"));
                    result.setPath('/' + relPath);
                    return result;
                }
            }
            break;
        default:
            break;
        }
        return url;
    }
};

int main(int argc, char *argv[])
{
    // Every line reaches the log at once: a crash must not take the last lines with it.
    std::setvbuf(stderr, nullptr, _IONBF, 0);
    qputenv("QT_QUICK_CONTROLS_STYLE", "Basic");
    qInstallMessageHandler(messageHandler);
    // QGC's image provider logs every icon it draws.
    QLoggingCategory::setFilterRules(QStringLiteral("QmlControls.ColoredSvgImageProvider.debug=false"));
    QGuiApplication app(argc, argv);
    QCoreApplication::setOrganizationName(QStringLiteral("VamaPreview"));
    QCoreApplication::setApplicationName(QStringLiteral("VamaPreview"));
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, QDir::tempPath() + "/vama-preview-settings");
    QSettings().remove(QStringLiteral("VamaSkydroid"));
    QSettings().remove(QStringLiteral("VamaFollow"));
    QSettings().remove(QStringLiteral("VamaMarks"));

    const QString outDir = argc > 1 ? QString::fromLocal8Bit(argv[1]) : QDir::currentPath();

    Vehicle vehicle;
    vehicle.position = QGeoCoordinate(18.520430, 73.856743);
    vehicle.gps.set("lock", 3);
    vehicle.vehicle.set("heading", 40.0);
    vehicle.vehicle.set("roll", 0.0);
    vehicle.vehicle.set("pitch", 0.0);
    vehicle.vehicle.set("altitudeAMSL", 620.0);
    MultiVehicleManager::instance()->setActiveVehicle(&vehicle);

    FakeCamera camera;
    SkydroidLink::setProbeTargets({}, {});  // no real network from a preview
    // Added cameras (task AA) in a folder of the preview's own, empty at the start.
    const QString cameraFolder = QDir::tempPath() + QStringLiteral("/vama-preview-cameras");
    QDir(cameraFolder).removeRecursively();
    SkydroidLink::setCameraFolder(cameraFolder);
    SkydroidLink link(nullptr);
    link.setHost(QStringLiteral("127.0.0.1"));
    link.setPort(camera.port());
    link.setEnabled(true);

    // Follow Me, with a stand-in drone that hovers where the vehicle is, in Loiter, its nose
    // to 40 degrees. Its telemetry comes five times a second while the timer runs.
    auto *drone = new PreviewDrone;
    ObjectFollow follow(&link, drone);
    skydroid::follow::Drone telemetry;
    telemetry.latDeg = vehicle.position.latitude();
    telemetry.lonDeg = vehicle.position.longitude();
    telemetry.heightM = 60.0;
    telemetry.northMps = 0.0;
    telemetry.eastMps = 0.0;
    telemetry.headingDeg = 40.0;
    telemetry.armed = true;
    telemetry.flying = true;
    telemetry.gps3d = true;
    telemetry.mode = skydroid::follow::Mode::Holding;
    QTimer telemetryTimer;
    telemetryTimer.setInterval(200);
    QObject::connect(&telemetryTimer, &QTimer::timeout, drone, [&]() { drone->tell(telemetry); });
    int guidedBefore = 0;

    // The marks on people and vehicles, with a stand-in finder on its own thread, as in the app.
    auto seen = std::make_shared<PreviewFinder::Seen>();
    ObjectMarks marks(&link, new PreviewFinder(seen), true);

    FakeGlobal global;
    qmlRegisterSingletonInstance("QGroundControl", 1, 0, "SkydroidLink", &link);
    qmlRegisterSingletonInstance("QGroundControl", 1, 0, "ObjectFollow", &follow);
    qmlRegisterSingletonInstance("QGroundControl", 1, 0, "ObjectMarks", &marks);
    qmlRegisterSingletonInstance("QGroundControl", 1, 0, "QGroundControl", &global);

    OverrideInterceptor interceptor;  // declared first: the engine must not outlive it
    QQmlApplicationEngine engine;
    engine.addUrlInterceptor(&interceptor);
    engine.addImageProvider(QLatin1String(ColoredSvgImageProvider::ProviderId), new ColoredSvgImageProvider);
    engine.addImportPath(QStringLiteral(VAMA_PREVIEW_DIR "/stubs"));
    engine.rootContext()->setContextProperty(QStringLiteral("previewCamera"), &camera);
    engine.load(QUrl::fromLocalFile(QStringLiteral(VAMA_PREVIEW_DIR "/main.qml")));
    if (engine.rootObjects().isEmpty()) {
        return 2;
    }
    auto *window = qobject_cast<QQuickWindow *>(engine.rootObjects().first());
    if (!window) {
        return 3;
    }

    auto shot = [&](const char *name) {
        const QImage image = window->grabWindow();
        const QString path = outDir + "/" + QString::fromLatin1(name);
        image.save(path);
        std::fprintf(stderr, "saved %s\n", qPrintable(path));
    };
    // With VAMA_PREVIEW_TOUCH set (run_preview.ps1 -Touch), every press, move and release is a
    // touch of one finger, as on the remote control, through Qt's own touch path. (Client video
    // 2026-10-15, a tap on a mark that locked nothing: every check passed with touches too.)
    QPointingDevice *touchScreen = qEnvironmentVariableIsSet("VAMA_PREVIEW_TOUCH") ? QTest::createTouchDevice() : nullptr;
    auto mouseOn = [&](QQuickWindow *target, QEvent::Type type, QPointF pos) {
        if (touchScreen) {
            if (type == QEvent::MouseButtonPress) {
                QTest::touchEvent(target, touchScreen).press(0, pos.toPoint(), target);
            } else if (type == QEvent::MouseMove) {
                QTest::touchEvent(target, touchScreen).move(0, pos.toPoint(), target);
            } else {
                QTest::touchEvent(target, touchScreen).release(0, pos.toPoint(), target);
            }
            return;
        }
        const Qt::MouseButtons buttons = type == QEvent::MouseButtonRelease ? Qt::NoButton : Qt::LeftButton;
        const Qt::MouseButton button = type == QEvent::MouseMove ? Qt::NoButton : Qt::LeftButton;
        QMouseEvent event(type, pos, target->mapToGlobal(pos), button, buttons, Qt::NoModifier);
        QCoreApplication::sendEvent(target, &event);
    };
    auto mouse = [&](QEvent::Type type, QPointF pos) { mouseOn(window, type, pos); };
    // Press and release in the middle of the item with this objectName.
    auto tap = [&](const char *objectName) {
        auto *item = window->findChild<QQuickItem *>(QString::fromLatin1(objectName));
        if (!item) {
            std::fprintf(stderr, "%s not found\n", objectName);
            ++g_problems;
            return;
        }
        const QPointF centre = item->mapToScene(QPointF(item->width() / 2, item->height() / 2));
        mouse(QEvent::MouseButtonPress, centre);
        mouse(QEvent::MouseButtonRelease, centre);
    };
    auto expect = [&](bool ok, const char *what, const QString &got) {
        std::fprintf(stderr, "%s %s (%s)\n", ok ? "OK  " : "FAIL", what, qPrintable(got));
        if (!ok) {
            ++g_problems;
        }
    };

    int anglesBeforeLock = 0;
    int anglesBeforePoint = 0;  // a point from the picture: the camera must not be turned,
    int lasersBeforePoint = 0;  // and the laser not asked
    int laserFramesBefore = 0;  // the laser button's steps (task Y)

    // Application Settings (client feedback 2026-10-07, tasks E and F).
    QQuickWindow *settingsWindow = nullptr;
    std::function<void(QQuickItem *, QList<QQuickItem *> &)> collect = [&](QQuickItem *item, QList<QQuickItem *> &out) {
        out.append(item);
        for (QQuickItem *child : item->childItems()) {
            collect(child, out);
        }
    };
    auto openAppSettings = [&]() {
        const qsizetype windows = engine.rootObjects().size();
        engine.load(QUrl::fromLocalFile(QStringLiteral(VAMA_PREVIEW_DIR "/settings.qml")));
        if (engine.rootObjects().size() == windows ||
            !(settingsWindow = qobject_cast<QQuickWindow *>(engine.rootObjects().last()))) {
            std::fprintf(stderr, "settings window not loaded\n");
            ++g_problems;
            return;
        }
        // As MainWindow.showSettingsTool() does: QGC's own address.
        QMetaObject::invokeMethod(settingsWindow, "showSettingsTool", Q_ARG(QVariant, QVariant(QStringLiteral(""))));
    };
    auto checkAppSettings = [&]() {
        if (!settingsWindow) {
            return;
        }
        QList<QQuickItem *> items;
        collect(settingsWindow->contentItem(), items);
        QObject *screen = nullptr;
        QQuickItem *general = nullptr;
        QQuickItem *help = nullptr;
        QStringList shown;
        int dividers = 0;
        for (QQuickItem *item : std::as_const(items)) {
            const QString name = item->objectName();
            if (name == QStringLiteral("toolDrawerLoader")) {
                screen = item->property("item").value<QObject *>();
            } else if (name.startsWith(QStringLiteral("settingsButton_"))) {
                const QString page = name.mid(15);
                if (page == QStringLiteral("General")) {
                    general = item;
                } else if (page == QStringLiteral("Help")) {
                    help = item;
                }
                if (item->isVisible()) {
                    shown.append(page);
                }
            } else if (name.startsWith(QStringLiteral("settingsDivider_")) && item->isVisible()) {
                ++dividers;
            }
        }
        // The preview has no QGC AppSettings.qml, so anything that loads is the copy.
        expect(screen && screen->property("_vamaLeftOutPages").isValid(),
               "settings: QGC's address opened the VAMA copy (custom.qrc)",
               screen ? QString::fromLatin1(screen->metaObject()->className()) : QStringLiteral("nothing loaded"));
        if (!screen) {
            return;
        }

        // Task E: the General button shows the VAMA mark, drawn by QGC's tinting.
        QQuickItem *content = general ? general->property("contentItem").value<QQuickItem *>() : nullptr;
        QQuickItem *icon = (content && !content->childItems().isEmpty()) ? content->childItems().first() : nullptr;
        QQuickItem *image = (icon && !icon->childItems().isEmpty()) ? icon->childItems().first() : nullptr;
        const QString iconSource = icon ? icon->property("source").toUrl().toString() : QString();
        const int imageStatus = image ? image->property("status").toInt() : -1;
        expect(general && general->isVisible() && iconSource == QStringLiteral("/custom/img/vama_logo_white.svg") &&
                   imageStatus == 1,
               "settings: the General page shows the VAMA mark (task E)",
               QStringLiteral("%1, image status %2").arg(iconSource).arg(imageStatus));

        // Task F: no Help page, every other page of the release APK, and no
        // divider left over around Help.
        expect(help && !help->isVisible(), "settings: no Help page (task F)",
               help ? (help->isVisible() ? QStringLiteral("shown") : QStringLiteral("hidden")) : QStringLiteral("no Help button"));
        const QStringList pages = {
            QStringLiteral("General"), QStringLiteral("Fly View"), QStringLiteral("3D View"), QStringLiteral("Plan View"),
            QStringLiteral("ADSB Server"), QStringLiteral("Comm Links"), QStringLiteral("App Logging"),
            QStringLiteral("App Log Viewer"), QStringLiteral("Maps"), QStringLiteral("NTRIP/RTK"),
            QStringLiteral("Remote ID"), QStringLiteral("Telemetry"), QStringLiteral("Video")};
        expect(shown == pages, "settings: QGC's other pages are all there", shown.join(QStringLiteral(", ")));
        expect(dividers == 1, "settings: one divider, none left over by the Help page", QString::number(dividers));

        const QImage picture = settingsWindow->grabWindow();
        const QString path = outDir + QStringLiteral("/17_app_settings.png");
        picture.save(path);
        std::fprintf(stderr, "saved %s\n", qPrintable(path));

        // Opening Help by name (showSettingsTool("Help")) must do nothing either.
        QMetaObject::invokeMethod(settingsWindow, "showSettingsTool", Q_ARG(QVariant, QVariant(QStringLiteral("Help"))));
        expect(screen->property("_selectedPageIndex").toInt() == 0, "settings: Help cannot be opened by name either",
               QStringLiteral("page %1").arg(screen->property("_selectedPageIndex").toInt()));
    };

    // The app menu with the admin password for Advanced Mode (task Z): menu.qml is QGC's drawer
    // around the real SelectViewDropdown.qml, made when it opens and destroyed when it closes.
    QQuickWindow *menuWindow = nullptr;
    auto openMenu = [&]() {
        if (!menuWindow) {
            const qsizetype windows = engine.rootObjects().size();
            engine.load(QUrl::fromLocalFile(QStringLiteral(VAMA_PREVIEW_DIR "/menu.qml")));
            if (engine.rootObjects().size() == windows ||
                !(menuWindow = qobject_cast<QQuickWindow *>(engine.rootObjects().last()))) {
                std::fprintf(stderr, "menu window not loaded\n");
                ++g_problems;
                return;
            }
        }
        QMetaObject::invokeMethod(menuWindow, "openMenu");
    };
    auto menuOpen = [&]() { return menuWindow && menuWindow->property("menuOpen").toBool(); };
    auto menuItem = [&](const char *objectName) -> QQuickItem * {
        return menuWindow ? menuWindow->findChild<QQuickItem *>(QString::fromLatin1(objectName)) : nullptr;
    };
    auto menuTapAt = [&](QPointF pos) {
        mouseOn(menuWindow, QEvent::MouseButtonPress, pos);
        mouseOn(menuWindow, QEvent::MouseButtonRelease, pos);
    };
    auto menuTap = [&](const char *objectName) {
        QQuickItem *item = menuItem(objectName);
        if (!item) {
            std::fprintf(stderr, "%s not found in the menu\n", objectName);
            ++g_problems;
            return;
        }
        menuTapAt(item->mapToScene(QPointF(item->width() / 2, item->height() / 2)));
    };
    // Keys as the keyboard sends them to the text field.
    auto typeInto = [&](QQuickItem *field, const QString &text) {
        for (const QChar c : text) {
            QKeyEvent press(QEvent::KeyPress, 0, Qt::NoModifier, QString(c));
            QCoreApplication::sendEvent(field, &press);
            QKeyEvent release(QEvent::KeyRelease, 0, Qt::NoModifier, QString(c));
            QCoreApplication::sendEvent(field, &release);
        }
    };
    auto pressEnter = [&](QQuickItem *field) {
        QKeyEvent press(QEvent::KeyPress, Qt::Key_Return, Qt::NoModifier);
        QCoreApplication::sendEvent(field, &press);
        QKeyEvent release(QEvent::KeyRelease, Qt::Key_Return, Qt::NoModifier);
        QCoreApplication::sendEvent(field, &release);
    };
    auto shotOf = [&](QQuickWindow *target, const char *name) {
        const QString path = outDir + "/" + QString::fromLatin1(name);
        target->grabWindow().save(path);
        std::fprintf(stderr, "saved %s\n", qPrintable(path));
    };
    auto menuText = [&](const char *objectName) {
        QQuickItem *item = menuItem(objectName);
        return item ? item->property("text").toString() : QStringLiteral("not there");
    };

    // The steps, one after another (delay in ms before each).
    // Thermal colours (client request 2026-10-07).
    auto findItem = [&](const char *objectName) -> QQuickItem * {
        QList<QQuickItem *> items;
        collect(window->contentItem(), items);
        for (QQuickItem *item : std::as_const(items)) {
            if (item->objectName() == QLatin1String(objectName)) {
                return item;
            }
        }
        return nullptr;
    };
    // An item's property set, or a problem when the item is not there.
    auto setOn = [&](const char *objectName, const char *property, const QVariant &value) {
        QQuickItem *item = findItem(objectName);
        if (!item) {
            std::fprintf(stderr, "%s not found\n", objectName);
            ++g_problems;
            return;
        }
        item->setProperty(property, value);
    };
    // A button's press as the button itself gives it (clicked), for a button that can be outside
    // the visible part of a window that scrolls.
    auto press = [&](const char *objectName) {
        QQuickItem *item = findItem(objectName);
        if (!item) {
            std::fprintf(stderr, "%s not found\n", objectName);
            ++g_problems;
            return;
        }
        QMetaObject::invokeMethod(item, "clicked");
    };
    auto tapItem = [&](QQuickItem *item) {
        const QPointF centre = item->mapToScene(QPointF(item->width() / 2, item->height() / 2));
        mouse(QEvent::MouseButtonPress, centre);
        mouse(QEvent::MouseButtonRelease, centre);
    };
    // A tap on an item found among the window's items (the rows of a Repeater are not found by
    // their QObject parents).
    auto tapFound = [&](const char *objectName) {
        QQuickItem *item = findItem(objectName);
        if (!item) {
            std::fprintf(stderr, "%s not found\n", objectName);
            ++g_problems;
            return;
        }
        tapItem(item);
    };
    // Steps run on fixed timers, but the app works in real time: on a busy PC a picture, a
    // grab or a turn of the stand-in camera comes later. waitFor lets events run (as QTest's
    // qWaitFor does) until ready() is true, at most maxMs. It says whether ready() came true.
    auto waitFor = [&](const std::function<bool()> &ready, int maxMs) -> bool {
        QElapsedTimer waited;
        waited.start();
        while (!ready()) {
            if (waited.elapsed() >= maxMs) {
                return false;
            }
            QEventLoop slice;
            QTimer::singleShot(20, &slice, &QEventLoop::quit);
            slice.exec();
        }
        return true;
    };
    // The lock button toggles: with a lock on it stops it, without one it starts or ends
    // picking (the hint shows while it picks). One press in the wrong state turned every press
    // after it the other way: a whole run failed with 34 problems (2026-10-15). So each press
    // says what it is for, finds the state it needs (and says so when a step before left
    // another one), and waits for the result.
    auto picking = [&]() {
        QQuickItem *hint = findItem("vamaLockHint");
        return hint && hint->isVisible();
    };
    auto lockState = [&]() {
        return QStringLiteral("lock %1, busy %2, picking %3: %4")
            .arg(link.lockActive()).arg(link.lockBusy()).arg(picking()).arg(link.lockMessage());
    };
    // Press the lock button to pick an object: no lock and no picking before, picking after.
    auto pressLockToPick = [&](const char *what) {
        if (link.lockActive() || link.lockBusy() || picking()) {
            expect(false, what, QStringLiteral("before the press: ") + lockState());
            link.stopLock();
            if (picking()) {
                tap("vamaLockButton");
            }
            waitFor([&]() { return !link.lockActive() && !link.lockBusy() && !picking(); }, 1000);
        }
        tap("vamaLockButton");
        if (!waitFor(picking, 1000)) {
            expect(false, what, QStringLiteral("after the press: ") + lockState());
        }
    };
    // Press the lock button to stop the lock: a lock before, none after.
    auto pressLockToStop = [&](const char *what) {
        if (!link.lockActive() && !link.lockBusy()) {
            expect(false, what, QStringLiteral("before the press, no lock: ") + lockState());
            if (picking()) {
                tap("vamaLockButton");
            }
            return;
        }
        tap("vamaLockButton");
        if (!waitFor([&]() { return !link.lockActive() && !link.lockBusy(); }, 1000)) {
            expect(false, what, QStringLiteral("after the press: ") + lockState());
        }
    };
    // Press the lock button to end picking: picking before, none after.
    auto pressLockToEndPicking = [&](const char *what) {
        if (!picking() || link.lockActive() || link.lockBusy()) {
            expect(false, what, QStringLiteral("before the press: ") + lockState());
            link.stopLock();
            if (picking()) {
                tap("vamaLockButton");
            }
            return;
        }
        tap("vamaLockButton");
        if (!waitFor([&]() { return !picking(); }, 1000)) {
            expect(false, what, QStringLiteral("after the press: ") + lockState());
        }
    };
    // The video alone: everything over it hidden for the grab.
    auto grabVideo = [&]() {
        window->setProperty("videoOnly", true);
        const QImage image = window->grabWindow();
        window->setProperty("videoOnly", false);
        return image;
    };
    // The lock: where an item is on the screen, and how far the walker is from the cross.
    auto sceneRect = [&](const char *objectName) -> QRectF {
        QQuickItem *item = findItem(objectName);
        return item ? item->mapRectToScene(QRectF(0, 0, item->width(), item->height())) : QRectF();
    };
    // In degrees: right of the cross and above it.
    auto walkerOff = [&]() -> QPointF {
        return QPointF(camera.walkerRight() - camera.lookRight(), camera.walkerUp() - camera.pitch());
    };
    auto offText = [&]() {
        const QPointF off = walkerOff();
        return QStringLiteral("%1 deg right, %2 deg up of the cross").arg(off.x(), 0, 'f', 1).arg(off.y(), 0, 'f', 1);
    };
    // The lock box on the screen against the walker on the screen.
    auto boxOnWalker = [&]() -> bool {
        const QRectF box = sceneRect("vamaLockBox");
        const QRectF walker = sceneRect("previewWalker");
        return !box.isEmpty() && !walker.isEmpty() && box.contains(walker.center()) &&
               std::abs(box.center().x() - walker.center().x()) < walker.width() &&
               std::abs(box.center().y() - walker.center().y()) < walker.height() / 2;
    };
    auto boxText = [&]() {
        const QRectF box = sceneRect("vamaLockBox");
        const QRectF walker = sceneRect("previewWalker");
        return QStringLiteral("box centre %1, %2; walker centre %3, %4")
            .arg(box.center().x(), 0, 'f', 0).arg(box.center().y(), 0, 'f', 0)
            .arg(walker.center().x(), 0, 'f', 0).arg(walker.center().y(), 0, 'f', 0);
    };
    auto boxLabel = [&]() {
        QQuickItem *label = findItem("vamaLockBoxLabel");
        return label ? label->property("text").toString() : QString();
    };
    double lookAtLock = 0.0;
    QRectF walkerOnScreen;
    int speedsBefore = 0;
    QPointF slideFrom;  // where a finger went down that slides a little
    QPointF lockButtonAt;  // the middle of the lock button, for a long press

    // The marks. Every item with this name that shows on the screen.
    auto findItems = [&](const char *objectName) -> QList<QQuickItem *> {
        QList<QQuickItem *> items;
        collect(window->contentItem(), items);
        QList<QQuickItem *> named;
        for (QQuickItem *item : std::as_const(items)) {
            if (item->objectName() == QLatin1String(objectName) && item->isVisible()) {
                named.append(item);
            }
        }
        return named;
    };
    // The mark of a kind on the screen (the first one), or an empty rectangle.
    auto markRect = [&](const char *kind) -> QRectF {
        for (QQuickItem *mark : findItems("vamaMark")) {
            if (mark->property("person").toBool() == (QLatin1String(kind) == QLatin1String("person"))) {
                return mark->mapRectToScene(QRectF(0, 0, mark->width(), mark->height()));
            }
        }
        return QRectF();
    };
    // A mark of this kind lies on the object: over its middle, and about its size.
    auto markOn = [&](const char *objectName, const char *kind) -> bool {
        const QRectF object = sceneRect(objectName);
        const QRectF mark = markRect(kind);
        return !object.isEmpty() && !mark.isEmpty() && mark.contains(object.center()) &&
               mark.width() < object.width() * 1.5 && mark.height() < object.height() * 1.5 &&
               mark.width() > object.width() * 0.6 && mark.height() > object.height() * 0.6;
    };
    auto marksText = [&]() {
        const QRectF person = markRect("person");
        const QRectF vehicle = markRect("vehicle");
        const QRectF walker = sceneRect("previewWalker");
        const QRectF car = sceneRect("previewCar");
        return QStringLiteral("person mark %1, %2 (%3 x %4), walker %5, %6; vehicle mark %7, %8, car %9, %10")
            .arg(person.center().x(), 0, 'f', 0).arg(person.center().y(), 0, 'f', 0).arg(person.width(), 0, 'f', 0)
            .arg(person.height(), 0, 'f', 0).arg(walker.center().x(), 0, 'f', 0).arg(walker.center().y(), 0, 'f', 0)
            .arg(vehicle.center().x(), 0, 'f', 0).arg(vehicle.center().y(), 0, 'f', 0).arg(car.center().x(), 0, 'f', 0)
            .arg(car.center().y(), 0, 'f', 0);
    };
    // What the stand-in finder is to find: the walker and the car where they are on the screen now.
    QQuickItem *walkerItem = findItem("previewWalker");
    QQuickItem *carItem = findItem("previewCar");
    QTimer seenTimer;
    seenTimer.setInterval(30);
    QObject::connect(&seenTimer, &QTimer::timeout, &app, [&]() {
        std::vector<skydroid::find::Found> things;
        const auto add = [&](QQuickItem *item, skydroid::find::Kind kind) {
            if (!item || !item->isVisible() || window->width() <= 0 || window->height() <= 0) {
                return;
            }
            const QRectF r = item->mapRectToScene(QRectF(0, 0, item->width(), item->height()));
            skydroid::find::Found thing;
            thing.kind = kind;
            thing.label = kind == skydroid::find::Kind::Person ? 0 : 2;
            thing.score = kind == skydroid::find::Kind::Person ? 0.8 : 0.7;
            thing.cx = r.center().x() / window->width();
            thing.cy = r.center().y() / window->height();
            thing.w = r.width() / window->width();
            thing.h = r.height() / window->height();
            things.push_back(thing);
        };
        add(walkerItem, skydroid::find::Kind::Person);
        add(carItem, skydroid::find::Kind::Vehicle);
        QMutexLocker lock(&seen->mutex);
        seen->things = things;
    });
    seenTimer.start();

    QImage thermalPlain;  // the thermal picture as the camera sends it
    // The 64 grey bands of the stand-in's thermal picture against VGCS's table
    // for the mode in use, and the black bar beside the picture.
    auto checkThermalColours = [&]() {
        QQuickItem *picture = findItem("previewThermalPicture");
        QQuickItem *video = findItem("videoContent");
        if (!picture || !video || thermalPlain.isNull()) {
            std::fprintf(stderr, "thermal picture not found\n");
            ++g_problems;
            return;
        }
        // VGCS's row for the mode in use, found by its id: a link whose list
        // drifted from VGCS's order would colour with the wrong row.
        int index = -1;
        for (int i = 0; i < kThermalPaletteCount; ++i) {
            if (link.thermalPalette() == QLatin1String(kThermalPaletteIds[i])) {
                index = i;
            }
        }
        if (index < 0) {
            std::fprintf(stderr, "thermal colours: %s is not a VGCS mode\n", qPrintable(link.thermalPalette()));
            ++g_problems;
            return;
        }
        const int row = video->property("_vamaPaletteRow").toInt();
        const QImage now = grabVideo();
        const qreal dpr = now.devicePixelRatio();
        auto pixel = [&](const QImage &image, qreal x, qreal y) {
            return image.pixelColor(qRound(x * dpr), qRound(y * dpr));
        };
        const QRectF r = picture->mapRectToScene(QRectF(0, 0, picture->width(), picture->height()));
        int wrong = 0;
        QString first;
        for (int band = 0; band < 64; ++band) {
            const qreal x = r.x() + (band + 0.5) * r.width() / 64;
            const qreal y = r.y() + r.height() / 4;
            const QColor plain = pixel(thermalPlain, x, y);
            const QColor got = pixel(now, x, y);
            const int level = plain.red();
            const unsigned char *want = kThermalPaletteColours[index][level];
            const bool grey = plain.green() == level && plain.blue() == level;
            if (!grey || qAbs(got.red() - want[0]) > 1 || qAbs(got.green() - want[1]) > 1 ||
                qAbs(got.blue() - want[2]) > 1) {
                if (wrong++ == 0) {
                    first = QStringLiteral("grey %1: %2 %3 %4, VGCS %5 %6 %7")
                                .arg(level).arg(got.red()).arg(got.green()).arg(got.blue())
                                .arg(want[0]).arg(want[1]).arg(want[2]);
                }
            }
        }
        const QColor bar = pixel(now, r.x() / 2, r.y() + r.height() / 2);
        const bool barBlack = bar.red() == 0 && bar.green() == 0 && bar.blue() == 0;
        const QByteArray what = "thermal colours: " + link.thermalPalette().toUtf8() +
                                " is VGCS's table at 64 grey levels, the bars stay black";
        expect(wrong == 0 && barBlack && row == index, what.constData(),
               wrong ? QStringLiteral("%1 wrong, first %2").arg(wrong).arg(first)
                     : QStringLiteral("row %1, bar %2 %3 %4").arg(row).arg(bar.red()).arg(bar.green()).arg(bar.blue()));
    };

    const QList<std::pair<int, std::function<void()>>> steps = {
        {1500, [&]() {
             link.zoom(1);
             link.zoom(1);
             link.zoom(1);
             // Tap the photo button to run its click handler.
             tap("vamaPhotoButton");
         }},
        // The laser button (task Y, Nish 2026-10-16): a press measures and shows the reading, the
        // next press hides it without a shot, and the press after that measures again.
        {500, [&]() {
             laserFramesBefore = camera.laserFrames;
             tap("vamaLaserButton");
         }},
        {1200, [&]() {
             QQuickItem *box = findItem("vamaResultBox");
             expect(link.laserValid() && box && box->isVisible() && camera.laserFrames > laserFramesBefore,
                    "laser button: a press measures and shows the reading", QStringLiteral("%1 m").arg(link.laserRangeM()));
             laserFramesBefore = camera.laserFrames;
             tap("vamaLaserButton");
         }},
        {500, [&]() {
             QQuickItem *box = findItem("vamaResultBox");
             expect(box && !box->isVisible() && camera.laserFrames == laserFramesBefore,
                    "laser button: the next press hides the reading, and does not measure",
                    QStringLiteral("%1 laser frames").arg(camera.laserFrames - laserFramesBefore));
             tap("vamaLaserButton");
         }},
        {1200, [&]() {
             QQuickItem *box = findItem("vamaResultBox");
             expect(link.laserValid() && box && box->isVisible() && camera.laserFrames > laserFramesBefore,
                    "laser button: the press after that measures again, and shows it", QString());
             tap("vamaLaserButton");  // hidden again for the steps after
         }},
        {500, [&]() { shot("1_main.png"); link.fireLaser(); }},
        {1500, [&]() { shot("2_laser.png"); mouse(QEvent::MouseButtonPress, QPointF(1000, 560)); }},
        {100, [&]() { mouse(QEvent::MouseMove, QPointF(1040, 550)); }},
        {100, [&]() { mouse(QEvent::MouseMove, QPointF(1150, 470)); }},
        {300, [&]() { shot("3_drag.png"); mouse(QEvent::MouseButtonRelease, QPointF(1150, 470)); }},
        {300, [&]() { tap("vamaIrButton"); }},
        {300, [&]() {
             // IR on: QGC shows the thermal stream, and the day address set in QGC is kept.
             const QString url = global.settings.video.rtspUrlFact.rawValue().toString();
             expect(url == link.thermalVideoUrl(), "IR on: QGC video is the thermal stream", url);
             expect(link.thermalPicture(), "IR on: the link is told that the thermal picture shows (it has another lens)", QString());
             expect(link.dayVideoUrl() == QStringLiteral("rtsp://192.168.144.108:554/main"),
                    "IR on: day address learned from QGC", link.dayVideoUrl());
             shot("7_ir_on.png");
             // Thermal colours: the button under IR shows only now, and opens the list.
             thermalPlain = grabVideo();
             // Beside IR, so the left column keeps its height (a sixth button in
             // it reached the small map in the corner).
             QQuickItem *button = findItem("vamaPaletteButton");
             QQuickItem *ir = findItem("vamaIrButton");
             const QRectF b = button ? button->mapRectToScene(QRectF(0, 0, button->width(), button->height())) : QRectF();
             const QRectF i = ir ? ir->mapRectToScene(QRectF(0, 0, ir->width(), ir->height())) : QRectF();
             expect(button && ir && button->isVisible() && qAbs(b.center().y() - i.center().y()) < 1 && b.left() > i.right(),
                    "IR on: the thermal colour button shows, beside IR",
                    QStringLiteral("button %1,%2 IR %3,%4").arg(b.x()).arg(b.y()).arg(i.x()).arg(i.y()));
             if (button) {
                 tapItem(button);
             }
         }},
        {500, [&]() {
             QObject *popup = window->findChild<QObject *>(QStringLiteral("vamaPalettePopup"));
             expect(popup && popup->property("opened").toBool(), "thermal colours: the button opens the list", QString());
             shot("8_thermal_colours_menu.png");
             QQuickItem *row = findItem("vamaPalette_ironbow");
             if (row) {
                 tapItem(row);
             } else {
                 std::fprintf(stderr, "vamaPalette_ironbow not found\n");
                 ++g_problems;
             }
         }},
        {500, [&]() {
             QObject *popup = window->findChild<QObject *>(QStringLiteral("vamaPalettePopup"));
             expect(link.thermalPalette() == QStringLiteral("ironbow") && popup && !popup->property("visible").toBool(),
                    "thermal colours: Ironbow picked in the list, the list closed", link.thermalPalette());
             shot("8_thermal_ironbow.png");
             checkThermalColours();
             link.setThermalPalette(QStringLiteral("black_hot"));
         }},
        {300, [&]() { checkThermalColours(); link.setThermalPalette(QStringLiteral("rainbow")); }},
        {300, [&]() { checkThermalColours(); link.setThermalPalette(QStringLiteral("red_hot")); }},
        {300, [&]() { checkThermalColours(); link.setThermalPalette(QStringLiteral("green")); }},
        {300, [&]() { checkThermalColours(); link.setThermalPalette(QStringLiteral("sepia")); }},
        {300, [&]() { checkThermalColours(); link.setThermalPalette(QStringLiteral("camera")); }},
        {300, [&]() {
             checkThermalColours();  // as received: the picture untouched
             // IR off with a colour mode chosen: the day picture must stay as it is.
             link.setThermalPalette(QStringLiteral("ironbow"));
             tap("vamaIrButton");
         }},
        {300, [&]() {
             const QString url = global.settings.video.rtspUrlFact.rawValue().toString();
             expect(url == QStringLiteral("rtsp://192.168.144.108:554/main"), "IR off: QGC video is the day stream again", url);
             expect(!link.thermalPicture(), "IR off: the link is told that the day picture shows", QString());
             QQuickItem *video = findItem("videoContent");
             QQuickItem *button = findItem("vamaPaletteButton");
             expect(video && video->property("_vamaPaletteRow").toInt() == 0 && button && !button->isVisible(),
                    "IR off: the day picture is not coloured, and the colour button hides", link.thermalPalette());
             link.setThermalPalette(QStringLiteral("camera"));
             // A single tap on the video, right of centre and up.
             mouse(QEvent::MouseButtonPress, QPointF(1300, 420));
             mouse(QEvent::MouseButtonRelease, QPointF(1300, 420));
         }},
        {700, [&]() {
             expect(camera.angleCommands >= 2, "tap: the camera was told to turn (GAY and GAP)",
                    QString::number(camera.angleCommands));
             shot("9_tap_aiming.png");
         }},
        {2500, [&]() {
             expect(link.laserValid() && link.targetValid(), "tap: the laser measured the point after the turn",
                    QStringLiteral("%1 m").arg(link.laserRangeM()));
             shot("10_tap_result.png");
             // Almost level, as in the field video of test build 4: the result then
             // carries the "less accurate" note, the tallest the box gets.
             camera.setPitch(3.0);
             // The walker stands 14 degrees right of where the camera looks, 4 below.
             camera.placeWalker(camera.lookRight() + 14.0, camera.pitch() - 4.0);
             pressLockToPick("lock: the lock button starts picking");
         }},
        {400, [&]() {
             shot("11_lock_pick.png");
             anglesBeforeLock = camera.angleCommands;
             speedsBefore = camera.speedCommands;
             lookAtLock = camera.lookRight();
             // Draw a box around the walker, a little larger than it.
             walkerOnScreen = sceneRect("previewWalker");
             expect(!walkerOnScreen.isEmpty() && walkerOnScreen.center().x() > 1000, "lock: the walker is in the picture, right of the cross",
                    QStringLiteral("%1, %2").arg(walkerOnScreen.center().x()).arg(walkerOnScreen.center().y()));
             mouse(QEvent::MouseButtonPress, walkerOnScreen.topLeft() - QPointF(14, 12));
         }},
        {100, [&]() { mouse(QEvent::MouseMove, walkerOnScreen.center()); }},
        {100, [&]() { mouse(QEvent::MouseMove, walkerOnScreen.bottomRight() + QPointF(14, 12)); }},
        {300, [&]() {
             shot("12_lock_drawing.png");
             expect(camera.angleCommands == anglesBeforeLock && camera.speedCommands == speedsBefore,
                    "lock: drawing the box does not move the camera", QString::number(camera.speedCommands - speedsBefore));
             mouse(QEvent::MouseButtonRelease, walkerOnScreen.bottomRight() + QPointF(14, 12));
         }},
        {2000, [&]() {
             expect(link.lockActive() && link.lockByApp() && link.lockSeen(), "lock: the app follows the object itself",
                    link.lockMessage());
             expect(camera.gotFrames == 0 && camera.sumConfirms == 0, "lock: the camera's own tracker is not used",
                    QStringLiteral("GOT %1, SUM 01 %2").arg(camera.gotFrames).arg(camera.sumConfirms));
             expect(camera.angleCommands == anglesBeforeLock && camera.speedCommands > speedsBefore,
                    "lock: the camera is turned with speed commands, not angle commands",
                    QStringLiteral("%1 speed frames, %2 angle frames").arg(camera.speedCommands - speedsBefore)
                        .arg(camera.angleCommands - anglesBeforeLock));
             const QPointF off = walkerOff();
             expect(std::abs(off.x()) < 1.5 && std::abs(off.y()) < 1.5, "lock: the camera turned until the walker was under the cross",
                    offText());
             expect(camera.lookRight() - lookAtLock > 11.0, "lock: it turned to the right, where the walker was",
                    QStringLiteral("%1 deg").arg(camera.lookRight() - lookAtLock, 0, 'f', 1));
             expect(boxOnWalker() && boxLabel() == QStringLiteral("LOCKED"), "lock: the box on the screen is on the walker",
                    boxText() + QStringLiteral(", label ") + boxLabel());
             const QRectF lockedBox = sceneRect("vamaLockBox");
             expect(lockedBox.height() > lockedBox.width() * 1.5 && lockedBox.width() > walkerOnScreen.width() &&
                        lockedBox.width() < walkerOnScreen.width() * 2.5,
                    "lock: the box has the shape that was drawn (tall, a little larger than the walker)",
                    QStringLiteral("%1 x %2, walker %3 x %4").arg(lockedBox.width(), 0, 'f', 0).arg(lockedBox.height(), 0, 'f', 0)
                        .arg(walkerOnScreen.width(), 0, 'f', 0).arg(walkerOnScreen.height(), 0, 'f', 0));
             shot("13_locked.png");
             camera.walkerSpeed = 4.0;  // the walker walks to the right
         }},
        {3000, [&]() {
             const QPointF off = walkerOff();
             expect(link.lockActive() && link.lockSeen(), "lock: still locked while the walker walks", link.lockMessage());
             expect(camera.lookRight() - lookAtLock > 22.0, "lock: the camera turned after the walking walker",
                    QStringLiteral("%1 deg since the lock").arg(camera.lookRight() - lookAtLock, 0, 'f', 1));
             // Behind a walking object by its speed over the gain, and by what the app takes off for the picture's age.
             expect(off.x() > -0.5 && off.x() < 4.5 && std::abs(off.y()) < 1.5, "lock: the walker stays near the cross while walking",
                    offText());
             expect(boxOnWalker(), "lock: the box stays on the walker", boxText());
             shot("14_lock_following.png");
             camera.walkerSpeed = 0.0;  // and stands
         }},
        {2600, [&]() {
             const QPointF off = walkerOff();
             expect(std::abs(off.x()) < 1.0 && std::abs(off.y()) < 1.0, "lock: the walker is under the cross once it stands", offText());
             expect(link.lockMessage().contains(QStringLiteral("follows the object")), "lock: the state says the camera follows",
                    link.lockMessage());
             expect(link.laserValid() && link.targetValid(), "lock: the laser measured the walker under the cross",
                    QStringLiteral("%1 m").arg(link.laserRangeM()));
             QQuickItem *numbers = findItem("vamaLockNumbers");
             expect(numbers && numbers->isVisible() && numbers->property("text").toString().contains(QStringLiteral("pictures/s")),
                    "lock: the line of numbers for a field video shows", numbers ? numbers->property("text").toString() : QString());
             shot("14b_lock_on_the_cross.png");
             // During a lock the laser button measures, as before, and the box keeps the readings (task Y).
             tap("vamaLaserButton");
         }},
        {700, [&]() {
             QQuickItem *box = findItem("vamaResultBox");
             expect(link.lockActive() && box && box->isVisible(), "lock: the laser button does not hide the box during a lock",
                    link.lockMessage());
             camera.setWalkerVisible(false);  // the walker is gone
         }},
        {700, [&]() {
             expect(link.lockActive() && !link.lockSeen(), "lock: it says when the object is not seen", link.lockMessage());
             expect(boxLabel() == QStringLiteral("NOT SEEN"), "lock: the box says NOT SEEN", boxLabel());
             expect(camera.yawRate == 0.0 && camera.pitchRate == 0.0, "lock: the camera waits, it does not turn after nothing",
                    QStringLiteral("%1 / %2 deg/s").arg(camera.yawRate).arg(camera.pitchRate));
             shot("14c_lock_not_seen.png");
         }},
        {2200, [&]() {
             expect(!link.lockActive() && link.lockMessage().contains(QStringLiteral("Lock lost")),
                    "lock: after two seconds without the object the lock ends and says why", link.lockMessage());
             QQuickItem *chip = findItem("vamaLockChip");
             expect(chip && chip->isVisible(), "lock: the reason is on the screen", link.lockMessage());
             shot("15_lock_lost.png");
             // The camera's own tracker, as test builds 4 to 6 did it. It is chosen as the operator
             // does, in the camera settings ("Object lock"). The walker stands 12 degrees left.
             QObject *modeBox = window->findChild<QObject *>(QStringLiteral("vamaLockModeBox"));
             expect(modeBox != nullptr, "camera's tracker: the camera settings have the choice \"Object lock\"", QString());
             if (modeBox) {
                 QMetaObject::invokeMethod(modeBox, "activated", Q_ARG(int, 1));
             }
             expect(link.lockMode() == QStringLiteral("camera"), "camera's tracker: choosing it in the camera settings sets it",
                    link.lockMode());
             camera.placeWalker(camera.lookRight() - 12.0, camera.pitch() - 2.0);
             anglesBeforeLock = camera.angleCommands;
             pressLockToPick("camera's tracker: the lock button starts picking");
         }},
        {400, [&]() {
             walkerOnScreen = sceneRect("previewWalker");
             mouse(QEvent::MouseButtonPress, walkerOnScreen.topLeft() - QPointF(14, 12));
         }},
        {100, [&]() { mouse(QEvent::MouseMove, walkerOnScreen.center()); }},
        {100, [&]() { mouse(QEvent::MouseMove, walkerOnScreen.bottomRight() + QPointF(14, 12)); }},
        {200, [&]() { mouse(QEvent::MouseButtonRelease, walkerOnScreen.bottomRight() + QPointF(14, 12)); }},
        {1800, [&]() {
             expect(link.lockActive() && !link.lockByApp(), "camera's tracker: locked after the turn", link.lockMessage());
             expect(camera.angleCommands >= anglesBeforeLock + 2, "camera's tracker: the camera turned to the box first (GAY and GAP)",
                    QString::number(camera.angleCommands - anglesBeforeLock));
             // The object is under the cross after the turn: the camera's own count for that is 672, 378.
             expect(camera.gotFrames == 1 && camera.lastGot == QStringLiteral("02A0017A") && camera.sumConfirms >= 1,
                    "camera's tracker: GOT at the camera's own middle (672, 378), then SUM confirm",
                    QStringLiteral("GOT %1 (%2), SUM 01 %3").arg(camera.gotFrames).arg(camera.lastGot).arg(camera.sumConfirms));
             expect(link.lockMessage().contains(QStringLiteral("camera's own tracker")) &&
                        !link.lockMessage().contains(QStringLiteral("is following")),
                    "camera's tracker: the text says whose tracker it is, and does not claim it follows", link.lockMessage());
         }},
        {3200, [&]() {
             expect(link.lockFollowSeen(), "camera's tracker: the camera is seen turning by itself",
                    QStringLiteral("%1 deg").arg(link.lockTurnedDeg()));
             QQuickItem *line = findItem("vamaLockLine");
             expect(line && line->property("text").toString().contains(QStringLiteral("(turned")),
                    "camera's tracker: how far it turned is on the screen", line ? line->property("text").toString() : QString());
             shot("15b_camera_tracker.png");
             pressLockToStop("camera's tracker: the lock button stops the lock");
         }},
        {400, [&]() {
             expect(!link.lockActive() && camera.sumStops >= 1, "camera's tracker: stopped with SUM stop",
                    QStringLiteral("SUM 00 %1").arg(camera.sumStops));
             shot("15c_lock_stopped.png");
             QObject *modeBox = window->findChild<QObject *>(QStringLiteral("vamaLockModeBox"));
             if (modeBox) {
                 QMetaObject::invokeMethod(modeBox, "activated", Q_ARG(int, 0));
             }
             expect(link.lockMode() == QStringLiteral("app"), "camera's tracker: the app's own lock can be chosen again", link.lockMode());
             camera.setWalkerVisible(false);
             // Indoors: the drone has no GPS lock. The laser still measures.
             vehicle.gps.set("lock", 0);
             link.fireLaser();
         }},
        {1500, [&]() {
             expect(link.laserValid() && !link.targetValid() && link.targetMessage().startsWith(QStringLiteral("No lat long")),
                    "no GPS lock: distance shown, and the text says only the lat long is missing", link.targetMessage());
             shot("16_no_gps_lock.png");
             vehicle.gps.set("lock", 3);
         }},
        {300, [&]() {
             // Follow Me. The drone hovers 60 m up in Loiter, its nose to 40 degrees. The camera
             // looks 40 degrees down at the walker, who stands 71 m away over the ground.
             vehicle.vehicle.set("altitudeRelative", 60.0);
             camera.setYaw(0.0);
             camera.setPitch(-40.0);
             camera.walkerSpeed = 0.0;
             camera.placeWalker(camera.lookRight(), camera.pitch());
             telemetryTimer.start();
             QQuickItem *button = findItem("vamaFollowButton");
             expect(button && !button->isVisible(), "follow: no follow button without a lock", QString());
             pressLockToPick("follow: the lock button starts picking");
         }},
        {500, [&]() {
             walkerOnScreen = sceneRect("previewWalker");
             mouse(QEvent::MouseButtonPress, walkerOnScreen.topLeft() - QPointF(14, 12));
         }},
        {100, [&]() { mouse(QEvent::MouseMove, walkerOnScreen.center()); }},
        {100, [&]() { mouse(QEvent::MouseMove, walkerOnScreen.bottomRight() + QPointF(14, 12)); }},
        {200, [&]() { mouse(QEvent::MouseButtonRelease, walkerOnScreen.bottomRight() + QPointF(14, 12)); }},
        {5000, [&]() {
             expect(link.lockActive() && link.lockByApp() && link.lockSeen(), "follow: the walker is locked by the app",
                    link.lockMessage());
             QQuickItem *button = findItem("vamaFollowButton");
             const QRectF followRect = sceneRect("vamaFollowButton");
             const QRectF lockRect = sceneRect("vamaLockButton");
             expect(button && button->isVisible() && followRect.left() > lockRect.right() &&
                        std::abs(followRect.center().y() - lockRect.center().y()) < 2.0,
                    "follow: the follow button shows beside the lock button while the app holds a lock",
                    QStringLiteral("at %1, %2; lock button at %3, %4").arg(followRect.center().x(), 0, 'f', 0)
                        .arg(followRect.center().y(), 0, 'f', 0).arg(lockRect.center().x(), 0, 'f', 0).arg(lockRect.center().y(), 0, 'f', 0));
             expect(follow.ready(), "follow: after a few seconds of lock the object's place is known", follow.notReadyText());
             expect(drone->sent.isEmpty() && drone->guidedAsked == 0, "follow: nothing goes to the drone before the operator starts it",
                    QStringLiteral("%1 speeds, %2 mode requests").arg(drone->sent.size()).arg(drone->guidedAsked));
             tap("vamaFollowButton");
         }},
        {600, [&]() {
             QObject *popup = window->findChild<QObject *>(QStringLiteral("vamaFollowPopup"));
             auto *what = window->findChild<QQuickItem *>(QStringLiteral("vamaFollowWhat"));
             auto *rules = window->findChild<QQuickItem *>(QStringLiteral("vamaFollowRules"));
             auto *start = window->findChild<QQuickItem *>(QStringLiteral("vamaFollowStart"));
             expect(popup && popup->property("opened").toBool() && !follow.active() && drone->sent.isEmpty(),
                    "follow: the button asks first, and nothing is sent yet", QString());
             const QString text = what ? what->property("text").toString() : QString();
             const QRegularExpressionMatch stays = QRegularExpression(QStringLiteral("It stays (\\d+) m from it")).match(text);
             const int kept = stays.hasMatch() ? stays.captured(1).toInt() : -1;
             expect(what && what->isVisible() && kept >= 69 && kept <= 73 && text.contains(QStringLiteral("at its height of 60 m")),
                    "follow: the window says what the drone will do, with the distance (71 m) and the height (60 m) it keeps", text);
             const QString rulesText = rules ? rules->property("text").toString() : QString();
             expect(rulesText.contains(QStringLiteral("never flies nearer to the object than 15 m")) &&
                        rulesText.contains(QStringLiteral("flies on for 3 s")) &&
                        rulesText.contains(QStringLiteral("The laser has measured")),
                    "follow: and its rules: the least distance, what happens when the link stops, where the distance comes from",
                    rulesText);
             expect(start && start->isEnabled(), "follow: \"Start following\" can be pressed", QString());
             shot("20_follow_ask.png");
             tap("vamaFollowStart");
         }},
        {700, [&]() {
             QObject *popup = window->findChild<QObject *>(QStringLiteral("vamaFollowPopup"));
             expect(follow.active() && popup && !popup->property("opened").toBool(), "follow: \"Start following\" starts it",
                    follow.message());
             expect(drone->guidedAsked == 1, "follow: the drone is asked to change to Guided", QString::number(drone->guidedAsked));
             // The drone changes to Guided, and the walker walks off to the right.
             telemetry.mode = skydroid::follow::Mode::Guided;
             drone->mode = QStringLiteral("Guided");
             camera.walkerSpeed = 3.0;
         }},
        {3500, [&]() {
             QQuickItem *line = findItem("vamaFollowLine");
             const QString text = line ? line->property("text").toString() : QString();
             expect(follow.active() && line && line->isVisible() && text.startsWith(QStringLiteral("Following. The object is")) &&
                        text.contains(QStringLiteral("m/s")),
                    "follow: the result box says that the drone follows, how far the object is and the speed asked", text);
             QQuickItem *numbers = findItem("vamaLockNumbers");
             expect(numbers && !numbers->isVisible(), "follow: the small line of lock numbers makes room for it", QString());
             double most = 0.0;
             for (const PreviewDrone::Sent &s : std::as_const(drone->sent)) {
                 most = std::max(most, std::hypot(s.northMps, s.eastMps));
             }
             const PreviewDrone::Sent last = drone->sent.isEmpty() ? PreviewDrone::Sent{} : drone->sent.constLast();
             const QString speed = QStringLiteral("%1 north, %2 east, heading %3; most %4 m/s")
                                       .arg(last.northMps, 0, 'f', 2).arg(last.eastMps, 0, 'f', 2).arg(last.headingDeg).arg(most, 0, 'f', 2);
             expect(!drone->sent.isEmpty() && std::hypot(last.northMps, last.eastMps) > 0.5 && most <= follow.speedLimit() + 1e-9 &&
                        last.headingDeg == 40.0,
                    "follow: the drone is sent speeds after the walking walker, with the heading to keep", speed);
             // The walker walks to the right of where the camera looks, and the drone's nose is to 40
             // degrees: that is towards 130 degrees, south-east.
             expect(last.eastMps > 0.0 && last.northMps < 0.0, "follow: the speed points the way the walker walks (south-east)", speed);
             shot("21_following.png");
             camera.walkerSpeed = 0.0;
             tap("vamaFollowButton");
         }},
        {500, [&]() {
             QQuickItem *chip = findItem("vamaFollowChip");
             QQuickItem *line = findItem("vamaFollowLine");
             QQuickItem *box = findItem("vamaResultBox");
             expect(!follow.active() && follow.message() == QStringLiteral("Follow stopped. The drone holds its place in Guided."),
                    "follow: a press on the green button stops it at once", follow.message());
             const PreviewDrone::Sent last = drone->sent.isEmpty() ? PreviewDrone::Sent{1.0, 1.0, 0.0} : drone->sent.constLast();
             expect(std::hypot(last.northMps, last.eastMps) == 0.0, "follow: the drone is told to stop",
                    QStringLiteral("%1 north, %2 east").arg(last.northMps).arg(last.eastMps));
             // In the result box, which shows for the lock: not in a second box under it.
             expect(box && box->isVisible() && line && line->isVisible() && line->property("text").toString() == follow.message() &&
                        chip && !chip->isVisible(),
                    "follow: why it ended is a line of the result box", line ? line->property("text").toString() : QString());
             const QRectF lockBox = sceneRect("vamaLockBox");
             const QRectF resultBox = sceneRect("vamaResultBox");
             expect(resultBox.bottom() < lockBox.top(), "follow: and the box stays clear of the object in the video",
                    QStringLiteral("box down to %1, object from %2").arg(resultBox.bottom(), 0, 'f', 0).arg(lockBox.top(), 0, 'f', 0));
             expect(link.lockActive(), "follow: the camera stays locked on the walker", link.lockMessage());
             shot("22_follow_stopped.png");
             if (box) {
                 tapItem(box);
             }
         }},
        {300, [&]() {
             QQuickItem *box = findItem("vamaResultBox");
             QQuickItem *line = findItem("vamaFollowLine");
             expect(follow.message().isEmpty() && line && !line->isVisible() && box && box->isVisible(),
                    "follow: a tap takes that line away, and the box stays", follow.message());
             // The drone has sunk to 6 m: Follow cannot start, and the window says why.
             telemetry.heightM = 6.0;
         }},
        {700, [&]() { tap("vamaFollowButton"); }},
        {600, [&]() {
             auto *why = window->findChild<QQuickItem *>(QStringLiteral("vamaFollowNotReady"));
             auto *what = window->findChild<QQuickItem *>(QStringLiteral("vamaFollowWhat"));
             auto *start = window->findChild<QQuickItem *>(QStringLiteral("vamaFollowStart"));
             const QString text = why ? why->property("text").toString() : QString();
             expect(why && why->isVisible() && text.contains(QStringLiteral("the drone is 6 m up. It needs 10 m.")) && what &&
                        !what->isVisible(),
                    "follow: when it cannot start, the window says why", text);
             expect(start && !start->isEnabled(), "follow: and \"Start following\" is greyed", QString());
             shot("23_follow_not_ready.png");
             guidedBefore = drone->guidedAsked;
             tap("vamaFollowStart");
         }},
        {300, [&]() {
             expect(!follow.active() && drone->guidedAsked == guidedBefore, "follow: a press on the greyed button starts nothing",
                    follow.message());
             tap("vamaFollowCancel");
         }},
        {300, [&]() {
             QObject *popup = window->findChild<QObject *>(QStringLiteral("vamaFollowPopup"));
             expect(popup && !popup->property("opened").toBool(), "follow: Cancel closes the window", QString());
             telemetry.heightM = 60.0;
             pressLockToStop("follow: the lock button stops the lock");
         }},
        {400, [&]() {
             QQuickItem *button = findItem("vamaFollowButton");
             expect(!link.lockActive() && button && !button->isVisible(), "follow: with the lock gone the follow button is gone",
                    link.lockMessage());
             // With no result box on the screen, why Follow does not start shows in a box of its own.
             QQuickItem *box = findItem("vamaResultBox");
             if (box && box->isVisible()) {
                 tapItem(box);
             }
             follow.start();
         }},
        {300, [&]() {
             QQuickItem *box = findItem("vamaResultBox");
             QQuickItem *chip = findItem("vamaFollowChip");
             QQuickItem *chipText = findItem("vamaFollowChipText");
             expect(box && !box->isVisible() && chip && chip->isVisible() && chipText &&
                        chipText->property("text").toString().contains(QStringLiteral("Lock the object first")),
                    "follow: without the result box, why it does not start shows in a box of its own",
                    chipText ? chipText->property("text").toString() : QString());
             if (chip) {
                 tapItem(chip);
             }
         }},
        {300, [&]() {
             QQuickItem *chip = findItem("vamaFollowChip");
             expect(follow.message().isEmpty() && chip && !chip->isVisible(), "follow: a tap takes it away", follow.message());
             telemetryTimer.stop();
             camera.setWalkerVisible(false);
         }},
        {300, [&]() {
             // Marks on people and vehicles. The camera looks 40 degrees down. A walker stands 9
             // degrees right of the cross and 5 below it, a car 15 degrees left of it and 14 below
             // (clear of where the walker walks later: a walker in front of a car is another test).
             camera.setYaw(0.0);
             camera.setPitch(-40.0);
             camera.walkerSpeed = 0.0;
             camera.placeWalker(camera.lookRight() + 9.0, camera.pitch() - 5.0);
             camera.placeCar(camera.lookRight() - 15.0, camera.pitch() - 14.0);
             expect(marks.enabled() && marks.usable() && !marks.running() && marks.count() == 0,
                    "marks: the switch is on, and nothing is looked at before the lock button", marks.state());
             speedsBefore = camera.speedCommands;
             pressLockToPick("marks: the lock button starts picking, with the marks");
         }},
        {1300, [&]() {
             // The stand-in finder's looks take real time: on a busy PC the first marks come later.
             waitFor([&]() { return marks.count() == 2 && findItems("vamaMark").size() == 2; }, 4000);
             const QList<QQuickItem *> shown = findItems("vamaMark");
             expect(marks.running() && marks.count() == 2 && shown.size() == 2,
                    "marks: after the lock button the walker and the car are marked",
                    QStringLiteral("%1 marks, %2 on the screen, %3").arg(marks.count()).arg(shown.size()).arg(marks.state()));
             expect(markOn("previewWalker", "person") && markOn("previewCar", "vehicle"), "marks: each box is on its object, with its kind",
                    marksText());
             QQuickItem *hint = findItem("vamaLockHintText");
             const QString hintText = hint ? hint->property("text").toString() : QString();
             expect(hint && hint->isVisible() && hintText.startsWith(QStringLiteral("Tap a marked object, or draw a box")),
                    "marks: the hint says to tap a marked object, or to draw a box", hintText);
             QQuickItem *state = findItem("vamaMarksState");
             const QString stateText = state ? state->property("text").toString() : QString();
             // A look of the stand-in finder takes about 0.3 s, more on a busy PC.
             static const QRegularExpression lookLine(QStringLiteral("^Marked: 1 person, 1 vehicle\\. A look takes 0\\.\\d s\\."));
             expect(state && state->isVisible() && lookLine.match(stateText).hasMatch() &&
                        stateText.contains(QStringLiteral("Getting a picture takes 0.")),
                    "marks: a line says what is marked, how long a look takes, and how long a picture takes to get", stateText);
             // This finder takes 0.12 s for a look of 640 x 384 points: the app looks closely at the
             // middle 960 x 544 of the camera's 1280 x 720 points (0.26 s), and shows that part with corners.
             // (Where the picture is on the screen is the lock box's own count, as for the marks.)
             const QRectF corners = sceneRect("vamaCloseLook");
             QQuickItem *cornersItem = findItem("vamaCloseLook");
             QQuickItem *drag = findItem("vamaVideoDrag");
             QQuickItem *lockBoxItem = findItem("vamaLockBox");
             const QRectF picture = lockBoxItem ? lockBoxItem->property("picture").toRectF() : QRectF();
             const QRectF look = marks.closeLook();
             const QRectF want = drag ? drag->mapRectToScene(QRectF(picture.x() + look.x() * picture.width(),
                                                                    picture.y() + look.y() * picture.height(),
                                                                    look.width() * picture.width(),
                                                                    look.height() * picture.height()))
                                      : QRectF();
             // Three quarters of the picture, each way, around its middle.
             expect(std::abs(look.width() - 0.75) < 0.01 && std::abs(look.height() - 0.76) < 0.02 &&
                        std::abs(look.center().x() - 0.5) < 0.01 && std::abs(look.center().y() - 0.5) < 0.01,
                    "marks: on this device the app looks closely at the middle 960 x 544 of the camera's 1280 x 720 points",
                    QStringLiteral("%1 x %2 of the picture").arg(look.width(), 0, 'f', 3).arg(look.height(), 0, 'f', 3));
             expect(cornersItem && cornersItem->isVisible() && picture.width() > 1000.0 &&
                        std::abs(corners.x() - want.x()) < 1.5 && std::abs(corners.y() - want.y()) < 1.5 &&
                        std::abs(corners.width() - want.width()) < 1.5 && std::abs(corners.height() - want.height()) < 1.5,
                    "marks: corners on the video show that part",
                    QStringLiteral("%1 x %2 at %3, %4; that part is %5 x %6 at %7, %8")
                        .arg(corners.width(), 0, 'f', 0).arg(corners.height(), 0, 'f', 0).arg(corners.center().x(), 0, 'f', 0)
                        .arg(corners.center().y(), 0, 'f', 0).arg(want.width(), 0, 'f', 0).arg(want.height(), 0, 'f', 0)
                        .arg(want.center().x(), 0, 'f', 0).arg(want.center().y(), 0, 'f', 0));
             expect(!link.lockActive() && !link.lockBusy() && camera.speedCommands == speedsBefore,
                    "marks: marking locks nothing and moves nothing", link.lockMessage());
             shot("24_marks.png");
             // A tap on the walker's mark.
             const QPointF at = sceneRect("previewWalker").center();
             mouse(QEvent::MouseButtonPress, at);
             mouse(QEvent::MouseButtonRelease, at);
         }},
        {100, [&]() {
             // At once: a mark is not a place to wait for a second tap.
             expect(link.lockActive() && link.lockByApp(), "marks: a tap on a mark locks its object at once", link.lockMessage());
             expect(!marks.running() && findItems("vamaMark").isEmpty(), "marks: the marks go when an object is locked",
                    QString::number(findItems("vamaMark").size()));
         }},
        {1500, [&]() {
             const QPointF off = walkerOff();
             expect(link.lockActive() && link.lockSeen() && boxOnWalker(), "marks: the lock's box is on the walker", boxText());
             expect(std::abs(off.x()) < 1.5 && std::abs(off.y()) < 1.5, "marks: and the camera has turned to the walker", offText());
             const QRectF lockedBox = sceneRect("vamaLockBox");
             const QRectF walker = sceneRect("previewWalker");
             expect(lockedBox.width() < walker.width() * 1.6 && lockedBox.height() < walker.height() * 1.4,
                    "marks: the box has the size of the mark, close around the walker",
                    QStringLiteral("%1 x %2, walker %3 x %4").arg(lockedBox.width(), 0, 'f', 0).arg(lockedBox.height(), 0, 'f', 0)
                        .arg(walker.width(), 0, 'f', 0).arg(walker.height(), 0, 'f', 0));
             shot("25_mark_locked.png");
             pressLockToStop("marks: the lock button stops the lock on the mark");
         }},
        {400, [&]() {
             // A walker who walks, 8 degrees a second to the right: the mark shows where they were
             // a moment ago. A tap on the mark has to lock the walker where they are now.
             expect(!link.lockActive(), "marks: the lock button stops the lock", link.lockMessage());
             camera.setYaw(0.0);
             camera.setPitch(-40.0);
             camera.placeWalker(camera.lookRight() - 22.0, camera.pitch() - 5.0);
             camera.walkerSpeed = 8.0;
             pressLockToPick("marks: the lock button starts picking for the walking walker");
         }},
        {1400, [&]() {
             const QRectF mark = markRect("person");
             const QRectF walker = sceneRect("previewWalker");
             expect(!mark.isEmpty() && walker.center().x() - mark.center().x() > 8.0,
                    "marks: the mark of a walking object is behind it (it shows where the object was)",
                    QStringLiteral("mark at %1, walker at %2").arg(mark.center().x(), 0, 'f', 0).arg(walker.center().x(), 0, 'f', 0));
             mouse(QEvent::MouseButtonPress, mark.center());
             mouse(QEvent::MouseButtonRelease, mark.center());
         }},
        {150, [&]() {
             // The box on the screen is where the app saw the walker in its last picture: within a
             // walker's width of them. (A lock on the mark's old place would be 100 points behind and
             // more, and stay on the ground there.) On a busy PC that picture comes later, and the
             // stand-in camera turns less than the app counts: the box is drawn ahead of the walker
             // until the next pictures come (2 of 4 runs with the CPU loaded). So up to 1 s.
             auto onWalker = [&]() {
                 const QRectF box = sceneRect("vamaLockBox");
                 const QRectF walker = sceneRect("previewWalker");
                 return link.lockActive() && link.lockSeen() && !box.isEmpty() &&
                        std::abs(box.center().x() - walker.center().x()) < walker.width();
             };
             waitFor(onWalker, 850);
             expect(onWalker(), "marks: a tap on that mark locks the walker where they are now",
                    boxText() + QStringLiteral("; ") + lockState() + QStringLiteral("; ") + link.lockNumbers());
         }},
        {1500, [&]() {
             expect(link.lockActive() && link.lockSeen() && boxOnWalker(), "marks: and the lock stays on the walking walker", boxText());
             camera.walkerSpeed = 0.0;
             pressLockToStop("marks: the lock button stops the lock on the walking walker");
         }},
        {400, [&]() {
             // A finger that slides a little while it taps a mark still means the mark, not a
             // small box drawn over it. The walker stands 9 degrees right of the cross again.
             camera.placeWalker(camera.lookRight() + 9.0, camera.pitch() - 5.0);
             pressLockToPick("marks: the lock button starts picking for the sliding tap");
         }},
        {1300, [&]() {
             slideFrom = sceneRect("previewWalker").center();
             mouse(QEvent::MouseButtonPress, slideFrom);
             mouse(QEvent::MouseMove, slideFrom + QPointF(18, 6));
         }},
        {80, [&]() { mouse(QEvent::MouseButtonRelease, slideFrom + QPointF(18, 6)); }},
        {1300, [&]() {
             const QRectF lockedBox = sceneRect("vamaLockBox");
             const QRectF walker = sceneRect("previewWalker");
             expect(link.lockActive() && link.lockSeen() && boxOnWalker() && lockedBox.height() > walker.height() * 0.7,
                    "marks: a tap that slides a little still locks the marked object, with the mark's box",
                    QStringLiteral("%1; box %2 x %3, walker %4 x %5").arg(boxText()).arg(lockedBox.width(), 0, 'f', 0)
                        .arg(lockedBox.height(), 0, 'f', 0).arg(walker.width(), 0, 'f', 0).arg(walker.height(), 0, 'f', 0));
             pressLockToStop("marks: the lock button stops the lock of the sliding tap");
         }},
        {400, [&]() {
             // A tap beside every mark, while the button waits: the place is locked as before the marks.
             pressLockToPick("marks: the lock button starts picking for a tap beside the marks");
         }},
        {900, [&]() {
             expect(marks.running() && marks.count() >= 1, "marks: marked again after the lock button", marks.state());
             const QPointF ground = sceneRect("previewLightBuilding").center();
             mouse(QEvent::MouseButtonPress, ground);
             mouse(QEvent::MouseButtonRelease, ground);
         }},
        {900, [&]() {
             expect(!marks.running() && (link.lockActive() || link.lockBusy() || link.lockMessage().contains(QStringLiteral("plain area"))),
                    "marks: a tap beside every mark locks the place tapped, as before", link.lockMessage());
             if (link.lockActive() || link.lockBusy()) {
                 pressLockToStop("marks: the lock button stops the lock on the place tapped");
             }
         }},
        {400, [&]() {
             // The switch, in the camera settings.
             QObject *popup = window->findChild<QObject *>(QStringLiteral("vamaCameraSettings"));
             if (popup) {
                 QMetaObject::invokeMethod(popup, "open");
             }
         }},
        {500, [&]() {
             auto *box = window->findChild<QQuickItem *>(QStringLiteral("vamaMarksBox"));
             auto *note = window->findChild<QQuickItem *>(QStringLiteral("vamaMarksNote"));
             QObject *flick = window->findChild<QObject *>(QStringLiteral("vamaCameraSettingsFlick"));
             expect(box && note && flick && box->property("checked").toBool() &&
                        note->property("text").toString().contains(QStringLiteral("inside the white corners")),
                    "marks: the camera settings have the switch, on, and say what it does", note ? note->property("text").toString() : QString());
             auto *name = window->findChild<QQuickItem *>(QStringLiteral("vamaFinderName"));
             expect(name && name->property("text").toString() == QStringLiteral("Object finder: a stand-in for the preview."),
                    "marks: and which finder this build has", name ? name->property("text").toString() : QString());
             if (box && flick) {
                 auto *content = qobject_cast<QQuickItem *>(flick->property("contentItem").value<QObject *>());
                 if (content) {
                     flick->setProperty("contentY", qMax(0.0, box->mapToItem(content, QPointF(0, 0)).y() - 200.0));
                 }
             }
         }},
        {300, [&]() {
             shot("26_marks_setting.png");
             auto *box = window->findChild<QQuickItem *>(QStringLiteral("vamaMarksBox"));
             if (box) {
                 tapItem(box);
             }
         }},
        {300, [&]() {
             expect(!marks.enabled(), "marks: the switch turns them off", QString());
             tap("vamaNoticesButton");
         }},
        {500, [&]() {
             QObject *popup = window->findChild<QObject *>(QStringLiteral("vamaNotices"));
             auto *text = window->findChild<QQuickItem *>(QStringLiteral("vamaNoticesText"));
             const QString notices = text ? text->property("text").toString() : QString();
             expect(popup && popup->property("opened").toBool() && notices.contains(QStringLiteral("BSD 3-Clause License")) &&
                        notices.contains(QStringLiteral("Apache License")) && notices.contains(QStringLiteral("NanoDet")),
                    "marks: the licences of the finder can be read in the app", notices.left(60));
             shot("27_notices.png");
             tap("vamaNoticesClose");
         }},
        {300, [&]() {
             QObject *notices = window->findChild<QObject *>(QStringLiteral("vamaNotices"));
             expect(notices && !notices->property("opened").toBool(), "marks: Close closes them", QString());
             QObject *popup = window->findChild<QObject *>(QStringLiteral("vamaCameraSettings"));
             if (popup) {
                 QMetaObject::invokeMethod(popup, "close");
             }
         }},
        {400, [&]() { pressLockToPick("marks: with the switch off the lock button still starts picking"); }},
        {900, [&]() {
             QQuickItem *hint = findItem("vamaLockHintText");
             const QString hintText = hint ? hint->property("text").toString() : QString();
             QQuickItem *state = findItem("vamaMarksState");
             expect(!marks.running() && marks.count() == 0 && findItems("vamaMark").isEmpty() &&
                        hintText.startsWith(QStringLiteral("Draw a box around the object, or tap it.")) && state && !state->isVisible(),
                    "marks: with the switch off the lock button marks nothing, and the hint is the old one", hintText);
             pressLockToEndPicking("marks: the lock button ends picking");
             marks.setEnabled(true);
             camera.setCarVisible(false);
             camera.setWalkerVisible(false);
         }},
        {200, [&]() {
             // Picking that ends without a lock says so in the lock box, with why and what the
             // video got meanwhile. (Client video 2026-10-15: picking ended at a tap on a mark,
             // nothing was locked, and the box still said "Lock stopped." of the lock before.)
             QQuickItem *chip = findItem("vamaLockChip");
             QQuickItem *chipText = findItem("vamaLockChipText");
             const QString text = chipText ? chipText->property("text").toString() : QString();
             expect(chip && chip->isVisible() &&
                        text == QStringLiteral("No object locked: the lock button was pressed again. Touches on the video meanwhile: none."),
                    "picking: when the lock button ends it, the lock box says no object was locked, and why", text);
             shot("28_picking_stopped.png");
             pressLockToPick("picking: the lock button starts it again");
         }},
        {300, [&]() {
             // A touch on the video that the system cuts off (as Android does when another
             // window takes the screen), then the map becomes the main view.
             mouse(QEvent::MouseButtonPress, QPointF(700, 700));
             if (QQuickItem *drag = findItem("vamaVideoDrag")) {
                 drag->ungrabMouse();
             }
             window->setProperty("mapIsMain", true);
         }},
        {200, [&]() {
             QQuickItem *chipText = findItem("vamaLockChipText");
             const QString text = chipText ? chipText->property("text").toString() : QString();
             expect(!picking() && text == QStringLiteral("No object locked: the video was no longer the main view. "
                                                         "Touches on the video meanwhile: 1, the last one cut off by the system."),
                    "picking: when the video is no longer the main view it ends, and the box says why and what the video got",
                    text);
             window->setProperty("mapIsMain", false);
             mouse(QEvent::MouseButtonRelease, QPointF(700, 700));  // the finger lifts: nothing has it any more
         }},
        {300, [&]() {
             pressLockToPick("picking: and again, for a long press on the lock button");
             lockButtonAt = sceneRect("vamaLockButton").center();
         }},
        {300, [&]() { mouse(QEvent::MouseButtonPress, lockButtonAt); }},
        {1300, [&]() { mouse(QEvent::MouseButtonRelease, lockButtonAt); }},
        {200, [&]() {
             // A press held this long is no tap: a finger resting on the button, or a touch the
             // screen did not end, would end picking like this. The box says how long it was.
             QQuickItem *chipText = findItem("vamaLockChipText");
             const QString text = chipText ? chipText->property("text").toString() : QString();
             static const QRegularExpression held(QStringLiteral(
                 "^No object locked: the lock button was held down for [1-9]\\d*\\.\\d s\\. Touches on the video meanwhile: none\\.$"));
             expect(!picking() && held.match(text).hasMatch(),
                    "picking: a long press on the lock button ends it, and the box says how long the button was held", text);
         }},
        {300, [&]() {
             // A point from the picture, without the laser. It is chosen as the operator does, in
             // the camera settings. The drone flies 60 m up and the camera looks 25 degrees down.
             vehicle.vehicle.set("altitudeRelative", 60.0);
             camera.setPitch(-25.0);
             QObject *pointBox = window->findChild<QObject *>(QStringLiteral("vamaPointModeBox"));
             expect(pointBox != nullptr, "picture point: the camera settings have the choice \"Tap on the video\"", QString());
             if (pointBox) {
                 QMetaObject::invokeMethod(pointBox, "activated", Q_ARG(int, 1));
             }
             expect(link.pointMode() == QStringLiteral("picture"), "picture point: choosing it in the camera settings sets it",
                    link.pointMode());
         }},
        {500, [&]() {
             anglesBeforePoint = camera.angleCommands;
             lasersBeforePoint = camera.laserFrames;
             // A tap right of the cross and below it.
             mouse(QEvent::MouseButtonPress, QPointF(1300, 620));
             mouse(QEvent::MouseButtonRelease, QPointF(1300, 620));
         }},
        {1100, [&]() {
             QQuickItem *title = findItem("vamaResultTitle");
             QQuickItem *note = findItem("vamaResultNote");
             const QString titleText = title ? title->property("text").toString() : QString();
             const QString noteText = note ? note->property("text").toString() : QString();
             expect(link.targetValid() && link.targetFromPicture(), "picture point: a tap gives the position without the laser",
                    link.laserMessage());
             expect(camera.angleCommands == anglesBeforePoint && camera.laserFrames == lasersBeforePoint,
                    "picture point: the camera was not turned and the laser was not asked",
                    QStringLiteral("%1 turns, %2 laser frames").arg(camera.angleCommands - anglesBeforePoint)
                        .arg(camera.laserFrames - lasersBeforePoint));
             expect(title && title->isVisible() && titleText.startsWith(QStringLiteral("About ")),
                    "picture point: the distance on the screen says \"About\"", titleText);
             expect(note && note->isVisible() && noteText.contains(QStringLiteral("From the picture, not by laser")) &&
                        noteText.contains(QStringLiteral("One degree of camera angle is")),
                    "picture point: the result says it is from the picture, and how far to trust it", noteText);
             shot("17a_picture_point.png");
             // The operator taps the result away to see the video.
             if (title) {
                 const QPointF onTheBox = title->mapToScene(QPointF(title->width() / 2, title->height() / 2));
                 mouse(QEvent::MouseButtonPress, onTheBox);
                 mouse(QEvent::MouseButtonRelease, onTheBox);
             }
         }},
        {300, [&]() {
             QQuickItem *title = findItem("vamaResultTitle");
             expect(title && !title->isVisible(), "picture point: a tap on the result hides it", QString());
             // The drone on the ground: no position, and the reason. The next tap brings the result back.
             vehicle.vehicle.set("altitudeRelative", 0.4);
             mouse(QEvent::MouseButtonPress, QPointF(1300, 620));
             mouse(QEvent::MouseButtonRelease, QPointF(1300, 620));
         }},
        {1100, [&]() {
             QQuickItem *title = findItem("vamaResultTitle");
             const QString titleText = title ? title->property("text").toString() : QString();
             expect(!link.targetValid() && title && title->isVisible() && titleText == QStringLiteral("No position") &&
                        link.laserMessage().contains(QStringLiteral("it needs 2.5 m")),
                    "picture point: on the ground it says \"No position\" and why", titleText + QStringLiteral(" / ") + link.laserMessage());
             shot("17b_picture_no_position.png");
             // By laser again. The laser gives no reading (too far): the position comes from the
             // picture, and the result says both.
             QObject *pointBox = window->findChild<QObject *>(QStringLiteral("vamaPointModeBox"));
             if (pointBox) {
                 QMetaObject::invokeMethod(pointBox, "activated", Q_ARG(int, 0));
             }
             expect(link.pointMode() == QStringLiteral("laser"), "picture point: the laser can be chosen again", link.pointMode());
             vehicle.vehicle.set("altitudeRelative", 60.0);
             camera.laserAnswers = false;
             link.fireLaser();
         }},
        {2500, [&]() {
             QQuickItem *title = findItem("vamaResultTitle");
             const QString titleText = title ? title->property("text").toString() : QString();
             expect(!link.laserValid() && link.laserMessage() == QStringLiteral("No laser reading.") && link.targetValid() &&
                        link.targetFromPicture() && titleText.startsWith(QStringLiteral("About ")),
                    "no laser reading: the position comes from the picture, and the screen says both",
                    titleText + QStringLiteral(" / ") + link.laserMessage() + QStringLiteral(" / ") + link.targetMessage());
             shot("17c_no_laser_reading.png");
             camera.laserAnswers = true;
             vehicle.vehicle.remove("altitudeRelative");
         }},
        {300, [&]() {
             QObject *popup = window->findChild<QObject *>(QStringLiteral("vamaCameraSettings"));
             if (!popup) {
                 std::fprintf(stderr, "settings popup not found\n");
                 ++g_problems;
             } else {
                 QMetaObject::invokeMethod(popup, "open");
             }
         }},
        {700, [&]() {
             shot("4_settings.png");
             QObject *flick = window->findChild<QObject *>(QStringLiteral("vamaCameraSettingsFlick"));
             if (!flick) {
                 std::fprintf(stderr, "settings scroll area not found\n");
                 ++g_problems;
                 return;
             }
             const double bottom = flick->property("contentHeight").toDouble() - flick->property("height").toDouble();
             flick->setProperty("contentY", qMax(0.0, bottom));
         }},
        {400, [&]() {
             shot("5_settings_wheel.png");
             QObject *popup = window->findChild<QObject *>(QStringLiteral("vamaCameraSettings"));
             if (popup) {
                 QMetaObject::invokeMethod(popup, "close");
             }
             link.setEnabled(false);
         }},
        {600, [&]() { shot("6_camera_off.png"); openAppSettings(); }},
        {1000, [&]() {
             checkAppSettings();
             if (settingsWindow) {
                 settingsWindow->close();
             }
             // QGC's warning texts (client video of test build 6): indoors, no GPS, a pre-arm error.
             link.setEnabled(true);
             global.vehicles.vehicle.position.set(false);
             global.vehicles.vehicle.setPrearmError(QStringLiteral("PreArm: GPS blending unhealthy"));
         }},
        {500, [&]() {
             QQuickItem *warnings = findItem("vamaVehicleWarnings");
             QQuickItem *advice = findItem("vamaVehicleWarningsAdvice");
             if (!warnings || !advice) {
                 std::fprintf(stderr, "the warning texts did not load\n");
                 ++g_problems;
                 return;
             }
             // The box is drawn where its transform puts it.
             const QRectF box = warnings->mapRectToScene(QRectF(0, 0, warnings->width(), warnings->height()));
             const qreal middle = window->height() / 2.0;
             expect(warnings->isVisible() && warnings->property("_small").toBool(),
                    "warnings: with the video as the main view they are the small form", QString());
             expect(box.top() > middle + window->height() * 0.08 && box.bottom() < window->height(),
                    "warnings: they are in the lower part, clear of the middle of the picture",
                    QStringLiteral("from %1 to %2 of %3").arg(box.top(), 0, 'f', 0).arg(box.bottom(), 0, 'f', 0).arg(window->height()));
             expect(std::abs(box.center().x() - window->width() / 2.0) < 4, "warnings: still centred left to right",
                    QString::number(box.center().x()));
             expect(box.height() < window->height() * 0.15, "warnings: three short lines, not a third of the picture",
                    QStringLiteral("%1 high").arg(box.height(), 0, 'f', 0));
             expect(advice->property("text").toString() == QStringLiteral("Fix this before the vehicle can be armed."),
                    "warnings: the advice is one short line", advice->property("text").toString());
             shot("18_warnings_small.png");
             window->setProperty("mapIsMain", true);
         }},
        {400, [&]() {
             QQuickItem *warnings = findItem("vamaVehicleWarnings");
             QQuickItem *advice = findItem("vamaVehicleWarningsAdvice");
             if (!warnings || !advice) {
                 return;
             }
             const QRectF box = warnings->mapRectToScene(QRectF(0, 0, warnings->width(), warnings->height()));
             expect(!warnings->property("_small").toBool() && std::abs(box.center().y() - window->height() / 2.0) < window->height() * 0.06,
                    "warnings: with the map as the main view they are as QGC shows them, in the middle",
                    QStringLiteral("centre at %1").arg(box.center().y(), 0, 'f', 0));
             expect(advice->property("text").toString().startsWith(QStringLiteral("The vehicle has failed a pre-arm check.")),
                    "warnings: with QGC's own words", advice->property("text").toString());
             shot("19_warnings_map.png");
             window->setProperty("mapIsMain", false);
             // With a GPS position and no pre-arm error there is nothing to show.
             global.vehicles.vehicle.position.set(true);
             global.vehicles.vehicle.setPrearmError(QString());
         }},
        {300, [&]() {
             QQuickItem *warnings = findItem("vamaVehicleWarnings");
             expect(warnings && !warnings->isVisible(), "warnings: nothing shows when there is nothing to warn of", QString());
             // QGC's 3D view (task H) with the video as the main view: the 3D scene covers the
             // video, so a tap and a drag there are for the 3D view, never for the camera.
             // (A MouseArea's own "enabled" is not Item.enabled: it is read as a property.)
             QQuickItem *drag = findItem("vamaVideoDrag");
             expect(drag && drag->property("enabled").toBool() && drag->isVisible(),
                    "3D view: before it, the video takes taps and drags", QString());
             window->setProperty("view3D", true);
         }},
        {300, [&]() {
             QQuickItem *drag = findItem("vamaVideoDrag");
             QObject *layer = window->property("layer").value<QObject *>();
             QObject *manager = engine.singletonInstance<QObject *>("QGroundControl.Viewer3D", "QGCViewer3DManager");
             expect(drag && !drag->property("enabled").toBool() && !drag->isVisible(), "3D view: the video's tap and drag area is off",
                    QStringLiteral("display mode %1, the layer sees the 3D view: %2")
                        .arg(manager ? manager->property("displayMode").toString() : QStringLiteral("no manager"))
                        .arg(layer ? layer->property("_is3DView").toString() : QStringLiteral("no layer")));
             anglesBeforePoint = camera.angleCommands;
             lasersBeforePoint = camera.laserFrames;
             speedsBefore = camera.speedCommands;
             mouse(QEvent::MouseButtonPress, QPointF(1300, 620));
             mouse(QEvent::MouseButtonRelease, QPointF(1300, 620));
             mouse(QEvent::MouseButtonPress, QPointF(1000, 560));
         }},
        {100, [&]() { mouse(QEvent::MouseMove, QPointF(1040, 550)); }},
        {100, [&]() { mouse(QEvent::MouseMove, QPointF(1150, 470)); }},
        {300, [&]() { mouse(QEvent::MouseButtonRelease, QPointF(1150, 470)); }},
        {1500, [&]() {
             expect(camera.angleCommands == anglesBeforePoint && camera.laserFrames == lasersBeforePoint &&
                        camera.speedCommands == speedsBefore && !link.aimBusy(),
                    "3D view: a tap and a drag there do not turn the camera or fire the laser",
                    QStringLiteral("%1 turns, %2 laser frames, %3 speed frames").arg(camera.angleCommands - anglesBeforePoint)
                        .arg(camera.laserFrames - lasersBeforePoint).arg(camera.speedCommands - speedsBefore));
             window->setProperty("view3D", false);
         }},
        {300, [&]() {
             QQuickItem *drag = findItem("vamaVideoDrag");
             expect(drag && drag->property("enabled").toBool() && drag->isVisible(),
                    "3D view: back on the video, taps and drags work again", QString());
             // The admin password for Advanced Mode (task Z), in QGC's app menu.
             openMenu();
         }},
        {400, [&]() {
             QQuickItem *fly = menuItem("toolbar_viewFly");
             QQuickItem *analyze = menuItem("toolbar_viewAnalyze");
             expect(menuOpen() && fly && fly->isVisible() && !menuItem("vamaAdminPasswordField"),
                    "admin: the app menu opens with its buttons, no password asked", QString());
             expect(analyze && !analyze->isVisible(), "admin: no Analyze button outside Advanced Mode", QString());
             menuTap("vamaAdvancedModeTap");
         }},
        {300, [&]() {
             QQuickItem *field = menuItem("vamaAdminPasswordField");
             QQuickItem *fly = menuItem("toolbar_viewFly");
             QQuickItem *ok = menuItem("vamaAdminPasswordOk");
             expect(menuOpen() && field && field->isVisible() && fly && !fly->isVisible(),
                    "admin: the hidden tap asks for the admin password, in the menu in place of its buttons", QString());
             expect(field && field->hasFocus(), "admin: the password field has the keyboard", QString());
             expect(field && field->property("echoMode").toInt() == 2,  // TextInput.Password
                    "admin: the password shows as dots, not as typed",
                    field ? QStringLiteral("echo mode %1").arg(field->property("echoMode").toInt()) : QString());
             const qreal fieldBottom = field ? field->mapToScene(QPointF(0, field->height())).y() : 1e9;
             expect(fieldBottom < menuWindow->height() / 2.0, "admin: the field is in the top half, clear of the RC's keyboard",
                    QStringLiteral("down to %1 of %2").arg(fieldBottom, 0, 'f', 0).arg(menuWindow->height()));
             expect(!global.core.showAdvancedUI(), "admin: Advanced Mode is still off", QString());
             expect(ok && !ok->isEnabled(), "admin: OK is off while the field is empty", QString());
             shotOf(menuWindow, "29_admin_password.png");
             if (field) {
                 typeInto(field, QStringLiteral("not the password"));
             }
             menuTap("vamaAdminPasswordOk");
             // At once, not only at the panel's next timer tick (250 ms).
             expect(menuText("vamaAdminPasswordMessage") == QStringLiteral("Wrong password. Try again in 5 s."),
                    "admin: the wait shows at once", menuText("vamaAdminPasswordMessage"));
         }},
        {300, [&]() {
             QQuickItem *field = menuItem("vamaAdminPasswordField");
             QQuickItem *ok = menuItem("vamaAdminPasswordOk");
             expect(global.core.tries == 1 && !global.core.showAdvancedUI() && menuOpen(),
                    "admin: a wrong password leaves Advanced Mode off and the menu open", QStringLiteral("%1 tries").arg(global.core.tries));
             expect(menuText("vamaAdminPasswordMessage") == QStringLiteral("Wrong password. Try again in 5 s.") &&
                        menuItem("vamaAdminPasswordMessage")->isVisible(),
                    "admin: it says the password was wrong, and how long to wait", menuText("vamaAdminPasswordMessage"));
             expect(menuText("vamaAdminPasswordField").isEmpty(), "admin: the field is emptied", menuText("vamaAdminPasswordField"));
             // The right password now: during the wait OK is off, and the Enter key checks nothing.
             if (field) {
                 typeInto(field, QString::fromLatin1(vama::kTestAdminPassword));
             }
             expect(ok && !ok->isEnabled(), "admin: OK is off during the wait", QString());
             if (field) {
                 pressEnter(field);
             }
             shotOf(menuWindow, "30_admin_wrong.png");
         }},
        {300, [&]() {
             expect(global.core.tries == 1 && !global.core.showAdvancedUI(), "admin: during the wait Enter checks nothing",
                    QStringLiteral("%1 tries").arg(global.core.tries));
             // A tap beside the menu closes it, as in the app.
             menuTapAt(QPointF(1500, 900));
         }},
        {300, [&]() {
             expect(!menuOpen(), "admin: a tap beside the menu closes it", QString());
             openMenu();
         }},
        {400, [&]() {
             QQuickItem *fly = menuItem("toolbar_viewFly");
             expect(menuOpen() && fly && fly->isVisible() && !menuItem("vamaAdminPasswordField"),
                    "admin: opened again, the menu has its buttons (the panel went with the old menu)", QString());
             menuTap("vamaAdvancedModeTap");
         }},
        {300, [&]() {
             expect(menuText("vamaAdminPasswordMessage") == QStringLiteral("Try again in 5 s."),
                    "admin: the wait goes on in the new menu", menuText("vamaAdminPasswordMessage"));
             global.core.nowMs += 5000;  // the wait is over
         }},
        {500, [&]() {
             QQuickItem *message = menuItem("vamaAdminPasswordMessage");
             QQuickItem *field = menuItem("vamaAdminPasswordField");
             expect(message && !message->isVisible(), "admin: after the wait the message goes", menuText("vamaAdminPasswordMessage"));
             if (field) {
                 typeInto(field, QString::fromLatin1(vama::kTestAdminPassword));
                 pressEnter(field);  // the keyboard's Enter (Done on the RC)
             }
         }},
        {300, [&]() {
             expect(global.core.tries == 2 && global.core.showAdvancedUI(), "admin: the right password turns Advanced Mode on",
                    QStringLiteral("%1 tries").arg(global.core.tries));
             expect(!menuOpen(), "admin: and the menu closes", QString());
             openMenu();
         }},
        {400, [&]() {
             QQuickItem *analyze = menuItem("toolbar_viewAnalyze");
             expect(analyze && analyze->isVisible(), "admin: Advanced Mode shows the Analyze button", QString());
             menuTap("vamaAdvancedModeTap");
         }},
        {300, [&]() {
             expect(menuWindow->property("offQuestions").toInt() == 1 && !menuOpen() && !menuItem("vamaAdminPasswordField"),
                    "admin: in Advanced Mode the tap asks QGC's question to turn it off, with no password",
                    QStringLiteral("%1 questions").arg(menuWindow->property("offQuestions").toInt()));
             menuWindow->close();
             // The cameras the admin adds (task AA), from the camera settings.
             global.core.setShowAdvancedUI(false);
             QMetaObject::invokeMethod(window->findChild<QObject *>(QStringLiteral("vamaCameraSettings")), "open");
         }},
        {400, [&]() {
             QQuickItem *button = findItem("vamaCamerasButton");
             expect(button && !button->isVisible(), "cameras: no Cameras button outside Advanced Mode", QString());
             global.core.setShowAdvancedUI(true);
         }},
        {300, [&]() {
             QQuickItem *button = findItem("vamaCamerasButton");
             expect(button && button->isVisible(), "cameras: in Advanced Mode the camera settings have a Cameras button", QString());
             // The window was left scrolled down by an earlier step: back to its top, where the button is.
             QObject *flick = window->findChild<QObject *>(QStringLiteral("vamaCameraSettingsFlick"));
             if (flick) {
                 flick->setProperty("contentY", 0);
             }
         }},
        {200, [&]() { tap("vamaCamerasButton"); }},
        {500, [&]() {
             QObject *editor = window->findChild<QObject *>(QStringLiteral("vamaCameraEditor"));
             expect(editor && editor->property("opened").toBool(), "cameras: the button opens the camera editor", QString());
             QQuickItem *copy = findItem("vamaCameraCopy_C13");
             QQuickItem *change = findItem("vamaCameraEdit_C13");
             QQuickItem *remove = findItem("vamaCameraDelete_C13");
             expect(copy && copy->isVisible() && change && !change->isVisible() && remove && !remove->isVisible(),
                    "cameras: a built-in camera can be copied, not changed or deleted", QString());
             shot("31_cameras.png");
             tapFound("vamaCameraCopy_C13");
         }},
        {400, [&]() {
             QQuickItem *name = findItem("vamaCameraName");
             expect(name && name->isVisible() && name->property("text").toString() == QStringLiteral("V13 copy"),
                    "cameras: Copy opens the form with the V13's values", name ? name->property("text").toString() : QStringLiteral("no form"));
             if (!name) {
                 return;
             }
             // A camera without laser, photo, recording, zoom, thermal picture or tracker of its own,
             // at the preview's fake camera. The port is wrong first.
             name->setProperty("text", QStringLiteral("Plain cam"));
             setOn("vamaCameraHost", "text", QStringLiteral("127.0.0.1"));
             setOn("vamaCameraPort", "text", QStringLiteral("0"));
             setOn("vamaCameraDayVideo", "text", QStringLiteral("rtsp://127.0.0.1:8554/plain"));
             setOn("vamaCameraThermalVideo", "text", QString());
             setOn("vamaCameraZoom", "currentIndex", 0);
             for (const char *box : {"vamaCameraLaser", "vamaCameraPhoto", "vamaCameraRecord", "vamaCameraOwnTracker"}) {
                 setOn(box, "checked", false);
             }
             // The buttons at the end of the form can be below the visible part: pressed as the button does.
             press("vamaCameraSave");
         }},
        {300, [&]() {
             QQuickItem *problem = findItem("vamaCameraProblem");
             const QString says = problem ? problem->property("text").toString() : QString();
             expect(problem && problem->isVisible() && says.contains(QStringLiteral("control port")) && !link.models().contains(QStringLiteral("plain-cam")),
                    "cameras: a wrong value is not saved, and the form says why", says);
             setOn("vamaCameraPort", "text", QString::number(camera.port()));
             shot("32_camera_form.png");
             press("vamaCameraSave");
         }},
        {400, [&]() {
             expect(link.models().contains(QStringLiteral("plain-cam")) && QFile::exists(cameraFolder + QStringLiteral("/plain-cam.json")),
                    "cameras: Save adds the camera, as a file", link.models().join(QStringLiteral(", ")));
             QQuickItem *change = findItem("vamaCameraEdit_plain-cam");
             QQuickItem *remove = findItem("vamaCameraDelete_plain-cam");
             expect(change && change->isVisible() && remove && remove->isVisible(), "cameras: an added camera can be changed and deleted", QString());
             press("vamaCameraEditorClose");
         }},
        {300, [&]() {
             // Chosen in the camera settings' list, as the user does.
             QQuickItem *box = findItem("vamaCameraModelBox");
             if (box) {
                 QMetaObject::invokeMethod(box, "activated", Q_ARG(int, int(link.models().indexOf(QStringLiteral("plain-cam")))));
             } else {
                 expect(false, "cameras: the camera settings have the camera list", QString());
             }
         }},
        {500, [&]() {
             expect(link.model() == QStringLiteral("plain-cam") && link.host() == QStringLiteral("127.0.0.1") && link.port() == camera.port() &&
                        link.dayVideoUrl() == QStringLiteral("rtsp://127.0.0.1:8554/plain") && link.thermalVideoUrl().isEmpty(),
                    "cameras: choosing it uses it, with its addresses",
                    QStringLiteral("%1 at %2:%3, %4").arg(link.model(), link.host()).arg(link.port()).arg(link.dayVideoUrl()));
             const QString video = global.settings.video.rtspUrlFact.rawValue().toString();
             expect(video == QStringLiteral("rtsp://127.0.0.1:8554/plain"), "cameras: QGC's video is its day picture", video);
             QMetaObject::invokeMethod(window->findChild<QObject *>(QStringLiteral("vamaCameraSettings")), "close");
             // Its thermal address is empty: an empty video address must not count as IR.
             global.settings.video.rtspUrlFact.setRawValue(QString());
         }},
        {300, [&]() {
             QQuickItem *palette = findItem("vamaPaletteButton");
             expect(!link.thermalPicture() && palette && !palette->isVisible(),
                    "cameras: with no thermal picture, an empty video address is not IR", QString());
             global.settings.video.rtspUrlFact.setRawValue(QStringLiteral("rtsp://127.0.0.1:8554/plain"));
         }},
        {400, [&]() {
             QStringList shown;
             for (const char *button : {"vamaLaserButton", "vamaPhotoButton", "vamaRecordButton", "vamaIrButton", "vamaZoomInButton"}) {
                 QQuickItem *item = findItem(button);
                 if (!item || item->isVisible()) {
                     shown.append(QString::fromLatin1(button));
                 }
             }
             QQuickItem *lock = findItem("vamaLockButton");
             expect(shown.isEmpty() && lock && lock->isVisible(),
                    "cameras: its screen has no laser, photo, record, IR or zoom button, and still the lock", shown.join(QStringLiteral(", ")));
             shot("33_plain_camera.png");
             // Deleted while in use: the V13 again, with the addresses as they are.
             QMetaObject::invokeMethod(window->property("layer").value<QObject *>(), "_openCameraEditor");
         }},
        {400, [&]() { tapFound("vamaCameraDelete_plain-cam"); }},
        {300, [&]() { tapFound("vamaCameraDeleteYes"); }},
        {400, [&]() {
             expect(link.model() == QStringLiteral("C13") && !link.models().contains(QStringLiteral("plain-cam")) &&
                        !QFile::exists(cameraFolder + QStringLiteral("/plain-cam.json")) && link.host() == QStringLiteral("127.0.0.1"),
                    "cameras: deleting the camera in use: the file is gone, the V13 is used, the addresses stay",
                    QStringLiteral("%1 at %2").arg(link.model(), link.host()));
             QQuickItem *laser = findItem("vamaLaserButton");
             expect(laser && laser->isVisible(), "cameras: the V13's buttons are back", QString());
             press("vamaCameraEditorClose");
             // A map pack loading, then loaded (task AB).
             global.core.setMapPackLoading(QStringLiteral("Loading map pack \"kalol\": 2374 map pictures, zoom 10 to 18. "
                                                          "The map shows them when this is done."));
         }},
        {300, [&]() {
             QQuickItem *chip = findItem("vamaMapPackLoading");
             QQuickItem *text = findItem("vamaMapPackLoadingText");
             expect(chip && chip->isVisible() && text && text->property("text").toString().startsWith(QStringLiteral("Loading map pack \"kalol\"")),
                    "map pack: while it loads, the fly view says so", text ? text->property("text").toString() : QString());
             shot("34_map_pack_loading.png");
             global.core.setMapPackLoading(QString());
             // Loaded, and the drone has no position: the map goes to the pack.
             global.vehicles.vehicle.position.set(false);
             emit global.core.mapPackLoaded(20.4458, 72.8634);
         }},
        {300, [&]() {
             QQuickItem *chip = findItem("vamaMapPackLoading");
             expect(chip && !chip->isVisible(), "map pack: the line goes when loading has ended", QString());
             QObject *layer = window->property("layer").value<QObject *>();
             QObject *map = layer ? layer->property("mapControl").value<QObject *>() : nullptr;
             const QGeoCoordinate centre = map ? map->property("center").value<QGeoCoordinate>() : QGeoCoordinate();
             expect(map && std::abs(centre.latitude() - 20.4458) < 1e-9 && std::abs(centre.longitude() - 72.8634) < 1e-9 &&
                        map->property("zoomLevel").toDouble() == 12.0,
                    "map pack: with no position from the drone, the small map goes to the pack, 3 zooms out",
                    QStringLiteral("%1, %2 at zoom %3").arg(centre.latitude(), 0, 'f', 4).arg(centre.longitude(), 0, 'f', 4)
                        .arg(map ? map->property("zoomLevel").toDouble() : 0.0));
             expect(std::abs(global.flightMapPosition.latitude() - 20.4458) < 1e-9 && global.flightMapZoom == 15.0,
                    "map pack: and QGC's map position is the pack's, at zoom 15",
                    QStringLiteral("zoom %1").arg(global.flightMapZoom));
             // With a position from the drone, QGC's map stays with the drone.
             global.vehicles.vehicle.position.set(true);
             if (map) {
                 map->setProperty("center", QVariant::fromValue(QGeoCoordinate(18.52, 73.85)));
             }
             emit global.core.mapPackLoaded(20.4458, 72.8634);
         }},
        {300, [&]() {
             QObject *layer = window->property("layer").value<QObject *>();
             QObject *map = layer ? layer->property("mapControl").value<QObject *>() : nullptr;
             const QGeoCoordinate centre = map ? map->property("center").value<QGeoCoordinate>() : QGeoCoordinate();
             expect(map && std::abs(centre.latitude() - 18.52) < 1e-9, "map pack: with a position from the drone, the map is not moved",
                    QStringLiteral("%1").arg(centre.latitude(), 0, 'f', 4));
         }},
        {100, [&]() { QCoreApplication::quit(); }},
    };
    // Each step starts its delay when the step before it has finished. Timers
    // counted from the start were "coarse" past 2 s (Qt allows 5 %), so late
    // steps could swap: once the quit ran before the Application Settings check.
    qsizetype stepsRun = 0;
    std::function<void()> runNext = [&]() {
        if (stepsRun >= steps.size()) {
            return;
        }
        const auto &step = steps.at(stepsRun);
        QTimer::singleShot(step.first, Qt::PreciseTimer, &app, [&, run = step.second]() {
            ++stepsRun;
            run();
            runNext();
        });
    };
    runNext();

    const int code = app.exec();
    if (stepsRun != steps.size()) {
        std::fprintf(stderr, "only %d of %d steps ran\n", int(stepsRun), int(steps.size()));
        ++g_problems;
    }
    std::fprintf(stderr, "QML problems: %d\n", g_problems);
    return (code == 0 && g_problems == 0) ? 0 : 1;
}

#include "main.moc"
