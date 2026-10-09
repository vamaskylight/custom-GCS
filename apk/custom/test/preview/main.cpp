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
//   17_app_settings.png   Application Settings: the VAMA mark on General, no Help page
//   18_warnings_small.png QGC's "No GPS Lock" and pre-arm texts, small at the bottom while the video is the main view
//   19_warnings_map.png   the same texts as QGC shows them, with the map as the main view
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
#include <QtCore/QSettings>
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

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <utility>

#include "ColoredSvgImageProvider.h"
#include "MultiVehicleManager.h"
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

class FakeGlobal : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QObject *videoManager READ videoManager CONSTANT)
    Q_PROPERTY(QObject *settingsManager READ settingsManager CONSTANT)
    Q_PROPERTY(QObject *multiVehicleManager READ multiVehicleManager CONSTANT)
    Q_PROPERTY(qreal zOrderTopMost READ zOrderTopMost CONSTANT)

public:
    QObject *videoManager() { return &_videoManager; }
    qreal zOrderTopMost() const { return 1000; }  // QGC's value
    QObject *settingsManager() { return &settings; }
    QObject *multiVehicleManager() { return &vehicles; }
    FakeSettingsManager settings;
    FakeQmlVehicleManager vehicles;

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

private slots:
    void _move()
    {
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
};

/// The app's override rule (CustomOverrideInterceptor in CustomPlugin.cc): a
/// qrc address is answered by its copy under /Custom when custom.qrc has one.
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
    SkydroidLink link;
    link.setHost(QStringLiteral("127.0.0.1"));
    link.setPort(camera.port());
    link.setEnabled(true);

    FakeGlobal global;
    qmlRegisterSingletonInstance("QGroundControl", 1, 0, "SkydroidLink", &link);
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
    auto mouse = [&](QEvent::Type type, QPointF pos) {
        const Qt::MouseButtons buttons = type == QEvent::MouseButtonRelease ? Qt::NoButton : Qt::LeftButton;
        const Qt::MouseButton button = type == QEvent::MouseMove ? Qt::NoButton : Qt::LeftButton;
        QMouseEvent event(type, pos, window->mapToGlobal(pos), button, buttons, Qt::NoModifier);
        QCoreApplication::sendEvent(window, &event);
    };
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
    auto tapItem = [&](QQuickItem *item) {
        const QPointF centre = item->mapToScene(QPointF(item->width() / 2, item->height() / 2));
        mouse(QEvent::MouseButtonPress, centre);
        mouse(QEvent::MouseButtonRelease, centre);
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
             tap("vamaLockButton");
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
             tap("vamaLockButton");
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
             tap("vamaLockButton");
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
