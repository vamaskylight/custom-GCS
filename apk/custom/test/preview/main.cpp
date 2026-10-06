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
// It also taps the IR button twice and checks QGC's video address each time.
// Every QML warning or error is printed; the exit code is 1 when there was one.

#include <QtCore/QDir>
#include <QtCore/QSettings>
#include <QtCore/QTimer>
#include <QtGui/QGuiApplication>
#include <QtGui/QImage>
#include <QtGui/QMouseEvent>
#include <QtNetwork/QNetworkDatagram>
#include <QtNetwork/QUdpSocket>
#include <QtQml/QQmlApplicationEngine>
#include <QtQuick/QQuickItem>
#include <QtQuick/QQuickWindow>

#include <cstdio>
#include <functional>
#include <utility>

#include "MultiVehicleManager.h"
#include "SkydroidLink.h"
#include "SkydroidTop.h"
#include "Vehicle.h"

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

class FakeGlobal : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QObject *videoManager READ videoManager CONSTANT)
    Q_PROPERTY(QObject *settingsManager READ settingsManager CONSTANT)

public:
    QObject *videoManager() { return &_videoManager; }
    QObject *settingsManager() { return &settings; }
    FakeSettingsManager settings;

private:
    FakeVideoManager _videoManager;
};

/// Answers angle questions and laser reads like a C13.
class FakeCamera : public QObject
{
    Q_OBJECT

public:
    FakeCamera()
    {
        _socket.bind(QHostAddress::LocalHost, 0);
        connect(&_socket, &QUdpSocket::readyRead, this, &FakeCamera::_read);
    }
    quint16 port() const { return _socket.localPort(); }

    double yaw = -12.4;
    double pitch = -31.5;
    int angleCommands = 0;  // GAY and GAP frames received (tap aiming)

private slots:
    void _read()
    {
        while (_socket.hasPendingDatagrams()) {
            const QNetworkDatagram d = _socket.receiveDatagram();
            const auto f = top::parseTpFrame(std::string(d.data().constData(), static_cast<size_t>(d.data().size())));
            // The gimbal turns to angle commands at once.
            if (f && (f->tag == "GAY" || f->tag == "GAP") && f->ctrl == 'w' && f->data.size() >= 4) {
                if (const auto angle = top::decodeAttitudeField4(f->data.substr(0, 4))) {
                    (f->tag == "GAY" ? yaw : pitch) = *angle;
                }
                ++angleCommands;
                continue;
            }
            if (!f || f->ctrl != 'r') {
                continue;
            }
            std::string reply;
            if (f->tag == "GAC") {
                reply = top::buildTpFrame('U', 'r', "GAC",
                                          top::encodeAttitudeField4(yaw) + top::encodeAttitudeField4(pitch) +
                                              top::encodeAttitudeField4(0.0),
                                          'G', 1);
            } else if (f->tag == "SLR" && f->address.size() == 2 && f->address[1] == 'E') {
                reply = top::buildTpFrame('U', 'r', "SLR", "03A0", 'E', 1);  // 92.8 m
            }
            if (!reply.empty()) {
                _socket.writeDatagram(reply.data(), static_cast<qint64>(reply.size()), d.senderAddress(),
                                      d.senderPort());
            }
        }
    }

private:
    QUdpSocket _socket;
};

int main(int argc, char *argv[])
{
    qputenv("QT_QUICK_CONTROLS_STYLE", "Basic");
    qInstallMessageHandler(messageHandler);
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

    QQmlApplicationEngine engine;
    engine.addImportPath(QStringLiteral(VAMA_PREVIEW_DIR "/stubs"));
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

    // The steps, one after another (delay in ms before each).
    const QList<std::pair<int, std::function<void()>>> steps = {
        {1500, [&]() {
             link.zoom(1);
             link.zoom(1);
             link.zoom(1);
             // Tap the photo button (left column, top) to run its click handler.
             mouse(QEvent::MouseButtonPress, QPointF(244, 336));
             mouse(QEvent::MouseButtonRelease, QPointF(244, 336));
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
             expect(link.dayVideoUrl() == QStringLiteral("rtsp://192.168.144.108:554/main"),
                    "IR on: day address learned from QGC", link.dayVideoUrl());
             shot("7_ir_on.png");
             tap("vamaIrButton");
         }},
        {300, [&]() {
             const QString url = global.settings.video.rtspUrlFact.rawValue().toString();
             expect(url == QStringLiteral("rtsp://192.168.144.108:554/main"), "IR off: QGC video is the day stream again", url);
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
        {600, [&]() { shot("6_camera_off.png"); }},
        {100, [&]() { QCoreApplication::quit(); }},
    };
    int delay = 0;
    for (const auto &step : steps) {
        delay += step.first;
        QTimer::singleShot(delay, &app, step.second);
    }

    const int code = app.exec();
    std::fprintf(stderr, "QML problems: %d\n", g_problems);
    return (code == 0 && g_problems == 0) ? 0 : 1;
}

#include "main.moc"
