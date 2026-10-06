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

public:
    bool fullScreen = false;

signals:
    void fullScreenChanged();
};

class FakeGlobal : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QObject *videoManager READ videoManager CONSTANT)

public:
    QObject *videoManager() { return &_videoManager; }

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

private slots:
    void _read()
    {
        while (_socket.hasPendingDatagrams()) {
            const QNetworkDatagram d = _socket.receiveDatagram();
            const auto f = top::parseTpFrame(std::string(d.data().constData(), static_cast<size_t>(d.data().size())));
            if (!f || f->ctrl != 'r') {
                continue;
            }
            std::string reply;
            if (f->tag == "GAC") {
                reply = top::buildTpFrame('U', 'r', "GAC",
                                          top::encodeAttitudeField4(-12.4) + top::encodeAttitudeField4(-31.5) +
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
