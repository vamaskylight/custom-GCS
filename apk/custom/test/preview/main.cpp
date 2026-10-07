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
//   12_lock_drawing.png   a box being drawn around the object
//   13_locked.png         locked: box on the centre mark, lock state under the laser result
//   14_lock_following.png the camera follows (the fake camera turns by itself)
//   15_lock_stopped.png   after the lock button again
//   16_no_gps_lock.png    a laser shot while the drone has no GPS lock: distance, and why no lat long
//   17_app_settings.png   Application Settings: the VAMA mark on General, no Help page
// It also taps the IR button twice and checks QGC's video address each time,
// and checks the frames the camera got for the tap and the lock.
// With IR on it picks Ironbow in the thermal colour list, then checks every
// mode against VGCS's colour table at 64 grey levels of the thermal picture,
// and that the black bars beside the picture stay black. The video is QGC's
// own address, answered by the VAMA copy (FlightDisplayViewVideoOutput.qml).
// Application Settings is opened by QGC's own address, through the app's
// override rule, and checked for the client feedback of 2026-10-07 (tasks E
// and F in DOCS/APK-V1-DEV-PLAN.md).
// Every QML warning or error is printed; the exit code is 1 when there was one.

#include <QtCore/QDir>
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
#include <QtQuick/QQuickItem>
#include <QtQuick/QQuickWindow>

#include <cstdio>
#include <functional>
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

class FakeGlobal : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QObject *videoManager READ videoManager CONSTANT)
    Q_PROPERTY(QObject *settingsManager READ settingsManager CONSTANT)
    Q_PROPERTY(qreal zOrderTopMost READ zOrderTopMost CONSTANT)

public:
    QObject *videoManager() { return &_videoManager; }
    qreal zOrderTopMost() const { return 1000; }  // QGC's value
    QObject *settingsManager() { return &settings; }
    FakeSettingsManager settings;

private:
    FakeVideoManager _videoManager;
};

/// Answers angle questions and laser reads like the client's V13 (C13). Like it,
/// it ignores long gimbal frames that start with a lower-case "#tp" (test build 3).
class FakeCamera : public QObject
{
    Q_OBJECT

public:
    FakeCamera()
    {
        _socket.bind(QHostAddress::LocalHost, 0);
        connect(&_socket, &QUdpSocket::readyRead, this, &FakeCamera::_read);
        connect(&_follow, &QTimer::timeout, this, [this]() { yaw += 0.3; });
    }
    quint16 port() const { return _socket.localPort(); }

    double yaw = -12.4;
    double pitch = -31.5;
    int angleCommands = 0;  // GAY and GAP frames received (tap aiming)
    int gotFrames = 0;      // lock: GOT
    int sumConfirms = 0;    // lock: SUM 01
    int sumStops = 0;       // lock: SUM 00

private slots:
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
                    (f->tag == "GAY" ? yaw : pitch) = *angle;
                }
                ++angleCommands;
                continue;
            }
            if (f && f->tag == "GOT" && f->ctrl == 'w') {
                ++gotFrames;
                continue;
            }
            // After a SUM confirm the camera follows an object that moves slowly (3 deg/s).
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
    QTimer _follow;
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
             camera.pitch = 3.0;
             tap("vamaLockButton");
         }},
        {400, [&]() {
             shot("11_lock_pick.png");
             anglesBeforeLock = camera.angleCommands;
             // Draw a box around the light building, left of the centre.
             mouse(QEvent::MouseButtonPress, QPointF(700, 380));
         }},
        {100, [&]() { mouse(QEvent::MouseMove, QPointF(760, 430)); }},
        {100, [&]() { mouse(QEvent::MouseMove, QPointF(860, 520)); }},
        {300, [&]() {
             shot("12_lock_drawing.png");
             expect(camera.angleCommands == anglesBeforeLock, "lock: drawing the box does not move the camera",
                    QString::number(camera.angleCommands));
             mouse(QEvent::MouseButtonRelease, QPointF(860, 520));
         }},
        {1800, [&]() {
             expect(link.lockActive(), "lock: locked after the turn", link.lockMessage());
             expect(camera.angleCommands >= anglesBeforeLock + 2, "lock: the camera turned to the box first (GAY and GAP)",
                    QString::number(camera.angleCommands));
             expect(camera.gotFrames == 1 && camera.sumConfirms >= 1, "lock: GOT, then SUM confirm",
                    QStringLiteral("GOT %1, SUM 01 %2").arg(camera.gotFrames).arg(camera.sumConfirms));
             shot("13_locked.png");
         }},
        {3500, [&]() {
             expect(link.lockFollowSeen(), "lock: the camera is seen following",
                    QStringLiteral("%1 deg").arg(link.lockTurnedDeg()));
             expect(link.laserValid(), "lock: the laser measured the locked object",
                    QStringLiteral("%1 m").arg(link.laserRangeM()));
             shot("14_lock_following.png");
             tap("vamaLockButton");
         }},
        {400, [&]() {
             expect(!link.lockActive() && camera.sumStops >= 1, "lock: stopped with SUM stop",
                    QStringLiteral("SUM 00 %1").arg(camera.sumStops));
             shot("15_lock_stopped.png");
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
        {1000, [&]() { checkAppSettings(); }},
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
