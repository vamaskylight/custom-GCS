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
//   14d_lock_check_not_there.png  the finder sees no person on the box in three looks: NOT SEEN (task AC)
//   14e_lock_found_again.png   the walker came out further on, and the finder found them again (task AC)
//   14c_lock_not_seen.png      the walker is gone: the box turns red, the camera waits
//   15_lock_lost.png      eight seconds later the lock has ended and says why
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
//   33_plain_camera.png   that camera in use: its buttons only (and photo and record, for the copy on the RC)
//   34_map_pack_loading.png  a map pack loading: the fly view says so (task AB)
//   35_photo_note.png     after the photo button: a note beside it says the photo is on the camera's SD card,
//                         and that a copy is on the RC (task AL)
//   36_recording.png      the camera records and says so ("REC"), and the RC records a copy ("RC REC")
//   37_no_sd_card.png     a camera without an SD card: the status says so, and the note of a photo
//   38_digital_zoom.png   the app's own zoom of the picture at 2x, and its button ("D 2x", B10)
//   40_voice.png          the app menu's Voice panel: the voice the app speaks with (the client, 2026-10-30)
//   40b_voice_note.png    the same without an Indian English voice: the note says where to add one
//   41_vehicle_config.png         Vehicle Configuration outside Advanced Mode: Summary, Radio, Sensors
//   41b_vehicle_config_advanced.png  the same in Advanced Mode: every line, and Parameters
//   41c_vehicle_config_why.png    a hidden component opened another way: why, not its page
//   42_proximity_view.png         the round proximity view's box: front and right within the range (B25, 2026-10-31)
//   42b_proximity_view_moved.png  the same box, moved and made bigger by the user
//   43_obstacle_range.png         the app menu's panel for the range of the round view
//   44_license_lock.png           the license lock over the whole window: the RC's device code (row A9)
//   44b_license_wrong.png         after a key for another RC: why it does not open the app
//   44c_license_open.png          after the right key: the lock is gone, QGC's dialog under it takes taps
//   44d_license_plain.png         the plain screen when the lock screen cannot be made: why, the device code
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
// Real mouse and key input from the PC is dropped and counted (RealInputBlocker): the developer
// works on the same PC while the preview runs.

#include <QtCore/QDir>
#include <QtCore/QElapsedTimer>
#include <QtCore/QFile>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QLoggingCategory>
#include <QtCore/QMutex>
#include <QtCore/QPointer>
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
#include <QtQml/QQmlComponent>
#include <QtQml/QQmlContext>
#include <QtQuick/QQuickItem>
#include <QtQuick/QQuickWindow>
#include <QtTest/QTest>

#include <array>
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
#include "PositionSourceStore.h"
#include "ProximityStore.h"
#include "SkydroidLink.h"
#include "SkydroidTop.h"
#include "VamaLicense.h"
#include "VamaVoice.h"
#include "Vehicle.h"

// Keys and device codes made by VGCS's own rules (gen_license_vectors.py), for the license lock (row A9).
namespace licenseVectors {
#include "../license_key_vectors.inc"
}

// VGCS's thermal colour modes (made by apk/make_thermal_palettes.py).
#include "../thermal_palette_vectors.inc"

namespace top = skydroid::top;

namespace {

int g_problems = 0;
// Warnings a step makes on purpose (the plain lock screen, row A9): each one is passed over once, and
// the step checks that it came.
QStringList g_expectedWarnings;

void messageHandler(QtMsgType type, const QMessageLogContext &, const QString &message)
{
    std::fprintf(stderr, "%s\n", qPrintable(message));
    // Windows says this when another program holds the clipboard at that moment: about the PC,
    // not the app (it cost one run of six a problem, 2026-10-15).
    if (message == QLatin1String("Retrying to obtain clipboard.")) {
        return;
    }
    for (qsizetype i = 0; i < g_expectedWarnings.size(); ++i) {
        if (type == QtWarningMsg && message.startsWith(g_expectedWarnings.at(i))) {
            g_expectedWarnings.removeAt(i);
            return;
        }
    }
    if (type != QtDebugMsg && type != QtInfoMsg) {
        ++g_problems;
    }
}

} // namespace

/// Real mouse, wheel and key input from the PC is not for the preview: the developer works on the
/// same PC while it runs. On 2026-10-26 a click meant for another window landed on the preview's
/// video, measured a point there, and the laser button hid that reading in place of measuring. The
/// preview's own input is sent with sendEvent, which is not "spontaneous", so it still arrives.
/// (Not with -Touch: QTest's touches come the way real ones do.)
class RealInputBlocker : public QObject
{
public:
    int blocked = 0;

protected:
    bool eventFilter(QObject *watched, QEvent *event) override
    {
        if (event->spontaneous()) {
            switch (event->type()) {
            case QEvent::MouseButtonPress:
            case QEvent::MouseButtonRelease:
            case QEvent::MouseButtonDblClick:
            case QEvent::MouseMove:
            case QEvent::Wheel:
            case QEvent::KeyPress:
            case QEvent::KeyRelease:
                ++blocked;
                return true;
            default:
                break;
            }
        }
        return QObject::eventFilter(watched, event);
    }
};

class FakeVideoManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool fullScreen MEMBER fullScreen NOTIFY fullScreenChanged)
    Q_PROPERTY(double aspectRatio READ aspectRatio CONSTANT)
    // QGC's recording of the video it receives, and its picture of it: the copy on the RC (task AL).
    Q_PROPERTY(bool recording READ recording NOTIFY recordingChanged)
    Q_PROPERTY(bool decoding READ decoding NOTIFY decodingChanged)

public:
    bool fullScreen = false;
    double aspectRatio() const { return 16.0 / 9.0; }  // the C13 stream, 1280 x 720
    bool recording() const { return _recording; }
    bool decoding() const { return _decoding; }
    void setDecoding(bool on)
    {
        if (on != _decoding) {
            _decoding = on;
            emit decodingChanged();
        }
    }
    // As QGC's: it names the file, and the video item (FlightDisplayViewVideoOutput.qml) grabs its
    // picture and writes it there. Here into the preview's output folder.
    Q_INVOKABLE void grabImage(const QString &file = QString())
    {
        ++grabs;
        lastImageFile = file.isEmpty() ? QStringLiteral("%1/rc_copy_%2.jpg").arg(photoFolder).arg(grabs) : file;
        QFile::remove(lastImageFile);  // one of a run before must not count
        emit imageFileChanged(lastImageFile);
    }
    Q_INVOKABLE void startRecording(const QString & = QString())
    {
        ++recordStarts;
        if (!_recording) {
            _recording = true;
            emit recordingChanged(true);
        }
    }
    // QGC's: opens the video again (B13, after a Viewpro's sensor switch).
    Q_INVOKABLE void startVideo() { ++videoStarts; }
    Q_INVOKABLE void stopRecording()
    {
        ++recordStops;
        if (_recording) {
            _recording = false;
            emit recordingChanged(false);
        }
    }
    // QGC's recording ending by itself, with nobody pressing stop: its video restarted (a GStreamer
    // error, as on the client's RC with build 14, an IR switch, a lost link).
    void failRecording()
    {
        if (_recording) {
            _recording = false;
            emit recordingChanged(false);
        }
    }
    int grabs = 0;
    int recordStarts = 0;
    int recordStops = 0;
    int videoStarts = 0;
    QString photoFolder;
    QString lastImageFile;

signals:
    void fullScreenChanged();
    void imageFileChanged(const QString &filename);  // QGC's video snapshot
    void recordingChanged(bool recording);
    void decodingChanged();

private:
    bool _recording = false;
    bool _decoding = true;  // the stand-in video shows pictures
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

/// QGC's AppSettings: the folders of the copies on the RC. Empty while QGC has no save folder, as on
/// Android without the Storage permission (B14).
class FakeAppSettings : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString videoSavePath READ videoSavePath NOTIFY savePathsChanged)
    Q_PROPERTY(QString photoSavePath READ photoSavePath NOTIFY savePathsChanged)

public:
    QString videoSavePath() const { return hasFolder ? QStringLiteral("VAMA-GCS/Video") : QString(); }
    QString photoSavePath() const { return hasFolder ? QStringLiteral("VAMA-GCS/Photo") : QString(); }
    void setHasFolder(bool on)
    {
        hasFolder = on;
        emit savePathsChanged();
    }
    bool hasFolder = true;

signals:
    void savePathsChanged();
};

class FakeSettingsManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QObject *videoSettings READ videoSettings CONSTANT)
    Q_PROPERTY(QObject *appSettings READ appSettings CONSTANT)

public:
    QObject *videoSettings() { return &video; }
    QObject *appSettings() { return &app; }
    FakeVideoSettings video;
    FakeAppSettings app;
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

/// One component of QGC's Vehicle Configuration list, as VehicleConfigView.qml reads it. Its setup
/// page is a made-up address with QGC's file name, which is what the VAMA copy goes by.
class FakeVehicleComponent : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString name MEMBER name CONSTANT)
    Q_PROPERTY(QUrl setupSource MEMBER setupSource CONSTANT)
    Q_PROPERTY(QUrl summaryQmlSource READ summaryQmlSource CONSTANT)
    Q_PROPERTY(QString iconResource READ iconResource CONSTANT)
    Q_PROPERTY(bool setupComplete READ setupComplete CONSTANT)
    Q_PROPERTY(bool requiresSetup READ requiresSetup CONSTANT)
    Q_PROPERTY(QStringList sectionIds READ sectionIds CONSTANT)
    Q_PROPERTY(QVariantMap sectionKeywords READ sectionKeywords CONSTANT)
    Q_PROPERTY(QUrl vehicleConfigJson READ vehicleConfigJson CONSTANT)
    Q_PROPERTY(bool showFirstSectionOnRootClick READ showFirstSectionOnRootClick CONSTANT)

public:
    FakeVehicleComponent(const QString &componentName, const QString &page, QObject *parent)
        : QObject(parent), name(componentName), setupSource(QStringLiteral("qrc:/preview/vehicle/") + page)
    {
    }
    QUrl summaryQmlSource() const { return QUrl(); }
    QString iconResource() const { return QString(); }
    bool setupComplete() const { return true; }
    bool requiresSetup() const { return false; }
    QStringList sectionIds() const { return {}; }
    QVariantMap sectionKeywords() const { return {}; }
    QUrl vehicleConfigJson() const { return QUrl(); }
    bool showFirstSectionOnRootClick() const { return false; }

    QString name;
    QUrl setupSource;
};

/// The ArduPilot plugin's components, with QGC's setup page names (APMAutoPilotPlugin).
class FakeAutopilotPlugin : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVariantList vehicleComponents READ vehicleComponents CONSTANT)

public:
    explicit FakeAutopilotPlugin(QObject *parent) : QObject(parent)
    {
        const QList<QPair<QString, QString>> components = {
            {QStringLiteral("Frame"), QStringLiteral("APMAirframeComponent.qml")},
            {QStringLiteral("Radio"), QStringLiteral("RadioComponent.qml")},
            {QStringLiteral("Sensors"), QStringLiteral("APMSensorsComponent.qml")},
            {QStringLiteral("Failsafes"), QStringLiteral("APMFailsafesComponent.qml")},
            {QStringLiteral("Motors"), QStringLiteral("APMMotorComponent.qml")},
        };
        for (const auto &[name, page] : components) {
            _components.append(QVariant::fromValue<QObject *>(new FakeVehicleComponent(name, page, this)));
        }
    }
    QVariantList vehicleComponents() const { return _components; }
    Q_INVOKABLE QString prerequisiteSetup(QObject *component) const
    {
        Q_UNUSED(component);
        return QString();
    }

private:
    QVariantList _components;
};

/// QGC's parameter manager as VehicleConfigView.qml reads it: all parameters are there.
class FakeParameterManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(double loadProgress READ loadProgress CONSTANT)
    Q_PROPERTY(bool missingParameters READ missingParameters CONSTANT)
    Q_PROPERTY(bool parameterDownloadSkipped READ parameterDownloadSkipped CONSTANT)

public:
    using QObject::QObject;
    double loadProgress() const { return 0.0; }
    bool missingParameters() const { return false; }
    bool parameterDownloadSkipped() const { return false; }
    Q_INVOKABLE void refreshAllParameters() {}
};

/// What QGC's VehicleWarnings.qml reads of the active vehicle.
class FakeQmlVehicle : public QObject
{
    Q_OBJECT
    // Vehicle Configuration (the client, 2026-10-30).
    Q_PROPERTY(QObject *autopilotPlugin READ autopilotPlugin CONSTANT)
    Q_PROPERTY(QObject *parameterManager READ parameterManager CONSTANT)
    Q_PROPERTY(int flowImageIndex READ flowImageIndex CONSTANT)
    Q_PROPERTY(bool usingHighLatencyLink READ usingHighLatencyLink CONSTANT)
    Q_PROPERTY(int id READ id CONSTANT)
    Q_PROPERTY(bool requiresGpsFix READ requiresGpsFix CONSTANT)
    Q_PROPERTY(bool armed READ armed CONSTANT)
    Q_PROPERTY(QObject *coordinate READ coordinate CONSTANT)
    Q_PROPERTY(QString prearmError READ prearmError NOTIFY prearmErrorChanged)
    Q_PROPERTY(QObject *healthAndArmingCheckReport READ report CONSTANT)

public:
    int id() const { return 1; }  // its MAVLink system id
    bool requiresGpsFix() const { return true; }
    bool armed() const { return false; }
    QObject *coordinate() { return &position; }
    QObject *report() { return &_report; }
    QObject *autopilotPlugin() { return &_autopilot; }
    QObject *parameterManager() { return &_parameters; }
    int flowImageIndex() const { return 1; }  // it sends optical flow pictures: QGC shows an Optical Flow line
    bool usingHighLatencyLink() const { return false; }
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
    FakeAutopilotPlugin _autopilot{this};
    FakeParameterManager _parameters{this};
};

class FakeQmlVehicleManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QObject *activeVehicle READ activeVehicle CONSTANT)
    Q_PROPERTY(bool parameterReadyVehicleAvailable READ parameterReadyVehicleAvailable
                   NOTIFY parameterReadyVehicleAvailableChanged)

public:
    QObject *activeVehicle() { return &vehicle; }
    bool parameterReadyVehicleAvailable() const { return true; }
    FakeQmlVehicle vehicle;

signals:
    void parameterReadyVehicleAvailableChanged(bool parametersReady);
};

/// What the app menu reads of QGC's options.
class FakeCoreOptions : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool combineSettingsAndSetup READ combineSettingsAndSetup CONSTANT)
    Q_PROPERTY(bool showFirmwareUpgrade READ showFirmwareUpgrade CONSTANT)

public:
    bool combineSettingsAndSetup() const { return false; }  // QGC's default
    bool showFirmwareUpgrade() const { return false; }
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
    // CustomPlugin's ask for a camera window from the app menu (task AG).
    Q_INVOKABLE void requestCameraWindow(const QString &which) { emit cameraWindowRequested(which); }

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
    void cameraWindowRequested(const QString &which);

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
    Q_PROPERTY(QString appName READ appName CONSTANT)
    Q_PROPERTY(qreal zOrderTopMost READ zOrderTopMost CONSTANT)
    // QGC's saved map position and zoom (QGroundControlQmlGlobal).
    Q_PROPERTY(QGeoCoordinate flightMapPosition MEMBER flightMapPosition NOTIFY flightMapChanged)
    Q_PROPERTY(double flightMapZoom MEMBER flightMapZoom NOTIFY flightMapChanged)

public:
    QObject *videoManager() { return &videos; }
    qreal zOrderTopMost() const { return 1000; }  // QGC's value
    QObject *settingsManager() { return &settings; }
    QObject *multiVehicleManager() { return &vehicles; }
    QObject *corePlugin() { return &core; }
    QString qgcVersion() const { return QStringLiteral("v5.1.5"); }
    QString appName() const { return QStringLiteral("VAMA GCS"); }
    // QGC's test of its speech (General settings, and the voice in the app menu): counted, not spoken.
    Q_INVOKABLE void testAudioOutput() { ++audioTests; }
    int audioTests = 0;
    FakeSettingsManager settings;
    FakeQmlVehicleManager vehicles;
    FakeCorePlugin core;
    FakeVideoManager videos;
    QGeoCoordinate flightMapPosition;
    double flightMapZoom = 16.0;

signals:
    void flightMapChanged();
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
    // Photos and recording (task AL): it echoes CAP and REC as TOP says a camera does, records
    // from REC 01 to REC 00 (not without an SD card), and answers REC and SDC questions.
    int photos = 0;
    bool recording = false;
    std::string sdCard = "03200076E4";  // free MB, then total MB: 12.5 GB free of 29.7 GB
    static constexpr const char *kNoSdCard = "0000000000";
    QStringList texts;  // what came as text, not as a TOP frame: the V20's torch (row B27)
    int datagrams = 0;  // everything that came (row A9: nothing while the app is locked)

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
            ++datagrams;
            const std::string raw(d.data().constData(), static_cast<size_t>(d.data().size()));
            if (raw.rfind("#tp", 0) == 0) {
                continue;
            }
            if (raw.rfind("AT+", 0) == 0) {
                texts << QString::fromStdString(raw);
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
            if (f && (f->tag == "CAP" || f->tag == "REC") && f->ctrl == 'w') {
                if (f->tag == "CAP") {
                    ++photos;
                } else {
                    recording = f->data == "01" && sdCard != kNoSdCard;
                }
                const std::string echo = top::buildTpFrame('U', 'w', f->tag, f->data, 'D', 0);
                _socket.writeDatagram(echo.data(), static_cast<qint64>(echo.size()), d.senderAddress(), d.senderPort());
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
            } else if (f->tag == "REC") {
                reply = top::buildTpFrame('U', 'r', "REC", recording ? "01" : "00", 'D', 0);
            } else if (f->tag == "SDC") {
                reply = top::buildTpFrame('U', 'r', "SDC", sdCard, 'D', 1);
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

/// The "transport" a camera file holds (A1, the C20), or what is wrong with the file.
QString cameraTransport(const QString &path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        return QStringLiteral("no file %1").arg(path);
    }
    return QJsonDocument::fromJson(file.readAll()).object().value(QStringLiteral("transport")).toString(QStringLiteral("no transport"));
}

/// One value of a camera file (task AK): nothing when there is no file.
QJsonValue cameraFileValue(const QString &path, const char *key)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        return QJsonValue(QJsonValue::Undefined);
    }
    return QJsonDocument::fromJson(file.readAll()).object().value(QLatin1String(key));
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
    QSettings().remove(QStringLiteral("VamaVoice"));
    QSettings().remove(QStringLiteral("VamaProximity"));
    QSettings().remove(QStringLiteral("VamaProximityWindow"));  // the round view's place and size (B25)
    QSettings().remove(QStringLiteral("VamaLicense"));          // the saved license key (row A9)

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
    global.videos.photoFolder = outDir;  // the copies on the "RC" (task AL)
    qmlRegisterSingletonInstance("QGroundControl", 1, 0, "SkydroidLink", &link);
    qmlRegisterSingletonInstance("QGroundControl", 1, 0, "ObjectFollow", &follow);
    qmlRegisterSingletonInstance("QGroundControl", 1, 0, "ObjectMarks", &marks);
    // Where the drone's position comes from (A8 step 4), on a clock the steps move.
    PositionSourceStore positionStore(nullptr);
    qint64 positionMs = 1000;
    positionStore.setClock([&positionMs]() { return positionMs; });
    qmlRegisterSingletonInstance("QGroundControl", 1, 0, "PositionSourceStore", &positionStore);
    // The obstacle sensor (B19), on a clock that stands still: a reading stays until the steps change it.
    ProximityStore proximity(nullptr);
    proximity.setClock([]() { return qint64(1000); });
    qmlRegisterSingletonInstance("QGroundControl", 1, 0, "ProximityStore", &proximity);
    // The voice the app speaks with (the client, 2026-10-30): an older Google engine's English voices,
    // whose names tell the gender. No engine: nothing is spoken.
    using vama::voice::Gender;
    const std::vector<vama::voice::Voice> olderVoices = {
        {"en-us-x-sfg#female_1-local", "en", "US", Gender::Female},
        {"en-in-x-cxx#male_1-local", "en", "IN", Gender::Male},
        {"en-in-x-cxx#female_2-network", "en", "IN", Gender::Female},
        {"en-in-x-cxx#female_1-local", "en", "IN", Gender::Female},
    };
    VamaVoice voice(nullptr);
    voice.setVoices(olderVoices);
    int voiceListsAsked = 0;  // the panel asking the engine for its voices again (B23)
    voice.setLister([&voiceListsAsked]() { ++voiceListsAsked; });
    qmlRegisterSingletonInstance("QGroundControl", 1, 0, "VamaVoice", &voice);
    qmlRegisterSingletonInstance("QGroundControl", 1, 0, "QGroundControl", &global);
    // The license lock (row A9): a stand-in RC (an ID and a day), the test key pair, and a clipboard of
    // its own (the PC's is the developer's).
    VamaLicense license(nullptr);
    license.setAndroidId(QString::fromLatin1(licenseVectors::kDevices[0].androidId));
    license.setPublicKey(QByteArray::fromHex(licenseVectors::kTestPublicKeyHex));
    license.setToday(QDate(2026, 11, 4));
    QString licenseClipboard;
    license.setClipboard([&licenseClipboard]() { return licenseClipboard; },
                         [&licenseClipboard](const QString &text) { licenseClipboard = text; });
    // The same RC, with a lock screen that cannot be made (row A9, 2026-11-06): a plain screen instead.
    VamaLicense plainLicense(nullptr);
    plainLicense.setAndroidId(QString::fromLatin1(licenseVectors::kDevices[0].androidId));
    plainLicense.setPublicKey(QByteArray::fromHex(licenseVectors::kTestPublicKeyHex));
    plainLicense.setToday(QDate(2026, 11, 4));
    plainLicense.setLockScreenUrl(QUrl(QStringLiteral("qrc:/Custom/qml/Vama/NoSuchLockScreen.qml")));

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
    RealInputBlocker realInput;
    if (!touchScreen) {
        app.installEventFilter(&realInput);
    }
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
    int flyViewsBefore = 0;     // the menu's switches to the fly view (task AG)

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
    // A real tap on a line of an open list in the menu. Qt's ComboBox then sets its index itself,
    // from C++, and a binding on it stays. False when the line is not there.
    auto menuTapListLine = [&](const QString &text) {
        QList<QQuickItem *> items;
        if (menuWindow) {
            collect(menuWindow->contentItem(), items);
        }
        for (QQuickItem *item : std::as_const(items)) {
            if (item->inherits("QQuickAbstractButton") && item->isVisible() && item->property("text").toString() == text) {
                menuTapAt(item->mapToScene(QPointF(item->width() / 2, item->height() / 2)));
                return true;
            }
        }
        return false;
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
    // Vehicle Configuration (the client, 2026-10-30): config.qml is QGC's MainWindow around the page.
    // Its lines come from a Repeater, so they are looked for in the window's items, not its QObjects.
    QQuickWindow *configWindow = nullptr;
    auto configItem = [&](const char *objectName) -> QQuickItem * {
        if (!configWindow) {
            return nullptr;
        }
        QList<QQuickItem *> items;
        collect(configWindow->contentItem(), items);
        for (QQuickItem *item : std::as_const(items)) {
            if (item->objectName() == QLatin1String(objectName)) {
                return item;
            }
        }
        return nullptr;
    };
    auto configShown = [&](const char *objectName) {
        QQuickItem *item = configItem(objectName);
        return item && item->isVisible();
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
    // The round proximity view (B25). Whether the object red (#f02020) is drawn where the box's plot puts
    // an object in this direction (degrees from the nose, to the right) at this distance, as
    // VamaProximityWindow.qml works it out; and how many red points the plot has.
    auto proximityPlotRect = [&]() -> QRectF {
        QQuickItem *plot = findItem("vamaProximityPlot");
        return plot ? plot->mapRectToScene(QRectF(0, 0, plot->width(), plot->height())) : QRectF();
    };
    auto proximityRedAt = [&](double directionDeg, double metres) -> bool {
        QQuickItem *box = findItem("vamaProximityWindow");
        const QRectF plot = proximityPlotRect();
        if (!box || !box->isVisible() || plot.isEmpty()) {
            return false;
        }
        const double font = std::max(9.0, std::min(14.0, std::round(std::min(plot.width(), plot.height()) / 26.0)));
        const double radius = std::max(10.0, std::min(plot.width(), plot.height()) / 2.0 - font * 1.9);
        const double r = radius * metres / proximity.barRange();
        const double a = (directionDeg - 90.0) * std::acos(-1.0) / 180.0;
        const QPointF at(plot.center().x() + r * std::cos(a), plot.center().y() + r * std::sin(a));
        const QImage image = window->grabWindow();
        const qreal dpr = image.devicePixelRatio();
        for (int dy = -3; dy <= 3; ++dy) {
            for (int dx = -3; dx <= 3; ++dx) {
                const QColor c = image.pixelColor(qRound((at.x() + dx) * dpr), qRound((at.y() + dy) * dpr));
                if (c.red() > 200 && c.green() < 90 && c.blue() < 90) {
                    return true;
                }
            }
        }
        return false;
    };
    auto proximityRedCount = [&]() -> int {
        const QRectF plot = proximityPlotRect();
        const QImage image = window->grabWindow();
        const qreal dpr = image.devicePixelRatio();
        int n = 0;
        for (int y = qRound(plot.top() * dpr); y < qRound(plot.bottom() * dpr); ++y) {
            for (int x = qRound(plot.left() * dpr); x < qRound(plot.right() * dpr); ++x) {
                const QColor c = image.pixelColor(x, y);
                n += (c.red() > 200 && c.green() < 90 && c.blue() < 90) ? 1 : 0;
            }
        }
        return n;
    };
    auto shownAt = [&](const char *objectName) -> QRectF {
        QQuickItem *item = findItem(objectName);
        return item ? item->mapRectToScene(QRectF(0, 0, item->width(), item->height())) : QRectF();
    };
    QRectF proximityBoxBefore;
    // The rings' distances as drawn, on the line ahead (right of it, from the middle to the outer ring).
    QImage proximityRingWords;
    auto proximityRingWordsNow = [&]() -> QImage {
        const QRectF plot = proximityPlotRect();
        if (plot.isEmpty()) {
            return QImage();
        }
        const double font = std::max(9.0, std::min(14.0, std::round(std::min(plot.width(), plot.height()) / 26.0)));
        const double radius = std::max(10.0, std::min(plot.width(), plot.height()) / 2.0 - font * 1.9);
        const QImage image = window->grabWindow();
        const qreal dpr = image.devicePixelRatio();
        return image.copy(QRect(qRound((plot.center().x() + 2) * dpr), qRound((plot.center().y() - radius - 2) * dpr),
                                qRound(70 * dpr), qRound((radius + 2) * dpr)));
    };
    // One DISTANCE_SENSOR for each direction, as ArduPilot sends them, in metres. Negative: nothing in range.
    auto hearObstacle = [&](const std::array<double, 8> &metres) {
        for (int i = 0; i < 8; ++i) {
            proximity.hear(1, ProximitySectors::kFirstId + i, i, metres[i] < 0 ? 4000 : qRound(metres[i] * 100.0), 4000);
        }
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
    // The note of a photo or record press (task AL): at the top in the middle, and clear of the
    // boxes there (beside the buttons it ran into the result box and the lock message).
    auto noteInItsPlace = [&]() -> bool {
        QQuickItem *note = findItem("vamaCameraNote");
        if (!note || !note->isVisible()) {
            return false;
        }
        const QRectF n = note->mapRectToScene(QRectF(0, 0, note->width(), note->height()));
        bool ok = qAbs(n.center().x() - window->width() / 2.0) < 2.0;
        for (const char *name : {"vamaResultBox", "vamaLockChip", "vamaFollowChip"}) {
            QQuickItem *box = findItem(name);
            if (box && box->isVisible() && box->mapRectToScene(QRectF(0, 0, box->width(), box->height())).intersects(n)) {
                std::fprintf(stderr, "the note overlaps %s\n", name);
                ok = false;
            }
        }
        return ok;
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
    double carScore = 0.7;  // how sure it is of the car (under 0.5 for an unsure mark, task AD)
    bool finderSeesWalker = true;  // false: it does not take the walker for a person (task AC)
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
            thing.score = kind == skydroid::find::Kind::Person ? 0.8 : carScore;
            thing.cx = r.center().x() / window->width();
            thing.cy = r.center().y() / window->height();
            thing.w = r.width() / window->width();
            thing.h = r.height() / window->height();
            things.push_back(thing);
        };
        if (finderSeesWalker) {
            add(walkerItem, skydroid::find::Kind::Person);
        }
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

    int grabsBeforeNoFolder = 0;  // B14's step: no picture is grabbed without a save folder
    QPointF zoomTap;              // B10's steps: the same tap at 1x and at 2x
    double yawAt1x = 0.0;
    // The license lock (row A9): license.qml stands in for QGC's main window under the lock. The lock is a
    // popup, so its items are looked for in the window's items (the overlay), not its QObjects.
    QQuickWindow *licenseWindow = nullptr;
    QPointer<QObject> licenseLock;
    QPointer<QObject> plainLock;  // the plain screen, when the lock screen cannot be made (2026-11-06)
    int heldDatagrams = 0;        // what had come to the camera once its link was held (row A9)
    auto licenseItem = [&](const char *objectName) -> QQuickItem * {
        if (!licenseWindow) {
            return nullptr;
        }
        QList<QQuickItem *> items;
        collect(licenseWindow->contentItem(), items);
        for (QQuickItem *item : std::as_const(items)) {
            if (item->objectName() == QLatin1String(objectName)) {
                return item;
            }
        }
        return nullptr;
    };
    auto licenseShown = [&](const char *objectName) {
        QQuickItem *item = licenseItem(objectName);
        return item && item->isVisible();
    };
    auto licenseText = [&](const char *objectName) {
        QQuickItem *item = licenseItem(objectName);
        return item ? item->property("text").toString() : QStringLiteral("not there");
    };
    auto licenseTap = [&](const char *objectName) {
        QQuickItem *item = licenseItem(objectName);
        if (!item) {
            std::fprintf(stderr, "%s not found in the license window\n", objectName);
            ++g_problems;
            return;
        }
        const QPointF centre = item->mapToScene(QPointF(item->width() / 2, item->height() / 2));
        mouseOn(licenseWindow, QEvent::MouseButtonPress, centre);
        mouseOn(licenseWindow, QEvent::MouseButtonRelease, centre);
    };
    const QString licenseDeviceCode = QString::fromLatin1(licenseVectors::kDevices[0].code);
    const QString licenseKeyHere = QString::fromLatin1(licenseVectors::kKeys[0].key);
    const QString licenseKeyElsewhere = QString::fromLatin1(licenseVectors::kKeys[3].key);  // for another RC
    const QList<std::pair<int, std::function<void()>>> steps = {
        {1500, [&]() {
             link.zoom(1);
             link.zoom(1);
             link.zoom(1);
             // Tap the photo button to run its click handler.
             tap("vamaPhotoButton");
         }},
        // The photo (task AL, client 2026-10-26: "photos and recordings are not saved in rc"): the
        // camera echoes the command, and a note at the top says where the photo is.
        {200, [&]() {
             const bool shown = waitFor([&]() {
                 QQuickItem *note = findItem("vamaCameraNote");
                 return note && note->isVisible();
             }, 2000);
             QQuickItem *note = findItem("vamaCameraNote");
             QQuickItem *text = findItem("vamaCameraNoteText");
             QQuickItem *rcText = findItem("vamaRcNoteText");
             const QString says = text ? text->property("text").toString() : QString();
             expect(shown && camera.photos == 1 && says == QStringLiteral("Photo taken. It is saved on the camera's SD card.") &&
                        noteInItsPlace(),
                    "photo: a note at the top says the photo is on the camera's SD card", says);
             // And the copy on the RC (the client's Stage 1.1): QGC's picture of the video, written by
             // the app's video item.
             const QString rcSays = rcText ? rcText->property("text").toString() : QString();
             const bool written = waitFor([&]() { return QFile::exists(global.videos.lastImageFile); }, 2000);
             expect(global.videos.grabs == 1 && written && rcSays == QStringLiteral("A copy is saved on this RC, in its Photo folder."),
                    "photo: a copy goes to the RC's Photo folder (the video item writes it), and the note says so",
                    rcSays + QStringLiteral(" | ") + global.videos.lastImageFile);
             shot("35_photo_note.png");
             if (note) {
                 tapItem(note);  // a tap closes it
             }
         }},
        {300, [&]() {
             QQuickItem *note = findItem("vamaCameraNote");
             expect(note && !note->isVisible(), "photo: a tap on the note closes it", QString());
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
             // The finder's check (task AC): the drawn box holds a person, so the finder looks.
             expect(link.lockWantsLooks() && marks.checking() && !marks.running() && findItems("vamaMark").isEmpty(),
                    "lock check: a drawn box on a person is checked by the finder, with no marks shown",
                    QStringLiteral("looks %1, checking %2").arg(link.lockWantsLooks()).arg(marks.checking()));
             // From now on the finder does not take the walker for a person: as the board stacks it stays
             // on, the box holds something else.
             finderSeesWalker = false;
         }},
        {300, [&]() {
             // Three looks in a row, about a second apart.
             waitFor([&]() { return !link.lockSeen(); }, 6000);
             expect(link.lockActive() && !link.lockSeen() &&
                        link.lockMessage() == QStringLiteral("Locked, but the app sees no person in the box any more. The camera waits."),
                    "lock check: no person on the box in three looks: NOT SEEN, and the state says why", link.lockMessage());
             expect(boxLabel() == QStringLiteral("NOT SEEN"), "lock check: the box says NOT SEEN", boxLabel());
             expect(camera.yawRate == 0.0 && camera.pitchRate == 0.0, "lock check: the camera waits",
                    QStringLiteral("%1 / %2 deg/s").arg(camera.yawRate).arg(camera.pitchRate));
             // (The line of numbers comes with the next picture.)
             QQuickItem *numbers = findItem("vamaLockNumbers");
             waitFor([&]() { return numbers && numbers->property("text").toString().endsWith(QStringLiteral(", check: not there")); }, 1000);
             const QString line = numbers ? numbers->property("text").toString() : QString();
             expect(line.endsWith(QStringLiteral(", check: not there")), "lock check: the line of numbers says so", line);
             shot("14d_lock_check_not_there.png");
             finderSeesWalker = true;
         }},
        {300, [&]() {
             waitFor([&]() { return link.lockSeen(); }, 3000);
             expect(link.lockActive() && link.lockSeen() && link.lockMessage().startsWith(QStringLiteral("Locked.")),
                    "lock check: the person on the box again: the lock goes on", link.lockMessage());
             // The walker goes behind a tree...
             camera.setWalkerVisible(false);
         }},
        {1000, [&]() {
             expect(link.lockActive() && !link.lockSeen(), "lock check: behind the tree the object is not seen", link.lockMessage());
             // ... and comes out three of its widths further on, where the tracker does not look.
             camera.placeWalker(camera.walkerRight() + 9.0, camera.walkerUp());
             camera.setWalkerVisible(true);
         }},
        {300, [&]() {
             waitFor([&]() { return link.lockSeen() && boxOnWalker(); }, 4000);
             expect(link.lockActive() && link.lockSeen() && link.lockMessage().startsWith(QStringLiteral("Found the person again.")),
                    "lock check: a person coming out near where the object went is found again, and the state says so",
                    link.lockMessage());
             expect(boxOnWalker(), "lock check: the box is on the walker there", boxText());
             shot("14e_lock_found_again.png");
         }},
        {1500, [&]() {
             const QPointF off = walkerOff();
             expect(std::abs(off.x()) < 2.0 && std::abs(off.y()) < 1.5, "lock check: and the camera has turned to them", offText());
             camera.setWalkerVisible(false);  // the walker is gone
         }},
        {700, [&]() {
             expect(link.lockActive() && !link.lockSeen(), "lock: it says when the object is not seen", link.lockMessage());
             expect(boxLabel() == QStringLiteral("NOT SEEN"), "lock: the box says NOT SEEN", boxLabel());
             expect(camera.yawRate == 0.0 && camera.pitchRate == 0.0, "lock: the camera waits, it does not turn after nothing",
                    QStringLiteral("%1 / %2 deg/s").arg(camera.yawRate).arg(camera.pitchRate));
             shot("14c_lock_not_seen.png");
         }},
        {7800, [&]() {
             expect(!link.lockActive() && link.lockMessage().contains(QStringLiteral("Lock lost")),
                    "lock: after eight seconds without the object the lock ends and says why", link.lockMessage());
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
             carScore = 0.46;  // the finder is unsure of the car (task AD)
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
             static const QRegularExpression lookLine(
                 QStringLiteral("^Marked: 1 person, 1 vehicle \\(1 unsure\\)\\. A look takes 0\\.\\d s\\."));
             expect(state && state->isVisible() && lookLine.match(stateText).hasMatch() &&
                        stateText.contains(QStringLiteral("Getting a picture takes 0.")),
                    "marks: a line says what is marked (and how much of it unsure), how long a look takes, and how long a picture takes to get",
                    stateText);
             // Each mark says how sure the finder is. The car's (46 %) is unsure: thinner, faint, with a "?" (task AD).
             QString labels;
             bool personSure = false;
             bool vehicleUnsure = false;
             for (QQuickItem *box : findItems("vamaMark")) {
                 QString text;
                 for (QQuickItem *child : box->childItems()) {
                     if (child->objectName() == QStringLiteral("vamaMarkLabel")) {
                         text = child->property("text").toString();
                     }
                 }
                 QObject *border = box->property("border").value<QObject *>();
                 const double borderWidth = border ? border->property("width").toDouble() : 0.0;
                 labels += text + QStringLiteral(" (border %1, opacity %2); ").arg(borderWidth).arg(box->opacity());
                 if (box->property("person").toBool()) {
                     personSure = text == QStringLiteral("Person 80%") && borderWidth == 2.0 && box->opacity() == 1.0;
                 } else {
                     vehicleUnsure = text == QStringLiteral("Vehicle 46%?") && borderWidth == 1.0 && box->opacity() < 0.7;
                 }
             }
             expect(personSure, "marks: a mark says how sure the finder is (Person 80%)", labels);
             expect(vehicleUnsure, "marks: a mark it is unsure of is thinner and faint, with a \"?\" (Vehicle 46%?)", labels);
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
             carScore = 0.7;
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
             // B13 d: with no channel chosen, the wheel part says that the wheel is off and how to set it.
             QObject *wheel = window->findChild<QObject *>(QStringLiteral("vamaWheelStatus"));
             const QString off = wheel ? wheel->property("text").toString() : QString();
             expect(off.startsWith(QStringLiteral("The RC wheel is off: no channel is chosen.")),
                    "RC wheel: with no channel chosen, it says that the wheel is off", off);
             link.setWheelPitchChannel(9);
         }},
        {300, [&]() {
             QObject *wheel = window->findChild<QObject *>(QStringLiteral("vamaWheelStatus"));
             const QString set = wheel ? wheel->property("text").toString() : QString();
             expect(!set.contains(QStringLiteral("The RC wheel is off")) &&
                        (set.contains(QStringLiteral("Channel 9 now")) || set.contains(QStringLiteral("No RC values from the drone yet."))),
                    "RC wheel: with a channel chosen, it shows that channel's value, not the off line", set);
             link.setWheelPitchChannel(0);
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
             QQuickItem *cameraSettings = menuItem("vamaMenuCameraSettings");
             QQuickItem *cameras = menuItem("vamaMenuCameras");
             expect(cameraSettings && cameraSettings->isVisible() && cameras && !cameras->isVisible(),
                    "menu: Camera settings is in the menu, Cameras only in Advanced Mode (task AG)", QString());
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
             // Task AG: the camera windows from the menu. The settings were left scrolled down.
             QQuickItem *cameras = menuItem("vamaMenuCameras");
             expect(cameras && cameras->isVisible(), "menu: Advanced Mode shows Cameras in the menu (task AG)", QString());
             shotOf(menuWindow, "29b_menu_cameras.png");
             QObject *flick = window->findChild<QObject *>(QStringLiteral("vamaCameraSettingsFlick"));
             if (flick) {
                 flick->setProperty("contentY", 120);
             }
             flyViewsBefore = menuWindow->property("flyViews").toInt();
             menuTap("vamaMenuCameraSettings");
         }},
        {400, [&]() {
             QObject *settings = window->findChild<QObject *>(QStringLiteral("vamaCameraSettings"));
             QObject *flick = window->findChild<QObject *>(QStringLiteral("vamaCameraSettingsFlick"));
             expect(!menuOpen() && settings && settings->property("opened").toBool(),
                    "menu: Camera settings closes the menu and opens the camera settings", QString());
             expect(flick && flick->property("contentY").toReal() == 0.0,
                    "menu: the camera settings open at their top, with the camera model and the Cameras button",
                    flick ? QString::number(flick->property("contentY").toReal()) : QStringLiteral("no flick"));
             QQuickItem *button = findItem("vamaCamerasButton");
             expect(button && button->isVisible(), "menu: there, the Cameras button is shown in Advanced Mode", QString());
             expect(menuWindow->property("flyViews").toInt() == flyViewsBefore + 1,
                    "menu: Camera settings also shows the fly view, where the camera windows are",
                    QString::number(menuWindow->property("flyViews").toInt() - flyViewsBefore));
             if (settings) {
                 QMetaObject::invokeMethod(settings, "close");
             }
             openMenu();
         }},
        {400, [&]() {
             flyViewsBefore = menuWindow->property("flyViews").toInt();
             menuTap("vamaMenuCameras");
         }},
        {500, [&]() {
             QObject *editor = window->findChild<QObject *>(QStringLiteral("vamaCameraEditor"));
             QObject *settings = window->findChild<QObject *>(QStringLiteral("vamaCameraSettings"));
             expect(!menuOpen() && editor && editor->property("opened").toBool(),
                    "menu: Cameras closes the menu and opens the camera editor", QString());
             expect(settings && !settings->property("opened").toBool(), "menu: and only the editor, not the settings too",
                    QString());
             expect(menuWindow->property("flyViews").toInt() == flyViewsBefore + 1, "menu: Cameras also shows the fly view",
                    QString::number(menuWindow->property("flyViews").toInt() - flyViewsBefore));
             shot("29c_cameras_from_menu.png");
             if (editor) {
                 QMetaObject::invokeMethod(editor, "close");
             }
             openMenu();
         }},
        {400, [&]() { menuTap("vamaAdvancedModeTap"); }},
        {300, [&]() {
             expect(menuWindow->property("offQuestions").toInt() == 1 && !menuOpen() && !menuItem("vamaAdminPasswordField"),
                    "admin: in Advanced Mode the tap asks QGC's question to turn it off, with no password",
                    QStringLiteral("%1 questions").arg(menuWindow->property("offQuestions").toInt()));
             menuWindow->close();
             // The cameras the admin adds (task AA), from the camera settings.
             global.core.setShowAdvancedUI(false);
             // The editor is for Advanced Mode only, also when it is asked for by name (task AG).
             global.core.requestCameraWindow(QStringLiteral("editor"));
             QMetaObject::invokeMethod(window->findChild<QObject *>(QStringLiteral("vamaCameraSettings")), "open");
         }},
        {400, [&]() {
             QQuickItem *button = findItem("vamaCamerasButton");
             expect(button && !button->isVisible(), "cameras: no Cameras button outside Advanced Mode", QString());
             QObject *editor = window->findChild<QObject *>(QStringLiteral("vamaCameraEditor"));
             expect(editor && !editor->property("opened").toBool(),
                    "menu: outside Advanced Mode an ask for the editor opens nothing (task AG)", QString());
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
             for (const char *button : {"vamaLaserButton", "vamaIrButton", "vamaZoomInButton", "vamaTorchButton"}) {
                 QQuickItem *item = findItem(button);
                 if (!item || item->isVisible()) {
                     shown.append(QString::fromLatin1(button));
                 }
             }
             // Photo and record stay (task AL): they keep a copy on the RC, also for a camera without them.
             QQuickItem *lock = findItem("vamaLockButton");
             QQuickItem *photo = findItem("vamaPhotoButton");
             QQuickItem *record = findItem("vamaRecordButton");
             expect(shown.isEmpty() && lock && lock->isVisible() && photo && photo->isVisible() && record && record->isVisible(),
                    "cameras: its screen has no laser, IR, zoom or torch button, and still the lock, and photo and record for the copy on the RC",
                    shown.join(QStringLiteral(", ")));
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
             // The form's control link (the C20, A1): the box is shown and saved both ways. Only
             // the file is checked: the preview's stand-in camera speaks UDP, and the link's TCP
             // has its own tests.
             press("vamaCameraCopy_C20");
         }},
        {400, [&]() {
             QQuickItem *transport = findItem("vamaCameraTransport");
             expect(transport && transport->isVisible() && transport->property("currentIndex").toInt() == 1,
                    "cameras: a copy of the V20 has TCP as its control link",
                    transport ? transport->property("currentText").toString() : QStringLiteral("no box"));
             // Its zoom on its camera port (row B10): a TOP camera has five zooms to choose from.
             QQuickItem *zoom = findItem("vamaCameraZoom");
             const QString zoomText = zoom ? zoom->property("currentText").toString() : QStringLiteral("no box");
             expect(zoom && zoom->property("count").toInt() == 5 &&
                        zoomText.startsWith(QStringLiteral("Zoom on its camera port 8100, which says the zoom")),
                    "cameras: and the V20's zoom on its camera port", zoomText);
             press("vamaCameraSave");
         }},
        {400, [&]() {
             const QString file = cameraFolder + QStringLiteral("/v20-copy.json");
             expect(cameraTransport(file) == QStringLiteral("tcp") && cameraFileValue(file, "zoom") == QJsonValue(QStringLiteral("c20")) &&
                        cameraFileValue(file, "maxZoom").toDouble() == 22.0,
                    "cameras: Save writes TCP and the camera port zoom into its file, with the lens's range",
                    cameraTransport(file) + QStringLiteral(" | ") + cameraFileValue(file, "zoom").toString());
             press("vamaCameraEdit_v20-copy");
         }},
        {400, [&]() {
             QQuickItem *transport = findItem("vamaCameraTransport");
             expect(transport && transport->property("currentIndex").toInt() == 1, "cameras: Change shows the TCP it was saved with",
                    transport ? transport->property("currentText").toString() : QStringLiteral("no box"));
             setOn("vamaCameraTransport", "currentIndex", 0);
             press("vamaCameraSave");
         }},
        {400, [&]() {
             expect(cameraTransport(cameraFolder + QStringLiteral("/v20-copy.json")) == QStringLiteral("udp") &&
                        link.model() == QStringLiteral("C13"),
                    "cameras: UDP picked in the box is saved as UDP, and the camera in use stays",
                    cameraTransport(cameraFolder + QStringLiteral("/v20-copy.json")));
             press("vamaCameraDelete_v20-copy");
         }},
        {300, [&]() { tapFound("vamaCameraDeleteYes"); }},
        {400, [&]() {
             expect(!QFile::exists(cameraFolder + QStringLiteral("/v20-copy.json")) && link.model() == QStringLiteral("C13"),
                    "cameras: the copy is deleted", link.model());
             // The V20's torch (row B27): its box in the form, its button on the screen. A copy on
             // 127.0.0.1 over UDP, so that the preview's stand-in camera gets the torch's text.
             press("vamaCameraCopy_C20");
         }},
        {400, [&]() {
             QQuickItem *torch = findItem("vamaCameraTorch");
             expect(torch && torch->isVisible() && torch->property("checked").toBool(), "torch: a copy of the V20 has its torch ticked",
                    QString());
             setOn("vamaCameraTorch", "checked", false);  // saved both ways
             setOn("vamaCameraHost", "text", QStringLiteral("127.0.0.1"));
             setOn("vamaCameraPort", "text", QString::number(camera.port()));
             setOn("vamaCameraTransport", "currentIndex", 0);
             setOn("vamaCameraDayVideo", "text", QStringLiteral("rtsp://127.0.0.1:8554/v20"));
             press("vamaCameraSave");
         }},
        {400, [&]() {
             const QString file = cameraFolder + QStringLiteral("/v20-copy.json");
             expect(cameraFileValue(file, "torch") == QJsonValue(false), "torch: Save writes an unticked torch into the file",
                    cameraFileValue(file, "torch").toVariant().toString());
             press("vamaCameraEdit_v20-copy");
         }},
        {400, [&]() {
             QQuickItem *torch = findItem("vamaCameraTorch");
             expect(torch && !torch->property("checked").toBool(), "torch: Change shows it unticked, as it was saved", QString());
             setOn("vamaCameraTorch", "checked", true);
             press("vamaCameraSave");
         }},
        {400, [&]() {
             const QString file = cameraFolder + QStringLiteral("/v20-copy.json");
             expect(cameraFileValue(file, "torch") == QJsonValue(true), "torch: and a ticked one", cameraTransport(file));
             press("vamaCameraEditorClose");
         }},
        {300, [&]() {
             SkydroidLink::setC20CameraPort(1);  // its camera port: nothing there, on 127.0.0.1
             QMetaObject::invokeMethod(window->property("layer").value<QObject *>(), "_chooseCamera",
                                       Q_ARG(QVariant, QVariant(QStringLiteral("v20-copy"))));
         }},
        {500, [&]() {
             QQuickItem *button = findItem("vamaTorchButton");
             expect(link.model() == QStringLiteral("v20-copy") && button && button->isVisible() && !link.torchOn(),
                    "torch: a camera with one has the torch button, off at the start", link.model());
             tap("vamaTorchButton");
         }},
        {400, [&]() {
             expect(link.torchOn() && camera.texts.contains(QStringLiteral("AT+LED -t1\r\n")),
                    "torch: a tap switches it on, by the text of Skydroid's SDK", camera.texts.join(QStringLiteral(" | ")).trimmed());
             tap("vamaTorchButton");
         }},
        {400, [&]() {
             expect(!link.torchOn() && camera.texts.contains(QStringLiteral("AT+LED -t0\r\n")), "torch: the next tap switches it off",
                    camera.texts.join(QStringLiteral(" | ")).trimmed());
             // Deleted while in use: the V13 again, with the addresses as they are.
             QMetaObject::invokeMethod(window->property("layer").value<QObject *>(), "_openCameraEditor");
         }},
        {400, [&]() { tapFound("vamaCameraDelete_v20-copy"); }},
        {300, [&]() { tapFound("vamaCameraDeleteYes"); }},
        {400, [&]() {
             SkydroidLink::setC20CameraPort(c20::kPort);
             QQuickItem *button = findItem("vamaTorchButton");
             expect(link.model() == QStringLiteral("C13") && link.host() == QStringLiteral("127.0.0.1") && button && !button->isVisible(),
                    "torch: the V13 has no torch button", link.model());
             // A copy of the Viewpro A20KTR (task AK): the form shows its protocol and what that has.
             press("vamaCameraCopy_A20KTR");
         }},
        {400, [&]() {
             QQuickItem *protocol = findItem("vamaCameraProtocol");
             QQuickItem *transport = findItem("vamaCameraTransport");
             QQuickItem *zoom = findItem("vamaCameraZoom");
             QQuickItem *laser = findItem("vamaCameraLaser");
             QQuickItem *bySwitch = findItem("vamaCameraThermalBySwitch");
             QQuickItem *thermal = findItem("vamaCameraThermalVideo");
             const QString zoomText = zoom ? zoom->property("currentText").toString() : QString();
             expect(protocol && protocol->property("text").toString() == QStringLiteral("VPro, over TCP") &&
                        transport && !transport->isVisible() && laser && laser->isVisible() && laser->property("checked").toBool() &&
                        zoom && zoom->property("count").toInt() == 2 && zoomText.startsWith(QStringLiteral("The camera says its zoom")) &&
                        bySwitch && bySwitch->isVisible() && bySwitch->property("checked").toBool() && thermal && !thermal->isVisible(),
                    "cameras: a copy of the A20KTR shows Viewpro, the zoom it says, the thermal picture by a switch, its laser, no control link",
                    protocol ? protocol->property("text").toString() + QStringLiteral(" | ") + zoomText : QStringLiteral("no form"));
             // On 127.0.0.1, never the camera's own address: the preview reaches no network.
             setOn("vamaCameraHost", "text", QStringLiteral("127.0.0.1"));
             setOn("vamaCameraPort", "text", QString::number(camera.port()));
             setOn("vamaCameraDayVideo", "text", QStringLiteral("rtsp://127.0.0.1:8554/viewpro"));
             press("vamaCameraSave");
         }},
        {400, [&]() {
             const QString file = cameraFolder + QStringLiteral("/vpro-copy.json");
             expect(cameraFileValue(file, "protocol") == QJsonValue(QStringLiteral("viewpro")) &&
                        cameraFileValue(file, "zoom") == QJsonValue(QStringLiteral("live")) &&
                        cameraFileValue(file, "thermalBySwitch") == QJsonValue(true) &&
                        cameraFileValue(file, "thermalVideoUrl") == QJsonValue(QString()) &&
                        cameraFileValue(file, "transport") == QJsonValue(QStringLiteral("tcp")) &&
                        cameraFileValue(file, "laser") == QJsonValue(true) && cameraFileValue(file, "laserMaxM") == QJsonValue(4000.0),
                    "cameras: Save writes a Viewpro camera, its thermal picture by a switch and its laser", cameraTransport(file));
             press("vamaCameraEditorClose");
         }},
        {300, [&]() {
             // Chosen as the camera list chooses it (the list is in the camera settings, closed now).
             QMetaObject::invokeMethod(window->property("layer").value<QObject *>(), "_chooseCamera",
                                       Q_ARG(QVariant, QVariant(QStringLiteral("vpro-copy"))));
         }},
        {500, [&]() {
             const QString video = global.settings.video.rtspUrlFact.rawValue().toString();
             QQuickItem *ir = findItem("vamaIrButton");
             expect(link.model() == QStringLiteral("vpro-copy") && video == QStringLiteral("rtsp://127.0.0.1:8554/viewpro") &&
                        ir && ir->isVisible() && !link.thermalPicture(),
                    "cameras: the Viewpro copy in use has an IR button, on its day picture", video);
         }},
        {300, [&]() { tap("vamaIrButton"); }},
        {300, [&]() {
             const QString video = global.settings.video.rtspUrlFact.rawValue().toString();
             expect(link.thermalPicture() && video == QStringLiteral("rtsp://127.0.0.1:8554/viewpro"),
                    "cameras: its IR button switches the camera, and the video address stays", video);
             shot("33b_viewpro_ir.png");
             // B17 (build 16): its thermal picture takes the colour modes too, though its address stays.
             link.setThermalPalette(QStringLiteral("ironbow"));
         }},
        {300, [&]() {
             QQuickItem *video = findItem("videoContent");
             QQuickItem *button = findItem("vamaPaletteButton");
             const int row = video ? video->property("_vamaPaletteRow").toInt() : -1;
             expect(row > 0 && button && button->isVisible(),
                    "cameras: a Viewpro's thermal picture is coloured in the chosen mode, and the colour button shows",
                    QStringLiteral("row %1").arg(row));
             tap("vamaIrButton");
         }},
        {300, [&]() {
             expect(!link.thermalPicture() && global.settings.video.rtspUrlFact.rawValue().toString() == QStringLiteral("rtsp://127.0.0.1:8554/viewpro"),
                    "cameras: and back to the day picture", QString());
             QQuickItem *video = findItem("videoContent");
             expect(video && video->property("_vamaPaletteRow").toInt() == 0,
                    "cameras: the Viewpro's day picture is not coloured", link.thermalPalette());
             link.setThermalPalette(QStringLiteral("camera"));
             QMetaObject::invokeMethod(window->property("layer").value<QObject *>(), "_openCameraEditor");
         }},
        {200, [&]() {
             // A Viewpro's video after its sensor switch (B13): the link asks, and the screen opens
             // QGC's video again, only while it shows none.
             global.videos.setDecoding(false);
             emit link.videoReopenWanted();
             global.videos.setDecoding(true);
             emit link.videoReopenWanted();
             expect(global.videos.videoStarts == 1, "Viewpro thermal switch: the video opens again only while it shows none",
                    QString::number(global.videos.videoStarts));
         }},
        {400, [&]() { tapFound("vamaCameraDelete_vpro-copy"); }},
        {300, [&]() { tapFound("vamaCameraDeleteYes"); }},
        {400, [&]() {
             expect(link.model() == QStringLiteral("C13") && !QFile::exists(cameraFolder + QStringLiteral("/vpro-copy.json")),
                    "cameras: the Viewpro copy deleted, the V13 in use again", link.model());
             press("vamaCameraCopy_ZR10");
         }},
        {400, [&]() {
             QQuickItem *protocol = findItem("vamaCameraProtocol");
             QQuickItem *zoom = findItem("vamaCameraZoom");
             QQuickItem *bySwitch = findItem("vamaCameraThermalBySwitch");
             QQuickItem *tracker = findItem("vamaCameraOwnTracker");
             QQuickItem *laser = findItem("vamaCameraLaser");
             const QString zoomText = zoom ? zoom->property("currentText").toString() : QString();
             expect(protocol && protocol->property("text").toString() == QStringLiteral("SIYI, over UDP (as the VR10)") && zoom &&
                        zoom->property("count").toInt() == 2 && zoomText.startsWith(QStringLiteral("It zooms and does not say")) &&
                        bySwitch && !bySwitch->isVisible() && tracker && !tracker->isVisible() && laser && !laser->isVisible(),
                    "cameras: a copy of the ZR10 shows SIYI and its zoom that says nothing, no thermal switch, laser or tracker",
                    protocol ? protocol->property("text").toString() + QStringLiteral(" | ") + zoomText : QStringLiteral("no form"));
             press("vamaCameraCancel");
         }},
        {300, [&]() {
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
             // Recording (task AL): the second press stops the camera's recording (every press
             // started it until test build 13), and the screen says what the camera did.
             tap("vamaRecordButton");
         }},
        {200, [&]() {
             const bool recording = waitFor([&]() { return camera.recording && link.recordingConfirmed(); }, 3000);
             QQuickItem *rec = findItem("vamaRecLabel");
             QQuickItem *rcRec = findItem("vamaRcRecLabel");
             QQuickItem *text = findItem("vamaCameraNoteText");
             QQuickItem *rcText = findItem("vamaRcNoteText");
             const QString says = text ? text->property("text").toString() : QString();
             const QString rcSays = rcText ? rcText->property("text").toString() : QString();
             expect(recording && rec && rec->isVisible() && rec->property("text").toString() == QStringLiteral("REC") &&
                        says == QStringLiteral("Recording on the camera's SD card."),
                    "record: the camera records and says so, and the note says where the video goes", says);
             // Here the result box and the lock message show at the top: the note goes under them.
             expect(noteInItsPlace(), "record: the note is under the boxes at the top, not over them", QString());
             expect(global.videos.recording() && rcRec && rcRec->isVisible() &&
                        rcSays == QStringLiteral("A copy is recorded on this RC, in its Video folder."),
                    "record: the RC records a copy as well, and the status and the note say so", rcSays);
             shot("36_recording.png");
             tap("vamaRecordButton");
         }},
        {200, [&]() {
             const bool stopped = waitFor([&]() { return !camera.recording && link.recordingConfirmed(); }, 3000);
             QQuickItem *rec = findItem("vamaRecLabel");
             QQuickItem *rcRec = findItem("vamaRcRecLabel");
             QQuickItem *text = findItem("vamaCameraNoteText");
             QQuickItem *rcText = findItem("vamaRcNoteText");
             const QString says = text ? text->property("text").toString() : QString();
             const QString rcSays = rcText ? rcText->property("text").toString() : QString();
             expect(stopped && rec && !rec->isVisible() && says == QStringLiteral("Recording stopped. The video is on the camera's SD card."),
                    "record: the second press stops the camera's recording", says);
             expect(!global.videos.recording() && global.videos.recordStops == 1 && rcRec && !rcRec->isVisible() &&
                        rcSays == QStringLiteral("The copy is saved on this RC, in its Video folder."),
                    "record: and the RC's copy, which is saved", rcSays);
             // The RC's copy ending by itself (task AM, client 2026-10-27: build 14 said "The copy is
             // saved" over an empty file): its video restarts, and a new copy starts when it is back.
             tap("vamaRecordButton");
         }},
        {200, [&]() {
             const bool both = waitFor([&]() { return camera.recording && global.videos.recording(); }, 3000);
             expect(both && global.videos.recordStarts == 2, "record: a new recording, on the camera and on the RC", QString());
             global.videos.setDecoding(false);
             global.videos.failRecording();
         }},
        {300, [&]() {
             QQuickItem *rcText = findItem("vamaRcNoteText");
             const QString rcSays = rcText ? rcText->property("text").toString() : QString();
             QObject *layer = window->property("layer").value<QObject *>();
             expect(rcSays == QStringLiteral("The copy on this RC stopped: its video restarted. A new copy starts when the video is back.") &&
                        global.videos.recordStops == 1 && camera.recording && layer && layer->property("_recordingShown").toBool(),
                    "RC copy stopped by itself: the note says so, not \"saved\", nothing was stopped, the camera records on", rcSays);
             global.videos.setDecoding(true);
         }},
        {300, [&]() {
             const bool again = waitFor([&]() { return global.videos.recording(); }, 3000);
             QQuickItem *rcText = findItem("vamaRcNoteText");
             const QString rcSays = rcText ? rcText->property("text").toString() : QString();
             expect(again && global.videos.recordStarts == 3 && rcSays == QStringLiteral("A new copy is recorded on this RC, in its Video folder."),
                    "RC copy stopped by itself: a new copy starts when the video is back", rcSays);
             global.videos.failRecording();  // the video runs on: a new copy a moment later
         }},
        {300, [&]() {
             const bool again = waitFor([&]() { return global.videos.recording(); }, 3000);
             expect(again && global.videos.recordStarts == 4, "RC copy stopped by itself with the video running: a new copy a moment later",
                    QString::number(global.videos.recordStarts));
             global.videos.failRecording();
         }},
        {300, [&]() {
             const bool again = waitFor([&]() { return global.videos.recording(); }, 3000);
             expect(again && global.videos.recordStarts == 5, "RC copy: a third new copy", QString::number(global.videos.recordStarts));
             global.videos.failRecording();
         }},
        {2000, [&]() {
             QQuickItem *rcText = findItem("vamaRcNoteText");
             const QString rcSays = rcText ? rcText->property("text").toString() : QString();
             expect(!global.videos.recording() && global.videos.recordStarts == 5 &&
                        rcSays == QStringLiteral("The copy on this RC keeps stopping, so no new one starts. The camera's own recording goes on."),
                    "RC copy that keeps stopping: no fourth new copy, and the note says the camera records on", rcSays);
             tap("vamaRecordButton");  // stops the camera's recording
         }},
        {200, [&]() {
             const bool stopped = waitFor([&]() { return !camera.recording; }, 3000);
             // The link says what the camera did when it has asked it (up to 2.6 s after the press):
             // wait for that note, or it comes during the SD card steps below and hides theirs.
             QQuickItem *text = findItem("vamaCameraNoteText");
             const QString stopNote = QStringLiteral("Recording stopped. The video is on the camera's SD card.");
             const bool said = waitFor([&]() { return text && text->property("text").toString() == stopNote; }, 4000);
             expect(stopped && said && global.videos.recordStarts == 5 && global.videos.recordStops == 1,
                    "RC copy: the next press stops the camera's recording, and starts no copy",
                    text ? text->property("text").toString() : QStringLiteral("no note"));
             // No SD card. The camera is asked about its card a second after a photo.
             camera.sdCard = FakeCamera::kNoSdCard;
             tap("vamaPhotoButton");
         }},
        {200, [&]() {
             const bool told = waitFor([&]() { return link.sdCardWarning() == QStringLiteral("No SD card"); }, 3000);
             QQuickItem *warning = findItem("vamaSdCardWarning");
             expect(told && warning && warning->isVisible() && warning->property("text").toString() == QStringLiteral("No SD card"),
                    "SD card: the camera status says when the camera has none", link.sdCardWarning());
             tap("vamaPhotoButton");
         }},
        {600, [&]() {
             QQuickItem *text = findItem("vamaCameraNoteText");
             QQuickItem *rcText = findItem("vamaRcNoteText");
             const QString says = text ? text->property("text").toString() : QString();
             expect(says == QStringLiteral("The camera has no SD card. The photo is not saved.") && rcText &&
                        rcText->property("text").toString() == QStringLiteral("A copy is saved on this RC, in its Photo folder."),
                    "SD card: a photo without a card says that it is not saved, and the RC has its copy", says);
             shot("37_no_sd_card.png");
             // The camera cannot record (no card), the RC can: the button shows the RC's recording, and
             // its next press stops it.
             tap("vamaRecordButton");
         }},
        {200, [&]() {
             QQuickItem *text = findItem("vamaCameraNoteText");
             const bool told = waitFor([&]() {
                 return text && text->property("text").toString() == QStringLiteral("Recording did not start. The camera has no SD card.");
             }, 4000);
             QObject *layer = window->property("layer").value<QObject *>();
             expect(told && !link.recording() && global.videos.recording() && layer && layer->property("_recordingShown").toBool(),
                    "record without an SD card: the camera did not start, the RC records, and the button shows it",
                    text ? text->property("text").toString() : QString());
             tap("vamaRecordButton");
         }},
        {300, [&]() {
             QQuickItem *rcText = findItem("vamaRcNoteText");
             expect(!global.videos.recording() && camera.recording == false && rcText &&
                        rcText->property("text").toString() == QStringLiteral("The copy is saved on this RC, in its Video folder."),
                    "record without an SD card: the next press stops the RC's copy", QString());
             // No video: no copy, and the note says why.
             global.videos.setDecoding(false);
             tap("vamaPhotoButton");
         }},
        {300, [&]() {
             QQuickItem *rcText = findItem("vamaRcNoteText");
             const QString says = rcText ? rcText->property("text").toString() : QString();
             expect(global.videos.grabs == 3 && says == QStringLiteral("No copy on this RC: there is no video."),
                    "photo without video: no copy on the RC, and the note says why", says);
             global.videos.setDecoding(true);
             QMetaObject::invokeMethod(window->findChild<QObject *>(QStringLiteral("vamaCameraSettings")), "open");
         }},
        {400, [&]() {
             QQuickItem *storage = findItem("vamaCameraStorage");
             const QString says = storage ? storage->property("text").toString() : QString();
             expect(storage && storage->isVisible() &&
                        says == QStringLiteral("The camera saves its photos and videos on its own SD card. The camera says it has no SD card. "
                                               "A copy of each is saved on this remote control, in its Photo and Video folders."),
                    "camera settings: where photos and videos go, and what the camera says of its SD card", says);
             QMetaObject::invokeMethod(window->findChild<QObject *>(QStringLiteral("vamaCameraSettings")), "close");
         }},
        {100, [&]() {
             // The note of the last photo goes by itself, some seconds after it came.
             const bool gone = waitFor([&]() {
                 QQuickItem *note = findItem("vamaCameraNote");
                 return note && !note->isVisible();
             }, 7000);
             expect(gone, "photo: the note goes by itself after a few seconds", QString());
             // B14: QGC has no save folder (Android without the Storage permission): no copy on the
             // RC, and the note says how to allow it.
             global.settings.app.setHasFolder(false);
             grabsBeforeNoFolder = global.videos.grabs;
             tap("vamaPhotoButton");
         }},
        {300, [&]() {
             QQuickItem *rcText = findItem("vamaRcNoteText");
             const QString says = rcText ? rcText->property("text").toString() : QString();
             expect(global.videos.grabs == grabsBeforeNoFolder &&
                        says.startsWith(QStringLiteral("No copy on this RC: the app has no folder to save it in.")) &&
                        says.contains(QStringLiteral("Allow Storage for VAMA GCS")),
                    "photo without a save folder: no copy on the RC, and the note says how to allow it", says);
             tap("vamaRecordButton");
         }},
        {300, [&]() {
             QQuickItem *rcText = findItem("vamaRcNoteText");
             const QString says = rcText ? rcText->property("text").toString() : QString();
             expect(!global.videos.recording() && says.startsWith(QStringLiteral("No copy on this RC: the app has no folder to save it in.")),
                    "record without a save folder: no copy on the RC, and the note says how to allow it", says);
             global.settings.app.setHasFolder(true);
             // B10: the app's own zoom of the picture. Its button goes 1x, 2x, 4x and back.
             tap("vamaDigitalZoomButton");
         }},
        {300, [&]() {
             QQuickItem *video = findItem("videoContent");
             expect(link.digitalZoom() == 2.0 && video && video->scale() == 2.0,
                    "digital zoom: its button makes it 2x, and the picture is scaled", QString::number(video ? video->scale() : 0.0));
             shot("38_digital_zoom.png");
             tap("vamaDigitalZoomButton");
         }},
        {300, [&]() {
             expect(link.digitalZoom() == 4.0, "digital zoom: then 4x", QString::number(link.digitalZoom()));
             tap("vamaDigitalZoomButton");
         }},
        {300, [&]() {
             QQuickItem *video = findItem("videoContent");
             expect(link.digitalZoom() == 1.0 && video && video->scale() == 1.0, "digital zoom: then 1x again",
                    QString::number(link.digitalZoom()));
             // A tap is a place in the whole picture: the same tap turns the camera half as far at 2x.
             link.setPointMode(QStringLiteral("laser"));
             camera.setYaw(0.0);
             camera.setPitch(-30.0);
         }},
        {1200, [&]() {
             // On the picture, clear of the boxes in the middle (the lock box and the notes), the
             // gimbal buttons and QGC's telemetry.
             QQuickItem *drag = findItem("vamaVideoDrag");
             zoomTap = drag ? drag->mapToScene(QPointF(drag->width() * 0.78, drag->height() * 0.70)) : QPointF();
             mouse(QEvent::MouseButtonPress, zoomTap);
             mouse(QEvent::MouseButtonRelease, zoomTap);
         }},
        {2500, [&]() {
             yawAt1x = camera.yaw();
             camera.setYaw(0.0);
             camera.setPitch(-30.0);
             link.setDigitalZoom(2.0);
         }},
        {1200, [&]() {
             // The link has heard the camera's angles again: a tap aims from where the camera looks.
             mouse(QEvent::MouseButtonPress, zoomTap);
             mouse(QEvent::MouseButtonRelease, zoomTap);
         }},
        {2500, [&]() {
             const double yawAt2x = camera.yaw();
             QQuickItem *drag = findItem("vamaVideoDrag");
             expect(std::abs(yawAt1x) > 2.0 && std::abs(yawAt2x * 2.0 - yawAt1x) < 0.1 * std::abs(yawAt1x),
                    "digital zoom: a tap is a place in the whole picture (at 2x the same tap turns the camera half as far)",
                    QStringLiteral("%1 degrees at 1x, %2 at 2x; tap at %3,%4, tap area %5, point mode %6, aim %7, laser %8 "
                                   "\"%9\", angles %10")
                        .arg(yawAt1x).arg(yawAt2x).arg(zoomTap.x()).arg(zoomTap.y())
                        .arg(drag && drag->property("enabled").toBool() ? QStringLiteral("on") : QStringLiteral("off"))
                        .arg(link.pointMode()).arg(link.aimBusy()).arg(link.laserBusy()).arg(link.laserMessage())
                        .arg(link.attitudeValid()));
             link.setDigitalZoom(1.0);
             // A8 step 4: where the drone's position comes from. In the air GPS 1 loses its 3D fix and
             // the camera's position (GPS 2) takes over: once that has lasted 2 s, a note says so.
             positionStore.setSettings(1, {{"GPS2_TYPE", 14.0}, {"GPS_AUTO_SWITCH", 4.0}});
             positionStore.setArmed(1, true);
             positionStore.hearGps1(1, 3);
             positionStore.hearGps2(1, 3, 1500, 10);
             positionMs = 3500;
             positionStore.publishAll();  // GPS 1 from the start is no news
             positionStore.hearGps1(1, 0);
             positionStore.hearGps2(1, 3, 1500, 10);
             positionMs = 6000;
             positionStore.hearGps2(1, 3, 1500, 10);
         }},
        {300, [&]() {
             QQuickItem *note = findItem("vamaPositionNote");
             QQuickItem *text = findItem("vamaPositionNoteText");
             QObject *border = note ? note->property("border").value<QObject *>() : nullptr;
             const QString says = text ? text->property("text").toString() : QStringLiteral("not there");
             expect(note && note->isVisible() && says == QStringLiteral("GPS 1 has no 3D fix: the position comes from the camera (GPS 2)"),
                    "position source: the camera takes over in the air, and a note says so", says);
             expect(border && border->property("color").value<QColor>() != QColor(Qt::transparent),
                    "position source: the note is a warning (orange border)", QString());
             shot("39_position_note.png");
             tap("vamaPositionNote");
         }},
        {300, [&]() {
             QQuickItem *note = findItem("vamaPositionNote");
             expect(note && !note->isVisible(), "position source: a tap closes the note", QString());
             // GPS 1 is back: once that has lasted 2 s, a note without the warning's border.
             positionStore.hearGps1(1, 3);
             positionMs = 8600;
             positionStore.hearGps1(1, 3);
         }},
        {300, [&]() {
             QQuickItem *note = findItem("vamaPositionNote");
             QQuickItem *text = findItem("vamaPositionNoteText");
             QObject *border = note ? note->property("border").value<QObject *>() : nullptr;
             const QString says = text ? text->property("text").toString() : QStringLiteral("not there");
             expect(note && note->isVisible() && says == QStringLiteral("The position comes from GPS 1 again") && border &&
                        border->property("color").value<QColor>().alpha() == 0,
                    "position source: GPS 1 back, a note without the warning's border", says);
             // The note sits under the boxes at the top, never over one.
             bool clear = note != nullptr;
             for (const char *name : {"vamaResultBox", "vamaLockChip", "vamaFollowChip", "vamaCameraNote", "vamaLockHint",
                                      "vamaMapPackLoading"}) {
                 QQuickItem *box = findItem(name);
                 if (note && box && box->isVisible()) {
                     const QRectF a = note->mapRectToScene(QRectF(0, 0, note->width(), note->height()));
                     const QRectF b = box->mapRectToScene(QRectF(0, 0, box->width(), box->height()));
                     clear = clear && !a.intersects(b);
                 }
             }
             expect(clear, "position source: the note lies over no box at the top", QString());
             tap("vamaPositionNote");
         }},
        // The round proximity view (B25, Nish 2026-10-31: "Just like the mission planner proximity", "this proximity
        // into hidden button", "adjustable dialogue box", "the round view replace them").
        {300, [&]() {
             QQuickItem *button = findItem("vamaProximityButton");
             QQuickItem *box = findItem("vamaProximityWindow");
             expect(button && !button->isVisible() && box && !box->isVisible(),
                    "proximity: no button and no box before the sensor says anything", QString());
             expect(!findItem("vamaObstacleFront") && !findItem("vamaObstacleBack"), "proximity: B19's four bars are gone", QString());
             QQuickItem *qgcRadar = findItem("qgcVideoRadar");
             QObject *radar = qgcRadar ? qgcRadar->property("item").value<QObject *>() : nullptr;
             expect(radar && radar->objectName() == QStringLiteral("vamaNoVideoRadar") && !radar->property("visible").toBool(),
                    "proximity: QGC's moving radar on the video is replaced, its address draws nothing", QString());
             hearObstacle({3.0, -1, 2.2, -1, -1, -1, 5.5, -1});  // left: just past the range, inside the plot
         }},
        {300, [&]() {
             QQuickItem *button = findItem("vamaProximityButton");
             QQuickItem *box = findItem("vamaProximityWindow");
             expect(button && button->isVisible() && box && !box->isVisible(),
                    "proximity: with the sensor the button shows, and the box stays closed until it is pressed", QString());
             QQuickItem *column = findItem("vamaActionColumn");
             const QRectF b = shownAt("vamaProximityButton");
             const QRectF c = shownAt("vamaActionColumn");
             expect(column && b.left() >= c.right() && !b.intersects(c), "proximity: the button is beside the left buttons, on none", QString());
             tap("vamaProximityButton");
         }},
        {400, [&]() {
             QQuickItem *box = findItem("vamaProximityWindow");
             expect(box && box->isVisible(), "proximity: the button opens the box", QString());
             const QRectF shown = shownAt("vamaProximityWindow");
             const QRectF screen = shownAt("vamaProximityButton").united(shownAt("vamaActionColumn"));
             QQuickItem *layerItem = qobject_cast<QQuickItem *>(window->property("layer").value<QObject *>());
             const QRectF layer = layerItem ? layerItem->mapRectToScene(QRectF(0, 0, layerItem->width(), layerItem->height())) : QRectF();
             expect(!layer.isEmpty() && qAbs(shown.center().x() - layer.center().x()) < 2 && qAbs(shown.center().y() - layer.center().y()) < 2 &&
                        !shown.intersects(screen),
                    "proximity: it opens in the middle the first time, clear of the buttons",
                    QStringLiteral("%1,%2 middle %3,%4").arg(shown.center().x()).arg(shown.center().y()).arg(layer.center().x()).arg(layer.center().y()));
             // Over the screen's notes and hints too (some lie at z 1), wherever the user drags it.
             qreal highestOther = -1000;
             for (QQuickItem *other : (layerItem ? layerItem->childItems() : QList<QQuickItem *>())) {
                 if (other != box) {
                     highestOther = std::max(highestOther, other->z());
                 }
             }
             expect(box && box->z() > highestOther, "proximity: the box lies over every other item of the screen",
                    QStringLiteral("z %1, the highest other %2").arg(box ? box->z() : 0).arg(highestOther));
             expect(proximityRedAt(0.0, 3.0) && proximityRedAt(90.0, 2.2),
                    "proximity: each object a red arc in its direction (nose up) at its distance", QString());
             expect(!proximityRedAt(270.0, 5.5) && !proximityRedAt(270.0, 4.9), "proximity: nothing beyond the range (5.5 m of 5 m)", QString());
             const QString title = findItem("vamaProximityTitle") ? findItem("vamaProximityTitle")->property("text").toString() : QString();
             const QString status = findItem("vamaProximityStatus") ? findItem("vamaProximityStatus")->property("text").toString() : QString();
             expect(title == QStringLiteral("Proximity, within 5 m") && status == QStringLiteral("Object within 5 m"),
                    "proximity: the box says its range and that an object is within it", title + " | " + status);
             shot("42_proximity_view.png");
             hearObstacle({4.8, -1, 2.2, -1, -1, -1, 5.5, -1});  // the object ahead moves away
         }},
        {300, [&]() {
             expect(proximityRedAt(0.0, 4.8) && !proximityRedAt(0.0, 3.0), "proximity: the arc moves out with the object", QString());
             // Dragged by its top.
             proximityBoxBefore = shownAt("vamaProximityWindow");
             const QRectF top = shownAt("vamaProximityDrag");
             mouse(QEvent::MouseButtonPress, top.center());
             // Qt starts a drag only after the first move of about 10 points: the box follows from there.
             mouse(QEvent::MouseMove, top.center() + QPointF(20, 10));
             mouse(QEvent::MouseMove, top.center() + QPointF(100, 40));
             mouse(QEvent::MouseButtonRelease, top.center() + QPointF(100, 40));
         }},
        {700, [&]() {
             const QRectF now = shownAt("vamaProximityWindow");
             expect(qAbs(now.left() - (proximityBoxBefore.left() + 80)) < 2 && qAbs(now.top() - (proximityBoxBefore.top() + 30)) < 2 &&
                        qAbs(now.width() - proximityBoxBefore.width()) < 0.5,
                    "proximity: the box goes where its top is dragged",
                    QStringLiteral("%1,%2 from %3,%4").arg(now.left()).arg(now.top()).arg(proximityBoxBefore.left()).arg(proximityBoxBefore.top()));
             // Saved when it is let go (QtCore's Settings writes half a second later).
             QQuickItem *dragged = findItem("vamaProximityWindow");
             const QSettings savedPlace;
             expect(dragged && qAbs(savedPlace.value(QStringLiteral("VamaProximityWindow/x")).toDouble() - dragged->x()) < 0.5 &&
                        qAbs(savedPlace.value(QStringLiteral("VamaProximityWindow/y")).toDouble() - dragged->y()) < 0.5,
                    "proximity: the place it is dragged to is saved",
                    QStringLiteral("%1,%2").arg(savedPlace.value(QStringLiteral("VamaProximityWindow/x")).toDouble())
                        .arg(savedPlace.value(QStringLiteral("VamaProximityWindow/y")).toDouble()));
             // Made bigger by its corner.
             proximityBoxBefore = now;
             const QRectF corner = shownAt("vamaProximityResize");
             mouse(QEvent::MouseButtonPress, corner.center());
             mouse(QEvent::MouseMove, corner.center() + QPointF(20, 20));
             mouse(QEvent::MouseMove, corner.center() + QPointF(100, 70));
             mouse(QEvent::MouseButtonRelease, corner.center() + QPointF(100, 70));
         }},
        {400, [&]() {
             const QRectF now = shownAt("vamaProximityWindow");
             expect(qAbs(now.width() - (proximityBoxBefore.width() + 100)) < 2 && qAbs(now.height() - (proximityBoxBefore.height() + 70)) < 2 &&
                        qAbs(now.left() - proximityBoxBefore.left()) < 0.5,
                    "proximity: its corner makes the box bigger, and the view with it",
                    QStringLiteral("%1 x %2").arg(now.width()).arg(now.height()));
             expect(proximityRedAt(0.0, 4.8) && proximityRedAt(90.0, 2.2), "proximity: the bigger view draws the objects in their places", QString());
             shot("42b_proximity_view_moved.png");
             proximityBoxBefore = now;
             window->setProperty("mapIsMain", true);
         }},
        {300, [&]() {
             const QRectF now = shownAt("vamaProximityWindow");
             expect(findItem("vamaProximityWindow")->isVisible() && qAbs(now.left() - proximityBoxBefore.left()) < 0.5 &&
                        qAbs(now.top() - proximityBoxBefore.top()) < 0.5 && proximityRedAt(0.0, 4.8),
                    "proximity: the same with the map as the main view", QString());
             window->setProperty("mapIsMain", false);
             tap("vamaProximityClose");
         }},
        {300, [&]() {
             QQuickItem *box = findItem("vamaProximityWindow");
             expect(box && !box->isVisible() && findItem("vamaProximityButton")->isVisible(),
                    "proximity: X closes the box, the button stays", QString());
             tap("vamaProximityButton");
         }},
        {300, [&]() {
             const QRectF now = shownAt("vamaProximityWindow");
             expect(findItem("vamaProximityWindow")->isVisible() && qAbs(now.left() - proximityBoxBefore.left()) < 0.5 &&
                        qAbs(now.width() - proximityBoxBefore.width()) < 0.5,
                    "proximity: opened again where the user left it, as big", QString());
             QQuickItem *shownBox = findItem("vamaProximityWindow");
             const QSettings saved;
             const auto savedAs = [&](const char *key, double value) {
                 return qAbs(saved.value(QStringLiteral("VamaProximityWindow/") + QLatin1String(key)).toDouble() - value) < 0.5;
             };
             expect(shownBox && savedAs("x", shownBox->x()) && savedAs("y", shownBox->y()) && savedAs("w", shownBox->width()) &&
                        savedAs("h", shownBox->height()),
                    "proximity: its place and size are saved for the next start",
                    QStringLiteral("%1 x %2").arg(saved.value(QStringLiteral("VamaProximityWindow/w")).toDouble())
                        .arg(saved.value(QStringLiteral("VamaProximityWindow/h")).toDouble()));
             // The next start: a new box from the same file opens where this one was left, as big.
             QQmlComponent fileOfTheBox(&engine, QUrl(QStringLiteral("qrc:/Custom/qml/QGroundControl/FlyView/VamaProximityWindow.qml")));
             QQuickItem *again = qobject_cast<QQuickItem *>(fileOfTheBox.createWithInitialProperties({{QStringLiteral("visible"), false}}));
             if (again && shownBox) {
                 again->setParentItem(shownBox->parentItem());
                 again->setVisible(true);
             }
             const QRectF next = again ? again->mapRectToScene(QRectF(0, 0, again->width(), again->height())) : QRectF();
             expect(again && qAbs(next.left() - now.left()) < 0.5 && qAbs(next.top() - now.top()) < 0.5 &&
                        qAbs(next.width() - now.width()) < 0.5 && qAbs(next.height() - now.height()) < 0.5,
                    "proximity: at the next start it opens where it was left, as big",
                    again ? QStringLiteral("%1,%2 %3 x %4").arg(next.left()).arg(next.top()).arg(next.width()).arg(next.height())
                          : fileOfTheBox.errorString());
             if (again) {
                 again->setVisible(false);
                 again->deleteLater();
             }
             hearObstacle({-1, -1, -1, -1, -1, -1, 8.0, -1});
         }},
        {300, [&]() {
             const QString status = findItem("vamaProximityStatus") ? findItem("vamaProximityStatus")->property("text").toString() : QString();
             expect(proximityRedCount() == 0 && status == QStringLiteral("Clear within 5 m"),
                    "proximity: the objects gone or beyond the range, no red left", status);
             // Ten rings, one every tenth of the range (as Mission Planner's), counted on the line just
             // below the middle, right and left of the drone, up to the outer ring.
             {
                 const QRectF plot = proximityPlotRect();
                 const double font = std::max(9.0, std::min(14.0, std::round(std::min(plot.width(), plot.height()) / 26.0)));
                 const double radius = std::max(10.0, std::min(plot.width(), plot.height()) / 2.0 - font * 1.9);
                 const QImage image = window->grabWindow();
                 const qreal dpr = image.devicePixelRatio();
                 QList<double> rings[2];  // where each ring starts, out from the middle
                 for (int side = 0; side < 2; ++side) {
                     bool inRing = false;
                     for (double d = radius * 0.05; d <= radius + 1.0; d += 0.5) {
                         const double x = plot.center().x() + (side == 0 ? d : -d);
                         const QColor c = image.pixelColor(qRound(x * dpr), qRound((plot.center().y() + 3) * dpr));
                         const bool grid = c.red() > 55 && c.red() < 150 && qAbs(c.red() - c.blue()) < 15 && qAbs(c.green() - c.blue()) < 15;
                         if (grid && !inRing) {
                             rings[side].append(d);
                         }
                         inRing = grid;
                     }
                 }
                 bool placed = true;
                 for (const QList<double> &found : rings) {
                     placed = placed && found.size() == 10 && qAbs(found.first() - radius / 10.0) < 2.5 && qAbs(found.last() - radius) < 2.5;
                 }
                 expect(placed, "proximity: ten rings, one every tenth of the range, the last on it",
                        QStringLiteral("%1 right, %2 left, the last at %3 of %4").arg(rings[0].size()).arg(rings[1].size())
                            .arg(rings[0].isEmpty() ? 0.0 : rings[0].last()).arg(radius));
             }
             tap("vamaProximityButton");  // the button closes it too
         }},
        {300, [&]() {
             expect(!findItem("vamaProximityWindow")->isVisible(), "proximity: the button closes the box too", QString());
         }},
        // The voice the app speaks with (the client, Nish 2026-10-30: Indian English, female).
        // The admin steps closed the menu's window: shown again first, or its pictures cannot be taken.
        {300, [&]() {
             if (menuWindow) {
                 menuWindow->show();
             }
             openMenu();
         }},
        {400, [&]() {
             QQuickItem *item = menuItem("vamaMenuVoice");
             expect(menuOpen() && item && item->isVisible(), "voice: the menu has a Voice line", QString());
             menuTap("vamaMenuVoice");
         }},
        {400, [&]() {
             QQuickItem *box = menuItem("vamaVoiceBox");
             QQuickItem *fly = menuItem("toolbar_viewFly");
             const QStringList choices = voice.choices();
             expect(box && box->isVisible() && fly && !fly->isVisible(),
                    "voice: the Voice panel takes the place of the menu's buttons", QString());
             expect(voiceListsAsked == 0, "voice: with an Indian English voice, the panel asks the engine nothing (B23)",
                    QString::number(voiceListsAsked));
             expect(voice.current() == QStringLiteral("en-in-x-cxx#female_1-local"),
                    "voice: automatic, the offline Indian English female voice speaks", voice.current());
             expect(choices.size() == 5 &&
                        choices.at(0) == QStringLiteral("Automatic: English (India), female, en-in-x-cxx#female_1-local") &&
                        choices.at(1).endsWith(QStringLiteral(", en-in-x-cxx#female_1-local")) &&
                        choices.at(2) == QStringLiteral("English (India), male, en-in-x-cxx#male_1-local") &&
                        choices.at(3) == QStringLiteral("English (India), female, needs the internet, en-in-x-cxx#female_2-network") &&
                        choices.at(4).startsWith(QStringLiteral("English (United States), female")),
                    "voice: the list, Indian English first (offline, then female), then the others",
                    choices.join(QStringLiteral(" | ")));
             expect(box && box->property("currentIndex").toInt() == 0 && menuText("vamaVoiceNote").isEmpty(),
                    "voice: Automatic is shown, and no note when Android tells the genders", menuText("vamaVoiceNote"));
             shotOf(menuWindow, "40_voice.png");
             menuTap("vamaVoiceBox");  // opens the list
         }},
        {300, [&]() {
             // A real tap on the third line: the panel's binding must stay (the check after the RC's
             // voices change, below, needs it).
             const QStringList choices = voice.choices();
             expect(choices.size() > 2 && menuTapListLine(choices.at(2)), "voice: a tap on the list opens it, with the voices in it",
                    QString());
         }},
        {300, [&]() {
             QQuickItem *box = menuItem("vamaVoiceBox");
             const QString saved = QSettings().value(QLatin1String(VamaVoice::kSettingsKey)).toString();
             expect(voice.choiceIndex() == 2 && voice.current() == QStringLiteral("en-in-x-cxx#male_1-local") && box &&
                        box->property("currentIndex").toInt() == 2,
                    "voice: a voice chosen in the list speaks, and the list shows it", voice.current());
             expect(saved == QStringLiteral("en-in-x-cxx#male_1-local"), "voice: the choice is saved for the next start", saved);
             menuTap("vamaVoiceTest");
         }},
        {300, [&]() {
             expect(global.audioTests == 1, "voice: Test is QGC's own test of its speech, with the voice in use",
                    QString::number(global.audioTests));
             // The RC's voices change while the panel is open (the engine became ready, or another
             // engine): a second offline Indian English female voice moves the chosen one a line down.
             std::vector<vama::voice::Voice> more = olderVoices;
             more.push_back({"en-in-x-ahp#female_3-local", "en", "IN", vama::voice::Gender::Female});
             voice.setVoices(more);
         }},
        {300, [&]() {
             QQuickItem *box = menuItem("vamaVoiceBox");
             expect(voice.choiceIndex() == 3 && voice.current() == QStringLiteral("en-in-x-cxx#male_1-local") && box &&
                        box->property("count").toInt() == 6 && box->property("currentIndex").toInt() == 3,
                    "voice: the RC's voices change, the chosen voice still speaks, and the list shows it on its new line",
                    box ? QStringLiteral("line %1 of %2").arg(box->property("currentIndex").toInt()).arg(box->property("count").toInt())
                        : QStringLiteral("no list"));
             if (box) {
                 QMetaObject::invokeMethod(box, "activated", Q_ARG(int, 0));
             }
         }},
        {300, [&]() {
             expect(voice.choiceIndex() == 0 && voice.current() == QStringLiteral("en-in-x-cxx#female_1-local") &&
                        !QSettings().contains(QLatin1String(VamaVoice::kSettingsKey)),
                    "voice: back to Automatic, and the saved choice is gone", voice.current());
             // A newer Google engine: no gender in the names.
             voice.setVoices({{"en-us-x-iol-local", "en", "US", vama::voice::Gender::Unknown},
                              {"en-in-x-ena-network", "en", "IN", vama::voice::Gender::Unknown},
                              {"en-in-x-enc-local", "en", "IN", vama::voice::Gender::Unknown}});
         }},
        {300, [&]() {
             QQuickItem *note = menuItem("vamaVoiceNote");
             const QString says = menuText("vamaVoiceNote");
             expect(voice.current() == QStringLiteral("en-in-x-enc-local") && note && note->isVisible() &&
                        says.startsWith(QStringLiteral("Android does not say which of these voices are female")),
                    "voice: without genders, the first offline Indian English voice, and a note to try them with Test", says);
             // An engine without Indian English voice data.
             voice.setVoices({{"en-us-x-iol-local", "en", "US", vama::voice::Gender::Unknown}});
         }},
        {300, [&]() {
             const QString says = menuText("vamaVoiceNote");
             const QStringList choices = voice.choices();
             expect(says.startsWith(QStringLiteral("This remote control has no Indian English voice")) &&
                        choices.size() == 2 && choices.at(0) == QStringLiteral("Automatic"),
                    "voice: no Indian English voice, and the note says where to add one", says);
             shotOf(menuWindow, "40b_voice_note.png");
             menuTap("vamaVoiceDone");
         }},
        {300, [&]() {
             expect(!menuOpen(), "voice: Done closes the menu", QString());
             // Still no Indian English voice: the note says to add one and choose it here, so the
             // panel asks the engine again when it opens (B23: messages no longer do).
             openMenu();
         }},
        {400, [&]() { menuTap("vamaMenuVoice"); }},
        {400, [&]() {
             expect(voiceListsAsked == 1, "voice: without an Indian English voice, the panel asks the engine again (B23)",
                    QString::number(voiceListsAsked));
             voice.setVoices(olderVoices);
             menuTap("vamaVoiceDone");
         }},
        {300, [&]() {
             expect(!menuOpen(), "voice: Done closes the panel again", QString());
             // The range of the round proximity view, in the app menu (B19, B25).
             openMenu();
         }},
        {400, [&]() {
             QQuickItem *item = menuItem("vamaMenuObstacleRange");
             expect(menuOpen() && item && item->isVisible() && item->property("text").toString() == QStringLiteral("Proximity range"),
                    "proximity range: the menu has a line for it", item ? item->property("text").toString() : QString());
             menuTap("vamaMenuObstacleRange");
         }},
        {400, [&]() {
             QQuickItem *box = menuItem("vamaObstacleRangeBox");
             QQuickItem *fly = menuItem("toolbar_viewFly");
             expect(box && box->isVisible() && fly && !fly->isVisible(),
                    "proximity range: its panel takes the place of the menu's buttons", QString());
             const QString shown = box ? box->property("currentText").toString() : QStringLiteral("no list");
             expect(shown == QStringLiteral("5 m"), "proximity range: 5 m until the user chooses, the client's example", shown);
             shotOf(menuWindow, "43_obstacle_range.png");
             menuTap("vamaObstacleRangeBox");
         }},
        {300, [&]() {
             expect(menuTapListLine(QStringLiteral("10 m")), "proximity range: a tap on the list opens it, with the ranges in it", QString());
         }},
        {300, [&]() {
             QQuickItem *box = menuItem("vamaObstacleRangeBox");
             const QString shown = box ? box->property("currentText").toString() : QStringLiteral("no list");
             expect(proximity.barRange() == 10.0 && shown == QStringLiteral("10 m") &&
                        QSettings().value(QLatin1String(ProximityStore::kBarRangeKey)).toDouble() == 10.0,
                    "proximity range: the range chosen counts, the list shows it, and it is saved for the next start", shown);
             menuTap("vamaObstacleRangeDone");
         }},
        {300, [&]() {
             expect(!menuOpen(), "proximity range: Done closes the menu", QString());
             tap("vamaProximityButton");
         }},
        {400, [&]() {
             const QString title = findItem("vamaProximityTitle") ? findItem("vamaProximityTitle")->property("text").toString() : QString();
             expect(proximityRedAt(270.0, 8.0) && title == QStringLiteral("Proximity, within 10 m"),
                    "proximity range: with 10 m the view reaches the object at 8 m", title);
             // The drone gone while the box is open (its sensor with it), then a new range: no reading draws
             // the view again then, so the range itself must (the rings' distances follow it).
             if (QQuickItem *box = findItem("vamaProximityWindow")) {
                 box->setProperty("sectors", QVariant::fromValue<QObject *>(nullptr));
             }
         }},
        {300, [&]() {
             proximityRingWords = proximityRingWordsNow();
             proximity.setBarRange(5.0);
         }},
        {300, [&]() {
             expect(!proximityRingWords.isNull() && proximityRingWordsNow() != proximityRingWords,
                    "proximity range: without a drone, a new range draws the rings' distances again", QString());
             tap("vamaProximityButton");
             // Vehicle Configuration (the client, Nish 2026-10-30): outside Advanced Mode only Summary, Radio, Sensors.
             global.core.setShowAdvancedUI(false);
             const qsizetype windows = engine.rootObjects().size();
             engine.load(QUrl::fromLocalFile(QStringLiteral(VAMA_PREVIEW_DIR "/config.qml")));
             if (engine.rootObjects().size() == windows ||
                 !(configWindow = qobject_cast<QQuickWindow *>(engine.rootObjects().last()))) {
                 std::fprintf(stderr, "Vehicle Configuration window not loaded\n");
                 ++g_problems;
             }
         }},
        {600, [&]() {
             expect(configItem("vehicleConfig_root") != nullptr,
                    "vehicle configuration: QGC's address opens the VAMA copy", QString());
             expect(configShown("vehicleConfig_summary") && configShown("vehicleConfig_comp_Radio") &&
                        configShown("vehicleConfig_comp_Sensors"),
                    "vehicle configuration: outside Advanced Mode, Summary, Radio and Sensors", QString());
             expect(!configShown("vehicleConfig_comp_Frame") && !configShown("vehicleConfig_comp_Failsafes") &&
                        !configShown("vehicleConfig_comp_Motors") && !configShown("vehicleConfig_opticalFlowButton") &&
                        !configShown("vehicleConfig_parametersButton"),
                    "vehicle configuration: and nothing else (no Frame, Failsafes, Motors, Optical Flow or Parameters)",
                    QString());
             if (configWindow) {
                 shotOf(configWindow, "41_vehicle_config.png");
             }
             global.core.setShowAdvancedUI(true);
         }},
        {300, [&]() {
             expect(configShown("vehicleConfig_comp_Frame") && configShown("vehicleConfig_comp_Radio") &&
                        configShown("vehicleConfig_comp_Sensors") && configShown("vehicleConfig_comp_Failsafes") &&
                        configShown("vehicleConfig_comp_Motors") && configShown("vehicleConfig_opticalFlowButton") &&
                        configShown("vehicleConfig_parametersButton"),
                    "vehicle configuration: in Advanced Mode every line, Optical Flow and Parameters", QString());
             if (configWindow) {
                 shotOf(configWindow, "41b_vehicle_config_advanced.png");
             }
             // An admin opens Failsafes (the fourth), then turns Advanced Mode off with the page still open.
             if (QQuickItem *root = configItem("vehicleConfig_root")) {
                 QMetaObject::invokeMethod(root, "_navigateToComponent", Q_ARG(QVariant, 3), Q_ARG(QVariant, -1));
             }
         }},
        {300, [&]() {
             QQuickItem *root = configItem("vehicleConfig_root");
             expect(root && root->property("_selectedComponentIndex").toInt() == 3 && configItem("previewFailsafesPage"),
                    "vehicle configuration: in Advanced Mode Failsafes opens", QString());
             global.core.setShowAdvancedUI(false);
         }},
        {300, [&]() {
             QQuickItem *root = configItem("vehicleConfig_root");
             expect(!configShown("vehicleConfig_comp_Failsafes") && configShown("vehicleConfig_comp_Radio"),
                    "vehicle configuration: Advanced Mode off hides them again", QString());
             expect(root && root->property("_selectedSpecial").toString() == QStringLiteral("summary") &&
                        root->property("_selectedComponentIndex").toInt() == -1 && !configItem("previewFailsafesPage"),
                    "vehicle configuration: and the open Failsafes page closes (back to the Summary)",
                    root ? root->property("_selectedSpecial").toString() : QStringLiteral("no page"));
             QQuickItem *search = configItem("vehicleConfig_searchField");
             if (search) {
                 search->setProperty("text", QStringLiteral("fail"));
             }
         }},
        {300, [&]() {
             QQuickItem *root = configItem("vehicleConfig_root");
             expect(root && root->property("_searchQuery").toString() == QStringLiteral("fail") &&
                        !configShown("vehicleConfig_comp_Failsafes"),
                    "vehicle configuration: the search finds no hidden line outside Advanced Mode", QString());
             QQuickItem *search = configItem("vehicleConfig_searchField");
             if (search) {
                 search->setProperty("text", QString());
             }
             // Any other way to open a hidden component (Failsafes is the fourth) shows why.
             if (root) {
                 QMetaObject::invokeMethod(root, "_navigateToComponent", Q_ARG(QVariant, 3), Q_ARG(QVariant, -1));
             }
         }},
        {300, [&]() {
             QQuickItem *root = configItem("vehicleConfig_root");
             QQuickItem *panel = configItem("vehicleConfig_messagePanel");
             const QString says = root ? root->property("_messagePanelText").toString() : QStringLiteral("no page");
             expect(panel && panel->isVisible() && root->property("_selectedComponentIndex").toInt() == -1 &&
                        says == QStringLiteral("Failsafes opens only in Advanced Mode, which asks for the admin password."),
                    "vehicle configuration: a hidden component opened another way shows why, not its page", says);
             if (configWindow) {
                 shotOf(configWindow, "41c_vehicle_config_why.png");
                 configWindow->close();
             }
         }},
        // The license lock (row A9): the app opens only with a key made for this RC. The lock is made over
        // the stand-in window as CustomPlugin::createRootWindow makes it over QGC's (VamaLicense::showLock).
        {300, [&]() {
             license.load();
             // The camera link waits while the app is locked, as CustomPlugin holds the app's one
             // (SkydroidLink::holdAll, 2026-11-06).
             license.holdWhileLocked([&link](bool held) { link.setHeld(held); });
             expect(link.held() && !link.answering(), "license: while locked, the camera link is held", QString());
             const qsizetype windows = engine.rootObjects().size();
             engine.load(QUrl::fromLocalFile(QStringLiteral(VAMA_PREVIEW_DIR "/license.qml")));
             if (engine.rootObjects().size() == windows ||
                 !(licenseWindow = qobject_cast<QQuickWindow *>(engine.rootObjects().last()))) {
                 std::fprintf(stderr, "license window not loaded\n");
                 ++g_problems;
                 return;
             }
             licenseLock = license.lockWindow(&engine, licenseWindow);  // as CustomPlugin::createRootWindow
             expect(license.locked() && licenseLock && licenseLock->objectName() == QStringLiteral("vamaLicenseLock"),
                    "license: without a key the app is locked, and the lock screen is made", QString());
             // QGC opens its own dialogs at the start too (the first-run questions), after the lock.
             QMetaObject::invokeMethod(licenseWindow, "openQgcDialog");
         }},
        {500, [&]() {
             heldDatagrams = camera.datagrams;  // what was on its way when the link was held has come
             if (!licenseWindow || !licenseLock) {
                 return;
             }
             const qreal width = licenseLock->property("width").toReal();
             const qreal height = licenseLock->property("height").toReal();
             expect(licenseLock->property("opened").toBool() && qFuzzyCompare(width, qreal(licenseWindow->width())) &&
                        qFuzzyCompare(height, qreal(licenseWindow->height())),
                    "license: the lock screen covers the whole window", QStringLiteral("%1 x %2").arg(width).arg(height));
             expect(licenseText("vamaLicenseDeviceCode") == licenseDeviceCode,
                    "license: it shows this RC's device code, as VGCS's rules make it", licenseText("vamaLicenseDeviceCode"));
             expect(!licenseShown("vamaLicenseProblem"), "license: no words about a key before one is tried",
                    licenseText("vamaLicenseProblem"));
             shotOf(licenseWindow, "44_license_lock.png");
             licenseTap("previewUnderTheLock");
             licenseTap("vamaLicenseCopyButton");
             if (QQuickItem *field = licenseItem("vamaLicenseKeyField")) {
                 field->setProperty("text", licenseKeyElsewhere);
             }
             licenseTap("vamaLicenseActivateButton");
         }},
        {300, [&]() {
             if (!licenseWindow) {
                 return;
             }
             const int under = licenseWindow->property("pressesUnder").toInt();
             expect(under == 0, "license: no tap reaches the screen or QGC's dialog under the lock, and its buttons work "
                                "above a dialog opened after it", QString::number(under));
             expect(licenseClipboard == licenseDeviceCode && licenseText("vamaLicenseCopyButton") == QStringLiteral("Copied"),
                    "license: Copy puts the device code on the clipboard, and says so", licenseClipboard);
             const QString why = licenseText("vamaLicenseProblem");
             expect(licenseShown("vamaLicenseProblem") &&
                        why == QStringLiteral("This key is for another RC. Send the device code above to VAMA for a key "
                                              "for this one."),
                    "license: a key for another RC is refused, and the lock says why", why);
             expect(license.locked() && licenseLock, "license: and the app stays locked", QString());
             shotOf(licenseWindow, "44b_license_wrong.png");
             // QGC's question to close the app would open under the lock: the lock takes the back key. Sent
             // to the lock's content itself: the stand-in window need not be the active one, and a key sent to
             // an inactive window reaches no item and stays "accepted" (that once passed without the handler).
             QQuickItem *content = licenseItem("vamaLicenseLockContent");
             QKeyEvent back(QEvent::KeyPress, Qt::Key_Back, Qt::NoModifier);
             if (content) {
                 QCoreApplication::sendEvent(content, &back);
             }
             expect(content && back.isAccepted() && licenseLock && licenseLock->property("opened").toBool(),
                    "license: the RC's back key is taken by the lock, which stays", QString());
             licenseClipboard = QStringLiteral("\n") + licenseKeyHere + QStringLiteral("\n");  // as pasted from a message
             licenseTap("vamaLicensePasteButton");
         }},
        {300, [&]() {
             QQuickItem *field = licenseItem("vamaLicenseKeyField");
             const QString text = field ? field->property("text").toString() : QStringLiteral("no field");
             expect(text == licenseClipboard && !licenseShown("vamaLicenseProblem"),
                    "license: Paste puts the key in the field, and the words about the last key go", text.trimmed().left(20));
             expect(link.held() && camera.datagrams == heldDatagrams,
                    "license: nothing went to the camera while the app was locked",
                    QString::number(camera.datagrams - heldDatagrams));
             licenseTap("vamaLicenseActivateButton");
         }},
        {300, [&]() {
             expect(!license.locked() && licenseLock.isNull(), "license: the right key opens the app, and the lock screen goes",
                    license.keySummary());
             const bool talking = waitFor([&]() { return link.answering(); }, 3000);
             expect(!link.held() && talking && camera.datagrams > heldDatagrams,
                    "license: with the key, the camera link starts again", QString::number(camera.datagrams - heldDatagrams));
             license.holdWhileLocked([&link](bool held) { link.setHeld(held); });
             expect(!link.held(), "license: an open app holds nothing", QString());
             const QString saved = QSettings().value(QStringLiteral("VamaLicense/key")).toString();
             expect(saved == licenseKeyHere, "license: the key is saved for the next start", saved.left(20));
             licenseTap("previewQgcDialogButton");
         }},
        {300, [&]() {
             if (!licenseWindow) {
                 return;
             }
             const int under = licenseWindow->property("pressesUnder").toInt();
             expect(under == 100, "license: then QGC's dialog takes taps", QString::number(under));
             shotOf(licenseWindow, "44c_license_open.png");
             QMetaObject::invokeMethod(licenseWindow, "closeQgcDialog");
         }},
        {300, [&]() {
             licenseTap("previewUnderTheLock");
         }},
        {300, [&]() {
             if (!licenseWindow) {
                 return;
             }
             const int under = licenseWindow->property("pressesUnder").toInt();
             expect(under == 101, "license: and so does the screen", QString::number(under));
             licenseWindow->close();
         }},
        // When the lock screen cannot be made (row A9, 2026-11-06), a plain screen covers the app, made
        // as CustomPlugin makes it (VamaLicense::lockWindow). It says why and shows the device code, and
        // nothing under it takes a tap or a key. The same RC, its key gone again.
        {300, [&]() {
             QSettings().remove(QStringLiteral("VamaLicense"));
             plainLicense.load();
             const qsizetype windows = engine.rootObjects().size();
             engine.load(QUrl::fromLocalFile(QStringLiteral(VAMA_PREVIEW_DIR "/license.qml")));
             if (engine.rootObjects().size() == windows ||
                 !(licenseWindow = qobject_cast<QQuickWindow *>(engine.rootObjects().last()))) {
                 std::fprintf(stderr, "license window not loaded again\n");
                 ++g_problems;
                 return;
             }
             g_expectedWarnings = {QStringLiteral("the lock screen could not be made:"),
                                   QStringLiteral("a plain screen covers the app in place of the lock screen")};
             plainLock = plainLicense.lockWindow(&engine, licenseWindow);
             expect(plainLicense.locked() && plainLock && plainLock->objectName() == QStringLiteral("vamaLicensePlainLock"),
                    "license: when the lock screen cannot be made, a plain screen is made in its place",
                    plainLicense.lockScreenProblem().left(120));
             expect(g_expectedWarnings.isEmpty(), "license: and the app's log says so, with no other warning",
                    g_expectedWarnings.join(QStringLiteral(" | ")));
             g_expectedWarnings.clear();
         }},
        {300, [&]() {
             auto *plain = qobject_cast<QQuickItem *>(plainLock.data());
             if (!licenseWindow || !plain) {
                 return;
             }
             expect(plain->isVisible() && qFuzzyCompare(plain->width(), qreal(licenseWindow->width())) &&
                        qFuzzyCompare(plain->height(), qreal(licenseWindow->height())),
                    "license: the plain screen covers the whole window",
                    QStringLiteral("%1 x %2").arg(plain->width()).arg(plain->height()));
             const QString says = licenseText("vamaLicensePlainLockText");
             const QString why = plainLicense.lockScreenProblem();
             expect(!why.isEmpty() && says.contains(QStringLiteral("its lock screen could not open")) &&
                        says.contains(licenseDeviceCode) && says.contains(why),
                    "license: it says why, with the device code, for a photo to VAMA", says.left(120));
             shotOf(licenseWindow, "44d_license_plain.png");
             licenseTap("previewUnderTheLock");
             QKeyEvent back(QEvent::KeyPress, Qt::Key_Back, Qt::NoModifier);
             QCoreApplication::sendEvent(plain, &back);
             expect(back.isAccepted(), "license: the RC's back key stays on the plain screen", QString());
         }},
        {300, [&]() {
             if (!licenseWindow) {
                 return;
             }
             const int under = licenseWindow->property("pressesUnder").toInt();
             expect(under == 0, "license: no tap reaches the screen under the plain one", QString::number(under));
             licenseWindow->close();
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
    if (realInput.blocked > 0) {
        std::fprintf(stderr, "real input from the PC, not passed on: %d events\n", realInput.blocked);
    }
    std::fprintf(stderr, "QML problems: %d\n", g_problems);
    return (code == 0 && g_problems == 0) ? 0 : 1;
}

#include "main.moc"
