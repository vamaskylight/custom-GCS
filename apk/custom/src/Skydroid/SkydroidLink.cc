#include "SkydroidLink.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QSettings>
#include <QtNetwork/QNetworkDatagram>
#include <QtNetwork/QUdpSocket>
#include <QtPositioning/QGeoCoordinate>
#include <QtQml/QJSEngine>

#include "Fact.h"
#include "FactGroup.h"
#include "MultiVehicleManager.h"
#include "Vehicle.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <vector>

namespace top = skydroid::top;

namespace {

/// A saved or typed colour mode as a known id; anything else is the picture
/// as received (VGCS's normalize_palette_id).
QString normalizedThermalPalette(const QString &value)
{
    const QString id = value.trimmed().toLower();
    const QStringList ids = SkydroidLink::thermalPalettes();
    return ids.contains(id) ? id : ids.constFirst();
}

constexpr const char *kSettingsGroup = "VamaSkydroid";
constexpr const char *kDefaultHost = "192.168.144.108";
constexpr int kDefaultPort = 5000;
constexpr const char *kDefaultModel = "C13";
// C12, C13 and C14 Pro: day picture on RTSP port 554, thermal on 555
// (DOCS/SKYDROID-C13-OBSERVATION-SETUP.md; C14 Pro assumed the same).
constexpr const char *kDefaultDayVideoUrl = "rtsp://192.168.144.108:554/stream=1";
constexpr const char *kDefaultThermalVideoUrl = "rtsp://192.168.144.108:555/stream=2";
constexpr int kDefaultMaxSpeed = 20;     // deg/s at full deflection
constexpr int kMinMaxSpeed = 5;
constexpr int kMaxMaxSpeed = 60;         // GSY/GSP carry at most 63.5 deg/s

constexpr int kPollIntervalMs = 200;     // gimbal angle query, 5 per second
constexpr int kGaaHz = 5;                // VGCS turns the angle push on at 5 Hz
constexpr int kGaaEveryPollsWhenQuiet = 5;  // GAA again every second while no angles arrive
constexpr int kGaaEveryPolls = 50;          // and every 10 s anyway, in case the camera restarted
constexpr int kProbeFirstPoll = 8;          // other addresses 1.6 s after start,
constexpr int kProbeEveryPolls = 15;        // then every 3 s while no angles arrive
constexpr int kZoomPollEvery = 10;       // C14 Pro zoom step query every 2 s
constexpr int kAnswerCheckMs = 1000;
constexpr int kAnswerTimeoutMs = 3000;   // no reply this long = camera not answering
constexpr int kAttitudeFreshMs = 1500;   // older angles: send GAA again and try other addresses
constexpr int kLaserShotSettleMs = 120;  // VGCS _LRF_SLR_SHOT_SETTLE_S
constexpr int kLaserWaitMs = 1000;
constexpr int kZoomQueryAfterCmdMs = 350;  // VGCS _DZM_POLL_AFTER_CMD_S
constexpr double kC14ProLaserMaxM = 1500.0;
// VGCS counts each C12/C13 zoom step as 0.1x (ZOOM_STEP_SKYDROID; the C13
// has 30x digital zoom). Not measured on the camera yet: the camera's own
// zoom report (zoomReport) is shown in the settings to check it.
constexpr double kZoomPerStep = 0.1;
constexpr int kZoomHomeExtraSteps = 5;   // "1x" steps out this many more than counted
constexpr int kZoomOutIntervalMs = 60;

constexpr int kMotionIntervalMs = 100;   // speed refresh, 10 per second (VGCS: 80 ms)
constexpr int kTouchLeaseMs = 450;       // no setTouchMotion this long = the finger is gone
constexpr int kTouchMaxMs = 20000;       // one touch never turns the gimbal longer than this
constexpr double kTouchDeadZone = 0.05;
constexpr double kWheelDeadZone = 0.08;
constexpr int kRcCentre = 1500;
constexpr int kRcHalfRange = 400;        // full speed at 1100 or 1900 us
constexpr int kRcFreshMs = 1000;         // older RC values are ignored (RC link lost)
constexpr int kRcMin = 800;              // outside 800..2200 us is not a real channel value
constexpr int kRcMax = 2200;
constexpr int kWheelMoveUs = 25;         // a hold-position wheel must move this much before it drives the gimbal
constexpr double kWheelAngleStepDeg = 0.5;
constexpr int kWheelAngleIntervalMs = 150;
constexpr double kWheelApproachDps = 30.0;
constexpr double kWheelPitchMinDeg = -90.0;  // wheel at 1000 us
constexpr double kWheelPitchMaxDeg = 30.0;   // wheel at 2000 us (C13: +10, its limit)
constexpr double kC13PitchMaxDeg = 10.0;
constexpr double kWheelYawRangeDeg = 90.0;   // wheel at 1000 or 2000 us = -90 or +90 deg
constexpr int kDetectMs = 5000;
constexpr int kDetectMinSpanUs = 150;
constexpr int kMaxRcChannel = 18;
constexpr double kCenterYawDps = 30.0;
// Centre and look down use angle commands (GAY, GAP): the field test of test
// build 2 found the PTZ centre and look-down codes did nothing on the client's
// camera, while yaw centre (GAY) worked. Skydroid's TOP document does not list
// PTZ for the C13 at all.
constexpr double kAngleMoveDps = 30.0;

// Tap aiming, from VGCS's LRF click aim (vgcs/skydroid/adapter.py):
// offset in degrees = offset from the picture centre x half the field of view,
// with the C13's calibrated 83.4 x 46.9 degree view (also used for the C12),
// and the C13's GAC yaw running opposite to the picture (_LRF_C13_NEGATE_IMAGE_YAW).
constexpr double kC13AimFovHDeg = 83.4;
constexpr double kC13AimFovVDeg = 46.9;
constexpr bool kNegateImageYaw = true;
constexpr double kAimToleranceDeg = 1.0;  // close enough to the point to fire
constexpr int kAimSettleMs = 400;          // held that close this long (VGCS waits 0.5 s)
constexpr int kAimResendMs = 1500;         // send the angles once more (UDP can drop one)
constexpr int kAimTimeoutMs = 8000;
constexpr double kPi = 3.14159265358979323846;

// Object lock, mode "app": the app's own follow, after VGCS M14
// (vgcs/observe/gimbal_follow_control.py), which was tuned on a Skydroid C12
// in July 2026: nothing inside half a degree, 2.5 deg/s for each degree the
// object is off the cross, at most 40 deg/s.
constexpr double kFollowDeadbandDeg = 0.5;
constexpr double kFollowGain = 2.5;
constexpr double kFollowMaxDps = 40.0;
constexpr double kFollowMinMaxDps = 10.0;
// A picture is old when the app gets it (the video link, the decoder, the
// grab), so the camera has already turned part of the way the picture still
// shows. What it turned in that time is taken off before the speed is set.
// Without this, a start 20 degrees off went 7 degrees past the object with
// 0.3 s of delay, on paper; with it, about 2. A walking object is then
// followed a little further behind (its speed times this time).
constexpr double kFollowPictureAgeS = 0.30;
constexpr int kFollowLeaseMs = 400;          // no picture this long: the camera stops turning
constexpr int kFollowHistoryMs = 1000;
constexpr int kLockPictureWaitMs = 1500;     // no picture at all after the lock: the app cannot follow
constexpr int kLockNoPictureEndMs = 3000;    // the pictures stopped this long: the lock ends
constexpr int kLockLostMs = 2000;            // the object not seen this long: the lock ends
constexpr double kLockOnCrossDeg = 1.5;      // the laser measures the object only this near the cross
constexpr double kLockTurningDeg = 2.0;      // further off, the state says the camera is turning to it
constexpr double kLockDefaultBox = 0.12;     // a tap: a square box of this share of the picture's height
constexpr int kLockPictureMinSide = 64;      // a smaller picture is not looked at

// A point from the picture, without the laser (point mode "picture").
// The thermal lens is known for the C14 Pro only (VGCS c14pro_default). On the
// thermal picture of another camera the app can place the cross and nothing
// else: a tap this near the middle counts as the cross.
constexpr double kC14ThermalFovHDeg = 32.84;
constexpr double kC14ThermalFovVDeg = 26.35;
constexpr double kCrossTapNorm = 0.06;

// Object lock, mode "camera", as VGCS M13 does it (vgcs/skydroid/adapter.py and
// vgcs/map/observation/track_mixin.py): turn to the object when it is 1.5
// degrees or more from the centre, GOT where the object is in the picture
// (the 1280 x 720 frame of TOP 3.3.5), SUM confirm 80 ms later, SUM confirm
// again every 2 s, and SUM stop at the end.
constexpr double kLockTurnFirstDeg = 1.5;
// Where the camera takes the middle of the picture to be, in GOT points. In
// both field videos (a car at 21 m on 2026-10-06, a person at 8 m on
// 2026-10-09) the V13 turned about 2.4 degrees left and 1.2 up within a second
// of GOT at (640, 360), the middle of 1280 x 720, although the object was on
// the cross. That is 32 points across and 18 down, the same share (2.5 %) of
// both sides: as if GOT counted in a frame of 1344 x 756 of which the video
// shows the middle. If so, the tracker was started beside the object both
// times. So the GOT point is moved by that much. NOT proven on a camera.
constexpr int kGotCentreX = 672;
constexpr int kGotCentreY = 378;
constexpr int kGotFrameW = 1344;
constexpr int kGotFrameH = 756;
constexpr int kLockTickMs = 200;
constexpr int kLockConfirmDelayMs = 80;
constexpr int kLockAfterStopMs = 60;     // VGCS waits 50 ms between an old lock's stop and a new GOT
constexpr int kLockConfirmEveryMs = 2000;
constexpr int kLockFirstLaserMs = 400;
constexpr int kLockLaserEveryMs = 3000;  // VGCS _M13_SLR_FRESH_INTERVAL_S
constexpr double kLockFollowDeg = 0.8;
constexpr int kLockFollowWarnMs = 6000;
// The camera's first move after the lock command is its tracker taking over,
// not the camera following something. In the field video of test build 4 it
// jumped 2.6 degrees within half a second beside a parked car, and the app
// said "is following". So the angles to compare with are taken after this.
// A turn after that is no proof either: in the video of test build 6 the
// camera drifted 6 degrees in 17 s while the person walked 18 degrees.
constexpr int kLockSettleMs = 2000;

// Some C13 firmware takes zoom on these ports as well (VGCS _ZOOM_EXTRA_PORTS).
const int kC13ZoomExtraPorts[] = {9003, 19853};

// What VGCS tries when the camera is quiet (vgcs/skydroid/adapter.py
// _C13_PROBE_PORTS, vgcs/skydroid/targets.py): the camera itself, the RC
// relay, and the RC hotspot gateway for a tablet on the RC's Wi-Fi.
// The first host and port are the camera's own address (used for the laser).
QStringList g_probeHosts = {QStringLiteral("192.168.144.108"), QStringLiteral("192.168.144.12"),
                            QStringLiteral("192.168.43.1")};
QList<int> g_probePorts = {5000, 9003, 19856};

/// The same frame with the upper-case "#TP" header that Skydroid's TOP
/// documents use for every gimbal frame (the checksum covers the header).
std::string withUpperHeader(const std::string &frame)
{
    if (frame.size() < 5 || frame.compare(0, 3, "#tp") != 0) {
        return frame;
    }
    const std::string body = "#TP" + frame.substr(3, frame.size() - 5);
    return body + top::checksum(body);
}

/// The value encodeSpeed2 sends, in 0.5 deg/s units.
int speedUnits(double degPerSecond)
{
    return static_cast<int>(std::clamp<long long>(top::pyRound(degPerSecond / 0.5), -127, 127));
}

/// C14 Pro zoom for a reported DZM step, as VGCS labels it ("W2.9x", "T5.3x").
/// From vgcs/skydroid/command_map.py c14pro_default: short lens steps 0..85
/// (61.4 deg wide), long lens 86..184 (14.7 deg wide); each step crops 36 px
/// of a 3840 px readout, restarting at the lens change (Skydroid, 2026-09-19).
QString c14ProZoomLabel(int step)
{
    const bool longLens = step >= 86;
    const int m = longLens ? std::clamp(step - 86, 0, 98) : std::clamp(step, 0, 85);
    const double crop = 3840.0 / std::max(1, 3840 - 36 * m);
    const double lensFactor = longLens ? std::tan(61.4 * kPi / 360.0) / std::tan(14.7 * kPi / 360.0) : 1.0;
    return QStringLiteral("%1%2x").arg(longLens ? QStringLiteral("T") : QStringLiteral("W")).arg(crop * lensFactor, 0, 'f', 1);
}

} // namespace

SkydroidLink::SkydroidLink(QObject *parent)
    : QObject(parent)
{
    _pollTimer.setInterval(kPollIntervalMs);
    _answerTimer.setInterval(kAnswerCheckMs);
    _laserShotTimer.setSingleShot(true);
    _laserShotTimer.setInterval(kLaserShotSettleMs);
    _laserTimeoutTimer.setSingleShot(true);
    _laserTimeoutTimer.setInterval(kLaserWaitMs);
    _motionTimer.setInterval(kMotionIntervalMs);
    _wheelAngleTimer.setSingleShot(true);
    _detectTimer.setSingleShot(true);
    _detectTimer.setInterval(kDetectMs);
    _zoomOutTimer.setInterval(kZoomOutIntervalMs);
    _aimTimer.setInterval(100);
    _lockTimer.setInterval(kLockTickMs);

    connect(&_pollTimer, &QTimer::timeout, this, &SkydroidLink::_poll);
    connect(&_answerTimer, &QTimer::timeout, this, &SkydroidLink::_checkAnswering);
    connect(&_laserShotTimer, &QTimer::timeout, this, &SkydroidLink::_laserReadAfterShot);
    connect(&_laserTimeoutTimer, &QTimer::timeout, this, &SkydroidLink::_laserFinish);
    connect(&_motionTimer, &QTimer::timeout, this, &SkydroidLink::_motionTick);
    connect(&_wheelAngleTimer, &QTimer::timeout, this, &SkydroidLink::_sendWheelAngles);
    connect(&_detectTimer, &QTimer::timeout, this, &SkydroidLink::_finishWheelDetect);
    connect(&_zoomOutTimer, &QTimer::timeout, this, &SkydroidLink::_zoomOutTick);
    connect(&_aimTimer, &QTimer::timeout, this, &SkydroidLink::_aimTick);
    connect(&_lockTimer, &QTimer::timeout, this, &SkydroidLink::_lockTick);

    MultiVehicleManager *manager = MultiVehicleManager::instance();
    connect(manager, &MultiVehicleManager::activeVehicleChanged, this, &SkydroidLink::_activeVehicleChanged);
    _activeVehicleChanged(manager->activeVehicle());

    _loadSettings();
    _applyModel();
    _rebuildEndpoints();
    if (_enabled) {
        _start();
    }
}

SkydroidLink::~SkydroidLink()
{
    _stop();
}

SkydroidLink *SkydroidLink::instance()
{
    static SkydroidLink *link = new SkydroidLink(QCoreApplication::instance());
    return link;
}

SkydroidLink *SkydroidLink::create(QQmlEngine *qmlEngine, QJSEngine *jsEngine)
{
    Q_UNUSED(qmlEngine);
    Q_UNUSED(jsEngine);
    SkydroidLink *link = instance();
    QJSEngine::setObjectOwnership(link, QJSEngine::CppOwnership);
    return link;
}

QStringList SkydroidLink::models()
{
    return {QStringLiteral("C12"), QStringLiteral("C13"), QStringLiteral("C14 Pro")};
}

QStringList SkydroidLink::modelNames()
{
    // VAMA sells these Skydroid cameras under its own names (client, 2026-10-06).
    // Only the screen uses them: the code, the saved settings and VGCS keep
    // Skydroid's names, which the protocol notes are written against.
    return {QStringLiteral("V12"), QStringLiteral("V13"), QStringLiteral("V14 Pro")};
}

QStringList SkydroidLink::thermalPalettes()
{
    // VGCS's ids in its order, so the colour table made from VGCS
    // (apk/make_thermal_palettes.py) has one row per entry, in this order.
    return {QStringLiteral("camera"), QStringLiteral("black_hot"), QStringLiteral("ironbow"),
            QStringLiteral("rainbow"), QStringLiteral("red_hot"), QStringLiteral("green"),
            QStringLiteral("sepia")};
}

QStringList SkydroidLink::thermalPaletteNames()
{
    return {tr("White hot (as received)"), tr("Black hot"), tr("Ironbow"), tr("Rainbow"),
            tr("Red hot"), tr("Green"), tr("Sepia")};
}

QString SkydroidLink::modelName() const
{
    const qsizetype i = models().indexOf(_model);
    return i >= 0 ? modelNames().at(i) : _model;
}

void SkydroidLink::setProbeTargets(const QStringList &hosts, const QList<int> &ports)
{
    g_probeHosts = hosts;
    g_probePorts = ports;
}

// --- Settings --------------------------------------------------------------

void SkydroidLink::_loadSettings()
{
    QSettings settings;
    settings.beginGroup(kSettingsGroup);
    _enabled = settings.value(QStringLiteral("enabled"), false).toBool();
    _host = settings.value(QStringLiteral("host"), QString::fromLatin1(kDefaultHost)).toString().trimmed();
    _port = settings.value(QStringLiteral("port"), kDefaultPort).toInt();
    _model = settings.value(QStringLiteral("model"), QString::fromLatin1(kDefaultModel)).toString();
    _maxSpeed = settings.value(QStringLiteral("maxSpeed"), kDefaultMaxSpeed).toInt();
    _reverseYaw = settings.value(QStringLiteral("reverseYaw"), false).toBool();
    _reversePitch = settings.value(QStringLiteral("reversePitch"), false).toBool();
    _wheelPitchChannel = settings.value(QStringLiteral("wheelPitchChannel"), 0).toInt();
    _wheelYawChannel = settings.value(QStringLiteral("wheelYawChannel"), 0).toInt();
    _wheelHoldsPosition = settings.value(QStringLiteral("wheelHoldsPosition"), false).toBool();
    _wheelReverse = settings.value(QStringLiteral("wheelReverse"), false).toBool();
    _reverseTapYaw = settings.value(QStringLiteral("reverseTapYaw"), false).toBool();
    _dayVideoUrl = settings.value(QStringLiteral("dayVideoUrl")).toString().trimmed();
    _thermalVideoUrl = settings.value(QStringLiteral("thermalVideoUrl")).toString().trimmed();
    _thermalPalette = normalizedThermalPalette(settings.value(QStringLiteral("thermalPalette")).toString());
    _lockMode = settings.value(QStringLiteral("lockMode"), QStringLiteral("app")).toString();
    _pointMode = settings.value(QStringLiteral("pointMode"), QStringLiteral("laser")).toString();
    settings.endGroup();
    if (!lockModes().contains(_lockMode)) {
        _lockMode = QStringLiteral("app");
    }
    if (!pointModes().contains(_pointMode)) {
        _pointMode = QStringLiteral("laser");
    }
    if (_dayVideoUrl.isEmpty()) {
        _dayVideoUrl = QString::fromLatin1(kDefaultDayVideoUrl);
    }
    if (_thermalVideoUrl.isEmpty()) {
        _thermalVideoUrl = QString::fromLatin1(kDefaultThermalVideoUrl);
    }
    if (!models().contains(_model)) {
        _model = QString::fromLatin1(kDefaultModel);
    }
    if (_port <= 0 || _port > 65535) {
        _port = kDefaultPort;
    }
    _maxSpeed = std::clamp(_maxSpeed, kMinMaxSpeed, kMaxMaxSpeed);
    _wheelPitchChannel = std::clamp(_wheelPitchChannel, 0, kMaxRcChannel);
    _wheelYawChannel = std::clamp(_wheelYawChannel, 0, kMaxRcChannel);
}

void SkydroidLink::_saveSettings() const
{
    QSettings settings;
    settings.beginGroup(kSettingsGroup);
    settings.setValue(QStringLiteral("enabled"), _enabled);
    settings.setValue(QStringLiteral("host"), _host);
    settings.setValue(QStringLiteral("port"), _port);
    settings.setValue(QStringLiteral("model"), _model);
    settings.setValue(QStringLiteral("maxSpeed"), _maxSpeed);
    settings.setValue(QStringLiteral("reverseYaw"), _reverseYaw);
    settings.setValue(QStringLiteral("reversePitch"), _reversePitch);
    settings.setValue(QStringLiteral("wheelPitchChannel"), _wheelPitchChannel);
    settings.setValue(QStringLiteral("wheelYawChannel"), _wheelYawChannel);
    settings.setValue(QStringLiteral("wheelHoldsPosition"), _wheelHoldsPosition);
    settings.setValue(QStringLiteral("wheelReverse"), _wheelReverse);
    settings.setValue(QStringLiteral("reverseTapYaw"), _reverseTapYaw);
    settings.setValue(QStringLiteral("dayVideoUrl"), _dayVideoUrl);
    settings.setValue(QStringLiteral("thermalVideoUrl"), _thermalVideoUrl);
    settings.setValue(QStringLiteral("thermalPalette"), _thermalPalette);
    settings.setValue(QStringLiteral("lockMode"), _lockMode);
    settings.setValue(QStringLiteral("pointMode"), _pointMode);
    settings.endGroup();
}

void SkydroidLink::setEnabled(bool enabled)
{
    if (enabled == _enabled) {
        return;
    }
    _enabled = enabled;
    _saveSettings();
    if (_enabled) {
        _start();
    } else {
        _stop();
    }
    emit enabledChanged();
}

void SkydroidLink::setHost(const QString &host)
{
    const QString h = host.trimmed();
    if (h == _host) {
        return;
    }
    _host = h;
    _rebuildEndpoints();
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setPort(int port)
{
    if (port <= 0 || port > 65535 || port == _port) {
        return;
    }
    _port = port;
    _rebuildEndpoints();
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setModel(const QString &model)
{
    if (model == _model || !models().contains(model)) {
        return;
    }
    _model = model;
    _applyModel();
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setMaxSpeed(int degPerSecond)
{
    const int speed = std::clamp(degPerSecond, kMinMaxSpeed, kMaxMaxSpeed);
    if (speed == _maxSpeed) {
        return;
    }
    _maxSpeed = speed;
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setReverseYaw(bool reverse)
{
    if (reverse == _reverseYaw) {
        return;
    }
    _reverseYaw = reverse;
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setReversePitch(bool reverse)
{
    if (reverse == _reversePitch) {
        return;
    }
    _reversePitch = reverse;
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setWheelPitchChannel(int channel)
{
    const int c = std::clamp(channel, 0, kMaxRcChannel);
    if (c == _wheelPitchChannel) {
        return;
    }
    _wheelPitchChannel = c;
    _wheelStart[PitchAxis].reset();
    _wheelMoved[PitchAxis] = false;
    _wheelTarget[PitchAxis].reset();
    _wheelSent[PitchAxis].reset();
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setWheelYawChannel(int channel)
{
    const int c = std::clamp(channel, 0, kMaxRcChannel);
    if (c == _wheelYawChannel) {
        return;
    }
    _wheelYawChannel = c;
    _wheelStart[YawAxis].reset();
    _wheelMoved[YawAxis] = false;
    _wheelTarget[YawAxis].reset();
    _wheelSent[YawAxis].reset();
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setWheelHoldsPosition(bool holds)
{
    if (holds == _wheelHoldsPosition) {
        return;
    }
    _wheelHoldsPosition = holds;
    for (int a = 0; a < 2; ++a) {
        _wheelStart[a].reset();
        _wheelMoved[a] = false;
        _wheelTarget[a].reset();
        _wheelSent[a].reset();
    }
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setWheelReverse(bool reverse)
{
    if (reverse == _wheelReverse) {
        return;
    }
    _wheelReverse = reverse;
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setReverseTapYaw(bool reverse)
{
    if (reverse == _reverseTapYaw) {
        return;
    }
    _reverseTapYaw = reverse;
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setDayVideoUrl(const QString &url)
{
    const QString u = url.trimmed().isEmpty() ? QString::fromLatin1(kDefaultDayVideoUrl) : url.trimmed();
    if (u == _dayVideoUrl) {
        return;
    }
    _dayVideoUrl = u;
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setThermalVideoUrl(const QString &url)
{
    const QString u = url.trimmed().isEmpty() ? QString::fromLatin1(kDefaultThermalVideoUrl) : url.trimmed();
    if (u == _thermalVideoUrl) {
        return;
    }
    _thermalVideoUrl = u;
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setThermalPalette(const QString &id)
{
    const QString palette = normalizedThermalPalette(id);
    if (palette == _thermalPalette) {
        return;
    }
    _thermalPalette = palette;
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::_applyModel()
{
    // The lock's stop goes out in the old camera's format.
    if (_lockActive || _lockBusy) {
        _endLock(QString());
    }
    _options = top::Options{};
    if (_model == QStringLiteral("C14 Pro")) {
        // Same choices as the VGCS c14pro_default profile.
        _options.gClassUpperHeader = true;
        _options.slrMaxDm = top::slrMaxDmForRange(kC14ProLaserMaxM);
    }
    // Another camera: nothing known about its zoom yet.
    if (_zoomStep != -1 || _zoomCount != 0 || !_zoomReport.isEmpty()) {
        _zoomStep = -1;
        _zoomCount = 0;
        _zoomReport.clear();
        emit zoomChanged();
    }
}

// --- Addresses -------------------------------------------------------------

void SkydroidLink::_rebuildEndpoints()
{
    // The lock's stop goes to the camera that has the lock.
    if (_lockActive || _lockBusy) {
        _endLock(QString());
    }
    _configured = Endpoint{QHostAddress(_host), static_cast<quint16>(_port)};
    _endpoints.clear();
    auto add = [this](const QHostAddress &address, int port) {
        if (address.isNull() || port <= 0 || port > 65535) {
            return;
        }
        const Endpoint endpoint{address, static_cast<quint16>(port)};
        if (!_endpoints.contains(endpoint)) {
            _endpoints.append(endpoint);
        }
    };
    QStringList hosts{_host};
    for (const QString &h : g_probeHosts) {
        if (!hosts.contains(h)) {
            hosts.append(h);
        }
    }
    QList<int> ports{_port};
    for (int p : g_probePorts) {
        if (!ports.contains(p)) {
            ports.append(p);
        }
    }
    for (const QString &h : hosts) {
        for (int p : ports) {
            add(QHostAddress(h), p);
        }
    }
    _sinceAttitude.invalidate();
    _setActive(_configured);
}

void SkydroidLink::_setActive(const Endpoint &endpoint)
{
    if (endpoint == _active) {
        return;
    }
    _active = endpoint;
    emit activeEndpointChanged();
}

QString SkydroidLink::activeEndpoint() const
{
    if (_active.address.isNull()) {
        return QString();
    }
    return QStringLiteral("%1:%2").arg(_active.address.toString()).arg(_active.port);
}

SkydroidLink::Endpoint SkydroidLink::_knownEndpointFor(const QHostAddress &address, quint16 port) const
{
    const Endpoint exact{address, port};
    if (_endpoints.contains(exact)) {
        return exact;
    }
    // A relay can answer from another port. Keep using the address we send to on that host.
    if (address.isEqual(_active.address)) {
        return _active;
    }
    for (const Endpoint &endpoint : _endpoints) {
        if (endpoint.address.isEqual(address)) {
            return endpoint;
        }
    }
    return exact;
}

bool SkydroidLink::_attitudeFresh() const
{
    return _sinceAttitude.isValid() && _sinceAttitude.elapsed() < kAttitudeFreshMs;
}

void SkydroidLink::_probe()
{
    // Only harmless questions go to the other addresses: angle push on, and angles.
    const std::string gaa = top::buildGaaEnable(kGaaHz, _options);
    const std::string gac = top::buildGacQuery(_options);
    for (const Endpoint &endpoint : _endpoints) {
        if (endpoint == _active) {
            continue;
        }
        _sendTo(endpoint, gaa);
        _sendTo(endpoint, gac);
    }
}

QList<SkydroidLink::Endpoint> SkydroidLink::_laserEndpoints() const
{
    QList<Endpoint> list{_active};
    if (!list.contains(_configured)) {
        list.append(_configured);
    }
    // The camera's own address: an RC relay often passes the angles but not the laser.
    if (!g_probeHosts.isEmpty() && !g_probePorts.isEmpty()) {
        const Endpoint direct{QHostAddress(g_probeHosts.first()), static_cast<quint16>(g_probePorts.first())};
        if (!direct.address.isNull() && !list.contains(direct)) {
            list.append(direct);
        }
    }
    return list;
}

// --- Socket ----------------------------------------------------------------

void SkydroidLink::_start()
{
    _stop();
    _socket = new QUdpSocket(this);
    if (!_socket->bind(QHostAddress::AnyIPv4, 0)) {
        _socket->deleteLater();
        _socket = nullptr;
        return;
    }
    connect(_socket, &QUdpSocket::readyRead, this, &SkydroidLink::_readPending);
    _sinceReply.invalidate();
    _sinceAttitude.invalidate();
    _setActive(_configured);
    _pollCount = 0;
    _pollTimer.start();
    _answerTimer.start();
    _poll();
}

void SkydroidLink::_stop()
{
    // Never leave the camera following an object, or the gimbal turning, when
    // the link goes away.
    if (_lockActive || _lockBusy) {
        _endLock(QString());
    }
    if (_socket && (_moving || _stopRepeats > 0)) {
        _sendStop();
    }
    _motionTimer.stop();
    _moving = false;
    _stopRepeats = 0;
    _touchActive = false;
    _touchBlocked = false;
    _wheelAngleTimer.stop();
    _zoomOutTimer.stop();
    _zoomOutLeft = 0;
    _aimTimer.stop();
    if (_aimBusy) {
        _aimBusy = false;
        emit aimBusyChanged();
    }

    _pollTimer.stop();
    _answerTimer.stop();
    _laserShotTimer.stop();
    _laserTimeoutTimer.stop();
    if (_socket) {
        _socket->close();
        _socket->deleteLater();
        _socket = nullptr;
    }
    if (_laserBusy) {
        _laserBusy = false;
        emit laserBusyChanged();
    }
    if (_answering) {
        _answering = false;
        emit answeringChanged();
    }
    if (_attitudeValid) {
        _attitudeValid = false;
        emit attitudeChanged();
    }
}

void SkydroidLink::_sendTo(const Endpoint &endpoint, const std::string &frame)
{
    if (!_socket || frame.empty() || endpoint.address.isNull() || endpoint.port == 0) {
        return;
    }
    _socket->writeDatagram(frame.data(), static_cast<qint64>(frame.size()), endpoint.address, endpoint.port);
}

void SkydroidLink::_sendGimbal(const std::string &frame)
{
    // Both forms, the documented one first: test build 3 showed the V13
    // ignoring angle commands sent lower-case, as VGCS sends them. A camera
    // that takes both gets the same command twice, which changes nothing.
    const std::string upper = withUpperHeader(frame);
    if (upper != frame) {
        _send(upper);
    }
    _send(frame);
}

void SkydroidLink::_readPending()
{
    while (_socket && _socket->hasPendingDatagrams()) {
        const QNetworkDatagram datagram = _socket->receiveDatagram();
        const QByteArray data = datagram.data();
        const auto frame = top::parseTpFrame(std::string(data.constData(), static_cast<size_t>(data.size())), _options);
        if (frame) {
            const quint16 port = datagram.senderPort() > 0 ? static_cast<quint16>(datagram.senderPort()) : 0;
            _handleFrame(*frame, _knownEndpointFor(datagram.senderAddress(), port));
        }
    }
}

void SkydroidLink::_handleFrame(const top::DecodedFrame &frame, const Endpoint &from)
{
    _sinceReply.start();
    if (!_answering) {
        _answering = true;
        emit answeringChanged();
    }

    if (frame.tag == "GAC" && frame.yaw && frame.pitch) {
        if (!(from == _active)) {
            // Angles from another address. Move there only when the current one
            // has gone quiet, so two answering addresses never take turns.
            if (_attitudeFresh()) {
                return;
            }
            _setActive(from);
        }
        _sinceAttitude.start();
        _yaw = *frame.yaw;
        _pitch = *frame.pitch;
        _roll = frame.roll.value_or(0.0);
        _attitudeValid = true;
        emit attitudeChanged();
    } else if (frame.tag == "SLR" && _laserBusy) {
        const char source = frame.address.empty() ? ' ' : frame.address[0];
        if (source == 'E') {
            if (frame.slrDm) {
                _laserFromE = frame.slrDm;
                _laserFinish();  // the laser module answered: no need to wait
            }
        } else if (source == 'D' && frame.slrDm) {
            _laserFromD = frame.slrDm;
        }
    } else if (frame.tag == "DZM" && frame.ctrl == 'r') {
        // A read reply (a write is only echoed back). Kept as sent, so the
        // settings can show what a C12/C13 reports; only the C14 Pro's step
        // is understood (VGCS c14pro_default).
        bool changed = false;
        const QString report = QString::fromStdString(frame.data);
        if (report != _zoomReport) {
            _zoomReport = report;
            changed = true;
        }
        if (_isZoomStepCamera() && frame.dzmStep && *frame.dzmStep != _zoomStep) {
            _zoomStep = *frame.dzmStep;
            changed = true;
        }
        if (changed) {
            emit zoomChanged();
        }
    }
}

void SkydroidLink::_poll()
{
    ++_pollCount;
    const bool fresh = _attitudeFresh();
    // VGCS order: angle push on (GAA), then the angle question (GAC).
    if ((!fresh && (_pollCount % kGaaEveryPollsWhenQuiet) == 1) || (_pollCount % kGaaEveryPolls) == 0) {
        _send(top::buildGaaEnable(kGaaHz, _options));
    }
    _send(top::buildGacQuery(_options));
    if (_isZoomStepCamera() && (_pollCount % kZoomPollEvery) == 1) {
        _send(top::buildDzmQuery(_options));
    }
    if (!fresh && _pollCount >= kProbeFirstPoll && ((_pollCount - kProbeFirstPoll) % kProbeEveryPolls) == 0) {
        _probe();
    }
}

void SkydroidLink::_checkAnswering()
{
    const bool replyFresh = _sinceReply.isValid() && _sinceReply.elapsed() < kAnswerTimeoutMs;
    if (!replyFresh && _answering) {
        _answering = false;
        emit answeringChanged();
    }
    const bool attitudeFresh = _sinceAttitude.isValid() && _sinceAttitude.elapsed() < kAnswerTimeoutMs;
    if (!attitudeFresh && _attitudeValid) {
        // Old angles must never be used to place a target.
        _attitudeValid = false;
        emit attitudeChanged();
    }
}

// --- Gimbal ------------------------------------------------------------------

double SkydroidLink::shapedSpeed(double deflection, double maxSpeed, double deadZone)
{
    if (!std::isfinite(deflection) || !std::isfinite(maxSpeed)) {
        return 0.0;
    }
    const double d = std::clamp(deflection, -1.0, 1.0);
    const double a = std::abs(d);
    if (a <= deadZone || deadZone >= 1.0) {
        return 0.0;
    }
    // Squared, so half way gives a quarter of the speed: fine aim near the middle.
    const double m = (a - deadZone) / (1.0 - deadZone);
    const double speed = maxSpeed * m * m;
    return d < 0.0 ? -speed : speed;
}

void SkydroidLink::ptz(const QString &action)
{
    _endLockByHand();
    const QString a = action.trimmed().toLower();
    const std::string frame = top::buildPtz(a.toStdString(), _options);
    // VGCS ptz_stop_burst: C13 can coast after a single stop.
    const int repeats = (a == QStringLiteral("stop")) ? 3 : 1;
    for (int i = 0; i < repeats; ++i) {
        _send(frame);
    }
}

void SkydroidLink::setTouchMotion(double x, double y)
{
    if (!_enabled || !_socket || _touchBlocked || !std::isfinite(x) || !std::isfinite(y)) {
        return;
    }
    _endLockByHand();
    _touchX = std::clamp(x, -1.0, 1.0);
    _touchY = std::clamp(y, -1.0, 1.0);
    _touchLease.start();
    if (!_touchActive) {
        _touchActive = true;
        _touchStarted.start();
    }
    _startMotionTimer();
}

void SkydroidLink::stopTouchMotion()
{
    _touchActive = false;
    _touchBlocked = false;
    _touchX = 0.0;
    _touchY = 0.0;
    if (_motionTimer.isActive()) {
        _motionTick();  // stop now, not on the next tick
    }
}

void SkydroidLink::stopGimbal()
{
    stopLock();
    _touchActive = false;
    _touchX = 0.0;
    _touchY = 0.0;
    if (!_socket) {
        return;
    }
    _sendStop();
    _moving = false;
    _stopRepeats = 1;  // once more on the next tick: UDP can drop one
    if (!_motionTimer.isActive()) {
        _motionTimer.start();
    }
}

void SkydroidLink::_startMotionTimer()
{
    if (!_motionTimer.isActive()) {
        _motionTimer.start();
        _motionTick();  // answer the first input at once
    }
}

void SkydroidLink::_desiredSpeed(double &yawDps, double &pitchDps)
{
    yawDps = 0.0;
    pitchDps = 0.0;
    if (_touchActive) {
        if (!_touchLease.isValid() || _touchLease.elapsed() > kTouchLeaseMs) {
            _touchActive = false;  // updates stopped: the finger is gone even if no release came
        } else if (_touchStarted.elapsed() > kTouchMaxMs) {
            _touchActive = false;  // runaway cap: lift the finger to move again
            _touchBlocked = true;
        } else {
            yawDps = shapedSpeed(_touchX, _maxSpeed, kTouchDeadZone);
            pitchDps = shapedSpeed(_touchY, _maxSpeed, kTouchDeadZone);
        }
    }
    if (!_touchActive && !_wheelHoldsPosition) {
        yawDps = shapedSpeed(_wheelDeflection(_wheelYawChannel), _maxSpeed, kWheelDeadZone);
        pitchDps = shapedSpeed(_wheelDeflection(_wheelPitchChannel), _maxSpeed, kWheelDeadZone);
    }
    // The app's own lock: the speeds set from the last picture, for as long as
    // pictures keep coming. The lease runs only while that lock follows: a finger,
    // a wheel or a button ends the lock first (_endLockByHand), and with it the lease.
    if (_followLease.isValid() && _followLease.elapsed() <= kFollowLeaseMs) {
        yawDps = _followYawDps;
        pitchDps = _followPitchDps;
    }
    if (_reverseYaw) {
        yawDps = -yawDps;
    }
    if (_reversePitch) {
        pitchDps = -pitchDps;
    }
}

void SkydroidLink::_motionTick()
{
    if (!_socket) {
        _motionTimer.stop();
        return;
    }
    double yaw = 0.0;
    double pitch = 0.0;
    _desiredSpeed(yaw, pitch);
    if (speedUnits(yaw) != 0 || speedUnits(pitch) != 0) {
        _sendSpeed(yaw, pitch);
        _moving = true;
        _stopRepeats = 0;
        return;
    }
    if (_moving) {
        _sendStop();
        _moving = false;
        _stopRepeats = 1;  // once more on the next tick: UDP can drop one
        return;
    }
    if (_stopRepeats > 0) {
        _sendStop();
        --_stopRepeats;
        return;
    }
    // Idle. RC wheel updates and touches start the timer again.
    _motionTimer.stop();
}

void SkydroidLink::_sendSpeed(double yawDps, double pitchDps)
{
    // Like VGCS: one frame per moving axis (GSY yaw, GSP pitch), never the
    // combined GSM, which dropped yaw on a C12 in the field.
    const int yawUnits = speedUnits(yawDps);
    const int pitchUnits = speedUnits(pitchDps);
    // An axis that just stopped gets its zero on two ticks, so one dropped frame cannot leave it turning.
    if (yawUnits != 0) {
        _yawZeroRepeats = 0;
    } else if (_lastYawUnits != 0) {
        _yawZeroRepeats = 2;
    }
    if (pitchUnits != 0) {
        _pitchZeroRepeats = 0;
    } else if (_lastPitchUnits != 0) {
        _pitchZeroRepeats = 2;
    }
    if (yawUnits != 0 || _yawZeroRepeats > 0) {
        _send(top::buildGimbalSpeedAxis("GSY", yawUnits != 0 ? yawDps : 0.0, _options));
        if (yawUnits == 0) {
            --_yawZeroRepeats;
        }
    }
    if (pitchUnits != 0 || _pitchZeroRepeats > 0) {
        _send(top::buildGimbalSpeedAxis("GSP", pitchUnits != 0 ? pitchDps : 0.0, _options));
        if (pitchUnits == 0) {
            --_pitchZeroRepeats;
        }
    }
    _lastYawUnits = yawUnits;
    _lastPitchUnits = pitchUnits;
}

void SkydroidLink::_sendStop()
{
    // Each axis gets its own zero, then GSM zero (the stop VGCS sends), so the
    // gimbal stops whichever of these the firmware listens to.
    _send(top::buildGimbalSpeedAxis("GSY", 0.0, _options));
    _send(top::buildGimbalSpeedAxis("GSP", 0.0, _options));
    _sendGimbal(top::buildGimbalSpeed(0.0, 0.0, _options));
    _lastYawUnits = 0;
    _lastPitchUnits = 0;
    _yawZeroRepeats = 0;
    _pitchZeroRepeats = 0;
}

void SkydroidLink::center()
{
    _endLockByHand();
    // One axis per frame, like the speed commands (a combined frame dropped
    // yaw on a C12 in the field).
    _sendGimbal(top::buildGimbalAngleAxis("GAY", 0.0, kAngleMoveDps, _options));
    _sendGimbal(top::buildGimbalAngleAxis("GAP", 0.0, kAngleMoveDps, _options));
}

void SkydroidLink::centerYaw()
{
    _endLockByHand();
    _sendGimbal(top::buildGimbalAngleAxis("GAY", 0.0, kCenterYawDps, _options));
}

void SkydroidLink::pointDown()
{
    _endLockByHand();
    // Pitch -90 is straight down: the C13 tilts from -90 to +10, and its
    // gimbal angles read negative when looking down (VGCS tries -90 first too).
    _sendGimbal(top::buildGimbalAngleAxis("GAP", -90.0, kAngleMoveDps, _options));
}

void SkydroidLink::currentFov(double &horizontalDeg, double &verticalDeg) const
{
    auto narrow = [](double fovDeg, double ratio) {
        return 2.0 * std::atan(std::tan(fovDeg * kPi / 360.0) * ratio) * 180.0 / kPi;
    };
    if (_isZoomStepCamera()) {
        // C14 Pro: the lens and crop for the step it reported (VGCS c14pro_default).
        const int step = _zoomStep >= 0 ? _zoomStep : 0;
        const bool longLens = step >= 86;
        const int m = longLens ? std::clamp(step - 86, 0, 98) : std::clamp(step, 0, 85);
        horizontalDeg = narrow(longLens ? 14.7 : 61.4, (3840.0 - 36.0 * m) / 3840.0);
        verticalDeg = narrow(longLens ? 11.1 : 47.9, (2160.0 - 20.0 * m) / 2160.0);
        return;
    }
    // C12, C13: narrowed by the zoom we counted, as VGCS does (tan(fov/2) / zoom).
    const double zoomX = 1.0 + _zoomCount * kZoomPerStep;
    horizontalDeg = narrow(kC13AimFovHDeg, 1.0 / zoomX);
    verticalDeg = narrow(kC13AimFovVDeg, 1.0 / zoomX);
}

QStringList SkydroidLink::pointModes()
{
    return {QStringLiteral("laser"), QStringLiteral("picture")};
}

QStringList SkydroidLink::pointModeNames()
{
    return {tr("By laser (the camera turns to the point)"), tr("From the picture, without the laser")};
}

void SkydroidLink::setPointMode(const QString &mode)
{
    const QString m = pointModes().contains(mode) ? mode : QStringLiteral("laser");
    if (m == _pointMode) {
        return;
    }
    _pointMode = m;
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setThermalPicture(bool thermal)
{
    if (thermal != _thermalPicture) {
        _thermalPicture = thermal;
        emit thermalPictureChanged();
    }
}

void SkydroidLink::_measureFromPicture(double u, double v)
{
    // One result shows at a time: another point ends a lock, as a tap by laser does.
    if (_lockActive || _lockBusy) {
        _endLock(tr("Lock stopped, because another point was measured."));
    }
    _clearMeasurement();
    _measuredByPicture = true;
    QString whyNot;
    skydroid::geo::LaserInput view;
    if (!_socket) {
        whyNot = tr("Camera link is off. Turn it on in the camera settings.");
    } else if (!_attitudeValid) {
        // Never place a point on assumed gimbal angles.
        whyNot = tr("No position: the camera sends no gimbal angles.");
    } else if (_sampleVehiclePose(view, whyNot)) {
        double hfov = 0.0;
        double vfov = 0.0;
        currentFov(hfov, vfov);
        bool lensKnown = true;
        if (_thermalPicture) {
            if (_isZoomStepCamera()) {
                hfov = kC14ThermalFovHDeg;
                vfov = kC14ThermalFovVDeg;
            } else if (std::abs(u - 0.5) <= kCrossTapNorm && std::abs(v - 0.5) <= kCrossTapNorm) {
                u = 0.5;
                v = 0.5;
            } else {
                lensKnown = false;
                whyNot = tr("On the thermal picture the app can place the cross only: the thermal lens of this camera "
                            "is not known. Put the cross on the point, then tap the cross.");
            }
        }
        if (lensKnown) {
            view.gimbalYawDeg = _yaw;
            view.gimbalYawLeftPositive = (kNegateImageYaw != _reverseTapYaw);
            view.gimbalPitchDeg = _pitch;
            view.videoXNorm = std::clamp(u, 0.0, 1.0);
            view.videoYNorm = std::clamp(v, 0.0, 1.0);
            view.cameraHfovDeg = hfov;
            view.cameraVfovDeg = vfov;
            _setTargetFromPicture(view, _sampleHeight(), whyNot);
        }
    }
    _laserMessage = whyNot;
    emit laserChanged();
    emit targetChanged();
}

bool SkydroidLink::_setTargetFromPicture(const skydroid::geo::LaserInput &view, std::optional<double> heightM,
                                         QString &whyNot)
{
    skydroid::geo::PictureInput input;
    input.view = view;
    input.heightAboveGroundM = heightM;
    const skydroid::geo::PictureResult result = skydroid::geo::computePictureTarget(input);
    using skydroid::geo::PictureWhy;
    switch (result.why) {
    case PictureWhy::None:
        break;
    case PictureWhy::HeightUnknown:
        whyNot = tr("No position from the picture: the drone's height is not known.");
        return false;
    case PictureWhy::TooLow:
        whyNot = tr("No position from the picture: the drone is %1 m up, it needs %2 m. Take off first.")
                     .arg(heightM.value_or(0.0), 0, 'f', 1).arg(skydroid::geo::kPictureMinHeightM, 0, 'f', 1);
        return false;
    case PictureWhy::AtTheHorizon:
        whyNot = tr("No position from the picture: the point is at the horizon or above it. Tilt the camera down.");
        return false;
    case PictureWhy::TooFlat:
        whyNot = tr("No position from the picture: the point is looked at %1 degrees down, it needs %2. "
                    "Tilt the camera down, or fly nearer.")
                     .arg(result.lookDownDeg, 0, 'f', 0).arg(skydroid::geo::kPictureMinLookDownDeg, 0, 'f', 0);
        return false;
    }
    _target = result.point;
    _targetFromPicture = true;
    _targetSlantM = result.slantRangeM;
    // How much the point can be trusted: it moves this far for one degree of camera angle.
    _targetMessage = tr("From the picture, not by laser. Level ground is assumed. One degree of camera angle is %1 m here.")
                         .arg(result.metresPerDegree, 0, 'f', result.metresPerDegree < 10.0 ? 1 : 0);
    whyNot.clear();
    return true;
}

std::optional<double> SkydroidLink::_sampleHeight() const
{
    Vehicle *vehicle = MultiVehicleManager::instance()->activeVehicle();
    FactGroup *v = vehicle ? vehicle->vehicleFactGroup() : nullptr;
    const QString name = QStringLiteral("altitudeRelative");
    if (!v || !v->factExists(name)) {
        return std::nullopt;
    }
    bool ok = false;
    const double height = v->getFact(name)->rawValue().toDouble(&ok);
    return (ok && std::isfinite(height)) ? std::optional<double>(height) : std::nullopt;
}

void SkydroidLink::aimAndMeasure(double u, double v)
{
    if (_laserBusy || _aimBusy || !std::isfinite(u) || !std::isfinite(v)) {
        return;
    }
    if (_pointMode == QStringLiteral("picture")) {
        _measureFromPicture(u, v);
        return;
    }
    // The camera turns away from a locked object, so the lock ends.
    _endLockByHand();
    if (!_socket) {
        _laserValid = false;
        _laserMessage = tr("Camera link is off. Turn it on in the camera settings.");
        emit laserChanged();
        return;
    }
    // A new point: the last result no longer applies.
    _laserValid = false;
    _measuredByPicture = false;
    _target = skydroid::geo::LaserResult{};
    _targetFromPicture = false;
    _targetMessage.clear();
    emit targetChanged();
    if (!_attitudeValid) {
        _laserMessage = tr("No gimbal angles from the camera, so it cannot turn to the point.");
        emit laserChanged();
        return;
    }
    _setAimFor(u, v);
    _aimBusy = true;
    _startTurn();
    _laserMessage = tr("Turning to the point...");
    emit aimBusyChanged();
    emit laserChanged();
}

double SkydroidLink::_setAimFor(double u, double v)
{
    double hfov = 0.0;
    double vfov = 0.0;
    currentFov(hfov, vfov);
    const double dyawImage = (std::clamp(u, 0.0, 1.0) - 0.5) * hfov;    // right is +
    const double dpitchImage = (std::clamp(v, 0.0, 1.0) - 0.5) * vfov;  // down is +
    const bool negate = kNegateImageYaw != _reverseTapYaw;
    const double yawLimit = _isZoomStepCamera() ? 120.0 : 90.0;
    const double pitchTop = (_model == QStringLiteral("C13")) ? kC13PitchMaxDeg
                          : (_isZoomStepCamera() ? 60.0 : kWheelPitchMaxDeg);
    _aimPointYaw = _yaw + (negate ? -dyawImage : dyawImage);
    _aimPointPitch = _pitch - dpitchImage;
    _aimYaw = std::clamp(_aimPointYaw, -yawLimit, yawLimit);
    _aimPitch = std::clamp(_aimPointPitch, -90.0, pitchTop);
    return std::max(std::abs(dyawImage), std::abs(dpitchImage));
}

void SkydroidLink::_startTurn()
{
    _sendGimbal(top::buildGimbalAngleAxis("GAY", _aimYaw, kAngleMoveDps, _options));
    _sendGimbal(top::buildGimbalAngleAxis("GAP", _aimPitch, kAngleMoveDps, _options));
    _aimResent = false;
    _aimElapsed.start();
    _aimSettled.invalidate();
    _aimTimer.start();
}

void SkydroidLink::_aimPointInPicture(double &u, double &v) const
{
    // The aim maths backwards: the angle still to go, as a place in the picture.
    double hfov = 0.0;
    double vfov = 0.0;
    currentFov(hfov, vfov);
    const bool negate = kNegateImageYaw != _reverseTapYaw;
    const double yawToGo = _aimPointYaw - _yaw;
    const double dyawImage = negate ? -yawToGo : yawToGo;
    const double dpitchImage = _pitch - _aimPointPitch;
    u = std::clamp(0.5 + dyawImage / hfov, 0.0, 1.0);
    v = std::clamp(0.5 + dpitchImage / vfov, 0.0, 1.0);
}

void SkydroidLink::_aimTick()
{
    if (!_aimBusy && !(_lockBusy && !_lockByApp)) {
        _aimTimer.stop();
        return;
    }
    const bool close = _attitudeFresh() && std::abs(_yaw - _aimYaw) <= kAimToleranceDeg &&
                       std::abs(_pitch - _aimPitch) <= kAimToleranceDeg;
    if (close) {
        if (!_aimSettled.isValid()) {
            _aimSettled.start();
        } else if (_aimSettled.elapsed() >= kAimSettleMs) {
            _aimTimer.stop();
            if (_lockBusy && !_lockByApp) {
                // Stopped on the object, or at a gimbal limit short of it: lock
                // where the object is in the picture now.
                double u = 0.5;
                double v = 0.5;
                _aimPointInPicture(u, v);
                _armLock(u, v);
                return;
            }
            // On the point: measure it, with the pose of this moment.
            _aimBusy = false;
            emit aimBusyChanged();
            fireLaser();
            return;
        }
        return;
    }
    _aimSettled.invalidate();
    if (!_aimResent && _aimElapsed.elapsed() >= kAimResendMs) {
        _aimResent = true;
        _sendGimbal(top::buildGimbalAngleAxis("GAY", _aimYaw, kAngleMoveDps, _options));
        _sendGimbal(top::buildGimbalAngleAxis("GAP", _aimPitch, kAngleMoveDps, _options));
    }
    if (_aimElapsed.elapsed() >= kAimTimeoutMs) {
        _aimTimer.stop();
        if (_lockBusy && !_lockByApp) {
            // Never lock on whatever the camera happens to look at.
            _lockBusy = false;
            _setLockMessage(tr("The camera did not turn to the object, so it was not locked. Try again."));
            return;
        }
        // Never measure a point the camera is not looking at.
        _aimBusy = false;
        _laserValid = false;
        _laserMessage = tr("The camera did not turn to the point, so nothing was measured. "
                           "Try again, or put the cross on it and press the laser button.");
        emit aimBusyChanged();
        emit laserChanged();
    }
}

// --- Object lock ---------------------------------------------------------------

QStringList SkydroidLink::lockModes()
{
    return {QStringLiteral("app"), QStringLiteral("camera")};
}

QStringList SkydroidLink::lockModeNames()
{
    return {tr("The app follows the object"), tr("The camera's own tracker")};
}

void SkydroidLink::setLockMode(const QString &mode)
{
    const QString m = lockModes().contains(mode) ? mode : QStringLiteral("app");
    if (m == _lockMode) {
        return;
    }
    if (_lockActive || _lockBusy) {
        _endLock(tr("Lock stopped."));
    }
    _lockMode = m;
    _saveSettings();
    emit settingsChanged();
}

double SkydroidLink::followSpeed(double offDeg, double maxDps)
{
    if (!std::isfinite(offDeg) || !std::isfinite(maxDps) || std::abs(offDeg) <= kFollowDeadbandDeg) {
        return 0.0;
    }
    const double limit = std::abs(maxDps);
    return std::clamp(offDeg * kFollowGain, -limit, limit);
}

void SkydroidLink::_clearMeasurement()
{
    _laserValid = false;
    _measuredByPicture = false;
    _laserMessage.clear();
    emit laserChanged();
    _target = skydroid::geo::LaserResult{};
    _targetFromPicture = false;
    _targetMessage.clear();
    emit targetChanged();
}

void SkydroidLink::lockAt(double u, double v)
{
    lockAtBox(u, v, 0.0, 0.0);
}

void SkydroidLink::lockAtBox(double u, double v, double w, double h)
{
    if (_aimBusy || _lockBusy || !std::isfinite(u) || !std::isfinite(v) || !std::isfinite(w) || !std::isfinite(h)) {
        return;
    }
    const bool wasLocked = _lockActive;
    if (_lockActive) {
        _endLock(QString());  // a new object replaces the old one
    }
    if (!_socket) {
        _setLockMessage(tr("Camera link is off. Turn it on in the camera settings."));
        return;
    }
    if (_lockMode == QStringLiteral("camera")) {
        _lockByCamera(u, v, wasLocked);
        return;
    }
    // A new object: the last measurement no longer applies.
    _clearMeasurement();
    // The lock starts on the first picture that comes: the box is kept until then.
    _pendingU = std::clamp(u, 0.0, 1.0);
    _pendingV = std::clamp(v, 0.0, 1.0);
    _pendingW = std::clamp(w, 0.0, 0.9);
    _pendingH = std::clamp(h, 0.0, 0.9);
    _lockByApp = true;
    _lockBusy = true;
    _lockSeen = false;
    _lockBoxValid = false;
    _lockOffDeg = 0.0;
    _lockNumbers.clear();
    _lockTurnedDeg = 0.0;
    _lockJumpDeg = 0.0;
    _lockFollowSeen = false;
    _pictureAge.invalidate();
    _lockLost.invalidate();
    _followSent.clear();
    _lockElapsed.start();
    // The camera's own tracker must not pull against the app.
    _send(top::buildSumTrack(false, _options));
    _lockTimer.start();
    emit lockBoxChanged();
    _setLockMessage(tr("Locking..."));
}

void SkydroidLink::_lockByCamera(double u, double v, bool wasLocked)
{
    if (!_attitudeValid) {
        _setLockMessage(tr("No gimbal angles from the camera, so it cannot turn to the object."));
        return;
    }
    // A new object: the last measurement no longer applies.
    _clearMeasurement();

    _lockByApp = false;
    _lockBusy = true;
    if (_setAimFor(u, v) >= kLockTurnFirstDeg) {
        _startTurn();
        _setLockMessage(tr("Turning to the object..."));
        return;
    }
    // Near the centre already: lock where it was picked, without turning.
    if (!wasLocked) {
        _armLock(u, v);
        return;
    }
    _setLockMessage(tr("Locking..."));
    QTimer::singleShot(kLockAfterStopMs, this, [this, u, v]() {
        if (_lockBusy && !_lockByApp && !_aimTimer.isActive()) {
            _armLock(u, v);
        }
    });
}

void SkydroidLink::_armLock(double u, double v)
{
    // The point in the camera's own count (see kGotCentre).
    const int x = static_cast<int>(std::lround((std::clamp(u, 0.0, 1.0) - 0.5) * top::kLrfFrameW)) + kGotCentreX;
    const int y = static_cast<int>(std::lround((std::clamp(v, 0.0, 1.0) - 0.5) * top::kLrfFrameH)) + kGotCentreY;
    _sendGimbal(top::buildGotTarget(x, y, kGotFrameW, kGotFrameH, _options));
    QTimer::singleShot(kLockConfirmDelayMs, this, &SkydroidLink::_sendLockConfirm);
    _lockByApp = false;
    _lockBusy = false;
    _lockActive = true;
    _lockFollowSeen = false;
    _lockWarned = false;
    _lockSettled = false;
    _lockJumped = false;
    _lockJumpDeg = 0.0;
    _lockTurnedDeg = 0.0;
    _lockStartYaw = _yaw;
    _lockStartPitch = _pitch;
    _lockElapsed.start();
    _lockNextConfirmMs = kLockConfirmEveryMs;
    _lockNextLaserMs = kLockFirstLaserMs;
    _lockTimer.start();
    _setLockMessage(tr("Locked with the camera's own tracker. The app cannot see what the camera follows: watch the cross."));
}

void SkydroidLink::_sendLockConfirm()
{
    if (_lockActive && !_lockByApp) {
        _send(top::buildSumTrack(true, _options));
    }
}

void SkydroidLink::lockPicture(const QImage &image, double pictureX, double pictureY, double pictureW,
                               double pictureH)
{
    if (!lockWantsPictures() || image.isNull()) {
        return;
    }
    // The tracker uses the colours: a grey image stays grey, anything else becomes three bytes a point.
    const bool grey = image.format() == QImage::Format_Grayscale8;
    const QImage pixels = grey || image.format() == QImage::Format_RGB888 ? image
                                                                           : image.convertToFormat(QImage::Format_RGB888);
    if (pixels.isNull()) {
        return;
    }
    const skydroid::track::Picture picture{pixels.constBits(), pixels.width(), pixels.height(),
                                           static_cast<int>(pixels.bytesPerLine()), grey ? 1 : 3};
    // Where the video picture is inside this image.
    const double left = std::clamp(pictureX, 0.0, 1.0) * pixels.width();
    const double top = std::clamp(pictureY, 0.0, 1.0) * pixels.height();
    const double width = std::clamp(pictureW, 0.0, 1.0) * pixels.width();
    const double height = std::clamp(pictureH, 0.0, 1.0) * pixels.height();
    if (!std::isfinite(left + top + width + height) || width < kLockPictureMinSide ||
        height < kLockPictureMinSide / 2) {
        return;  // too small to find anything in
    }
    if (_pictureAge.isValid()) {
        const double seconds = std::clamp(_pictureAge.elapsed() / 1000.0, 0.02, 1.0);
        _pictureRate = 0.8 * _pictureRate + 0.2 / seconds;
    }
    _pictureAge.start();

    skydroid::track::Result result;
    if (_lockBusy) {
        skydroid::track::Box box;
        box.cx = left + _pendingU * width;
        box.cy = top + _pendingV * height;
        box.h = _pendingH > 0.0 ? _pendingH * height : kLockDefaultBox * height;
        box.w = _pendingW > 0.0 ? _pendingW * width : box.h;
        if (!_tracker.start(picture, box)) {
            _endLock(tr("The app cannot follow this: the box holds a plain area. Draw the box around the whole object."));
            return;
        }
        _lockBusy = false;
        _lockActive = true;
        _lockElapsed.start();
        _lockNextLaserMs = kLockFirstLaserMs;
        _followSent.clear();
        result.found = true;
        result.box = _tracker.box();
        result.alike = 1.0;
    } else {
        result = _tracker.update(picture);
    }
    _follow(result, left, top, width, height);
}

void SkydroidLink::_turnedSincePicture(double &yawDeg, double &pitchDeg) const
{
    yawDeg = 0.0;
    pitchDeg = 0.0;
    const qint64 now = _lockElapsed.elapsed();
    const qint64 from = now - static_cast<qint64>(kFollowPictureAgeS * 1000.0);
    // Each speed held from when it was set until the next one.
    qint64 until = now;
    for (auto it = _followSent.rbegin(); it != _followSent.rend(); ++it) {
        const qint64 since = std::max(it->atMs, from);
        if (until > since) {
            const double seconds = (until - since) / 1000.0;
            yawDeg += it->yawDps * seconds;
            pitchDeg += it->pitchDps * seconds;
        }
        until = it->atMs;
        if (it->atMs <= from) {
            break;
        }
    }
}

void SkydroidLink::_setFollowSpeed(double yawDps, double pitchDps)
{
    _followYawDps = yawDps;
    _followPitchDps = pitchDps;
    _followLease.start();
    const qint64 now = _lockElapsed.elapsed();
    _followSent.push_back({now, yawDps, pitchDps});
    while (!_followSent.empty() && _followSent.front().atMs < now - kFollowHistoryMs) {
        _followSent.pop_front();
    }
    _startMotionTimer();
}

void SkydroidLink::_follow(const skydroid::track::Result &result, double left, double top, double width,
                           double height)
{
    const bool wasSeen = _lockSeen;
    _lockSeen = result.found;
    _lockBoxValid = true;
    _lockBoxU = (result.box.cx - left) / width;
    _lockBoxV = (result.box.cy - top) / height;
    _lockBoxW = result.box.w / width;
    _lockBoxH = result.box.h / height;

    QString message;
    if (result.found) {
        _lockLost.invalidate();
        double hfov = 0.0;
        double vfov = 0.0;
        currentFov(hfov, vfov);
        // Where the object is from the cross: right and up are +, as the speed commands count.
        const double right = (_lockBoxU - 0.5) * hfov;
        const double up = (0.5 - _lockBoxV) * vfov;
        _lockOffDeg = std::max(std::abs(right), std::abs(up));
        // The picture is old: the camera has turned part of that way since.
        double turnedYaw = 0.0;
        double turnedPitch = 0.0;
        _turnedSincePicture(turnedYaw, turnedPitch);
        // Never so fast that the object leaves the tracker's reach between two pictures.
        const double maxYaw = std::clamp(0.5 * _lockBoxW * hfov * _pictureRate, kFollowMinMaxDps, kFollowMaxDps);
        const double maxPitch = std::clamp(0.5 * _lockBoxH * vfov * _pictureRate, kFollowMinMaxDps, kFollowMaxDps);
        _setFollowSpeed(followSpeed(right - turnedYaw, maxYaw), followSpeed(up - turnedPitch, maxPitch));
        message = _lockOffDeg > kLockTurningDeg ? tr("Locked. Turning the camera to the object...")
                                                : tr("Locked. The camera follows the object.");
    } else {
        // Not seen: the camera waits where it last saw the object. (Letting it turn on for a
        // second, as the object was going, was tried on the test bench: no walk more was kept.)
        _setFollowSpeed(0.0, 0.0);
        if (!_lockLost.isValid()) {
            _lockLost.start();
        }
        if (_lockLost.elapsed() >= kLockLostMs) {
            _endLock(tr("Lock lost: the app does not see the object any more. Lock it again."));
            return;
        }
        message = tr("Locked, but the object is not seen right now.");
    }
    _lockNumbers = QStringLiteral("off %1 deg, speed %2 / %3 deg/s, match %4, colours %5, %6 pictures/s")
                       .arg(_lockOffDeg, 0, 'f', 1)
                       .arg(_followYawDps, 0, 'f', 1)
                       .arg(_followPitchDps, 0, 'f', 1)
                       .arg(result.psr, 0, 'f', 0)
                       .arg(result.alike, 0, 'f', 2)
                       .arg(_pictureRate, 0, 'f', 0);
    emit lockBoxChanged();
    if (message != _lockMessage || wasSeen != _lockSeen) {
        _lockMessage = message;
        emit lockChanged();
    }
}

void SkydroidLink::_lockTick()
{
    if ((!_lockActive && !(_lockBusy && _lockByApp)) || !_socket) {
        _lockTimer.stop();
        return;
    }
    const qint64 now = _lockElapsed.elapsed();
    if (_lockByApp) {
        if (_lockBusy) {
            // Waiting for the first picture. With none, the app cannot follow.
            if (now >= kLockPictureWaitMs) {
                const double u = _pendingU;
                const double v = _pendingV;
                _lockBusy = false;
                _lockByApp = false;
                emit lockChanged();
                if (_attitudeValid) {
                    _lockByCamera(u, v, false);
                    if (_lockBusy || _lockActive) {
                        _setLockMessage(tr("No picture of the video reaches the app, so the camera's own tracker is used. ") +
                                        _lockMessage);
                    }
                } else {
                    _lockTimer.stop();
                    _setLockMessage(tr("No picture of the video reaches the app, so it cannot lock."));
                }
            }
            return;
        }
        if (_pictureAge.isValid() && _pictureAge.elapsed() >= kLockNoPictureEndMs) {
            _endLock(tr("Lock stopped: the video stopped."));
            return;
        }
        if (now >= _lockNextLaserMs && _lockSeen && _lockOffDeg <= kLockOnCrossDeg) {
            // The object is under the cross: measure it.
            _lockNextLaserMs = now + kLockLaserEveryMs;
            _measureLockedObject();
        }
        return;
    }
    if (now >= _lockNextConfirmMs) {
        // VGCS sends the confirm again every 2 s to keep the camera following.
        _send(top::buildSumTrack(true, _options));
        _lockNextConfirmMs = now + kLockConfirmEveryMs;
    }
    bool changed = false;
    if (_attitudeFresh()) {
        const double turned = std::max(std::abs(_yaw - _lockStartYaw), std::abs(_pitch - _lockStartPitch));
        if (!_lockSettled) {
            if (now >= kLockSettleMs) {
                // What the camera did before this is kept apart as the jump at the lock.
                _lockSettled = true;
                _lockJumpDeg = turned;
                _lockStartYaw = _yaw;
                _lockStartPitch = _pitch;
                if (turned > kLockFollowDeg) {
                    _lockJumped = true;
                    _lockMessage = tr("Locked with the camera's own tracker. The camera moved %1 degrees at the lock. "
                                      "Check that the cross is on the object.").arg(turned, 0, 'f', 1);
                }
                changed = true;
            }
        } else {
            if (std::abs(turned - _lockTurnedDeg) >= 0.1) {
                _lockTurnedDeg = turned;
                changed = true;
            }
            if (!_lockFollowSeen && turned > kLockFollowDeg) {
                // The camera turned by itself. The text does not say "is following":
                // the app cannot see what the camera follows.
                _lockFollowSeen = true;
                changed = true;
            }
        }
    }
    if (!_lockFollowSeen && !_lockWarned && now >= kLockFollowWarnMs) {
        _lockWarned = true;
        if (!_attitudeValid) {
            // Without angles the app cannot see the camera turn: say that, not "not following".
            _lockMessage = tr("Locked with the camera's own tracker, but the camera sends no angles, "
                              "so the app cannot tell if it turns.");
            changed = true;
        } else if (!_lockJumped) {
            _lockMessage = tr("Locked with the camera's own tracker, but the camera has not turned by itself yet. "
                              "If the object moved, the camera is not following it.");
            changed = true;
        }
        // After a jump the text already says what to check, and it stays.
    }
    if (changed) {
        emit lockChanged();
    }
    if (now >= _lockNextLaserMs) {
        _lockNextLaserMs = now + kLockLaserEveryMs;
        // While the camera follows, the object stays under the cross.
        _measureLockedObject();
    }
}

void SkydroidLink::_measureLockedObject()
{
    if (_pointMode != QStringLiteral("picture")) {
        _fireLaser(true);
        return;
    }
    // Without the laser: where the cross is on the ground, from the picture.
    // The last position stays up when this one cannot be worked out.
    skydroid::geo::LaserInput view;
    QString whyNot;
    if (!_attitudeValid || !_sampleVehiclePose(view, whyNot)) {
        return;
    }
    view.gimbalYawDeg = _yaw;
    view.gimbalYawLeftPositive = (kNegateImageYaw != _reverseTapYaw);
    view.gimbalPitchDeg = _pitch;
    if (_setTargetFromPicture(view, _sampleHeight(), whyNot)) {
        _laserValid = false;
        _measuredByPicture = true;
        _laserMessage.clear();
        emit laserChanged();
        emit targetChanged();
    }
}

void SkydroidLink::stopLock()
{
    if (_lockActive || _lockBusy) {
        _endLock(tr("Lock stopped."));
    }
}

void SkydroidLink::_endLock(const QString &message)
{
    const bool byApp = _lockByApp;
    if (_lockBusy) {
        _lockBusy = false;
        if (!byApp) {
            _aimTimer.stop();
        }
    }
    if (_lockActive) {
        _lockActive = false;
        if (!byApp) {
            // Twice, as UDP can drop one: a camera that kept following would fight the operator.
            _send(top::buildSumTrack(false, _options));
            _send(top::buildSumTrack(false, _options));
        }
    }
    _lockTimer.stop();
    if (byApp) {
        // The app's own lock: stop the gimbal where it is.
        _tracker.stop();
        _followYawDps = 0.0;
        _followPitchDps = 0.0;
        _followLease.invalidate();
        _followSent.clear();
        if (_socket && (_moving || _motionTimer.isActive())) {
            _sendStop();
            _moving = false;
            _stopRepeats = 1;  // once more on the next tick: UDP can drop one
            if (!_motionTimer.isActive()) {
                _motionTimer.start();
            }
        }
    }
    _lockByApp = false;
    _lockSeen = false;
    if (_lockBoxValid) {
        _lockBoxValid = false;
        emit lockBoxChanged();
    }
    _setLockMessage(message);
}

void SkydroidLink::_endLockByHand()
{
    if (_lockActive || _lockBusy) {
        _endLock(tr("Lock stopped, because the camera was moved."));
    }
}

void SkydroidLink::_endAppLockForZoom()
{
    // The app's tracker knows the object at one size: a zoom step changes it.
    if (_lockByApp && (_lockActive || _lockBusy)) {
        _endLock(tr("Lock stopped, because the zoom was changed. Lock the object again."));
    }
}

void SkydroidLink::_setLockMessage(const QString &message)
{
    _lockMessage = message;
    emit lockChanged();
}

// --- RC wheels -----------------------------------------------------------------

void SkydroidLink::_activeVehicleChanged(Vehicle *vehicle)
{
    if (_vehicle) {
        _vehicle->disconnect(this);
    }
    _vehicle = vehicle;
    _rc.clear();
    _rcAge.invalidate();
    for (int a = 0; a < 2; ++a) {
        _wheelStart[a].reset();
        _wheelMoved[a] = false;
        _wheelTarget[a].reset();
    }
    if (_vehicle) {
        connect(_vehicle, &Vehicle::rcChannelsRawChanged, this, &SkydroidLink::_rcChannelsReceived);
    }
    emit rcChannelsChanged();
}

QVariantList SkydroidLink::rcChannels() const
{
    QVariantList list;
    list.reserve(_rc.size());
    for (int value : _rc) {
        list.append(value);
    }
    return list;
}

std::optional<int> SkydroidLink::_rcValue(int channel) const
{
    if (channel < 1 || channel > _rc.size() || !_rcAge.isValid() || _rcAge.elapsed() > kRcFreshMs) {
        return std::nullopt;
    }
    const int value = _rc.at(channel - 1);
    if (value < kRcMin || value > kRcMax) {
        return std::nullopt;
    }
    return value;
}

double SkydroidLink::_wheelDeflection(int channel) const
{
    const std::optional<int> value = _rcValue(channel);
    if (!value) {
        return 0.0;
    }
    double d = static_cast<double>(*value - kRcCentre) / kRcHalfRange;
    if (_wheelReverse) {
        d = -d;
    }
    return std::clamp(d, -1.0, 1.0);
}

void SkydroidLink::_rcChannelsReceived(QVector<int> values)
{
    _rc = values;
    _rcAge.start();
    emit rcChannelsChanged();

    if (!_detectAxis.isEmpty()) {
        if (_detectMin.size() < values.size()) {
            _detectMin.resize(values.size(), INT_MAX);
            _detectMax.resize(values.size(), INT_MIN);
        }
        for (int i = 0; i < values.size(); ++i) {
            const int v = values.at(i);
            if (v < kRcMin || v > kRcMax) {
                continue;
            }
            _detectMin[i] = std::min(_detectMin[i], v);
            _detectMax[i] = std::max(_detectMax[i], v);
        }
    }

    if (!_enabled || !_socket) {
        return;
    }
    if (_wheelHoldsPosition) {
        _updateWheelAngles();
        return;
    }
    const bool wheelTurned = speedUnits(shapedSpeed(_wheelDeflection(_wheelYawChannel), _maxSpeed, kWheelDeadZone)) != 0 ||
                             speedUnits(shapedSpeed(_wheelDeflection(_wheelPitchChannel), _maxSpeed, kWheelDeadZone)) != 0;
    if (wheelTurned) {
        _endLockByHand();
        _startMotionTimer();
    }
}

void SkydroidLink::_updateWheelAngles()
{
    const int channels[2] = {_wheelYawChannel, _wheelPitchChannel};
    bool changed = false;
    for (int a = 0; a < 2; ++a) {
        const std::optional<int> value = _rcValue(channels[a]);
        if (!value) {
            continue;
        }
        // Never move the gimbal only because the link started: wait until the wheel is turned.
        if (!_wheelStart[a]) {
            _wheelStart[a] = *value;
            continue;
        }
        if (!_wheelMoved[a]) {
            if (std::abs(*value - *_wheelStart[a]) < kWheelMoveUs) {
                continue;
            }
            _wheelMoved[a] = true;
        }
        double t = std::clamp((*value - 1000) / 1000.0, 0.0, 1.0);
        if (_wheelReverse) {
            t = 1.0 - t;
        }
        const double pitchMax = (_model == QStringLiteral("C13")) ? kC13PitchMaxDeg : kWheelPitchMaxDeg;
        const double angle = (a == PitchAxis)
            ? kWheelPitchMinDeg + t * (pitchMax - kWheelPitchMinDeg)
            : -kWheelYawRangeDeg + t * 2.0 * kWheelYawRangeDeg;
        _wheelTarget[a] = angle;
        if (!_wheelSent[a] || std::abs(*_wheelSent[a] - angle) >= kWheelAngleStepDeg) {
            changed = true;
        }
    }
    if (!changed) {
        return;
    }
    _endLockByHand();
    if (!_wheelSentAt.isValid() || _wheelSentAt.elapsed() >= kWheelAngleIntervalMs) {
        _sendWheelAngles();
    } else if (!_wheelAngleTimer.isActive()) {
        _wheelAngleTimer.start(static_cast<int>(kWheelAngleIntervalMs - _wheelSentAt.elapsed()));
    }
}

void SkydroidLink::_sendWheelAngles()
{
    if (!_socket) {
        return;
    }
    const char *const tags[2] = {"GAY", "GAP"};
    for (int a = 0; a < 2; ++a) {
        if (!_wheelTarget[a]) {
            continue;
        }
        if (_wheelSent[a] && std::abs(*_wheelSent[a] - *_wheelTarget[a]) < kWheelAngleStepDeg) {
            continue;
        }
        _sendGimbal(top::buildGimbalAngleAxis(tags[a], *_wheelTarget[a], kWheelApproachDps, _options));
        _wheelSent[a] = _wheelTarget[a];
    }
    _wheelSentAt.start();
}

void SkydroidLink::detectWheel(const QString &axis)
{
    if (axis != QStringLiteral("pitch") && axis != QStringLiteral("yaw")) {
        return;
    }
    _detectAxis = axis;
    _detectMin = QVector<int>(_rc.size(), INT_MAX);
    _detectMax = QVector<int>(_rc.size(), INT_MIN);
    for (int i = 0; i < _rc.size(); ++i) {
        const int v = _rc.at(i);
        if (v >= kRcMin && v <= kRcMax) {
            _detectMin[i] = v;
            _detectMax[i] = v;
        }
    }
    _detectTimer.start();
    emit detectingWheelChanged();
}

void SkydroidLink::_finishWheelDetect()
{
    const QString axis = _detectAxis;
    _detectAxis.clear();
    int best = 0;
    int bestSpan = kDetectMinSpanUs - 1;
    const int count = std::min<int>(std::min(_detectMin.size(), _detectMax.size()), kMaxRcChannel);
    // Channel 5 and up: 1 to 4 are the sticks, which may be touched by accident.
    for (int i = 4; i < count; ++i) {
        if (_detectMin[i] == INT_MAX) {
            continue;
        }
        const int span = _detectMax[i] - _detectMin[i];
        if (span > bestSpan) {
            bestSpan = span;
            best = i + 1;
        }
    }
    if (best > 0) {
        if (axis == QStringLiteral("pitch")) {
            setWheelPitchChannel(best);
        } else {
            setWheelYawChannel(best);
        }
    }
    emit detectingWheelChanged();
    emit wheelDetected(axis, best);
}

// --- Camera ------------------------------------------------------------------

void SkydroidLink::takePhoto()
{
    _send(top::buildPhoto(_options));
}

void SkydroidLink::toggleRecord()
{
    _send(top::buildRecord(_options));
    _recording = !_recording;
    emit recordingChanged();
}

void SkydroidLink::zoom(int direction)
{
    if (direction == 0) {
        return;
    }
    const int d = direction > 0 ? 1 : -1;
    _endAppLockForZoom();
    if (_isZoomStepCamera()) {
        for (const std::string &frame : top::buildDzmStepFrames(d, _options)) {
            _send(frame);
        }
        QTimer::singleShot(kZoomQueryAfterCmdMs, this, [this]() { _send(top::buildDzmQuery(_options)); });
        return;
    }
    // A tap ends a "back to 1x" run that is still stepping out.
    _zoomOutLeft = 0;
    _zoomOutTimer.stop();
    _sendZoomStepFrames(d);
    QTimer::singleShot(kZoomQueryAfterCmdMs, this, [this]() { _send(top::buildDzmQuery(_options)); });
    const int count = std::clamp(_zoomCount + d, 0, _zoomStepsMax());
    if (count != _zoomCount) {
        _zoomCount = count;
        emit zoomChanged();
    }
}

void SkydroidLink::zoomHome()
{
    _endAppLockForZoom();
    if (_isZoomStepCamera()) {
        for (const std::string &frame : top::buildDzmHomeFrames(_options)) {
            _send(frame);
        }
        QTimer::singleShot(kZoomQueryAfterCmdMs, this, [this]() { _send(top::buildDzmQuery(_options)); });
        return;
    }
    // The absolute 1x VGCS sends, and the 1x preset. Some C13 firmware ignores
    // absolute zoom, so the zoom also steps back out as many steps as it went
    // in (plus a few), one every 60 ms: steps are what zoomed in the field.
    std::vector<std::string> frames = top::buildOpticalZoomFrames(1.0, _options);
    frames.push_back(top::buildDzmPreset(1, _options));
    _sendZoomFrames(frames);
    _zoomOutLeft = _zoomCount + kZoomHomeExtraSteps;
    _zoomOutTimer.start();
    if (_zoomCount != 0) {
        _zoomCount = 0;
        emit zoomChanged();
    }
}

void SkydroidLink::_zoomOutTick()
{
    if (_zoomOutLeft <= 0 || !_socket) {
        _zoomOutLeft = 0;
        _zoomOutTimer.stop();
        QTimer::singleShot(kZoomQueryAfterCmdMs, this, [this]() { _send(top::buildDzmQuery(_options)); });
        return;
    }
    _sendZoomStepFrames(-1);
    --_zoomOutLeft;
}

void SkydroidLink::_sendZoomStepFrames(int direction)
{
    // One C12/C13 step as VGCS sends it: lens zoom start (ZMC), DZM step, lens stop.
    _sendZoomFrames(top::buildC13ZoomStepFrames(direction, _options));
}

void SkydroidLink::_sendZoomFrames(const std::vector<std::string> &frames)
{
    for (const std::string &frame : frames) {
        _send(frame);
    }
    for (int extraPort : kC13ZoomExtraPorts) {
        if (extraPort == _active.port) {
            continue;
        }
        const Endpoint extra{_active.address, static_cast<quint16>(extraPort)};
        for (const std::string &frame : frames) {
            _sendTo(extra, frame);
        }
    }
}

int SkydroidLink::_zoomStepsMax() const
{
    // C13: 30x digital zoom, C12: 4x (DOCS/SKYDROID-CAMERA-SPECS.md).
    const double maxZoom = (_model == QStringLiteral("C12")) ? 4.0 : 30.0;
    return static_cast<int>(std::lround((maxZoom - 1.0) / kZoomPerStep));
}

QString SkydroidLink::zoomLabel() const
{
    if (_isZoomStepCamera()) {
        // The camera reports its step; until it does, show nothing rather than a guess.
        return _zoomStep >= 0 ? c14ProZoomLabel(_zoomStep) : QString();
    }
    return QStringLiteral("%1x").arg(1.0 + _zoomCount * kZoomPerStep, 0, 'f', 1);
}

// --- Laser ---------------------------------------------------------------------

void SkydroidLink::fireLaser()
{
    _fireLaser(false);
}

void SkydroidLink::_fireLaser(bool keepLastResult)
{
    if (_laserBusy || _aimBusy || _lockBusy) {  // a tap aim or a lock fires the laser itself when it arrives
        return;
    }
    if (!_socket) {
        _laserValid = false;
        _laserMessage = tr("Camera link is off. Turn it on in the camera settings.");
        emit laserChanged();
        return;
    }
    _laserBusy = true;
    _laserFromE.reset();
    _laserFromD.reset();

    // Pose and gimbal angles at the moment of the shot, not when the reply lands.
    _shotPose = skydroid::geo::LaserInput{};
    _shotPoseValid = _sampleVehiclePose(_shotPose, _shotPoseWhy);
    _shotGimbalValid = _attitudeValid;
    _shotPose.gimbalYawDeg = _yaw;
    // Which way that yaw counts: the same switch as the tap aiming, because it
    // is one camera. Without this the target landed on the wrong side of the
    // drone's nose whenever the camera was turned (field video, 2026-10-06).
    _shotPose.gimbalYawLeftPositive = (kNegateImageYaw != _reverseTapYaw);
    _shotPose.gimbalPitchDeg = _pitch;
    _shotHeightM = _sampleHeight();
    _measuredByPicture = false;
    if (!keepLastResult) {
        _target = skydroid::geo::LaserResult{};
        _targetFromPicture = false;
        _targetMessage.clear();
        emit targetChanged();
        _laserMessage = tr("Measuring...");
    }
    emit laserBusyChanged();
    emit laserChanged();

    // VGCS order: fire the shot at the laser module (E), read, settle, read again.
    for (const Endpoint &endpoint : _laserEndpoints()) {
        _sendTo(endpoint, top::buildSlrTrigger('E', _options));
        _sendTo(endpoint, top::buildSlrQuery('E', _options));
    }
    _laserShotTimer.start();
    _laserTimeoutTimer.start();
}

void SkydroidLink::_laserReadAfterShot()
{
    if (!_laserBusy) {
        return;
    }
    for (const Endpoint &endpoint : _laserEndpoints()) {
        _sendTo(endpoint, top::buildSlrQuery('E', _options));
        _sendTo(endpoint, top::buildSlrQuery('D', _options));
    }
}

void SkydroidLink::_laserFinish()
{
    if (!_laserBusy) {
        return;
    }
    _laserShotTimer.stop();
    _laserTimeoutTimer.stop();

    // Prefer the laser module (E); the system address (D) is the fallback.
    const std::optional<int> dm = _laserFromE ? _laserFromE : _laserFromD;
    if (dm) {
        _laserValid = true;
        _laserRangeM = *dm / 10.0;
        _laserMessage.clear();
    } else {
        _laserValid = false;
        _laserMessage = tr("No laser reading. The target may be out of range (5 to %1 m), or the laser did not answer.")
                            .arg(_options.slrMaxDm / 10);
    }
    _laserBusy = false;
    emit laserBusyChanged();
    emit laserChanged();
    _computeTarget();
}

bool SkydroidLink::_sampleVehiclePose(skydroid::geo::LaserInput &pose, QString &why) const
{
    // Why there is no lat long. Under a laser distance that was measured fine
    // (often indoors, with no GPS) the caller adds that the distance is still valid.
    Vehicle *vehicle = MultiVehicleManager::instance()->activeVehicle();
    if (!vehicle) {
        why = tr("No lat long: no drone is connected.");
        return false;
    }
    const QGeoCoordinate position = vehicle->coordinate();
    if (!position.isValid()) {
        why = tr("No lat long: the drone has no GPS lock yet.");
        return false;
    }
    FactGroup *gps = vehicle->gpsFactGroup();
    const int lock = (gps && gps->factExists(QStringLiteral("lock")))
        ? gps->getFact(QStringLiteral("lock"))->rawValue().toInt() : 0;
    if (lock < 3) {
        // QGC's lock value: 2 is a 2D fix, 3 and up are 3D.
        why = (lock == 2) ? tr("No lat long: the drone has only a 2D GPS fix (it needs 3D).")
                          : tr("No lat long: the drone has no GPS lock yet.");
        return false;
    }
    FactGroup *v = vehicle->vehicleFactGroup();
    auto value = [v](const char *name) -> double {
        const QString n = QString::fromLatin1(name);
        if (!v || !v->factExists(n)) {
            return std::nan("");
        }
        bool ok = false;
        const double d = v->getFact(n)->rawValue().toDouble(&ok);
        return ok ? d : std::nan("");
    };
    const double heading = value("heading");
    const double roll = value("roll");
    const double pitch = value("pitch");
    if (!std::isfinite(heading) || !std::isfinite(roll) || !std::isfinite(pitch)) {
        why = tr("No lat long: the drone's heading is not known yet.");
        return false;
    }
    pose.vehicleLatDeg = position.latitude();
    pose.vehicleLonDeg = position.longitude();
    pose.vehicleHeadingDeg = heading;
    pose.vehicleRollDeg = roll;
    pose.vehiclePitchDeg = pitch;
    const double altitude = value("altitudeAMSL");
    pose.vehicleAltMslM = std::isfinite(altitude) ? std::optional<double>(altitude) : std::nullopt;
    why.clear();
    return true;
}

void SkydroidLink::_computeTarget()
{
    _target = skydroid::geo::LaserResult{};
    _targetFromPicture = false;
    if (!_laserValid) {
        // No range from the laser (too far, or no answer). The picture may still say
        // where the cross is on the ground: the pose of the shot, which is at the cross.
        QString whyNot;
        if (_shotGimbalValid && _shotPoseValid && _setTargetFromPicture(_shotPose, _shotHeightM, whyNot)) {
            _laserMessage = tr("No laser reading.");
            emit laserChanged();
        } else {
            _targetMessage = tr("No laser range, so no lat long.");
        }
    } else if (!_shotGimbalValid) {
        // Never place a target on assumed gimbal angles.
        _targetMessage = tr("No lat long: the camera sends no gimbal angles. The distance is still valid.");
    } else if (!_shotPoseValid) {
        _targetMessage = _shotPoseWhy + QLatin1Char(' ') + tr("The distance is still valid.");
    } else {
        skydroid::geo::LaserInput input = _shotPose;
        input.slantRangeM = _laserRangeM;
        _target = skydroid::geo::computeLaserTarget(input);
        if (!_target.ok) {
            _targetMessage = QString::fromStdString(_target.error);
        } else if (_target.nearHorizon) {
            _targetMessage = tr("The camera looks almost level, so this position is less accurate.");
        } else {
            _targetMessage.clear();
        }
    }
    emit targetChanged();
}
