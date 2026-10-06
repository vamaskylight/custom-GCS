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

// Object lock, as VGCS M13 does it (vgcs/skydroid/adapter.py and
// vgcs/map/observation/track_mixin.py): turn to the object when it is 1.5
// degrees or more from the centre, GOT where the object is in the picture
// (the 1280 x 720 frame of TOP 3.3.5), SUM confirm 80 ms later, SUM confirm
// again every 2 s, and SUM stop at the end. The camera follows once it has
// turned more than 0.8 degrees by itself; VGCS warns after 6 s without that.
constexpr double kLockTurnFirstDeg = 1.5;
constexpr int kLockTickMs = 200;
constexpr int kLockConfirmDelayMs = 80;
constexpr int kLockAfterStopMs = 60;     // VGCS waits 50 ms between an old lock's stop and a new GOT
constexpr int kLockConfirmEveryMs = 2000;
constexpr int kLockFirstLaserMs = 400;
constexpr int kLockLaserEveryMs = 3000;  // VGCS _M13_SLR_FRESH_INTERVAL_S
constexpr double kLockFollowDeg = 0.8;
constexpr int kLockFollowWarnMs = 6000;

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
    settings.endGroup();
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

void SkydroidLink::aimAndMeasure(double u, double v)
{
    if (_laserBusy || _aimBusy || !std::isfinite(u) || !std::isfinite(v)) {
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
    _target = skydroid::geo::LaserResult{};
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
    if (!_aimBusy && !_lockBusy) {
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
            if (_lockBusy) {
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
        if (_lockBusy) {
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

void SkydroidLink::lockAt(double u, double v)
{
    if (_aimBusy || _lockBusy || !std::isfinite(u) || !std::isfinite(v)) {
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
    if (!_attitudeValid) {
        _setLockMessage(tr("No gimbal angles from the camera, so it cannot turn to the object."));
        return;
    }
    // A new object: the last measurement no longer applies.
    _laserValid = false;
    _laserMessage.clear();
    emit laserChanged();
    _target = skydroid::geo::LaserResult{};
    _targetMessage.clear();
    emit targetChanged();

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
        if (_lockBusy && !_aimTimer.isActive()) {
            _armLock(u, v);
        }
    });
}

void SkydroidLink::_armLock(double u, double v)
{
    const int x = static_cast<int>(std::lround(std::clamp(u, 0.0, 1.0) * top::kLrfFrameW));
    const int y = static_cast<int>(std::lround(std::clamp(v, 0.0, 1.0) * top::kLrfFrameH));
    _sendGimbal(top::buildGotTarget(x, y, top::kLrfFrameW, top::kLrfFrameH, _options));
    QTimer::singleShot(kLockConfirmDelayMs, this, &SkydroidLink::_sendLockConfirm);
    _lockBusy = false;
    _lockActive = true;
    _lockFollowSeen = false;
    _lockWarned = false;
    _lockTurnedDeg = 0.0;
    _lockStartYaw = _yaw;
    _lockStartPitch = _pitch;
    _lockElapsed.start();
    _lockNextConfirmMs = kLockConfirmEveryMs;
    _lockNextLaserMs = kLockFirstLaserMs;
    _lockTimer.start();
    _setLockMessage(tr("Locked. The camera should now follow the object."));
}

void SkydroidLink::_sendLockConfirm()
{
    if (_lockActive) {
        _send(top::buildSumTrack(true, _options));
    }
}

void SkydroidLink::_lockTick()
{
    if (!_lockActive || !_socket) {
        _lockTimer.stop();
        return;
    }
    const qint64 now = _lockElapsed.elapsed();
    if (now >= _lockNextConfirmMs) {
        // VGCS sends the confirm again every 2 s to keep the camera following.
        _send(top::buildSumTrack(true, _options));
        _lockNextConfirmMs = now + kLockConfirmEveryMs;
    }
    bool changed = false;
    if (_attitudeFresh()) {
        const double turned = std::max(std::abs(_yaw - _lockStartYaw), std::abs(_pitch - _lockStartPitch));
        if (std::abs(turned - _lockTurnedDeg) >= 0.1) {
            _lockTurnedDeg = turned;
            changed = true;
        }
        if (!_lockFollowSeen && turned > kLockFollowDeg) {
            _lockFollowSeen = true;
            _lockMessage = tr("Locked. The camera is following the object.");
            changed = true;
        }
    }
    if (!_lockFollowSeen && !_lockWarned && now >= kLockFollowWarnMs) {
        _lockWarned = true;
        // Without angles the app cannot see the camera turn: say that, not "not following".
        _lockMessage = _attitudeValid
            ? tr("Locked, but the camera has not turned by itself yet. "
                 "If the object moved, the camera is not following it.")
            : tr("Locked, but the camera sends no angles, so the app cannot tell if it follows.");
        changed = true;
    }
    if (changed) {
        emit lockChanged();
    }
    if (now >= _lockNextLaserMs) {
        _lockNextLaserMs = now + kLockLaserEveryMs;
        // While the camera follows, the object stays under the cross.
        _fireLaser(true);
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
    if (_lockBusy) {
        _lockBusy = false;
        _aimTimer.stop();
    }
    if (_lockActive) {
        _lockActive = false;
        _lockTimer.stop();
        // Twice, as UDP can drop one: a camera that kept following would fight the operator.
        _send(top::buildSumTrack(false, _options));
        _send(top::buildSumTrack(false, _options));
    }
    _setLockMessage(message);
}

void SkydroidLink::_endLockByHand()
{
    if (_lockActive || _lockBusy) {
        _endLock(tr("Lock stopped, because the camera was moved."));
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
    _shotPose.gimbalPitchDeg = _pitch;
    if (!keepLastResult) {
        _target = skydroid::geo::LaserResult{};
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
    // The operator sees these under a distance that was measured fine (often
    // indoors, with no GPS), so each one says that only the lat long is missing.
    Vehicle *vehicle = MultiVehicleManager::instance()->activeVehicle();
    if (!vehicle) {
        why = tr("No lat long: no drone is connected. The distance is still valid.");
        return false;
    }
    const QGeoCoordinate position = vehicle->coordinate();
    if (!position.isValid()) {
        why = tr("No lat long: the drone has no GPS lock yet. The distance is still valid.");
        return false;
    }
    FactGroup *gps = vehicle->gpsFactGroup();
    const int lock = (gps && gps->factExists(QStringLiteral("lock")))
        ? gps->getFact(QStringLiteral("lock"))->rawValue().toInt() : 0;
    if (lock < 3) {
        // QGC's lock value: 2 is a 2D fix, 3 and up are 3D.
        why = (lock == 2) ? tr("No lat long: the drone has only a 2D GPS fix (it needs 3D). The distance is still valid.")
                          : tr("No lat long: the drone has no GPS lock yet. The distance is still valid.");
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
        why = tr("No lat long: the drone's heading is not known yet. The distance is still valid.");
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
    if (!_laserValid) {
        _targetMessage = tr("No laser range, so no lat long.");
    } else if (!_shotGimbalValid) {
        // Never place a target on assumed gimbal angles.
        _targetMessage = tr("No lat long: the camera sends no gimbal angles. The distance is still valid.");
    } else if (!_shotPoseValid) {
        _targetMessage = _shotPoseWhy;
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
