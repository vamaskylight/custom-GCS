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

#include <cmath>
#include <vector>

namespace top = skydroid::top;

namespace {

constexpr const char *kSettingsGroup = "VamaSkydroid";
constexpr const char *kDefaultHost = "192.168.144.108";
constexpr int kDefaultPort = 5000;
constexpr const char *kDefaultModel = "C13";

constexpr int kPollIntervalMs = 200;     // gimbal angle query, 5 per second
constexpr int kZoomPollEvery = 10;       // C14 Pro zoom step query every 2 s
constexpr int kAnswerCheckMs = 1000;
constexpr int kAnswerTimeoutMs = 3000;   // no reply this long = camera not answering
constexpr int kLaserShotSettleMs = 120;  // VGCS _LRF_SLR_SHOT_SETTLE_S
constexpr int kLaserWaitMs = 1000;
constexpr int kZoomQueryAfterCmdMs = 350; // VGCS _DZM_POLL_AFTER_CMD_S
constexpr double kC14ProLaserMaxM = 1500.0;

// Some C13 firmware takes zoom on these ports as well (VGCS _ZOOM_EXTRA_PORTS).
const int kC13ZoomExtraPorts[] = {9003, 19853};

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

    connect(&_pollTimer, &QTimer::timeout, this, &SkydroidLink::_poll);
    connect(&_answerTimer, &QTimer::timeout, this, &SkydroidLink::_checkAnswering);
    connect(&_laserShotTimer, &QTimer::timeout, this, &SkydroidLink::_laserReadAfterShot);
    connect(&_laserTimeoutTimer, &QTimer::timeout, this, &SkydroidLink::_laserFinish);

    _loadSettings();
    _applyModel();
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

// --- Settings --------------------------------------------------------------

void SkydroidLink::_loadSettings()
{
    QSettings settings;
    settings.beginGroup(kSettingsGroup);
    _enabled = settings.value(QStringLiteral("enabled"), false).toBool();
    _host = settings.value(QStringLiteral("host"), QString::fromLatin1(kDefaultHost)).toString().trimmed();
    _port = settings.value(QStringLiteral("port"), kDefaultPort).toInt();
    _model = settings.value(QStringLiteral("model"), QString::fromLatin1(kDefaultModel)).toString();
    settings.endGroup();
    if (!models().contains(_model)) {
        _model = QString::fromLatin1(kDefaultModel);
    }
    if (_port <= 0 || _port > 65535) {
        _port = kDefaultPort;
    }
}

void SkydroidLink::_saveSettings() const
{
    QSettings settings;
    settings.beginGroup(kSettingsGroup);
    settings.setValue(QStringLiteral("enabled"), _enabled);
    settings.setValue(QStringLiteral("host"), _host);
    settings.setValue(QStringLiteral("port"), _port);
    settings.setValue(QStringLiteral("model"), _model);
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
    _address = QHostAddress(_host);
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setPort(int port)
{
    if (port <= 0 || port > 65535 || port == _port) {
        return;
    }
    _port = port;
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

void SkydroidLink::_applyModel()
{
    _options = top::Options{};
    if (_model == QStringLiteral("C14 Pro")) {
        // Same choices as the VGCS c14pro_default profile.
        _options.gClassUpperHeader = true;
        _options.slrMaxDm = top::slrMaxDmForRange(kC14ProLaserMaxM);
    }
    if (_zoomStep != -1) {
        _zoomStep = -1;
        emit zoomChanged();
    }
}

// --- Socket ----------------------------------------------------------------

void SkydroidLink::_start()
{
    _stop();
    _address = QHostAddress(_host);
    _socket = new QUdpSocket(this);
    if (!_socket->bind(QHostAddress::AnyIPv4, 0)) {
        _socket->deleteLater();
        _socket = nullptr;
        return;
    }
    connect(_socket, &QUdpSocket::readyRead, this, &SkydroidLink::_readPending);
    _sinceReply.invalidate();
    _pollCount = 0;
    _pollTimer.start();
    _answerTimer.start();
    _poll();
}

void SkydroidLink::_stop()
{
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

void SkydroidLink::_send(const std::string &frame)
{
    if (!_socket || frame.empty() || _address.isNull()) {
        return;
    }
    _socket->writeDatagram(frame.data(), static_cast<qint64>(frame.size()), _address, static_cast<quint16>(_port));
}

void SkydroidLink::_readPending()
{
    while (_socket && _socket->hasPendingDatagrams()) {
        const QNetworkDatagram datagram = _socket->receiveDatagram();
        const QByteArray data = datagram.data();
        const auto frame = top::parseTpFrame(std::string(data.constData(), static_cast<size_t>(data.size())), _options);
        if (frame) {
            _handleFrame(*frame);
        }
    }
}

void SkydroidLink::_handleFrame(const top::DecodedFrame &frame)
{
    _sinceReply.start();
    if (!_answering) {
        _answering = true;
        emit answeringChanged();
    }

    if (frame.tag == "GAC" && frame.yaw && frame.pitch) {
        _yaw = *frame.yaw;
        _pitch = *frame.pitch;
        _roll = frame.roll.value_or(0.0);
        _attitudeValid = true;
        emit attitudeChanged();
    } else if (frame.tag == "SLR" && _laserBusy) {
        const char source = frame.address.empty() ? ' ' : frame.address[0];
        if (source == 'E') {
            _laserFromE = frame.slrDm;
            if (_laserFromE) {
                _laserFinish();  // the laser module answered: no need to wait
            }
        } else if (source == 'D' && frame.slrDm) {
            _laserFromD = frame.slrDm;
        }
    } else if (frame.tag == "DZM" && frame.dzmStep) {
        if (*frame.dzmStep != _zoomStep) {
            _zoomStep = *frame.dzmStep;
            emit zoomChanged();
        }
    }
}

void SkydroidLink::_poll()
{
    _send(top::buildGacQuery(_options));
    if (_isZoomStepCamera() && (_pollCount % kZoomPollEvery) == 0) {
        _send(top::buildDzmQuery(_options));
    }
    ++_pollCount;
}

void SkydroidLink::_checkAnswering()
{
    const bool fresh = _sinceReply.isValid() && _sinceReply.elapsed() < kAnswerTimeoutMs;
    if (!fresh && _answering) {
        _answering = false;
        emit answeringChanged();
    }
    if (!fresh && _attitudeValid) {
        // Old angles must never be used to place a target.
        _attitudeValid = false;
        emit attitudeChanged();
    }
}

// --- Gimbal ------------------------------------------------------------------

void SkydroidLink::ptz(const QString &action)
{
    const QString a = action.trimmed().toLower();
    if (a == QStringLiteral("stop")) {
        stopGimbal();
        return;
    }
    _send(top::buildPtz(a.toStdString(), _options));
}

void SkydroidLink::gimbalSpeed(double yawDegPerSecond, double pitchDegPerSecond)
{
    // Same choice as VGCS _speed_commands_for: a single moving axis goes out
    // as GSY or GSP, so a zero on the other axis cannot mask it.
    const bool yawMoves = std::abs(yawDegPerSecond) >= 1e-6;
    const bool pitchMoves = std::abs(pitchDegPerSecond) >= 1e-6;
    if (yawMoves && !pitchMoves) {
        _send(top::buildGimbalSpeedAxis("GSY", yawDegPerSecond, _options));
    } else if (pitchMoves && !yawMoves) {
        _send(top::buildGimbalSpeedAxis("GSP", pitchDegPerSecond, _options));
    } else {
        _send(top::buildGimbalSpeed(yawDegPerSecond, pitchDegPerSecond, _options));
    }
}

void SkydroidLink::stopGimbal()
{
    // VGCS ptz_stop_burst: C13 can coast after a single stop.
    const std::string stop = top::buildPtz("stop", _options);
    for (int i = 0; i < 3; ++i) {
        _send(stop);
    }
}

void SkydroidLink::center()
{
    _send(top::buildPtz("center", _options));
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
    if (_isZoomStepCamera()) {
        for (const std::string &frame : top::buildDzmStepFrames(direction, _options)) {
            _send(frame);
        }
        QTimer::singleShot(kZoomQueryAfterCmdMs, this, [this]() { _send(top::buildDzmQuery(_options)); });
        return;
    }
    const std::vector<std::string> frames = top::buildC13ZoomStepFrames(direction, _options);
    for (const std::string &frame : frames) {
        _send(frame);
    }
    if (_socket && !_address.isNull()) {
        for (int extraPort : kC13ZoomExtraPorts) {
            if (extraPort == _port) {
                continue;
            }
            for (const std::string &frame : frames) {
                _socket->writeDatagram(frame.data(), static_cast<qint64>(frame.size()), _address,
                                       static_cast<quint16>(extraPort));
            }
        }
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
    for (const std::string &frame : top::buildOpticalZoomFrames(1.0, _options)) {
        _send(frame);
    }
}

// --- Laser ---------------------------------------------------------------------

void SkydroidLink::fireLaser()
{
    if (_laserBusy) {
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
    _target = skydroid::geo::LaserResult{};
    _targetMessage.clear();
    emit targetChanged();
    _laserMessage = tr("Measuring...");
    emit laserBusyChanged();
    emit laserChanged();

    // VGCS order: fire the shot at the laser module (E), read, settle, read again.
    _send(top::buildSlrTrigger('E', _options));
    _send(top::buildSlrQuery('E', _options));
    _laserShotTimer.start();
    _laserTimeoutTimer.start();
}

void SkydroidLink::_laserReadAfterShot()
{
    if (!_laserBusy) {
        return;
    }
    _send(top::buildSlrQuery('E', _options));
    _send(top::buildSlrQuery('D', _options));
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
    Vehicle *vehicle = MultiVehicleManager::instance()->activeVehicle();
    if (!vehicle) {
        why = tr("No drone connected, so no target position.");
        return false;
    }
    const QGeoCoordinate position = vehicle->coordinate();
    if (!position.isValid()) {
        why = tr("The drone has no position yet, so no target position.");
        return false;
    }
    FactGroup *gps = vehicle->gpsFactGroup();
    const int lock = (gps && gps->factExists(QStringLiteral("lock")))
        ? gps->getFact(QStringLiteral("lock"))->rawValue().toInt() : 0;
    if (lock < 3) {
        why = tr("The drone has no 3D GPS fix, so no target position.");
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
        why = tr("The drone attitude is not known yet, so no target position.");
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
        _targetMessage = tr("No laser range, so no target position.");
    } else if (!_shotGimbalValid) {
        // Never place a target on assumed gimbal angles.
        _targetMessage = tr("No gimbal angles from the camera, so no target position.");
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
