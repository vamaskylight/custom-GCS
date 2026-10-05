#pragma once

// UDP link to a Skydroid camera (TOP protocol, port 5000): gimbal, zoom,
// photo, record and the laser rangefinder. Frames come from SkydroidTop, the
// tested port of VGCS's protocol code.
//
// One shared instance; QML uses it as the singleton "SkydroidLink" from the
// QGC module.

#include <QtCore/QElapsedTimer>
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QTimer>
#include <QtNetwork/QHostAddress>
#include <QtQmlIntegration/QtQmlIntegration>

#include <optional>
#include <string>

#include "LaserGeo.h"
#include "SkydroidTop.h"

class QJSEngine;
class QQmlEngine;
class QUdpSocket;

class SkydroidLink : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON

    Q_PROPERTY(bool enabled READ enabled WRITE setEnabled NOTIFY enabledChanged)
    Q_PROPERTY(QString host READ host WRITE setHost NOTIFY settingsChanged)
    Q_PROPERTY(int port READ port WRITE setPort NOTIFY settingsChanged)
    Q_PROPERTY(QString model READ model WRITE setModel NOTIFY settingsChanged)
    Q_PROPERTY(QStringList models READ models CONSTANT)
    Q_PROPERTY(bool answering READ answering NOTIFY answeringChanged)
    Q_PROPERTY(bool attitudeValid READ attitudeValid NOTIFY attitudeChanged)
    Q_PROPERTY(double gimbalYaw READ gimbalYaw NOTIFY attitudeChanged)
    Q_PROPERTY(double gimbalPitch READ gimbalPitch NOTIFY attitudeChanged)
    Q_PROPERTY(double gimbalRoll READ gimbalRoll NOTIFY attitudeChanged)
    Q_PROPERTY(bool laserBusy READ laserBusy NOTIFY laserBusyChanged)
    Q_PROPERTY(bool laserValid READ laserValid NOTIFY laserChanged)
    Q_PROPERTY(double laserRangeM READ laserRangeM NOTIFY laserChanged)
    Q_PROPERTY(QString laserMessage READ laserMessage NOTIFY laserChanged)
    Q_PROPERTY(int zoomStep READ zoomStep NOTIFY zoomChanged)
    Q_PROPERTY(bool recording READ recording NOTIFY recordingChanged)
    Q_PROPERTY(bool targetValid READ targetValid NOTIFY targetChanged)
    Q_PROPERTY(double targetLat READ targetLat NOTIFY targetChanged)
    Q_PROPERTY(double targetLon READ targetLon NOTIFY targetChanged)
    Q_PROPERTY(bool targetHasAlt READ targetHasAlt NOTIFY targetChanged)
    Q_PROPERTY(double targetAltMsl READ targetAltMsl NOTIFY targetChanged)
    Q_PROPERTY(double targetHorizontalM READ targetHorizontalM NOTIFY targetChanged)
    Q_PROPERTY(double targetBearingDeg READ targetBearingDeg NOTIFY targetChanged)
    Q_PROPERTY(QString targetMessage READ targetMessage NOTIFY targetChanged)

public:
    explicit SkydroidLink(QObject *parent = nullptr);
    ~SkydroidLink() override;

    static SkydroidLink *instance();
    static SkydroidLink *create(QQmlEngine *qmlEngine, QJSEngine *jsEngine);

    bool enabled() const { return _enabled; }
    void setEnabled(bool enabled);
    QString host() const { return _host; }
    void setHost(const QString &host);
    int port() const { return _port; }
    void setPort(int port);
    QString model() const { return _model; }
    void setModel(const QString &model);
    static QStringList models();

    bool answering() const { return _answering; }
    bool attitudeValid() const { return _attitudeValid; }
    double gimbalYaw() const { return _yaw; }
    double gimbalPitch() const { return _pitch; }
    double gimbalRoll() const { return _roll; }
    bool laserBusy() const { return _laserBusy; }
    bool laserValid() const { return _laserValid; }
    double laserRangeM() const { return _laserRangeM; }
    QString laserMessage() const { return _laserMessage; }
    /// The zoom step the camera last reported (C14 Pro), or -1 when unknown.
    int zoomStep() const { return _zoomStep; }
    /// What we last asked for. The camera does not report recording state.
    bool recording() const { return _recording; }
    bool targetValid() const { return _target.ok; }
    double targetLat() const { return _target.targetLatDeg; }
    double targetLon() const { return _target.targetLonDeg; }
    bool targetHasAlt() const { return _target.targetAltMslM.has_value(); }
    double targetAltMsl() const { return _target.targetAltMslM.value_or(0.0); }
    double targetHorizontalM() const { return _target.horizontalRangeM; }
    double targetBearingDeg() const { return _target.bearingDeg; }
    QString targetMessage() const { return _targetMessage; }

    /// up, down, left, right, center, stop, nadir
    Q_INVOKABLE void ptz(const QString &action);
    /// Continuous move while a pad button is held, in degrees per second.
    Q_INVOKABLE void gimbalSpeed(double yawDegPerSecond, double pitchDegPerSecond);
    /// Release of a pad button: stop, sent a few times because UDP can drop one.
    Q_INVOKABLE void stopGimbal();
    Q_INVOKABLE void center();
    Q_INVOKABLE void takePhoto();
    Q_INVOKABLE void toggleRecord();
    /// +1 zoom in, -1 zoom out (one step).
    Q_INVOKABLE void zoom(int direction);
    Q_INVOKABLE void zoomHome();
    /// One laser shot at the centre of the picture.
    Q_INVOKABLE void fireLaser();

signals:
    void enabledChanged();
    void settingsChanged();
    void answeringChanged();
    void attitudeChanged();
    void laserBusyChanged();
    void laserChanged();
    void zoomChanged();
    void recordingChanged();
    void targetChanged();

private slots:
    void _readPending();
    void _poll();
    void _checkAnswering();
    void _laserReadAfterShot();
    void _laserFinish();

private:
    void _start();
    void _stop();
    void _loadSettings();
    void _saveSettings() const;
    void _applyModel();
    void _send(const std::string &frame);
    void _handleFrame(const skydroid::top::DecodedFrame &frame);
    bool _isZoomStepCamera() const { return _model == QStringLiteral("C14 Pro"); }
    /// Drone position and attitude now, from QGC. False when there is no usable GPS fix.
    bool _sampleVehiclePose(skydroid::geo::LaserInput &pose, QString &why) const;
    void _computeTarget();

    QUdpSocket *_socket = nullptr;
    QHostAddress _address;
    QTimer _pollTimer;
    QTimer _answerTimer;
    QTimer _laserShotTimer;
    QTimer _laserTimeoutTimer;
    QElapsedTimer _sinceReply;
    int _pollCount = 0;

    skydroid::top::Options _options;
    bool _enabled = false;
    QString _host;
    int _port = 5000;
    QString _model;

    bool _answering = false;
    bool _attitudeValid = false;
    double _yaw = 0.0;
    double _pitch = 0.0;
    double _roll = 0.0;

    bool _laserBusy = false;
    bool _laserValid = false;
    double _laserRangeM = 0.0;
    QString _laserMessage;
    std::optional<int> _laserFromE;  // decimeters from the laser module
    std::optional<int> _laserFromD;  // decimeters from the system address

    // Captured when the laser fires, so the target uses the pose of the shot.
    skydroid::geo::LaserInput _shotPose;
    bool _shotPoseValid = false;
    QString _shotPoseWhy;
    bool _shotGimbalValid = false;
    skydroid::geo::LaserResult _target;
    QString _targetMessage;

    int _zoomStep = -1;
    bool _recording = false;
};
