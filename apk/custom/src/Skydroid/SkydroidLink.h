#pragma once

// UDP link to a Skydroid camera (TOP protocol): gimbal, zoom, photo, record
// and the laser rangefinder. Frames come from SkydroidTop, the tested port
// of VGCS's protocol code.
//
// What the first field test (2026-10-06) and VGCS taught:
//  - Turn the angle push on (GAA) before asking for angles (GAC). VGCS always
//    does. The first test build only asked, and got the laser but no angles.
//  - The control address is not always the camera's own (192.168.144.108:5000).
//    Some systems relay it (192.168.144.12:19856), so other known addresses are
//    tried while the camera is quiet. The laser often answers only on the
//    camera's own address, so laser frames also go to the configured address.
//  - Move the gimbal with speed commands (GSY yaw, GSP pitch), sent again 10
//    times a second while motion is asked for, then stop. This is what VGCS
//    sends. PTZ arrows run at the camera's own fixed speed (much too fast) and
//    did not move yaw at all in the field test.
//  - The long gimbal frames (angles GAY/GAP, GSM, GOT) must start with an
//    upper-case "#TP", as Skydroid's TOP documents say. VGCS sends them to the
//    C12/C13 with a lower-case "#tp", and the third test build found the V13
//    ignoring the angle commands sent that way (0 and 90 degree buttons).
//
// Motion comes from a finger dragging on the video (setTouchMotion) or from
// RC wheels, read from the RC channels the flight controller reports.
//
// Object lock (lockAt) uses the camera's own tracker, as VGCS M13 does. It
// never worked on a C13 in VGCS, and TOP V1.1.6 lists GOT and SUM for the C12
// only; V1.2.0 adds the C13 and the C14 Pro. The camera reports nothing about
// the lock, so the gimbal angles are the only sign that it follows.
//
// One shared instance; QML uses it as the singleton "SkydroidLink" from the
// QGC module.

#include <QtCore/QElapsedTimer>
#include <QtCore/QList>
#include <QtCore/QObject>
#include <QtCore/QPointer>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QTimer>
#include <QtCore/QVariantList>
#include <QtCore/QVector>
#include <QtNetwork/QHostAddress>
#include <QtQmlIntegration/QtQmlIntegration>

#include <optional>
#include <string>
#include <vector>

#include "LaserGeo.h"
#include "SkydroidTop.h"

class QJSEngine;
class QQmlEngine;
class QUdpSocket;
class Vehicle;

class SkydroidLink : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON

    // Settings, saved between runs
    Q_PROPERTY(bool enabled READ enabled WRITE setEnabled NOTIFY enabledChanged)
    Q_PROPERTY(QString host READ host WRITE setHost NOTIFY settingsChanged)
    Q_PROPERTY(int port READ port WRITE setPort NOTIFY settingsChanged)
    Q_PROPERTY(QString model READ model WRITE setModel NOTIFY settingsChanged)
    Q_PROPERTY(QStringList models READ models CONSTANT)
    Q_PROPERTY(QStringList modelNames READ modelNames CONSTANT)
    Q_PROPERTY(QString modelName READ modelName NOTIFY settingsChanged)
    Q_PROPERTY(int maxSpeed READ maxSpeed WRITE setMaxSpeed NOTIFY settingsChanged)
    Q_PROPERTY(bool reverseYaw READ reverseYaw WRITE setReverseYaw NOTIFY settingsChanged)
    Q_PROPERTY(bool reversePitch READ reversePitch WRITE setReversePitch NOTIFY settingsChanged)
    Q_PROPERTY(int wheelPitchChannel READ wheelPitchChannel WRITE setWheelPitchChannel NOTIFY settingsChanged)
    Q_PROPERTY(int wheelYawChannel READ wheelYawChannel WRITE setWheelYawChannel NOTIFY settingsChanged)
    Q_PROPERTY(bool wheelHoldsPosition READ wheelHoldsPosition WRITE setWheelHoldsPosition NOTIFY settingsChanged)
    Q_PROPERTY(bool wheelReverse READ wheelReverse WRITE setWheelReverse NOTIFY settingsChanged)
    Q_PROPERTY(bool reverseTapYaw READ reverseTapYaw WRITE setReverseTapYaw NOTIFY settingsChanged)
    Q_PROPERTY(QString dayVideoUrl READ dayVideoUrl WRITE setDayVideoUrl NOTIFY settingsChanged)
    Q_PROPERTY(QString thermalVideoUrl READ thermalVideoUrl WRITE setThermalVideoUrl NOTIFY settingsChanged)

    // Camera state
    Q_PROPERTY(bool answering READ answering NOTIFY answeringChanged)
    Q_PROPERTY(QString activeEndpoint READ activeEndpoint NOTIFY activeEndpointChanged)
    Q_PROPERTY(bool attitudeValid READ attitudeValid NOTIFY attitudeChanged)
    Q_PROPERTY(double gimbalYaw READ gimbalYaw NOTIFY attitudeChanged)
    Q_PROPERTY(double gimbalPitch READ gimbalPitch NOTIFY attitudeChanged)
    Q_PROPERTY(double gimbalRoll READ gimbalRoll NOTIFY attitudeChanged)
    Q_PROPERTY(bool laserBusy READ laserBusy NOTIFY laserBusyChanged)
    /// True while the camera turns to a tapped point, before the laser fires.
    Q_PROPERTY(bool aimBusy READ aimBusy NOTIFY aimBusyChanged)
    Q_PROPERTY(bool laserValid READ laserValid NOTIFY laserChanged)
    Q_PROPERTY(double laserRangeM READ laserRangeM NOTIFY laserChanged)
    Q_PROPERTY(QString laserMessage READ laserMessage NOTIFY laserChanged)
    Q_PROPERTY(int zoomStep READ zoomStep NOTIFY zoomChanged)
    Q_PROPERTY(QString zoomLabel READ zoomLabel NOTIFY zoomChanged)
    Q_PROPERTY(QString zoomReport READ zoomReport NOTIFY zoomChanged)
    Q_PROPERTY(bool recording READ recording NOTIFY recordingChanged)
    Q_PROPERTY(bool targetValid READ targetValid NOTIFY targetChanged)
    Q_PROPERTY(double targetLat READ targetLat NOTIFY targetChanged)
    Q_PROPERTY(double targetLon READ targetLon NOTIFY targetChanged)
    Q_PROPERTY(bool targetHasAlt READ targetHasAlt NOTIFY targetChanged)
    Q_PROPERTY(double targetAltMsl READ targetAltMsl NOTIFY targetChanged)
    Q_PROPERTY(double targetHorizontalM READ targetHorizontalM NOTIFY targetChanged)
    Q_PROPERTY(double targetBearingDeg READ targetBearingDeg NOTIFY targetChanged)
    Q_PROPERTY(QString targetMessage READ targetMessage NOTIFY targetChanged)

    // Object lock with the camera's own tracker (GOT and SUM). The camera never
    // reports what it tracks, so the gimbal angles show whether it follows.
    /// True while the camera turns to the object, before the lock starts.
    Q_PROPERTY(bool lockBusy READ lockBusy NOTIFY lockChanged)
    /// True while the camera is told to follow the object.
    Q_PROPERTY(bool lockActive READ lockActive NOTIFY lockChanged)
    /// How far the camera has turned by itself since the lock settled, in degrees.
    Q_PROPERTY(double lockTurnedDeg READ lockTurnedDeg NOTIFY lockChanged)
    /// How far the camera moved in the first 2 s after the lock command, in degrees.
    /// That first move is its tracker taking over. It does not count as following.
    Q_PROPERTY(double lockJumpDeg READ lockJumpDeg NOTIFY lockChanged)
    /// True once the camera has turned by itself after the lock settled (it follows).
    Q_PROPERTY(bool lockFollowSeen READ lockFollowSeen NOTIFY lockChanged)
    Q_PROPERTY(QString lockMessage READ lockMessage NOTIFY lockChanged)

    // RC channels from the flight controller (index 0 is channel 1)
    Q_PROPERTY(QVariantList rcChannels READ rcChannels NOTIFY rcChannelsChanged)
    /// "pitch" or "yaw" while the wheel detection runs, else empty.
    Q_PROPERTY(QString detectingWheel READ detectingWheel NOTIFY detectingWheelChanged)

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
    /// Skydroid's name (C12, C13, C14 Pro): what the code and saved settings use.
    QString model() const { return _model; }
    void setModel(const QString &model);
    static QStringList models();
    /// The names VAMA shows for the same cameras (V12, V13, V14 Pro), in the order of models().
    static QStringList modelNames();
    QString modelName() const;
    /// Gimbal speed in degrees per second at full deflection.
    int maxSpeed() const { return _maxSpeed; }
    void setMaxSpeed(int degPerSecond);
    bool reverseYaw() const { return _reverseYaw; }
    void setReverseYaw(bool reverse);
    bool reversePitch() const { return _reversePitch; }
    void setReversePitch(bool reverse);
    /// RC channel (1 to 18) of the wheel that tilts the camera, 0 = off.
    int wheelPitchChannel() const { return _wheelPitchChannel; }
    void setWheelPitchChannel(int channel);
    /// RC channel (1 to 18) of the wheel that turns the camera left and right, 0 = off.
    int wheelYawChannel() const { return _wheelYawChannel; }
    void setWheelYawChannel(int channel);
    /// false: the wheel springs back to the middle, and how far it is turned sets the speed.
    /// true: the wheel stays where it is left, and its position sets the camera angle.
    bool wheelHoldsPosition() const { return _wheelHoldsPosition; }
    void setWheelHoldsPosition(bool holds);
    bool wheelReverse() const { return _wheelReverse; }
    void setWheelReverse(bool reverse);
    /// RTSP addresses of the day and thermal pictures. The IR button switches
    /// QGC's video between them; an empty value means the camera's default.
    /// Tap aiming turns the camera the other way left and right (for a camera
    /// whose yaw angle runs the other way than VGCS found on the C13).
    bool reverseTapYaw() const { return _reverseTapYaw; }
    void setReverseTapYaw(bool reverse);
    QString dayVideoUrl() const { return _dayVideoUrl; }
    void setDayVideoUrl(const QString &url);
    QString thermalVideoUrl() const { return _thermalVideoUrl; }
    void setThermalVideoUrl(const QString &url);

    bool answering() const { return _answering; }
    /// The address the camera answers on, "host:port".
    QString activeEndpoint() const;
    bool attitudeValid() const { return _attitudeValid; }
    double gimbalYaw() const { return _yaw; }
    double gimbalPitch() const { return _pitch; }
    double gimbalRoll() const { return _roll; }
    bool laserBusy() const { return _laserBusy; }
    bool aimBusy() const { return _aimBusy; }
    bool laserValid() const { return _laserValid; }
    double laserRangeM() const { return _laserRangeM; }
    QString laserMessage() const { return _laserMessage; }
    /// The zoom step the camera last reported (C14 Pro), or -1 when unknown.
    int zoomStep() const { return _zoomStep; }
    /// The zoom to show: "2.9x"-style from the C14 Pro's report, or for C12/C13
    /// counted from our steps like VGCS (1x plus 0.1x per step).
    QString zoomLabel() const;
    /// The data of the camera's last zoom read reply, as sent (for checking a C13).
    QString zoomReport() const { return _zoomReport; }
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
    bool lockBusy() const { return _lockBusy; }
    bool lockActive() const { return _lockActive; }
    double lockTurnedDeg() const { return _lockTurnedDeg; }
    double lockJumpDeg() const { return _lockJumpDeg; }
    bool lockFollowSeen() const { return _lockFollowSeen; }
    QString lockMessage() const { return _lockMessage; }
    QVariantList rcChannels() const;
    QString detectingWheel() const { return _detectAxis; }

    /// Raw PTZ code: up, down, left, right, center, stop (sent 3 times), nadir.
    Q_INVOKABLE void ptz(const QString &action);
    /// A finger dragging on the video. x is right, y is up, each -1 to 1.
    /// Call it about 10 times a second while the finger is down: motion stops
    /// on its own if the calls stop (a lost "release" must not leave the gimbal turning).
    Q_INVOKABLE void setTouchMotion(double x, double y);
    Q_INVOKABLE void stopTouchMotion();
    /// Stop every gimbal motion now.
    Q_INVOKABLE void stopGimbal();
    /// Yaw and pitch back to the middle (angle commands GAY 0 and GAP 0).
    Q_INVOKABLE void center();
    /// Yaw only back to the middle (GAY 0).
    Q_INVOKABLE void centerYaw();
    /// Straight down (GAP -90), yaw kept.
    Q_INVOKABLE void pointDown();
    /// Turn the camera to a point tapped on the video, then measure it with
    /// the laser. u and v run from 0 to 1 across and down the picture.
    Q_INVOKABLE void aimAndMeasure(double u, double v);
    /// Lock on the object at a point of the video (u, v as for aimAndMeasure):
    /// turn the camera to it, then tell the camera to follow it (GOT, then SUM
    /// confirm), and measure it with the laser every few seconds. Moving the
    /// camera by hand ends the lock.
    Q_INVOKABLE void lockAt(double u, double v);
    Q_INVOKABLE void stopLock();
    /// The picture's field of view now, in degrees, zoom included.
    void currentFov(double &horizontalDeg, double &verticalDeg) const;
    Q_INVOKABLE void takePhoto();
    Q_INVOKABLE void toggleRecord();
    /// +1 zoom in, -1 zoom out (one step).
    Q_INVOKABLE void zoom(int direction);
    Q_INVOKABLE void zoomHome();
    /// One laser shot at the centre of the picture.
    Q_INVOKABLE void fireLaser();
    /// Watch the RC channels for a few seconds and use the wheel that moves,
    /// for "pitch" or "yaw". Channels 1 to 4 (the sticks) are never picked.
    Q_INVOKABLE void detectWheel(const QString &axis);

    /// Degrees per second for a deflection of -1 to 1: nothing inside the dead
    /// zone, then a curve that is gentle near the middle for fine aiming.
    static double shapedSpeed(double deflection, double maxSpeed, double deadZone);
    /// Hosts and ports also tried while the camera does not answer on the
    /// configured address. Tests replace them with local ones.
    static void setProbeTargets(const QStringList &hosts, const QList<int> &ports);

signals:
    void enabledChanged();
    void settingsChanged();
    void answeringChanged();
    void activeEndpointChanged();
    void attitudeChanged();
    void laserBusyChanged();
    void aimBusyChanged();
    void laserChanged();
    void zoomChanged();
    void recordingChanged();
    void targetChanged();
    void lockChanged();
    void rcChannelsChanged();
    void detectingWheelChanged();
    /// Result of detectWheel: the channel now used, or 0 when no wheel moved.
    void wheelDetected(const QString &axis, int channel);

private slots:
    void _readPending();
    void _poll();
    void _checkAnswering();
    void _laserReadAfterShot();
    void _laserFinish();
    void _motionTick();
    void _finishWheelDetect();
    void _sendWheelAngles();
    void _zoomOutTick();
    void _aimTick();
    void _lockTick();
    void _sendLockConfirm();

private:
    // Plain functions, not slots: a slot taking Vehicle* would need the full
    // Vehicle class in this header.
    void _activeVehicleChanged(Vehicle *vehicle);
    void _rcChannelsReceived(QVector<int> values);

    struct Endpoint
    {
        QHostAddress address;
        quint16 port = 0;
        bool operator==(const Endpoint &other) const
        {
            return port == other.port && address.isEqual(other.address);
        }
    };

    enum Axis { YawAxis = 0, PitchAxis = 1 };

    void _start();
    void _stop();
    void _loadSettings();
    void _saveSettings() const;
    void _applyModel();
    void _rebuildEndpoints();
    void _setActive(const Endpoint &endpoint);
    void _sendTo(const Endpoint &endpoint, const std::string &frame);
    void _send(const std::string &frame) { _sendTo(_active, frame); }
    /// A long gimbal frame: the upper-case "#TP" form first, then the lower-case
    /// form VGCS uses, when the camera's frames are lower-case (C12, C13).
    void _sendGimbal(const std::string &frame);
    void _handleFrame(const skydroid::top::DecodedFrame &frame, const Endpoint &from);
    Endpoint _knownEndpointFor(const QHostAddress &address, quint16 port) const;
    bool _attitudeFresh() const;
    void _probe();
    QList<Endpoint> _laserEndpoints() const;
    bool _isZoomStepCamera() const { return _model == QStringLiteral("C14 Pro"); }
    void _sendZoomStepFrames(int direction);
    /// To the control port, and to the other ports some C13 firmware takes zoom on.
    void _sendZoomFrames(const std::vector<std::string> &frames);
    int _zoomStepsMax() const;
    /// Drone position and attitude now, from QGC. False when there is no usable GPS fix.
    bool _sampleVehiclePose(skydroid::geo::LaserInput &pose, QString &why) const;
    void _computeTarget();
    /// keepLastResult: a refresh during a lock keeps showing the last result until the new one arrives.
    void _fireLaser(bool keepLastResult);

    // Tap aiming and the lock
    /// Sets the angles to turn to for a point of the picture. Returns how far
    /// the point is from the picture centre, in degrees (the larger axis).
    double _setAimFor(double u, double v);
    /// Sends the aim angles and starts watching the turn.
    void _startTurn();
    /// Where the aimed point is in the picture now, from the gimbal angles.
    void _aimPointInPicture(double &u, double &v) const;
    void _armLock(double u, double v);
    /// Stops the lock (SUM stop) and shows the message.
    void _endLock(const QString &message);
    /// The operator moved the camera: the lock must not fight them.
    void _endLockByHand();
    void _setLockMessage(const QString &message);

    // Motion
    void _desiredSpeed(double &yawDps, double &pitchDps);
    void _sendSpeed(double yawDps, double pitchDps);
    void _sendStop();
    void _startMotionTimer();
    /// How far a speed-mode wheel is turned, -1 to 1, or 0 when it is off or has no fresh value.
    double _wheelDeflection(int channel) const;
    /// The RC value of a channel when it is fresh and plausible.
    std::optional<int> _rcValue(int channel) const;
    void _updateWheelAngles();

    QUdpSocket *_socket = nullptr;
    QTimer _pollTimer;
    QTimer _answerTimer;
    QTimer _laserShotTimer;
    QTimer _laserTimeoutTimer;
    QElapsedTimer _sinceReply;
    QElapsedTimer _sinceAttitude;
    int _pollCount = 0;

    Endpoint _configured;
    Endpoint _active;
    QList<Endpoint> _endpoints;  // configured first, then the probe targets

    skydroid::top::Options _options;
    bool _enabled = false;
    QString _host;
    int _port = 5000;
    QString _model;
    int _maxSpeed = 20;
    bool _reverseYaw = false;
    bool _reversePitch = false;
    int _wheelPitchChannel = 0;
    int _wheelYawChannel = 0;
    bool _wheelHoldsPosition = false;
    bool _wheelReverse = false;
    bool _reverseTapYaw = false;
    QString _dayVideoUrl;
    QString _thermalVideoUrl;

    // Tap aiming
    bool _aimBusy = false;
    bool _aimResent = false;
    double _aimYaw = 0.0;      // the angles sent (inside the gimbal's limits)
    double _aimPitch = 0.0;
    double _aimPointYaw = 0.0;    // where the point is, which can be past a limit
    double _aimPointPitch = 0.0;
    QTimer _aimTimer;
    QElapsedTimer _aimElapsed;
    QElapsedTimer _aimSettled;

    // Object lock
    bool _lockBusy = false;
    bool _lockActive = false;
    bool _lockFollowSeen = false;
    bool _lockWarned = false;
    bool _lockSettled = false;   // the first 2 s after the lock command are over
    bool _lockJumped = false;    // the camera moved in those 2 s
    double _lockJumpDeg = 0.0;
    double _lockTurnedDeg = 0.0;
    double _lockStartYaw = 0.0;
    double _lockStartPitch = 0.0;
    QString _lockMessage;
    QTimer _lockTimer;
    QElapsedTimer _lockElapsed;
    qint64 _lockNextConfirmMs = 0;
    qint64 _lockNextLaserMs = 0;

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
    int _zoomCount = 0;     // C12/C13: our zoom steps in from 1x
    int _zoomOutLeft = 0;   // steps still to send for "back to 1x"
    QTimer _zoomOutTimer;
    QString _zoomReport;
    bool _recording = false;

    // Motion
    QTimer _motionTimer;
    bool _touchActive = false;
    bool _touchBlocked = false;  // after the runaway cap, until the finger lifts
    double _touchX = 0.0;
    double _touchY = 0.0;
    QElapsedTimer _touchLease;    // since the last setTouchMotion call
    QElapsedTimer _touchStarted;  // since this touch started moving the gimbal
    bool _moving = false;
    int _stopRepeats = 0;
    int _lastYawUnits = 0;    // last GSY value sent, in 0.5 deg/s units
    int _lastPitchUnits = 0;  // last GSP value sent
    int _yawZeroRepeats = 0;
    int _pitchZeroRepeats = 0;

    // RC wheels
    QPointer<Vehicle> _vehicle;
    QVector<int> _rc;
    QElapsedTimer _rcAge;
    // Hold-position wheels: the value seen first, and whether the wheel has moved since.
    std::optional<int> _wheelStart[2];
    bool _wheelMoved[2] = {false, false};
    std::optional<double> _wheelTarget[2];
    std::optional<double> _wheelSent[2];
    QElapsedTimer _wheelSentAt;
    QTimer _wheelAngleTimer;
    // Wheel detection
    QString _detectAxis;
    QTimer _detectTimer;
    QVector<int> _detectMin;
    QVector<int> _detectMax;
};
