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
// Object lock (lockAtBox). The app follows the object itself: it finds the
// object in pictures of the video (ObjectTracker) and turns the camera with
// speed commands until the object is under the cross. VGCS follows a Skydroid
// camera the same way (its M14). The box on the screen is where the app sees
// the object, so the operator can check it.
//
// The first lock (test builds 4 to 6) used the camera's own tracker (GOT and
// SUM, as VGCS M13). In two field videos the V13 did not follow: a parked car
// on 2026-10-06 and a walking person on 2026-10-09. Both times the camera
// moved about 2.4 degrees left and 1.2 up right after the lock command, then
// the object walked off and the camera only drifted. The camera reports
// nothing about what it tracks. That way is kept as lock mode "camera", with
// the lock point moved by that much (see kGotCentre in the .cc), to be tried
// again. TOP V1.1.6 lists GOT and SUM for the C12 only; V1.2.0 adds the C13
// and the C14 Pro.
//
// Marked objects (ObjectMarks). While the lock button waits for an object, the
// app marks the people and vehicles it finds in the video. Such a mark is some
// tenths of a second old when the operator taps it, and a car has driven on by
// then. So the lock does not start on the newest picture with the old box: the
// last pictures are kept (keepPictures), the tracker starts on the kept picture
// of the moment the object was found in, and goes through the pictures since
// (lockAtFound). The box is then on the object where it is now.
//
// The finder's check (LockCheck.h, task AC). While a person or a vehicle is
// locked, ObjectMarks has the finder look at the video about once a second
// (lockWantsLooks, finderLooked). When the object has not been on the box in
// three looks in a row, the box holds something else (a look-alike, a tree it
// went behind): the screen says NOT SEEN and the camera waits. When the object
// is not seen and the finder sees one of its kind near where it went, the lock
// goes on there. A lock waits kLockLostMs for its object before it ends.
//
// Follow Me (ObjectFollow) flies the drone after the object of the app's lock.
// For that the link says where the object is on the ground, for every picture
// it is found in (objectPlaced): the camera's angle to the object, and the
// drone's height above the ground there.
//
// The camera (CameraProfile.h, task AA): its lens, zoom, limits, laser and
// features come from the camera in use, one of the built-in ones (V12, V13,
// V14 Pro) or one the admin added in the camera editor (a file in the app's
// folder "Cameras"). Choosing another camera (chooseCamera) also sets its
// addresses. setModel changes the camera alone, as it always did.
//
// Photos and recording (task AL, client 2026-10-26: "photos and recordings are not saved in
// rc"). The buttons tell the camera to take a photo (CAP) or to start or stop its recording
// (REC 01, REC 00). The camera saves them on its own SD card: the app keeps nothing on the RC.
// Until test build 13 every record press sent REC 01, so the second press never stopped the
// camera. The camera is asked whether it records (REC read) and for its SD card's space (SDC),
// and after each press the screen says what the camera did (cameraNote).
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
#include <QtCore/QVariantMap>
#include <QtCore/QVector>
#include <QtGui/QImage>
#include <QtNetwork/QHostAddress>
#include <QtQmlIntegration/QtQmlIntegration>

#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "CameraProfile.h"
#include "LaserGeo.h"
#include "LockCheck.h"
#include "ObjectTracker.h"
#include "SiyiCamera.h"
#include "SkydroidTop.h"
#include "ViewproCamera.h"

class QJSEngine;
class QQmlEngine;
class QTcpSocket;
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
    Q_PROPERTY(QStringList models READ models NOTIFY camerasChanged)
    Q_PROPERTY(QStringList modelNames READ modelNames NOTIFY camerasChanged)
    Q_PROPERTY(QString modelName READ modelName NOTIFY settingsChanged)
    /// The camera in use as the camera editor shows it (the fields of its file, with "id",
    /// "builtIn" and "thermal"). Its features decide which buttons the screen shows.
    Q_PROPERTY(QVariantMap camera READ camera NOTIFY settingsChanged)
    /// Every camera, for the camera editor: "id", "name" and "builtIn" of each.
    Q_PROPERTY(QVariantList cameras READ cameras NOTIFY camerasChanged)
    /// Where added cameras are kept: one file each.
    Q_PROPERTY(QString cameraFolder READ cameraFolder CONSTANT)
    /// Files in that folder that were not read, each with why.
    Q_PROPERTY(QStringList cameraNotes READ cameraNotes NOTIFY camerasChanged)
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
    /// Thermal colour mode: an id of thermalPalettes().
    Q_PROPERTY(QString thermalPalette READ thermalPalette WRITE setThermalPalette NOTIFY settingsChanged)
    /// Its place in thermalPalettes() (0: as received), which is also its row in thermal_palettes.png.
    Q_PROPERTY(int thermalPaletteIndex READ thermalPaletteIndex NOTIFY settingsChanged)
    Q_PROPERTY(QStringList thermalPalettes READ thermalPalettes CONSTANT)
    Q_PROPERTY(QStringList thermalPaletteNames READ thermalPaletteNames CONSTANT)

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
    /// The camera itself said lately that it records as the screen shows (REC read).
    Q_PROPERTY(bool recordingConfirmed READ recordingConfirmed NOTIFY recordingChanged)
    /// "No SD card", "SD card almost full", or empty: for the camera status.
    Q_PROPERTY(QString sdCardWarning READ sdCardWarning NOTIFY sdCardChanged)
    /// What the camera says of its SD card, in a sentence, or empty while that is not known.
    Q_PROPERTY(QString sdCardText READ sdCardText NOTIFY sdCardChanged)
    Q_PROPERTY(bool targetValid READ targetValid NOTIFY targetChanged)
    Q_PROPERTY(double targetLat READ targetLat NOTIFY targetChanged)
    Q_PROPERTY(double targetLon READ targetLon NOTIFY targetChanged)
    Q_PROPERTY(bool targetHasAlt READ targetHasAlt NOTIFY targetChanged)
    Q_PROPERTY(double targetAltMsl READ targetAltMsl NOTIFY targetChanged)
    Q_PROPERTY(double targetHorizontalM READ targetHorizontalM NOTIFY targetChanged)
    Q_PROPERTY(double targetBearingDeg READ targetBearingDeg NOTIFY targetChanged)
    Q_PROPERTY(QString targetMessage READ targetMessage NOTIFY targetChanged)
    /// The position that shows comes from the picture (the camera's angle and the
    /// drone's height), not from a laser range.
    Q_PROPERTY(bool targetFromPicture READ targetFromPicture NOTIFY targetChanged)
    /// How far that point is from the drone, as the picture gives it.
    Q_PROPERTY(double targetSlantM READ targetSlantM NOTIFY targetChanged)
    /// The last measurement did not ask the laser at all (point mode "picture").
    Q_PROPERTY(bool measuredByPicture READ measuredByPicture NOTIFY laserChanged)

    // The position of a point.
    /// "laser": a tap turns the camera to the point and the laser measures it.
    /// "picture": a tap gives the point's position at once, without the laser.
    Q_PROPERTY(QString pointMode READ pointMode WRITE setPointMode NOTIFY settingsChanged)
    Q_PROPERTY(QStringList pointModes READ pointModes CONSTANT)
    Q_PROPERTY(QStringList pointModeNames READ pointModeNames CONSTANT)
    /// The screen says that the thermal picture shows: it has another lens.
    Q_PROPERTY(bool thermalPicture READ thermalPicture WRITE setThermalPicture NOTIFY thermalPictureChanged)

    // Object lock.
    /// "app": the app follows the object in the video and turns the camera.
    /// "camera": the camera's own tracker (GOT and SUM).
    Q_PROPERTY(QString lockMode READ lockMode WRITE setLockMode NOTIFY settingsChanged)
    Q_PROPERTY(QStringList lockModes READ lockModes CONSTANT)
    Q_PROPERTY(QStringList lockModeNames READ lockModeNames CONSTANT)
    /// True while a lock is being started: the app waits for its first picture,
    /// or (camera mode) the camera turns to the object.
    Q_PROPERTY(bool lockBusy READ lockBusy NOTIFY lockChanged)
    /// True while an object is locked.
    Q_PROPERTY(bool lockActive READ lockActive NOTIFY lockChanged)
    /// True when this lock is the app's own (false: the camera's tracker).
    Q_PROPERTY(bool lockByApp READ lockByApp NOTIFY lockChanged)
    /// True while the app needs pictures of the video (lockPicture).
    Q_PROPERTY(bool lockWantsPictures READ lockWantsPictures NOTIFY lockChanged)
    /// True while the app sees the object in the pictures.
    Q_PROPERTY(bool lockSeen READ lockSeen NOTIFY lockChanged)
    /// The box around the object as the app sees it now, in parts of the picture (0 to 1): centre and size.
    Q_PROPERTY(bool lockBoxValid READ lockBoxValid NOTIFY lockBoxChanged)
    Q_PROPERTY(double lockBoxU READ lockBoxU NOTIFY lockBoxChanged)
    Q_PROPERTY(double lockBoxV READ lockBoxV NOTIFY lockBoxChanged)
    Q_PROPERTY(double lockBoxW READ lockBoxW NOTIFY lockBoxChanged)
    Q_PROPERTY(double lockBoxH READ lockBoxH NOTIFY lockBoxChanged)
    /// How far the object is from the cross, in degrees (the larger of across and up).
    Q_PROPERTY(double lockOffDeg READ lockOffDeg NOTIFY lockBoxChanged)
    /// For a field video: what the app measures and commands, in one line.
    Q_PROPERTY(QString lockNumbers READ lockNumbers NOTIFY lockBoxChanged)
    // Camera mode only: the camera never reports what it tracks, so the gimbal
    // angles are all the app has.
    /// How far the camera has turned by itself since the lock settled, in degrees.
    Q_PROPERTY(double lockTurnedDeg READ lockTurnedDeg NOTIFY lockChanged)
    /// How far the camera moved in the first 2 s after the lock command, in degrees.
    Q_PROPERTY(double lockJumpDeg READ lockJumpDeg NOTIFY lockChanged)
    /// True once the camera has turned by itself after the lock settled. That is
    /// not proof that it follows the object: on 2026-10-09 it was a slow drift.
    Q_PROPERTY(bool lockFollowSeen READ lockFollowSeen NOTIFY lockChanged)
    Q_PROPERTY(QString lockMessage READ lockMessage NOTIFY lockChanged)

    // RC channels from the flight controller (index 0 is channel 1)
    Q_PROPERTY(QVariantList rcChannels READ rcChannels NOTIFY rcChannelsChanged)
    /// "pitch" or "yaw" while the wheel detection runs, else empty.
    Q_PROPERTY(QString detectingWheel READ detectingWheel NOTIFY detectingWheelChanged)

public:
    /// No default for the parent, on purpose. QML makes a singleton (QML_SINGLETON) with a
    /// constructor that needs no argument when there is one, and with create() only when there
    /// is none (Qt's qqmlprivate.h, singletonConstructionMode). With a default here, QML made
    /// a link of its own for the screen, and Follow Me and the marks used instance(), a second
    /// link that was never locked (tasks X and V, builds 9 and 10).
    explicit SkydroidLink(QObject *parent);
    ~SkydroidLink() override;

    /// The app's one link. QML gets it too (create()).
    static SkydroidLink *instance();
    static SkydroidLink *create(QQmlEngine *qmlEngine, QJSEngine *jsEngine);

    bool enabled() const { return _enabled; }
    void setEnabled(bool enabled);
    QString host() const { return _host; }
    void setHost(const QString &host);
    int port() const { return _port; }
    void setPort(int port);
    /// The id of the camera in use: Skydroid's name for a built-in one (C12, C13, C14 Pro),
    /// which the code and the saved settings use, or an added camera's file name.
    QString model() const { return _camera.id; }
    /// Uses another camera, with the addresses as they are. An unknown id is ignored.
    void setModel(const QString &model);
    /// The ids of every camera: the built-in ones, then the added ones.
    QStringList models() const;
    /// The names on screen, in the order of models(): VAMA's for the built-in cameras (V12,
    /// V13, V14 Pro), the admin's for the added ones.
    QStringList modelNames() const;
    QString modelName() const { return _camera.name; }
    QVariantMap camera() const;
    QVariantList cameras() const;
    QString cameraFolder() const { return _cameraStore.folder(); }
    QStringList cameraNotes() const { return _cameraStore.notes(); }
    /// Another camera, chosen on the screen: setModel, and that camera's addresses (camera IP,
    /// control port, day and thermal video). Choosing the camera in use changes nothing.
    Q_INVOKABLE void chooseCamera(const QString &id);
    /// A camera's fields for the editor (as camera), or nothing for an unknown id.
    Q_INVOKABLE QVariantMap cameraProfile(const QString &id) const;
    /// Saves a camera from the editor's fields: an added one keeps its file, anything else (a
    /// copy of a built-in camera) becomes a new one. "problem": what is wrong (then nothing is
    /// saved), else empty; "id": the camera's id. The camera in use is used as saved, with its
    /// addresses.
    Q_INVOKABLE QVariantMap saveCamera(const QVariantMap &fields);
    /// Deletes an added camera's file. The problem, or empty. When it was in use, the V13 is.
    Q_INVOKABLE QString deleteCamera(const QString &id);
    /// Reads the camera folder again (files copied in while the app runs).
    Q_INVOKABLE void reloadCameras();
    /// The folder of added cameras. Tests and the screen preview give a folder of their own;
    /// else it is the app's own folder "Cameras" (Android/data/com.vama.gcs/files/Cameras).
    static void setCameraFolder(const QString &folder);
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
    /// Thermal colour modes, the same as VGCS (vgcs/video/thermal_palette.py):
    /// its ids in its menu order. The first is the picture as the camera sends
    /// it (white hot). While IR is on, FlightDisplayViewVideoOutput.qml draws
    /// the picture through the chosen mode's colour table.
    static QStringList thermalPalettes();
    /// The names on screen (VGCS's), in the order of thermalPalettes().
    static QStringList thermalPaletteNames();
    QString thermalPalette() const { return _thermalPalette; }
    /// An unknown id means the picture as received, as in VGCS.
    void setThermalPalette(const QString &id);
    int thermalPaletteIndex() const { return thermalPalettes().indexOf(_thermalPalette); }

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
    /// The camera records to its SD card: what it said when asked (every 2 s), and right after a
    /// press what the press asked for, until the camera answers.
    bool recording() const { return _recording; }
    bool recordingConfirmed() const { return _recordConfirmed; }
    QString sdCardWarning() const;
    QString sdCardText() const;
    bool targetValid() const { return _target.ok; }
    double targetLat() const { return _target.targetLatDeg; }
    double targetLon() const { return _target.targetLonDeg; }
    bool targetHasAlt() const { return _target.targetAltMslM.has_value(); }
    double targetAltMsl() const { return _target.targetAltMslM.value_or(0.0); }
    double targetHorizontalM() const { return _target.horizontalRangeM; }
    double targetBearingDeg() const { return _target.bearingDeg; }
    QString targetMessage() const { return _targetMessage; }
    bool targetFromPicture() const { return _target.ok && _targetFromPicture; }
    double targetSlantM() const { return _targetSlantM; }
    bool measuredByPicture() const { return _measuredByPicture; }
    static QStringList pointModes();
    /// The names on screen, in the order of pointModes().
    static QStringList pointModeNames();
    QString pointMode() const { return _pointMode; }
    /// An unknown mode means "laser".
    void setPointMode(const QString &mode);
    bool thermalPicture() const { return _thermalPicture; }
    void setThermalPicture(bool thermal);
    /// A camera whose thermal picture comes in its day video (a Viewpro, thermalBySwitch): switch
    /// its sensor. For the others the screen switches the video address.
    Q_INVOKABLE void switchThermal(bool on);
    static QStringList lockModes();
    /// The names on screen, in the order of lockModes().
    static QStringList lockModeNames();
    QString lockMode() const { return _lockMode; }
    /// An unknown mode means "app". Changing the mode ends a lock.
    void setLockMode(const QString &mode);
    bool lockBusy() const { return _lockBusy; }
    bool lockActive() const { return _lockActive; }
    bool lockByApp() const { return _lockByApp; }
    bool lockWantsPictures() const { return _keepPictures || (_lockByApp && (_lockBusy || _lockActive)); }
    bool lockSeen() const { return _lockSeen; }
    bool lockBoxValid() const { return _lockBoxValid; }
    double lockBoxU() const { return _lockBoxU; }
    double lockBoxV() const { return _lockBoxV; }
    double lockBoxW() const { return _lockBoxW; }
    double lockBoxH() const { return _lockBoxH; }
    double lockOffDeg() const { return _lockOffDeg; }
    QString lockNumbers() const { return _lockNumbers; }
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
    /// A point tapped on the video. u and v run from 0 to 1 across and down the picture.
    ///  - Point mode "laser": the camera turns to the point, then the laser
    ///    measures it. With no laser reading the position comes from the picture
    ///    when the picture can give one.
    ///  - Point mode "picture": the position comes from the picture at once (the
    ///    camera's angle and the drone's height over level ground). The camera
    ///    stays where it is and the laser is not fired.
    Q_INVOKABLE void aimAndMeasure(double u, double v);
    /// Lock on the object in a box of the video: its centre u, v (as for
    /// aimAndMeasure) and its size w, h, all in parts of the picture.
    ///  - Mode "app": the app follows the object in the pictures it is given
    ///    (lockPicture) and turns the camera so the object comes under the
    ///    cross and stays there.
    ///  - Mode "camera": the camera turns to the point and is told to follow
    ///    it (GOT, then SUM confirm).
    /// The object is measured with the laser every few seconds while it is
    /// under the cross. Moving the camera by hand ends the lock.
    /// True when a lock was started. False when none was: the lock message says
    /// why (the camera is still turning to a point that was tapped, the camera
    /// link is off, or in mode "camera" no gimbal angles). Also false, without a
    /// word, while a lock is starting, and for numbers that are not numbers.
    Q_INVOKABLE bool lockAtBox(double u, double v, double w, double h);
    /// The same with a box of the usual size (a tap).
    Q_INVOKABLE bool lockAt(double u, double v);
    /// While true, the last pictures given with lockPicture are kept (some two
    /// seconds of them, and lockWantsPictures is true), so that a lock can start
    /// on a picture of a moment ago. ObjectMarks turns this on while it marks objects.
    void keepPictures(bool keep);
    bool keepsPictures() const { return _keepPictures; }
    /// The clock the kept pictures are timed with, in milliseconds.
    qint64 pictureClockMs() const { return _pictureClock.elapsed(); }
    /// The time of the newest kept picture on that clock, or -1 when none is kept.
    qint64 newestKeptPictureMs() const { return _kept.empty() ? -1 : _kept.back().atMs; }
    /// Lock on an object the app found in a picture taken at atMs of the picture
    /// clock: the box it had then, as for lockAtBox. In mode "app" the lock starts
    /// on the kept picture of that moment and goes through the pictures since.
    /// With no kept picture of that moment (and in mode "camera") this is lockAtBox.
    /// kind and score: what the finder took it for, and how sure it was (the
    /// finder's check, LockCheck.h). Without a kind the first looks may say.
    /// True when a lock was started, as for lockAtBox.
    bool lockAtFound(double u, double v, double w, double h, qint64 atMs,
                     std::optional<skydroid::find::Kind> kind = std::nullopt, double score = 0.0);
    /// The finder's picture of this moment, for a lock that starts on it when no picture
    /// given with lockPicture is near enough to that moment. It is kept at the size of
    /// those (kept only while pictures are kept, and once one of those has come).
    void keepFinderPicture(const QImage &image, double pictureX, double pictureY, double pictureW, double pictureH);
    /// True while the app's lock wants the finder's looks: its object is a person or a
    /// vehicle (or may be one, for a drawn box), and the finder's check is on.
    bool lockWantsLooks() const;
    /// The finder looked at the picture of atMs (the picture clock) for the lock: what it
    /// found there, in parts of the picture, and the picture's width over its height.
    void finderLooked(const std::vector<skydroid::find::Found> &found, qint64 atMs, double aspect);
    Q_INVOKABLE void stopLock();
    /// A picture of the video for the lock (mode "app"), about 10 times a
    /// second while lockWantsPictures is true. The picture itself may be a
    /// part of the image (black bars beside a 5:4 thermal picture): its left
    /// top corner and size, in parts of the image.
    Q_INVOKABLE void lockPicture(const QImage &image, double pictureX, double pictureY, double pictureW,
                                 double pictureH);
    /// The speed to turn at, in degrees per second, for an object this far
    /// from the cross: nothing inside half a degree, then in proportion, and
    /// never faster than maxDps.
    static double followSpeed(double offDeg, double maxDps);
    /// The picture's field of view now, in degrees, zoom included.
    void currentFov(double &horizontalDeg, double &verticalDeg) const;
    /// The field of view of the picture on the screen: the day lens (zoom
    /// included) or, while the thermal picture shows, the thermal lens.
    /// False when that lens is known only from a data sheet (the C12's and the
    /// C13's thermal lens): good enough to turn the camera, not to place a point.
    bool pictureFov(double &horizontalDeg, double &verticalDeg) const;
    /// CAP: the camera takes a photo onto its SD card. cameraNote says how that went.
    Q_INVOKABLE void takePhoto();
    /// REC: starts the camera's recording when the screen shows none, else stops it.
    /// cameraNote says how that went.
    Q_INVOKABLE void toggleRecord();
    /// REC 01 (start) or REC 00 (stop), for a screen that records a copy on the RC as well and
    /// decides for both. Asking for what the screen shows already sends the frame again, to make
    /// sure, with no note.
    Q_INVOKABLE void setRecord(bool start);
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
    /// A camera was added, changed or deleted (the camera editor), or the folder read again.
    void camerasChanged();
    void answeringChanged();
    void activeEndpointChanged();
    void attitudeChanged();
    void laserBusyChanged();
    void aimBusyChanged();
    void laserChanged();
    void zoomChanged();
    void recordingChanged();
    void sdCardChanged();
    /// What the camera did with a photo or record press, for the screen. problem: it did not
    /// do it, it does not say, or its SD card is missing or almost full.
    void cameraNote(const QString &text, bool problem);
    void targetChanged();
    void thermalPictureChanged();
    /// A camera that switched the picture of its one video (a Viewpro, whose network stops for some
    /// seconds) answers on its video port again, or the wait for that is over: the screen opens the
    /// video again if it shows none.
    void videoReopenWanted();
    void lockChanged();
    void lockBoxChanged();
    /// A zoom step was asked for: what was found in the pictures so far has another size now.
    void zoomAsked();
    void rcChannelsChanged();
    void detectingWheelChanged();
    /// Result of detectWheel: the channel now used, or 0 when no wheel moved.
    void wheelDetected(const QString &axis, int channel);
    /// The object of the app's lock, placed on the ground from a picture it was found in
    /// (at most five a second). sigmaM: how sure the place is, one standard deviation.
    /// belowM: how far the object is below the drone when the laser measured that, else
    /// not a number (the ground is then taken as level). ageMs: how old the picture is.
    void objectPlaced(double latDeg, double lonDeg, double sigmaM, double belowM, int ageMs);

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
    void _photoVerdict();
    void _recordVerdict();
    void _sendDue();

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
    /// Uses this camera: its lens, zoom, limits and features, and with withAddresses its addresses.
    void _useCamera(const skydroid::camera::Profile &profile, bool withAddresses);
    void _applyModel();
    void _rebuildEndpoints();
    void _setActive(const Endpoint &endpoint);
    void _sendTo(const Endpoint &endpoint, const std::string &frame);
    void _send(const std::string &frame) { _sendTo(_active, frame); }
    /// A long gimbal frame: the upper-case "#TP" form first, then the lower-case
    /// form VGCS uses, when the camera's frames are lower-case (C12, C13).
    void _sendGimbal(const std::string &frame);
    void _handleFrame(const skydroid::top::DecodedFrame &frame, const Endpoint &from);
    /// A camera that takes TOP over TCP (the C20): one connection to its address while the
    /// link runs, opened again a moment after it is lost.
    void _syncTcp();
    void _openTcp();
    void _closeTcp();
    void _readTcp();
    void _tcpLost();
    // Viewpro and SIYI (task AK): their rules (ViewproCamera, SiyiCamera) make the bytes, these send
    // them (Viewpro over the TCP connection, SIYI by UDP) and take what the camera says. A TOP frame
    // never goes to such a camera (_sendTo).
    bool _top() const { return _camera.protocol == skydroid::camera::Protocol::Top; }
    long long _nowMs() const;
    void _sendBytes(const std::vector<std::uint8_t> &bytes);
    void _sendAllBytes(const std::vector<std::vector<std::uint8_t>> &frames);
    /// While a Viewpro or SIYI link runs, the rules' frames due later go out on _dueTimer.
    void _syncDueTimer();
    void _takeViewproStatus();
    /// A Viewpro or SIYI angle command on both axes, the other axis where the camera is now.
    void _sendAnglesOther(std::optional<double> yawDeg, std::optional<double> pitchDeg);
    void _markAnswering();
    void _takeAttitude(double yawDeg, double pitchDeg, double rollDeg);
    /// Which way the gimbal yaw counts against the picture (the screen's switch turns it round).
    bool _yawLeftPositive() const;
    Endpoint _knownEndpointFor(const QHostAddress &address, quint16 port) const;
    bool _attitudeFresh() const;
    void _probe();
    QList<Endpoint> _laserEndpoints() const;
    bool _isZoomStepCamera() const { return _camera.zoom == skydroid::camera::Zoom::Reported; }
    void _sendZoomStepFrames(int direction);
    void _lensZoom(int direction);
    void _lensHomeDone();
    /// To the control port, and to the other ports some C13 firmware takes zoom on.
    void _sendZoomFrames(const std::vector<std::string> &frames);
    int _zoomStepsMax() const;

    // Photos and recording
    /// The camera answered whether it records.
    void _recordAnswered(bool on);
    /// The state on the screen, and whether the camera's last answer says the same.
    void _setRecording(bool on);
    void _askSdCard();
    void _sdCardAnswered(const skydroid::top::SdCard &card);
    /// The camera's SD card has no room, in words (" It has no SD card."), or nothing.
    QString _sdCardProblem() const;
    /// Forgets what the camera said (another camera, or the link stopped).
    void _forgetCameraReports();
    /// Drone position and attitude now, from QGC. False when there is no usable GPS fix.
    bool _sampleVehiclePose(skydroid::geo::LaserInput &pose, QString &why) const;
    /// The drone's height above its take-off point, when it is known.
    std::optional<double> _sampleHeight() const;
    void _computeTarget();
    /// Point mode "picture": the position of a point of the picture, now, without the laser.
    void _measureFromPicture(double u, double v);
    /// The object of a lock is under the cross: by laser, or from the picture in point mode "picture".
    void _measureLockedObject();
    /// Works the point out from the picture and shows it. False, with the reason, when the picture gives none.
    bool _setTargetFromPicture(const skydroid::geo::LaserInput &view, std::optional<double> heightM, QString &whyNot);
    /// keepLastResult: a refresh during a lock keeps showing the last result until the new one arrives.
    void _fireLaser(bool keepLastResult);

    // Tap aiming and the lock
    /// Sets the angles to turn to for a point of the picture. Returns how far
    /// the point is from the picture centre, in degrees (the larger axis).
    double _setAimFor(double u, double v);
    /// Sends the aim angles and starts watching the turn.
    void _startTurn();
    void _sendAimAngles();
    /// Where the aimed point is in the picture now, from the gimbal angles.
    void _aimPointInPicture(double &u, double &v) const;
    void _armLock(double u, double v);
    /// Mode "camera": turn to the point, then GOT and SUM. False without gimbal angles (the lock message says so).
    bool _lockByCamera(double u, double v, bool wasLocked);
    /// Mode "app": one picture for the tracker (RGB888 or Grayscale8), with where the video
    /// picture is inside it, in points. It starts a lock that waits, or finds the object
    /// again. act false: the picture is an old one, and only the tracker learns from it.
    void _trackPicture(const QImage &pixels, double left, double top, double width, double height, bool act);
    void _keepPicture(const QImage &pixels, double left, double top, double width, double height);
    struct KeptPicture;  // (below, with the lock's other data)
    /// The kept pictures from moment atMs on: first a picture given with lockPicture no further than
    /// kFoundPictureGapMs from it, or else the finder's own picture of that moment, then the ones since.
    /// Empty when there is no picture of that moment.
    std::vector<KeptPicture> _picturesFrom(qint64 atMs) const;
    /// The tracker starts on the box waiting (_pending) in the first of these pictures and goes through
    /// the rest, as it would have then. The camera is turned for the newest only.
    void _playFrom(const std::vector<KeptPicture> &pictures);
    /// The finder found the object of the lock again (task AC): the lock goes on there.
    void _lockAgainOn(const skydroid::check::Box &box, qint64 atMs);
    /// The lock's state while the finder does not see the object on the box.
    QString _doubtMessage() const;
    /// Mode "app": what to do with what the tracker found in a picture.
    void _follow(const skydroid::track::Result &result, double left, double top, double width, double height);
    /// How far the camera has turned since the picture was taken, from the speeds sent, in degrees.
    void _turnedSincePicture(double &yawDeg, double &pitchDeg) const;
    void _setFollowSpeed(double yawDps, double pitchDps);
    /// Places the object of the app's lock on the ground, from the picture it was just found in:
    /// where it is from the cross, in degrees (right and up are +), seen from where the camera
    /// looks now, and the lens of that picture.
    void _placeObject(double rightDeg, double upDeg, double hfovDeg, double vfovDeg);
    /// A laser range on the locked object says how high the ground there is against the take-off point.
    void _learnGround();
    void _forgetGround();
    /// True when the picture shows that the video has stopped (see kVideoStoppedMs).
    bool _videoHasStopped(const QImage &pixels, int left, int top, int width, int height, int channels);
    void _clearMeasurement();
    /// Stops the lock (the gimbal, or SUM stop) and shows the message.
    void _endLock(const QString &message);
    /// The operator moved the camera: the lock must not fight them.
    void _endLockByHand();
    /// A zoom step changes the size of the object in the picture, which the app's tracker cannot take.
    void _endAppLockForZoom();
    void _setLockMessage(const QString &message);

    // Motion
    void _desiredSpeed(double &yawDps, double &pitchDps);
    void _sendSpeed(double yawDps, double pitchDps);
    /// A turn to an angle on one axis ("GAY" or "GAP"). The camera goes on turning after it.
    void _sendAngle(const char *axis, double degrees, double dps);
    void _sendStop();
    void _startMotionTimer();
    /// How far a speed-mode wheel is turned, -1 to 1, or 0 when it is off or has no fresh value.
    double _wheelDeflection(int channel) const;
    /// _wheelDeflection of an axis's wheel, or 0 until that wheel has been turned (_wheelTurnedSinceStart).
    double _speedWheelDeflection(int axis) const;
    /// Whether the wheel of an axis has been turned since the link first saw it. Never only
    /// because the link started, or because a channel rests away from its middle.
    bool _wheelTurnedSinceStart(int axis, int value);
    /// The RC value of a channel when it is fresh and plausible.
    std::optional<int> _rcValue(int channel) const;
    void _updateWheelAngles();
    // After a Viewpro's sensor switch (switchThermal): knock on its video port until it answers again.
    void _startVideoWait();
    void _probeVideoPort();
    void _videoPortAnswered(QTcpSocket *probe);
    void _endVideoWait(bool back);
    void _dropVideoProbe();

    QUdpSocket *_socket = nullptr;  // made while the link runs, also for a TCP camera
    QTcpSocket *_tcp = nullptr;
    QString _tcpHost;
    int _tcpPort = 0;
    std::string _tcpRx;             // what came over TCP that is not a whole frame yet
    QTimer _tcpRetryTimer;
    QTimer _tcpConnectTimer;
    viewpro::Camera _viewpro;       // the rules of a Viewpro camera in use (task AK)
    siyi::Camera _siyi;             // and of a SIYI camera
    QTimer _dueTimer;               // sends the rules' frames due later (a zoom stop, a hold after a jog)
    QElapsedTimer _rulesClock;
    double _liveZoomX = 0.0;        // the zoom a Viewpro says (0: not yet)
    double _liveFovHDeg = 0.0;      // and its view
    double _liveFovVDeg = 0.0;
    /// A SIYI's zoom is not known: it never went back to 1x here, or zoomed in since. Its view
    /// (and so a point from the picture) holds at 1x only.
    bool _zoomUnknown = true;
    QTimer _pollTimer;
    QTimer _answerTimer;
    QTimer _laserShotTimer;
    QTimer _laserTimeoutTimer;
    QElapsedTimer _sinceReply;
    QElapsedTimer _sinceAttitude;
    QElapsedTimer _cameraMovedAt;  // since the app last moved the camera, or saw its turn go on
    QElapsedTimer _angleSentAt;    // since the last turn to an angle was asked for
    int _pollCount = 0;

    Endpoint _configured;
    Endpoint _active;
    QList<Endpoint> _endpoints;  // configured first, then the probe targets

    skydroid::top::Options _options;
    bool _enabled = false;
    QString _host;
    int _port = 5000;
    skydroid::camera::Store _cameraStore;
    skydroid::camera::Profile _camera;  // the camera in use
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
    QString _thermalPalette;

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
    QString _lockMode;
    bool _lockByApp = false;
    skydroid::track::ObjectTracker _tracker;
    double _pendingU = 0.5;   // the box to lock, until the first picture comes
    double _pendingV = 0.5;
    double _pendingW = 0.0;   // 0: the usual size
    double _pendingH = 0.0;
    // The last pictures, for a lock that starts a moment ago (lockAtFound).
    struct KeptPicture
    {
        QImage pixels;
        double left = 0.0;  // where the video picture is inside it, in points
        double top = 0.0;
        double width = 0.0;
        double height = 0.0;
        qint64 atMs = 0;  // of _pictureClock
    };
    std::deque<KeptPicture> _kept;
    std::deque<KeptPicture> _finderKept;  // the finder's pictures, at the size of those (keepFinderPicture)
    bool _keepPictures = false;
    QElapsedTimer _pictureClock;
    // The finder's check of the lock (task AC).
    skydroid::check::LockCheck _check;
    bool _lockDoubt = false;           // the finder has not seen the object on the box for some looks
    struct LockBoxAt
    {
        qint64 atMs = 0;  // the picture's time, of _pictureClock
        skydroid::check::Box box;
        bool seen = false;
    };
    std::deque<LockBoxAt> _lockBoxes;  // the box in each of the last pictures
    skydroid::check::Box _lastSeenBox;  // where the app saw the object last
    qint64 _lastSeenAtMs = -1;          // and when, of _pictureClock (-1: not yet)
    qint64 _pictureAtMs = 0;            // the time of the picture the tracker has now
    QElapsedTimer _foundAgainAt;        // since the finder found the object again
    bool _lockSeen = false;
    bool _lockBoxValid = false;
    double _lockBoxU = 0.5;
    double _lockBoxV = 0.5;
    double _lockBoxW = 0.0;
    double _lockBoxH = 0.0;
    double _lockOffDeg = 0.0;
    QString _lockNumbers;
    QElapsedTimer _pictureAge;     // since the last picture for the tracker
    double _pictureRate = 10.0;    // pictures a second, lately
    QElapsedTimer _lockLost;       // since the object was last seen
    double _followYawDps = 0.0;    // the speeds wanted now: right is +, up is +
    double _followPitchDps = 0.0;
    QElapsedTimer _followLease;    // since those speeds were set
    struct FollowSent
    {
        qint64 atMs = 0;
        double yawDps = 0.0;
        double pitchDps = 0.0;
    };
    std::deque<FollowSent> _followSent;  // the last second of them
    // For Follow Me: the object's place on the ground.
    QElapsedTimer _placedAt;             // since the last place was given
    bool _shotAtLockedObject = false;    // the laser shot under way is the lock's own, at the object
    std::deque<double> _groundRises;     // from the last laser shots at the object, metres
    bool _groundKnown = false;
    double _groundRiseM = 0.0;           // the middle one of them
    QElapsedTimer _groundAt;             // since the last of them
    // A stopped video gives the same picture again and again.
    quint64 _pictureSignature = 0;
    QElapsedTimer _pictureSame;      // since the picture last changed
    bool _pictureSameAngles = false;  // the gimbal angles then, if the camera sent them
    double _pictureSameYaw = 0.0;
    double _pictureSamePitch = 0.0;
    bool _videoStopped = false;
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
    std::optional<double> _shotHeightM;
    skydroid::geo::LaserResult _target;
    QString _targetMessage;
    bool _targetFromPicture = false;
    double _targetSlantM = 0.0;
    bool _measuredByPicture = false;
    QString _pointMode;
    bool _thermalPicture = false;

    int _zoomStep = -1;
    int _zoomCount = 0;     // C12/C13: our zoom steps in from 1x
    int _zoomOutLeft = 0;   // steps still to send for "back to 1x"
    QTimer _zoomOutTimer;
    QTimer _lensStopTimer;  // the C20: its lens stops this long after the last zoom click
    QTimer _lensHomeTimer;  // the C20: the end of a "back to 1x" run
    QString _zoomReport;

    // The wait for a Viewpro's video after its sensor switch (_startVideoWait)
    QTimer _videoWaitTimer;
    QElapsedTimer _videoWaitClock;
    QTcpSocket *_videoProbe = nullptr;
    bool _videoWaitSawDown = false;  // a knock went unanswered: the camera's network went away

    // Photos and recording
    bool _recording = false;
    bool _recordConfirmed = false;
    std::optional<bool> _recordReported;  // the camera's last answer: it records
    QElapsedTimer _recordReportAt;        // since then
    bool _recordChecking = false;         // a press waits for the camera's answer
    bool _recordWanted = false;           // what that press asked for
    QElapsedTimer _recordPressAt;
    quint64 _recordPresses = 0;
    QTimer _recordVerdictTimer;
    bool _photoWaiting = false;           // a photo press waits for the camera's echo
    QTimer _photoVerdictTimer;
    std::optional<skydroid::top::SdCard> _sdCard;
    QElapsedTimer _sdCardAt;              // since its answer
    int _sdCardAsked = 0;                 // SDC questions since the last answer

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
