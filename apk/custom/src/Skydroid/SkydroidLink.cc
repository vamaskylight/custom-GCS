#include "SkydroidLink.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QSettings>
#include <QtCore/QStandardPaths>
#include <QtNetwork/QNetworkDatagram>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QUdpSocket>
#include <QtPositioning/QGeoCoordinate>
#include <QtQml/QJSEngine>

#ifdef Q_OS_ANDROID
#include <QtCore/QJniEnvironment>
#include <QtCore/QJniObject>
#include <QtCore/qcoreapplication_platform.h>
#endif

#include "Fact.h"
#include "FactGroup.h"
#include "MultiVehicleManager.h"
#include "Vehicle.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <type_traits>
#include <vector>

namespace top = skydroid::top;
namespace camera = skydroid::camera;

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
// The added cameras: a file each, in this folder of the app's own folder (as the map packs).
constexpr const char *kCameraFolder = "Cameras";
QString g_cameraFolder;  // given by tests and the screen preview
constexpr int kDefaultMaxSpeed = 20;     // deg/s at full deflection
constexpr int kMinMaxSpeed = 5;
constexpr int kMaxMaxSpeed = 60;         // GSY/GSP carry at most 63.5 deg/s

constexpr int kPollIntervalMs = 200;     // gimbal angle query, 5 per second
constexpr int kGaaHz = 5;                // VGCS turns the angle push on at 5 Hz
constexpr int kGaaEveryPollsWhenQuiet = 5;  // GAA again every second while no angles arrive
constexpr int kGaaEveryPolls = 50;          // and every 10 s anyway, in case the camera restarted
constexpr int kProbeFirstPoll = 8;          // other addresses 1.6 s after start,
constexpr int kProbeEveryPolls = 15;        // then every 3 s while no angles arrive
constexpr int kZoomPollEvery = 10;       // zoom step query every 2 s (a camera that reports it: C14 Pro)
constexpr int kAnswerCheckMs = 1000;
constexpr int kAnswerTimeoutMs = 3000;   // no reply this long = camera not answering
constexpr int kAttitudeFreshMs = 1500;   // older angles: send GAA again and try other addresses
constexpr int kLaserShotSettleMs = 120;  // VGCS _LRF_SLR_SHOT_SETTLE_S
constexpr int kLaserWaitMs = 1000;
constexpr int kZoomQueryAfterCmdMs = 350;  // VGCS _DZM_POLL_AFTER_CMD_S
constexpr int kZoomHomeExtraSteps = 5;   // "1x" steps out this many more than counted
constexpr int kZoomOutIntervalMs = 60;
constexpr int kTcpRetryMs = 2000;        // a TCP camera (C20) that is gone is tried again after this
constexpr int kTcpConnectMs = 3000;      // and a connection not made by then is given up
// Viewpro and SIYI (task AK): their rules want the status asked twice a second, and send some
// frames later (a zoom click's stop, the hold after a jog). Their clock starts at 1 s, as in the
// scripts they were checked with (VGCS takes a time of 0 as "never").
constexpr int kRulesPollMs = static_cast<int>(viewpro::Camera::kPollMs);
static_assert(viewpro::Camera::kPollMs == siyi::Camera::kPollMs, "one poll timer for both");
constexpr int kRulesDueMs = 25;
constexpr long long kRulesClockStartMs = 1000;
// "Back to 1x" on a Viewpro or SIYI: zoom out until the Viewpro says 1x, a SIYI this long (it does not say).
constexpr int kZoomHomeOutMs = 8000;
// The C20's lens (task AN, TOP's ZMC): it moves while it is told to, and gets its stop a moment
// after the last click. A held zoom button clicks at once, again 400 ms later, then every 120 ms
// (FlyViewCustomLayer.qml). So the first click waits well past that first gap (a shorter wait
// stopped the lens once at the start of every hold), and a click that comes while the lens moves
// waits well past 120 ms, but not long (so the lens stops soon after the button is let go). A tap
// moves the lens for the first wait. The margins are for a busy remote control, whose timers run late.
constexpr int kLensZoomPulseMs = 550;
constexpr int kLensZoomHeldMs = 300;

// Photos and recording (task AL). Both go to the camera's own SD card. While the camera answers,
// it is asked whether it records every 2 s (the RC's own button can start it too), and for its
// SD card every 10 s (every 2 s until it has answered once).
constexpr int kRecordPollEvery = 10;
constexpr int kRecordPollAt = 3;           // the first 0.6 s after start
constexpr int kSdCardPollEvery = 50;
constexpr int kSdCardPollEveryUnknown = 10;
constexpr int kSdCardPollAt = 4;
constexpr int kSdCardSilentAsks = 3;       // asked this often with no answer: the camera does not report it
constexpr int kSdCardFreshMs = 30000;      // an older answer is forgotten
constexpr int kSdCardLowMb = 100;          // less free: almost full (a minute of video or less)
constexpr int kSdCardAskAgainMs = 1000;    // asked again after a photo or a recording: its room changed
// After a record press: an answer this soon may answer a question asked before the press, so it
// is not taken. The camera is asked twice, and what it says then is the press's result.
constexpr int kRecordSettleMs = 500;
constexpr int kRecordAskAfterMs[] = {700, 1800};
constexpr int kRecordVerdictMs = 2600;
constexpr int kRecordFreshMs = 5000;       // an older answer no longer confirms the state on screen
// The camera echoes a command it takes ("Valid command: echoed back"), the photo's within this.
constexpr int kPhotoVerdictMs = 1500;

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
// A hold-position wheel's range, inside the camera's own limits (the C13 tilts up to +10).
constexpr double kWheelPitchMinDeg = -90.0;  // wheel at 1000 us
constexpr double kWheelPitchMaxDeg = 30.0;   // wheel at 2000 us
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
// with the camera's view (CameraProfile.h; for the C12 and C13 the C13's
// calibrated 83.4 x 46.9 degrees), and the C13's GAC yaw running opposite to
// the picture (_LRF_C13_NEGATE_IMAGE_YAW).
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
// The object not seen this long: the lock ends. 2 s until task AC: a person the client
// locked was behind a thin tree for about 6 s (video 2026-10-19), and the developer chose
// 8 s, so the finder can find them again when they come out (LockCheck.h).
constexpr int kLockLostMs = 8000;
constexpr double kLockOnCrossDeg = 1.5;      // the laser measures the object only this near the cross
constexpr double kLockTurningDeg = 2.0;      // further off, the state says the camera is turning to it
constexpr double kLockDefaultBox = 0.12;     // a tap: a square box of this share of the picture's height
constexpr int kLockPictureMinSide = 64;      // a smaller picture is not looked at
// A lock on a marked object starts on the picture the mark was found in. Pictures are kept
// this long (a mark is not shown longer), and no more than this many of them. A kept picture
// counts as "of that moment" when it is no further than this from it: at 14 pictures a
// second the nearest one is at most 36 ms away.
constexpr int kKeepPicturesMs = 2500;
constexpr int kKeepPicturesMost = 40;
constexpr int kFoundPictureGapMs = 120;
// The finder's own pictures are kept too (at the size of the others), for a lock that starts
// at a moment with no other picture near it: with 9 to 10 pictures a second, unevenly, there
// was none in the preview on a busy PC (2026-10-20), and a lock on the mark of a walker
// started on the mark's old place. No more than this many: the finder looks about twice a second.
constexpr int kFinderPicturesMost = 6;
// The box in each picture is kept this long, for the finder's looks at those pictures (task AC).
constexpr int kLockBoxesKeepMs = 3000;
// After the finder found the object again, the lock's state says so this long.
constexpr int kFoundAgainNoteMs = 3000;
// A video that has stopped still gives pictures: the last one, again and again. While the
// camera turns, the picture must change. The same picture this long, while the gimbal turned
// this far since it came (or, with no gimbal angles, while the app asks it to turn): the
// video has stopped. A slow video shows a picture for less (a picture a second, or more).
constexpr int kVideoStoppedMs = 500;
constexpr double kVideoStoppedTurnDeg = 0.5;
// A picture of one flat colour (each colour's bytes this near each other) is not a picture
// of the scene: the video shows nothing (not started, a restart, or a phone that cannot copy
// its video picture). The lock takes it as no picture at all.
constexpr int kBlankPictureSpread = 2;

// Follow Me needs the locked object's place on the ground (objectPlaced), for the pictures
// it is found in, at most five a second.
constexpr int kPlaceEveryMs = 180;
// How sure such a place is. The camera's angle to the object (the gimbal's angle, the lens,
// the box) is good to about half a degree, and the height above the ground at the object to
// about 2 m (the barometer, and ground that is not level). A laser range on the object takes
// most of both away. No place is taken as surer than a metre.
constexpr double kPlaceAngleSigmaDeg = 0.5;
constexpr double kPlaceHeightSigmaM = 2.0;
constexpr double kPlaceAngleSigmaLaserDeg = 0.25;
constexpr double kPlaceHeightSigmaLaserM = 0.7;
constexpr double kPlaceLeastSigmaM = 1.0;
// How high the ground at the object is against the take-off point: the laser's range and the
// camera's angle give how far the object is below the drone. The middle one of the last three
// shots is taken, so one shot that missed the object changes nothing. It counts this long.
// Only ground that is HIGHER than the take-off point is taken from the laser. Higher ground
// means the object is nearer than level ground says, and that must be known. A shot that
// passes the object and lands far behind it says "much lower ground", which would put the
// object further away than it is, and the drone would be flown nearer to it.
constexpr int kGroundShots = 3;
constexpr int kGroundFreshMs = 15000;

// A point from the picture is placed by the camera's angles of this moment, but the video
// shows the scene a little late. So after the app moved the camera (a finger, a wheel, a
// button, a turn to a tap, the lock), a tap for such a point waits this long.
constexpr int kStillBeforePictureMs = 1000;
// A turn to an angle goes on after its command (100 degrees take 3.3 s). For this long after
// one, gimbal angles that still change count as that turn. Later changes are not the app's
// doing (the drone turning under a steady camera) and do not hold a tap back.
constexpr int kAngleTurnMaxMs = 5000;
constexpr double kAngleTurnStepDeg = 0.1;

// The thermal lens. The thermal stream is a picture of its own (the C13's is 640 x 512,
// "hevc, 640x512" in the client's VGCS log of 2026-10-08), so a place in it is a place on
// the thermal lens, and the day camera's zoom does not narrow it. Each camera's thermal lens
// is in its profile (CameraProfile.cc). When it is known only from a data sheet (the C12's
// and the C13's), it is good enough to turn the camera and to follow a locked object, not to
// place a point from the picture: on its thermal picture that is the cross only. A tap this
// near the middle is the cross.
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

/// Two sentences, or the first alone when the second is empty.
QString joined(const QString &first, const QString &second)
{
    return second.isEmpty() ? first : first + QLatin1Char(' ') + second;
}

/// A size the camera gives in MB, as the screen shows it: "850 MB", "12.5 GB".
QString sizeText(int mb)
{
    if (mb < 1024) {
        return QStringLiteral("%1 MB").arg(mb);
    }
    return QStringLiteral("%1 GB").arg(mb / 1024.0, 0, 'f', 1);
}

/// The value encodeSpeed2 sends, in 0.5 deg/s units.
int speedUnits(double degPerSecond)
{
    return static_cast<int>(std::clamp<long long>(top::pyRound(degPerSecond / 0.5), -127, 127));
}

#ifdef Q_OS_ANDROID
QString pathOf(const QJniObject &file)
{
    if (!file.isValid()) {
        return QString();
    }
    return file.callObjectMethod("getAbsolutePath", "()Ljava/lang/String;").toString();
}
#endif

/// The app's folder of added cameras: on Android Android/data/com.vama.gcs/files/Cameras,
/// which a PC reaches by USB without any permission (as the map packs, MapPackLoader).
QString appCameraFolder()
{
    QString own;
#ifdef Q_OS_ANDROID
    QJniEnvironment env;
    const QJniObject context = QNativeInterface::QAndroidApplication::context();
    if (context.isValid()) {
        own = pathOf(context.callObjectMethod("getExternalFilesDir", "(Ljava/lang/String;)Ljava/io/File;", static_cast<jobject>(nullptr)));
    }
    (void) env.checkAndClearExceptions();
#endif
    if (own.isEmpty()) {
        own = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    }
    return own + QLatin1Char('/') + QLatin1String(kCameraFolder);
}

/// A camera for the camera editor and the screen: the fields of its file, and more.
QVariantMap cameraMap(const camera::Profile &profile)
{
    QVariantMap map = camera::toJson(profile).toVariantMap();
    map.insert(QStringLiteral("id"), profile.id);
    map.insert(QStringLiteral("builtIn"), profile.builtIn);
    map.insert(QStringLiteral("thermal"), profile.hasThermal());
    return map;
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
    _lensStopTimer.setSingleShot(true);
    _lensStopTimer.setInterval(kLensZoomPulseMs);
    _lensHomeTimer.setSingleShot(true);
    _lensHomeTimer.setInterval(kZoomHomeOutMs);
    _aimTimer.setInterval(100);
    _lockTimer.setInterval(kLockTickMs);
    _recordVerdictTimer.setSingleShot(true);
    _recordVerdictTimer.setInterval(kRecordVerdictMs);
    _photoVerdictTimer.setSingleShot(true);
    _photoVerdictTimer.setInterval(kPhotoVerdictMs);
    _tcpRetryTimer.setSingleShot(true);
    _tcpRetryTimer.setInterval(kTcpRetryMs);
    _tcpConnectTimer.setSingleShot(true);
    _tcpConnectTimer.setInterval(kTcpConnectMs);
    _dueTimer.setInterval(kRulesDueMs);
    _rulesClock.start();
    _pictureClock.start();

    connect(&_pollTimer, &QTimer::timeout, this, &SkydroidLink::_poll);
    connect(&_answerTimer, &QTimer::timeout, this, &SkydroidLink::_checkAnswering);
    connect(&_laserShotTimer, &QTimer::timeout, this, &SkydroidLink::_laserReadAfterShot);
    connect(&_laserTimeoutTimer, &QTimer::timeout, this, &SkydroidLink::_laserFinish);
    connect(&_motionTimer, &QTimer::timeout, this, &SkydroidLink::_motionTick);
    connect(&_wheelAngleTimer, &QTimer::timeout, this, &SkydroidLink::_sendWheelAngles);
    connect(&_detectTimer, &QTimer::timeout, this, &SkydroidLink::_finishWheelDetect);
    connect(&_zoomOutTimer, &QTimer::timeout, this, &SkydroidLink::_zoomOutTick);
    connect(&_lensStopTimer, &QTimer::timeout, this, [this]() { _send(top::buildLensZoom("stop", _options)); });
    connect(&_lensHomeTimer, &QTimer::timeout, this, &SkydroidLink::_lensHomeDone);
    connect(&_aimTimer, &QTimer::timeout, this, &SkydroidLink::_aimTick);
    connect(&_lockTimer, &QTimer::timeout, this, &SkydroidLink::_lockTick);
    connect(&_recordVerdictTimer, &QTimer::timeout, this, &SkydroidLink::_recordVerdict);
    connect(&_photoVerdictTimer, &QTimer::timeout, this, &SkydroidLink::_photoVerdict);
    connect(&_tcpRetryTimer, &QTimer::timeout, this, &SkydroidLink::_syncTcp);
    connect(&_tcpConnectTimer, &QTimer::timeout, this, &SkydroidLink::_tcpLost);
    connect(&_dueTimer, &QTimer::timeout, this, &SkydroidLink::_sendDue);

    MultiVehicleManager *manager = MultiVehicleManager::instance();
    connect(manager, &MultiVehicleManager::activeVehicleChanged, this, &SkydroidLink::_activeVehicleChanged);
    _activeVehicleChanged(manager->activeVehicle());

    // The cameras first: the saved camera can be an added one.
    _cameraStore.setFolder(g_cameraFolder.isEmpty() ? appCameraFolder() : g_cameraFolder);
    _cameraStore.load();
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

// QML calls create() only for a singleton that cannot be made without an argument (see the
// constructor in SkydroidLink.h). Else the screen gets a link of its own.
static_assert(!std::is_default_constructible_v<SkydroidLink>,
              "QML would make its own SkydroidLink instead of using SkydroidLink::create()");

QStringList SkydroidLink::models() const
{
    QStringList ids;
    for (const camera::Profile &p : _cameraStore.profiles()) {
        ids.append(p.id);
    }
    return ids;
}

QStringList SkydroidLink::modelNames() const
{
    // VAMA sells the built-in Skydroid cameras under its own names (client, 2026-10-06).
    // Only the screen uses them: the code, the saved settings and VGCS keep Skydroid's
    // names, which the protocol notes are written against.
    QStringList names;
    for (const camera::Profile &p : _cameraStore.profiles()) {
        names.append(p.name);
    }
    return names;
}

QVariantMap SkydroidLink::camera() const
{
    return cameraMap(_camera);
}

QVariantList SkydroidLink::cameras() const
{
    QVariantList list;
    for (const camera::Profile &p : _cameraStore.profiles()) {
        QVariantMap row;
        row.insert(QStringLiteral("id"), p.id);
        row.insert(QStringLiteral("name"), p.name);
        row.insert(QStringLiteral("builtIn"), p.builtIn);
        list.append(row);
    }
    return list;
}

QVariantMap SkydroidLink::cameraProfile(const QString &id) const
{
    const camera::Profile *p = _cameraStore.find(id);
    return p ? cameraMap(*p) : QVariantMap();
}

void SkydroidLink::chooseCamera(const QString &id)
{
    const camera::Profile *p = _cameraStore.find(id);
    if (!p || id == _camera.id) {
        return;
    }
    _useCamera(*p, true);
}

QVariantMap SkydroidLink::saveCamera(const QVariantMap &fields)
{
    QVariantMap result;
    QString why;
    QString id;
    const std::optional<camera::Profile> profile = camera::fromFields(QJsonObject::fromVariantMap(fields), &why);
    if (profile) {
        why = _cameraStore.save(*profile, &id);
    }
    if (why.isEmpty()) {
        if (id == _camera.id) {
            // The camera in use was changed: it is used as saved, with its addresses.
            _useCamera(*_cameraStore.find(id), true);
        }
        emit camerasChanged();
    }
    result.insert(QStringLiteral("problem"), why);
    result.insert(QStringLiteral("id"), id);
    return result;
}

QString SkydroidLink::deleteCamera(const QString &id)
{
    const QString why = _cameraStore.remove(id);
    if (!why.isEmpty()) {
        return why;
    }
    if (id == _camera.id) {
        // The camera in use is gone: the V13, with the addresses as they are.
        _useCamera(*_cameraStore.find(camera::defaultId()), false);
    }
    emit camerasChanged();
    return QString();
}

void SkydroidLink::reloadCameras()
{
    _cameraStore.load();
    const camera::Profile *p = _cameraStore.find(_camera.id);
    if (!p) {
        p = _cameraStore.find(camera::defaultId());  // its file is gone
    }
    if (p->id != _camera.id || camera::toJson(*p) != camera::toJson(_camera)) {
        _useCamera(*p, false);
    }
    emit camerasChanged();
}

void SkydroidLink::setCameraFolder(const QString &folder)
{
    g_cameraFolder = folder;
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
    // An unknown camera (an added one whose file is gone) is the V13.
    const camera::Profile *saved = _cameraStore.find(settings.value(QStringLiteral("model"), camera::defaultId()).toString());
    _camera = saved ? *saved : *_cameraStore.find(camera::defaultId());
    _enabled = settings.value(QStringLiteral("enabled"), false).toBool();
    _host = settings.value(QStringLiteral("host"), QString::fromLatin1(kDefaultHost)).toString().trimmed();
    _port = settings.value(QStringLiteral("port"), kDefaultPort).toInt();
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
    // Empty: the camera's own.
    if (_dayVideoUrl.isEmpty()) {
        _dayVideoUrl = _camera.dayVideoUrl;
    }
    if (_thermalVideoUrl.isEmpty()) {
        _thermalVideoUrl = _camera.thermalVideoUrl;
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
    settings.setValue(QStringLiteral("model"), _camera.id);
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
    const camera::Profile *p = _cameraStore.find(model);
    if (!p || model == _camera.id) {
        return;
    }
    _useCamera(*p, false);
}

void SkydroidLink::_useCamera(const camera::Profile &profile, bool withAddresses)
{
    // A C20's lens that is still zooming gets its stop while it is still the camera: frames go out
    // in the camera's own way (TCP for the C20).
    if (_lensStopTimer.isActive() || _lensHomeTimer.isActive()) {
        _send(top::buildLensZoom("stop", _options));
        _lensStopTimer.stop();
        _lensHomeTimer.stop();
    }
    _camera = profile;
    _applyModel();
    if (_camera.thermalBySwitch) {
        setThermalPicture(false);  // taken to start on its day picture (the screen does not switch an address)
    }
    if (withAddresses) {
        _host = profile.host;
        _port = profile.port;
        _dayVideoUrl = profile.dayVideoUrl;
        _thermalVideoUrl = profile.thermalVideoUrl;
        _rebuildEndpoints();
    } else if (_thermalVideoUrl.isEmpty()) {
        // From a camera without a thermal picture: this one's, if it has one.
        _thermalVideoUrl = profile.thermalVideoUrl;
    }
    _syncTcp();  // another camera can take TOP over the other transport
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
    const QString u = url.trimmed().isEmpty() ? _camera.dayVideoUrl : url.trimmed();
    if (u == _dayVideoUrl) {
        return;
    }
    _dayVideoUrl = u;
    _saveSettings();
    emit settingsChanged();
}

void SkydroidLink::setThermalVideoUrl(const QString &url)
{
    const QString u = url.trimmed().isEmpty() ? _camera.thermalVideoUrl : url.trimmed();
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
    // As VGCS's profiles: g_frames_upper_header and laser_max_m (the C14 Pro's choices).
    _options = top::Options{};
    _options.gClassUpperHeader = _camera.upperGimbalHeader;
    _options.slrMaxDm = top::slrMaxDmForRange(_camera.laserMaxM);
    // A Viewpro's or SIYI's rules start again (task AK): the status twice a second.
    _viewpro = viewpro::Camera{};
    _siyi = siyi::Camera{};
    _pollTimer.setInterval(_top() ? kPollIntervalMs : kRulesPollMs);
    _syncDueTimer();
    // Another camera: nothing known about its zoom yet.
    const bool zoomShown = _zoomStep != -1 || _zoomCount != 0 || !_zoomReport.isEmpty() || _liveZoomX != 0.0 ||
                           !_zoomUnknown;
    _zoomStep = -1;
    _zoomCount = 0;
    _zoomReport.clear();
    _liveZoomX = 0.0;
    _liveFovHDeg = 0.0;
    _liveFovVDeg = 0.0;
    _zoomUnknown = true;
    if (zoomShown) {
        emit zoomChanged();
    }
    // Nor about its recording and its SD card.
    _forgetCameraReports();
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
    _syncTcp();  // a TCP camera at another address: connect there
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
    // A TCP camera gets none of them (_sendTo sends it nothing for another address).
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
    _syncTcp();
    _syncDueTimer();
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
    if (_socket && (_lensStopTimer.isActive() || _lensHomeTimer.isActive())) {
        _send(top::buildLensZoom("stop", _options));  // never leave the C20's lens moving
    }
    _lensStopTimer.stop();
    _lensHomeTimer.stop();
    _aimTimer.stop();
    if (_aimBusy) {
        _aimBusy = false;
        emit aimBusyChanged();
    }

    _pollTimer.stop();
    _answerTimer.stop();
    _laserShotTimer.stop();
    _laserTimeoutTimer.stop();
    _tcpRetryTimer.stop();
    _closeTcp();  // after the stop above: it goes out over TCP too
    if (_socket) {
        _socket->close();
        _socket->deleteLater();
        _socket = nullptr;
    }
    _syncDueTimer();
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
    _forgetCameraReports();
}

void SkydroidLink::_sendTo(const Endpoint &endpoint, const std::string &frame)
{
    // TOP frames never go to a Viewpro or SIYI camera: it gets its own (_sendBytes).
    if (!_socket || !_top() || frame.empty() || endpoint.address.isNull() || endpoint.port == 0) {
        return;
    }
    if (_camera.tcp) {
        // One connection, to the camera's own address. What would go to other addresses over
        // UDP (questions to other ports, zoom to more ports, the laser's other addresses) is not sent.
        if (endpoint == _active && _tcp && _tcp->state() == QAbstractSocket::ConnectedState) {
            _tcp->write(frame.data(), static_cast<qint64>(frame.size()));
        }
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
        if (_camera.protocol == camera::Protocol::Siyi) {
            // From the camera's own address only (it answers from its port 37260).
            const auto *bytes = reinterpret_cast<const std::uint8_t *>(data.constData());
            if (datagram.senderAddress().isEqual(_active.address) &&
                _siyi.feed(siyi::Bytes(bytes, bytes + data.size()), _nowMs()) && _siyi.attitude()) {
                _markAnswering();
                _takeAttitude(_siyi.attitude()->yawDeg, _siyi.attitude()->pitchDeg, _siyi.attitude()->rollDeg);
            }
            continue;
        }
        if (!_top()) {
            continue;  // a Viewpro speaks over TCP only
        }
        const auto frame = top::parseTpFrame(std::string(data.constData(), static_cast<size_t>(data.size())), _options);
        if (frame) {
            const quint16 port = datagram.senderPort() > 0 ? static_cast<quint16>(datagram.senderPort()) : 0;
            _handleFrame(*frame, _knownEndpointFor(datagram.senderAddress(), port));
        }
    }
}

void SkydroidLink::_syncTcp()
{
    // While the link runs and the camera takes its commands over TCP (the C20, a Viewpro): one
    // connection to its address.
    if (!_socket || !_camera.controlOverTcp()) {
        _tcpRetryTimer.stop();
        _closeTcp();
        return;
    }
    if (_tcp && _tcpHost == _host && _tcpPort == _port) {
        return;  // connected there, or on the way
    }
    _closeTcp();
    _openTcp();
}

void SkydroidLink::_openTcp()
{
    _tcp = new QTcpSocket(this);
    _tcpHost = _host;
    _tcpPort = _port;
    _tcpRx.clear();
    connect(_tcp, &QTcpSocket::connected, this, [this]() {
        _tcpConnectTimer.stop();
        // Small frames go out at once. Set here: Qt 6.11 drops an option set before
        // the connection exists (measured 2026-10-26), so a jog could wait for the camera's ACK.
        _tcp->setSocketOption(QAbstractSocket::LowDelayOption, 1);
        if (_camera.protocol == camera::Protocol::Viewpro) {
            _sendBytes(_viewpro.poll());  // its status at once
            return;
        }
        // A new connection: angle push on, and the angles, at once (as a new UDP link does).
        _send(top::buildGaaEnable(kGaaHz, _options));
        _send(top::buildGacQuery(_options));
    });
    connect(_tcp, &QTcpSocket::readyRead, this, &SkydroidLink::_readTcp);
    connect(_tcp, &QTcpSocket::disconnected, this, &SkydroidLink::_tcpLost);
    connect(_tcp, &QTcpSocket::errorOccurred, this, &SkydroidLink::_tcpLost);
    // A camera that is not there may not refuse at all: the system would try for a long time.
    // Started first, so an error at once (which closes the socket and stops it) leaves none running.
    _tcpConnectTimer.start();
    _tcp->connectToHost(_host, static_cast<quint16>(_port));
}

void SkydroidLink::_closeTcp()
{
    if (!_tcp) {
        return;
    }
    QTcpSocket *tcp = _tcp;
    _tcp = nullptr;
    _tcpConnectTimer.stop();
    tcp->disconnect(this);  // a close asked for is not a lost camera
    tcp->flush();           // a stop just written still goes out
    tcp->abort();
    tcp->deleteLater();
    _tcpRx.clear();
}

void SkydroidLink::_tcpLost()
{
    // The camera went away, or did not take the connection: again in a moment, while the link runs.
    _closeTcp();
    if (_socket && _camera.controlOverTcp()) {
        _tcpRetryTimer.start();
    }
}

void SkydroidLink::_readTcp()
{
    if (!_tcp) {
        return;
    }
    const QByteArray data = _tcp->readAll();
    if (_camera.protocol == camera::Protocol::Viewpro) {
        // Its rules take the stream in any pieces.
        const auto *bytes = reinterpret_cast<const std::uint8_t *>(data.constData());
        if (_viewpro.feed(viewpro::Bytes(bytes, bytes + data.size()), _nowMs())) {
            _takeViewproStatus();
        }
        return;
    }
    _tcpRx.append(data.constData(), static_cast<size_t>(data.size()));
    // A stream: half a frame or several can come in one read.
    for (const std::string &raw : top::takeFrames(_tcpRx)) {
        if (const auto frame = top::parseTpFrame(raw, _options)) {
            _handleFrame(*frame, _active);
        }
    }
}

void SkydroidLink::_markAnswering()
{
    _sinceReply.start();
    if (!_answering) {
        _answering = true;
        emit answeringChanged();
    }
}

void SkydroidLink::_takeAttitude(double yawDeg, double pitchDeg, double rollDeg)
{
    if (_attitudeValid && _angleSentAt.isValid() && _angleSentAt.elapsed() < kAngleTurnMaxMs &&
        std::max(std::abs(yawDeg - _yaw), std::abs(pitchDeg - _pitch)) >= kAngleTurnStepDeg) {
        _cameraMovedAt.start();  // the turn that was asked for is still going on
    }
    _sinceAttitude.start();
    _yaw = yawDeg;
    _pitch = pitchDeg;
    _roll = rollDeg;
    _attitudeValid = true;
    emit attitudeChanged();
}

void SkydroidLink::_handleFrame(const top::DecodedFrame &frame, const Endpoint &from)
{
    _markAnswering();

    if (frame.tag == "GAC" && frame.yaw && frame.pitch) {
        if (!(from == _active)) {
            // Angles from another address. Move there only when the current one
            // has gone quiet, so two answering addresses never take turns.
            if (_attitudeFresh()) {
                return;
            }
            _setActive(from);
        }
        _takeAttitude(*frame.yaw, *frame.pitch, frame.roll.value_or(0.0));
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
    } else if (frame.tag == "REC" && frame.ctrl == 'r' && frame.recording) {
        _recordAnswered(*frame.recording);
    } else if (frame.tag == "CAP" && frame.ctrl == 'w' && _photoWaiting) {
        // The camera echoes a command it takes.
        _photoWaiting = false;
        _photoVerdictTimer.stop();
        const QString problem = _sdCardProblem();
        if (_sdCard && _sdCard->totalMb == 0 && _sdCard->freeMb == 0) {
            emit cameraNote(tr("The camera has no SD card. The photo is not saved."), true);
        } else if (!problem.isEmpty()) {
            emit cameraNote(joined(tr("Photo taken."), problem), true);
        } else {
            emit cameraNote(tr("Photo taken. It is saved on the camera's SD card."), false);
        }
        QTimer::singleShot(kSdCardAskAgainMs, this, &SkydroidLink::_askSdCard);
    } else if (frame.tag == "SDC" && frame.ctrl == 'r' && frame.sdCard) {
        _sdCardAnswered(*frame.sdCard);
    }
}

// --- Viewpro and SIYI (task AK) ----------------------------------------------

long long SkydroidLink::_nowMs() const
{
    return kRulesClockStartMs + _rulesClock.elapsed();
}

void SkydroidLink::_sendBytes(const std::vector<std::uint8_t> &bytes)
{
    if (!_socket || bytes.empty()) {
        return;
    }
    const char *data = reinterpret_cast<const char *>(bytes.data());
    const qint64 size = static_cast<qint64>(bytes.size());
    if (_camera.controlOverTcp()) {
        // A Viewpro: over its one connection, once it is there.
        if (_tcp && _tcp->state() == QAbstractSocket::ConnectedState) {
            _tcp->write(data, size);
        }
        return;
    }
    if (!_active.address.isNull() && _active.port != 0) {
        _socket->writeDatagram(data, size, _active.address, _active.port);
    }
}

void SkydroidLink::_sendAllBytes(const std::vector<std::vector<std::uint8_t>> &frames)
{
    for (const std::vector<std::uint8_t> &frame : frames) {
        _sendBytes(frame);
    }
}

void SkydroidLink::_syncDueTimer()
{
    if (_socket && !_top()) {
        if (!_dueTimer.isActive()) {
            _dueTimer.start();
        }
    } else {
        _dueTimer.stop();
    }
}

void SkydroidLink::_sendDue()
{
    const long long now = _nowMs();
    if (_camera.protocol == camera::Protocol::Viewpro) {
        _sendAllBytes(_viewpro.tick(now));
    } else if (_camera.protocol == camera::Protocol::Siyi) {
        _sendAllBytes(_siyi.tick(now));
    }
}

void SkydroidLink::_takeViewproStatus()
{
    const std::optional<viewpro::Status> &status = _viewpro.status();
    if (!status) {
        return;
    }
    _markAnswering();
    // Its angles as VGCS takes them: yaw positive right, pitch positive up (VGCS read that off the
    // speed command in the field, 2026-07-30; for the angles it is not proven).
    _takeAttitude(status->attitude.yawDeg, status->attitude.pitchDeg, status->attitude.rollDeg);
    // Its zoom and its view, once it says them.
    const double zoomX = _viewpro.zoomX().value_or(0.0);
    const double hfov = _viewpro.hfovDeg().value_or(0.0);
    const double vfov = _viewpro.vfovDeg().value_or(0.0);
    if (zoomX != _liveZoomX || hfov != _liveFovHDeg || vfov != _liveFovVDeg) {
        _liveZoomX = zoomX;
        _liveFovHDeg = hfov;
        _liveFovVDeg = vfov;
        emit zoomChanged();
    }
    // Whether it records, as the camera says (the RC's own button can start it too).
    if (status->camera && _camera.record) {
        _recordAnswered(_viewpro.recording());
    }
}

void SkydroidLink::_sendAnglesOther(std::optional<double> yawDeg, std::optional<double> pitchDeg)
{
    // One frame for both axes: the other one where the camera is now (0 before it said).
    const double yaw = yawDeg.value_or(_attitudeValid ? _yaw : 0.0);
    const double pitch = pitchDeg.value_or(_attitudeValid ? _pitch : 0.0);
    if (_camera.protocol == camera::Protocol::Viewpro) {
        _sendBytes(_viewpro.setAngle(yaw, pitch));
    } else {
        _sendAllBytes(_siyi.setAngle(yaw, pitch));
    }
    _angleSentAt.start();
    _cameraMovedAt.start();
}

bool SkydroidLink::_yawLeftPositive() const
{
    // The C13's GAC yaw runs opposite to the picture (VGCS _LRF_C13_NEGATE_IMAGE_YAW). VGCS takes a
    // Viewpro's and a SIYI's yaw as it comes (yaw_left_positive off). The screen's switch turns it round.
    const bool own = _top() ? kNegateImageYaw : false;
    return own != _reverseTapYaw;
}

void SkydroidLink::_poll()
{
    if (!_top()) {
        // A Viewpro: the heartbeat, which it answers with its status. A SIYI: the angle question.
        _sendBytes(_camera.protocol == camera::Protocol::Viewpro ? _viewpro.poll() : _siyi.poll());
        return;
    }
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
    // Photos and recording go to the camera's SD card: whether it records, and the card's room.
    if (_answering && _camera.record && (_pollCount % kRecordPollEvery) == kRecordPollAt) {
        _send(top::buildRecordQuery(_options));
    }
    if (_answering && (_camera.photo || _camera.record) &&
        (_pollCount % (_sdCard ? kSdCardPollEvery : kSdCardPollEveryUnknown)) == kSdCardPollAt) {
        _askSdCard();
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
    // Old answers about the recording and the SD card no longer say how things are.
    if (_recordReported && _recordReportAt.elapsed() >= kRecordFreshMs) {
        _recordReported.reset();
        _setRecording(_recording);
    }
    if (_sdCard && _sdCardAt.elapsed() >= kSdCardFreshMs) {
        _sdCard.reset();
        emit sdCardChanged();
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
    if (_camera.protocol == camera::Protocol::Viewpro) {
        _sendBytes(_viewpro.ptz(a.toStdString(), _nowMs()));
        return;
    }
    if (_camera.protocol == camera::Protocol::Siyi) {
        _sendAllBytes(_siyi.ptz(a.toStdString()));
        return;
    }
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

void SkydroidLink::_sendAngle(const char *axis, double degrees, double dps)
{
    if (!_top()) {
        // Both axes in one frame (two would undo each other): callers that turn both use _sendAnglesOther.
        const bool yaw = std::string(axis) == "GAY";
        _sendAnglesOther(yaw ? std::optional<double>(degrees) : std::nullopt,
                         yaw ? std::nullopt : std::optional<double>(degrees));
        return;
    }
    _sendGimbal(top::buildGimbalAngleAxis(axis, degrees, dps, _options));
    _angleSentAt.start();
    _cameraMovedAt.start();
}

void SkydroidLink::_sendSpeed(double yawDps, double pitchDps)
{
    if (!_top()) {
        // One frame for both axes. The link's speeds are VGCS's: yaw positive right, pitch positive up.
        if (speedUnits(yawDps) != 0 || speedUnits(pitchDps) != 0) {
            _cameraMovedAt.start();
        }
        if (_camera.protocol == camera::Protocol::Viewpro) {
            _sendBytes(_viewpro.gimbalSpeed(yawDps, pitchDps, _nowMs()));
        } else {
            _sendAllBytes(_siyi.gimbalSpeed(yawDps, pitchDps));
        }
        _lastYawUnits = speedUnits(yawDps);
        _lastPitchUnits = speedUnits(pitchDps);
        return;
    }
    // Like VGCS: one frame per moving axis (GSY yaw, GSP pitch), never the
    // combined GSM, which dropped yaw on a C12 in the field.
    const int yawUnits = speedUnits(yawDps);
    const int pitchUnits = speedUnits(pitchDps);
    if (yawUnits != 0 || pitchUnits != 0) {
        _cameraMovedAt.start();
    }
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
    if (!_top()) {
        // A Viewpro's rules pin the gimbal where it came to rest a moment later (its hold).
        if (_camera.protocol == camera::Protocol::Viewpro) {
            _sendBytes(_viewpro.gimbalSpeed(0.0, 0.0, _nowMs()));
        } else {
            _sendAllBytes(_siyi.gimbalSpeed(0.0, 0.0));
        }
        _lastYawUnits = 0;
        _lastPitchUnits = 0;
        return;
    }
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
    if (!_top()) {
        // Their own "home" command, as VGCS sends it.
        if (_camera.protocol == camera::Protocol::Viewpro) {
            _sendBytes(_viewpro.center());
        } else {
            _sendAllBytes(_siyi.center());
        }
        _angleSentAt.start();
        _cameraMovedAt.start();
        return;
    }
    // One axis per frame, like the speed commands (a combined frame dropped
    // yaw on a C12 in the field).
    _sendAngle("GAY", 0.0, kAngleMoveDps);
    _sendAngle("GAP", 0.0, kAngleMoveDps);
}

void SkydroidLink::centerYaw()
{
    _endLockByHand();
    _sendAngle("GAY", 0.0, kCenterYawDps);  // a Viewpro or SIYI keeps its tilt
}

void SkydroidLink::pointDown()
{
    _endLockByHand();
    if (!_top()) {
        // A Viewpro's own "look down"; a SIYI keeps its yaw and tilts to -90 (VGCS's default).
        if (_camera.protocol == camera::Protocol::Viewpro) {
            _sendBytes(_viewpro.lookDown());
        } else {
            _sendAllBytes(_siyi.pointDown());
        }
        _angleSentAt.start();
        _cameraMovedAt.start();
        return;
    }
    // Pitch -90 is straight down: the C13 tilts from -90 to +10, and its
    // gimbal angles read negative when looking down (VGCS tries -90 first too).
    _sendAngle("GAP", -90.0, kAngleMoveDps);
}

void SkydroidLink::currentFov(double &horizontalDeg, double &verticalDeg) const
{
    // A Viewpro's view as it says it (VGCS uses that too).
    if (_camera.zoom == camera::Zoom::Live && _liveFovHDeg > 0.0 && _liveFovVDeg > 0.0) {
        horizontalDeg = _liveFovHDeg;
        verticalDeg = _liveFovVDeg;
        return;
    }
    // The lens and crop for the step the camera reported (C14 Pro), or narrowed by the zoom
    // we counted (C12, C13), as VGCS does.
    camera::dayFov(_camera, _zoomCount, _zoomStep, horizontalDeg, verticalDeg);
}

bool SkydroidLink::pictureFov(double &horizontalDeg, double &verticalDeg) const
{
    if (!_thermalPicture) {
        currentFov(horizontalDeg, verticalDeg);
        // A SIYI's view holds at 1x only, and its zoom is known only after "back to 1x".
        return !(_camera.zoom == camera::Zoom::Pulse && _zoomUnknown);
    }
    horizontalDeg = _camera.thermalFovHDeg;
    verticalDeg = _camera.thermalFovVDeg;
    return _camera.thermalFovSure;
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

void SkydroidLink::switchThermal(bool on)
{
    if (!_camera.thermalBySwitch || !_socket) {
        return;
    }
    if (_camera.protocol == camera::Protocol::Viewpro) {
        _sendBytes(_viewpro.setSensor(on));
    }
    setThermalPicture(on);
    // A Viewpro drops its whole network while it switches: 5 to 40 s in VGCS's field tests.
    emit cameraNote(on ? tr("The camera switches to its thermal picture. Its video and link stop for some seconds.")
                       : tr("The camera switches to its day picture. Its video and link stop for some seconds."),
                    false);
}

void SkydroidLink::_measureFromPicture(double u, double v)
{
    // One result shows at a time: another point ends a lock, as a tap by laser does.
    if (_lockActive || _lockBusy) {
        _endLock(tr("Lock stopped, because another point was tapped."));
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
    } else if (_cameraMovedAt.isValid() && _cameraMovedAt.elapsed() < kStillBeforePictureMs) {
        // The picture tapped is older than the angles: it shows where the camera looked before.
        whyNot = tr("No position: the camera is still moving. Hold it still for a second, then tap again.");
    } else if (_sampleVehiclePose(view, whyNot)) {
        double hfov = 0.0;
        double vfov = 0.0;
        bool lensKnown = pictureFov(hfov, vfov);
        if (!lensKnown) {
            // The cross is the cross whatever the lens.
            if (std::abs(u - 0.5) <= kCrossTapNorm && std::abs(v - 0.5) <= kCrossTapNorm) {
                u = 0.5;
                v = 0.5;
                lensKnown = true;
            } else if (_thermalPicture) {
                whyNot = tr("On the thermal picture the app can place the cross only: the thermal lens of this camera "
                            "is not known exactly. Put the cross on the point, then tap the cross.");
            } else {
                whyNot = tr("This camera does not say how far it zoomed, so the app can place the cross only. Zoom back "
                            "to 1x first (four corner arrows), or put the cross on the point and tap the cross.");
            }
        }
        if (lensKnown) {
            view.gimbalYawDeg = _yaw;
            view.gimbalYawLeftPositive = _yawLeftPositive();
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
    // A camera without a laser: always from the picture.
    if (_pointMode == QStringLiteral("picture") || !_camera.laser) {
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
    // The lens of the picture tapped: on the thermal picture the thermal lens.
    double hfov = 0.0;
    double vfov = 0.0;
    pictureFov(hfov, vfov);
    const double dyawImage = (std::clamp(u, 0.0, 1.0) - 0.5) * hfov;    // right is +
    const double dpitchImage = (std::clamp(v, 0.0, 1.0) - 0.5) * vfov;  // down is +
    const bool negate = _yawLeftPositive();
    // The camera's own limits (C13: up to +10, C12: +30, C14 Pro: +60 and 120 either way).
    _aimPointYaw = _yaw + (negate ? -dyawImage : dyawImage);
    _aimPointPitch = _pitch - dpitchImage;
    _aimYaw = std::clamp(_aimPointYaw, -_camera.yawLimitDeg, _camera.yawLimitDeg);
    _aimPitch = std::clamp(_aimPointPitch, _camera.pitchMinDeg, _camera.pitchMaxDeg);
    return std::max(std::abs(dyawImage), std::abs(dpitchImage));
}

void SkydroidLink::_sendAimAngles()
{
    if (!_top()) {
        _sendAnglesOther(_aimYaw, _aimPitch);
        return;
    }
    _sendAngle("GAY", _aimYaw, kAngleMoveDps);
    _sendAngle("GAP", _aimPitch, kAngleMoveDps);
}

void SkydroidLink::_startTurn()
{
    _sendAimAngles();
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
    pictureFov(hfov, vfov);
    const bool negate = _yawLeftPositive();
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
        _sendAimAngles();
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

bool SkydroidLink::lockAt(double u, double v)
{
    return lockAtBox(u, v, 0.0, 0.0);
}

bool SkydroidLink::lockAtBox(double u, double v, double w, double h)
{
    if (!std::isfinite(u) || !std::isfinite(v) || !std::isfinite(w) || !std::isfinite(h) || _lockBusy) {
        return false;  // (while a lock starts the lock button stops it, so no tap gets here then)
    }
    if (_aimBusy) {
        // VAMA (client video 2026-10-15): the screen must say why a tap locked nothing.
        // Before, this went back without a word, and the tap looked lost.
        _setLockMessage(tr("The camera is still turning to the point you tapped. Lock again when it has stopped."));
        return false;
    }
    const bool wasLocked = _lockActive;
    if (_lockActive) {
        _endLock(QString());  // a new object replaces the old one
    }
    if (!_socket) {
        _setLockMessage(tr("Camera link is off. Turn it on in the camera settings."));
        return false;
    }
    // A camera without a tracker of its own: the app follows, whatever the setting says.
    if (_lockMode == QStringLiteral("camera") && _camera.ownTracker) {
        return _lockByCamera(u, v, wasLocked);
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
    _pictureSame.invalidate();
    _videoStopped = false;
    _lockLost.invalidate();
    _followSent.clear();
    _forgetGround();  // another object: the ground under it is not known
    // The finder's check (task AC): the finder's first looks may say what a drawn box holds.
    // (A lock on a mark knows it: lockAtFound.)
    _check.start();
    _lockDoubt = false;
    _lockBoxes.clear();
    _lastSeenAtMs = -1;
    _foundAgainAt.invalidate();
    _lockElapsed.start();
    // The camera's own tracker must not pull against the app.
    _send(top::buildSumTrack(false, _options));
    _lockTimer.start();
    emit lockBoxChanged();
    _setLockMessage(tr("Locking..."));
    return true;
}

bool SkydroidLink::_lockByCamera(double u, double v, bool wasLocked)
{
    if (!_attitudeValid) {
        _setLockMessage(tr("No gimbal angles from the camera, so it cannot turn to the object."));
        return false;
    }
    // A new object: the last measurement no longer applies.
    _clearMeasurement();

    _lockByApp = false;
    _lockBusy = true;
    if (_setAimFor(u, v) >= kLockTurnFirstDeg) {
        _startTurn();
        _setLockMessage(tr("Turning to the object..."));
        return true;
    }
    // Near the centre already: lock where it was picked, without turning.
    if (!wasLocked) {
        _armLock(u, v);
        return true;
    }
    _setLockMessage(tr("Locking..."));
    QTimer::singleShot(kLockAfterStopMs, this, [this, u, v]() {
        if (_lockBusy && !_lockByApp && !_aimTimer.isActive()) {
            _armLock(u, v);
        }
    });
    return true;
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

// True for a picture of one flat colour (see kBlankPictureSpread).
static bool pictureIsBlank(const QImage &pixels, int left, int top, int width, int height, int channels)
{
    const int x0 = std::clamp(left, 0, pixels.width());
    const int y0 = std::clamp(top, 0, pixels.height());
    const int x1 = std::clamp(left + width, x0, pixels.width());
    const int y1 = std::clamp(top + height, y0, pixels.height());
    int lowest[3] = {255, 255, 255};
    int highest[3] = {0, 0, 0};
    for (int y = y0; y < y1; ++y) {
        const uchar *point = pixels.constScanLine(y) + static_cast<qsizetype>(x0) * channels;
        for (int x = x0; x < x1; ++x, point += channels) {
            for (int c = 0; c < channels; ++c) {
                lowest[c] = std::min<int>(lowest[c], point[c]);
                highest[c] = std::max<int>(highest[c], point[c]);
                if (highest[c] - lowest[c] > kBlankPictureSpread) {
                    return false;
                }
            }
        }
    }
    return true;
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
    // Where the video picture is inside this image.
    const double left = std::clamp(pictureX, 0.0, 1.0) * pixels.width();
    const double top = std::clamp(pictureY, 0.0, 1.0) * pixels.height();
    const double width = std::clamp(pictureW, 0.0, 1.0) * pixels.width();
    const double height = std::clamp(pictureH, 0.0, 1.0) * pixels.height();
    if (!std::isfinite(left + top + width + height) || width < kLockPictureMinSide ||
        height < kLockPictureMinSide / 2) {
        return;  // too small to find anything in
    }
    const int areaLeft = static_cast<int>(left);
    const int areaTop = static_cast<int>(top);
    const int areaWidth = static_cast<int>(width);
    const int areaHeight = static_cast<int>(height);
    const int channels = grey ? 1 : 3;
    const bool blank = pictureIsBlank(pixels, areaLeft, areaTop, areaWidth, areaHeight, channels);
    if (_keepPictures && !blank) {
        _keepPicture(pixels, left, top, width, height);
    }
    if (!_lockByApp || !(_lockBusy || _lockActive)) {
        return;  // no lock of the app's: the picture was for keeping only
    }
    if (blank && _lockBusy) {
        // No picture of the scene yet. With none the camera's own tracker takes over (kLockPictureWaitMs).
        return;
    }
    if (blank || _videoHasStopped(pixels, areaLeft, areaTop, areaWidth, areaHeight, channels)) {
        // An old picture: the camera waits, as for an object not seen. With no new picture
        // the lock ends (kLockNoPictureEndMs), so this picture does not count as one.
        _setFollowSpeed(0.0, 0.0);
        if (!_videoStopped) {
            _videoStopped = true;
            _lockSeen = false;  // the object is not seen anew: the box says so, and the laser waits
            _lockMessage = tr("Locked, but the video has stopped. The camera waits.");
            emit lockBoxChanged();
            emit lockChanged();
        }
        return;
    }
    _videoStopped = false;
    if (_pictureAge.isValid()) {
        const double seconds = std::clamp(_pictureAge.elapsed() / 1000.0, 0.02, 1.0);
        _pictureRate = 0.8 * _pictureRate + 0.2 / seconds;
    }
    _pictureAge.start();
    _pictureAtMs = _pictureClock.elapsed();
    _trackPicture(pixels, left, top, width, height, true);
}

void SkydroidLink::_trackPicture(const QImage &pixels, double left, double top, double width, double height,
                                 bool act)
{
    const skydroid::track::Picture picture{pixels.constBits(), pixels.width(), pixels.height(),
                                           static_cast<int>(pixels.bytesPerLine()),
                                           pixels.format() == QImage::Format_Grayscale8 ? 1 : 3};
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
    if (act) {
        _follow(result, left, top, width, height);
    }
}

void SkydroidLink::_keepPicture(const QImage &pixels, double left, double top, double width, double height)
{
    // Pictures of another size or shape (the thermal picture came on) are of another view.
    if (!_kept.empty()) {
        const KeptPicture &last = _kept.back();
        if (last.pixels.size() != pixels.size() || last.pixels.format() != pixels.format() || last.left != left ||
            last.top != top || last.width != width || last.height != height) {
            _kept.clear();
            _finderKept.clear();
        }
    }
    KeptPicture kept;
    kept.pixels = pixels.copy();  // its own bytes: the picture given may be gone by the time it is used
    kept.left = left;
    kept.top = top;
    kept.width = width;
    kept.height = height;
    kept.atMs = _pictureClock.elapsed();
    _kept.push_back(kept);
    while (!_kept.empty() && (_kept.size() > static_cast<size_t>(kKeepPicturesMost) ||
                              kept.atMs - _kept.front().atMs > kKeepPicturesMs)) {
        _kept.pop_front();
    }
}

void SkydroidLink::keepPictures(bool keep)
{
    if (keep == _keepPictures) {
        return;
    }
    const bool wanted = lockWantsPictures();
    _keepPictures = keep;
    if (!keep) {
        _kept.clear();
        _finderKept.clear();
    }
    if (wanted != lockWantsPictures()) {
        emit lockChanged();
    }
}

void SkydroidLink::keepFinderPicture(const QImage &image, double pictureX, double pictureY, double pictureW,
                                     double pictureH)
{
    if (!_keepPictures || _kept.empty() || image.isNull()) {
        return;
    }
    // At the size of the pictures given for the lock, so that the tracker takes it as one of them.
    // The two are copies of one video item, at two sizes: the video picture is at the same part of each.
    const KeptPicture &last = _kept.back();
    const double width = last.pixels.width();
    const double height = last.pixels.height();
    if (!std::isfinite(pictureX + pictureY + pictureW + pictureH) ||
        std::abs(std::clamp(pictureX, 0.0, 1.0) * width - last.left) > 1.5 ||
        std::abs(std::clamp(pictureY, 0.0, 1.0) * height - last.top) > 1.5 ||
        std::abs(std::clamp(pictureW, 0.0, 1.0) * width - last.width) > 1.5 ||
        std::abs(std::clamp(pictureH, 0.0, 1.0) * height - last.height) > 1.5) {
        return;  // another view
    }
    KeptPicture kept;
    kept.pixels = image.scaled(last.pixels.size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation)
                      .convertToFormat(last.pixels.format());
    if (kept.pixels.isNull()) {
        return;
    }
    kept.left = last.left;
    kept.top = last.top;
    kept.width = last.width;
    kept.height = last.height;
    kept.atMs = _pictureClock.elapsed();
    _finderKept.push_back(kept);
    while (!_finderKept.empty() && (_finderKept.size() > static_cast<size_t>(kFinderPicturesMost) ||
                                    kept.atMs - _finderKept.front().atMs > kKeepPicturesMs)) {
        _finderKept.pop_front();
    }
}

bool SkydroidLink::lockAtFound(double u, double v, double w, double h, qint64 atMs,
                               std::optional<skydroid::find::Kind> kind, double score)
{
    // Copies of the pictures from that moment on (they share their bytes): a new lock may clear the kept ones.
    const std::vector<KeptPicture> pictures = _picturesFrom(atMs);
    if (!lockAtBox(u, v, w, h)) {
        return false;  // no lock now: the lock message says why
    }
    if (_lockByApp && kind) {
        _check.start(kind, score);  // what the finder took it for
    }
    if (pictures.empty() || !_lockByApp || !_lockBusy) {
        return true;  // mode "camera", or no picture of that moment: locked as a drawn box is
    }
    _playFrom(pictures);
    return true;
}

std::vector<SkydroidLink::KeptPicture> SkydroidLink::_picturesFrom(qint64 atMs) const
{
    // A picture given for the lock near that moment.
    int from = -1;
    qint64 nearest = kFoundPictureGapMs + 1;
    for (size_t i = 0; i < _kept.size(); ++i) {
        const qint64 gap = std::abs(_kept[i].atMs - atMs);
        if (gap < nearest) {
            nearest = gap;
            from = static_cast<int>(i);
        }
    }
    if (from >= 0) {
        return std::vector<KeptPicture>(_kept.begin() + from, _kept.end());
    }
    // None: the finder's own picture of that moment, then the ones given since (kFinderPicturesMost).
    const KeptPicture *finder = nullptr;
    for (const KeptPicture &kept : _finderKept) {
        const qint64 gap = std::abs(kept.atMs - atMs);
        if (gap < nearest) {
            nearest = gap;
            finder = &kept;
        }
    }
    if (!finder) {
        return {};
    }
    std::vector<KeptPicture> pictures{*finder};
    for (const KeptPicture &kept : _kept) {
        if (kept.atMs > finder->atMs) {
            pictures.push_back(kept);
        }
    }
    return pictures;
}

void SkydroidLink::_playFrom(const std::vector<KeptPicture> &pictures)
{
    // The tracker starts on the object where it was found, and follows it through the pictures
    // since, as it would have then. The camera is turned for the last of them only: the newest.
    for (size_t i = 0; i < pictures.size() && (_lockBusy || _lockActive); ++i) {
        const KeptPicture &kept = pictures[i];
        _pictureAtMs = kept.atMs;
        _trackPicture(kept.pixels, kept.left, kept.top, kept.width, kept.height, i + 1 == pictures.size());
    }
    // The camera swings over to the object only now. The tracker's first pictures (task Q's soft
    // start) are for that swing: the play-back, up to kKeepPicturesMost pictures, must not use
    // them up. A slow finder (as on the remote control) makes the mark older and the play-back longer.
    _tracker.settleAgain();
    if (_lockActive) {
        // The newest of them is the last picture that came: if none follows, the lock ends
        // as any lock does when the video stops (kLockNoPictureEndMs).
        _pictureAge.start();
    }
}

bool SkydroidLink::lockWantsLooks() const
{
    return _lockByApp && _lockActive && _check.wanted();
}

void SkydroidLink::finderLooked(const std::vector<skydroid::find::Found> &found, qint64 atMs, double aspect)
{
    if (!lockWantsLooks()) {
        return;
    }
    // The lock box in the picture the finder looked at.
    const LockBoxAt *at = nullptr;
    qint64 nearest = kFoundPictureGapMs + 1;
    for (const LockBoxAt &box : _lockBoxes) {
        const qint64 gap = std::abs(box.atMs - atMs);
        if (gap < nearest) {
            nearest = gap;
            at = &box;
        }
    }
    if (!at) {
        return;  // no picture of the lock near that moment
    }
    // Where the app had seen the object last by then, and how long before.
    skydroid::check::Box lastSeen;
    double sinceSeenS = 0.0;
    if (at->seen) {
        lastSeen = at->box;
    } else {
        bool known = false;
        for (auto it = _lockBoxes.rbegin(); it != _lockBoxes.rend(); ++it) {
            if (it->seen && it->atMs <= atMs) {
                lastSeen = it->box;
                sinceSeenS = static_cast<double>(atMs - it->atMs) / 1000.0;
                known = true;
                break;
            }
        }
        if (!known && _lastSeenAtMs >= 0 && _lastSeenAtMs <= atMs) {
            lastSeen = _lastSeenBox;
            sinceSeenS = static_cast<double>(atMs - _lastSeenAtMs) / 1000.0;
        }
    }
    const skydroid::check::Box box = at->box;
    const bool seen = at->seen;
    const bool wanted = _check.wanted();
    const skydroid::check::Answer answer = _check.look(found, box, seen, lastSeen, sinceSeenS, aspect);
    switch (answer.verdict) {
    case skydroid::check::Verdict::OnTheBox:
    case skydroid::check::Verdict::Nothing:
        // The object is on the box (or cannot be checked): no doubt. The next picture shows it as seen.
        _lockDoubt = false;
        break;
    case skydroid::check::Verdict::NotThere:
        if (!_lockDoubt) {
            // The box holds something else: at once the box says NOT SEEN. With the next picture the
            // camera waits, and the lock ends as for an object not seen, unless it is found again.
            _lockDoubt = true;
            _lockSeen = false;
            _lockMessage = _doubtMessage();
            emit lockBoxChanged();
            emit lockChanged();
        }
        break;
    case skydroid::check::Verdict::FoundAgain:
        _lockAgainOn(answer.again, atMs);
        break;
    case skydroid::check::Verdict::Missed:
        break;
    }
    if (wanted != _check.wanted()) {
        emit lockChanged();  // a drawn box that holds no person or vehicle: no more looks
    }
}

void SkydroidLink::_lockAgainOn(const skydroid::check::Box &box, qint64 atMs)
{
    const std::vector<KeptPicture> pictures = _picturesFrom(atMs);
    if (pictures.empty()) {
        return;  // no picture of that moment: a later look may find it again
    }
    // The tracker starts again on the object where the finder found it, and goes through the
    // pictures since, as a lock on a mark does. The lock itself goes on.
    _pendingU = std::clamp(box.cx, 0.0, 1.0);
    _pendingV = std::clamp(box.cy, 0.0, 1.0);
    _pendingW = std::clamp(box.w, 0.0, 0.9);
    _pendingH = std::clamp(box.h, 0.0, 0.9);
    _lockActive = false;
    _lockBusy = true;
    _lockDoubt = false;
    _lockLost.invalidate();
    _foundAgainAt.start();
    _playFrom(pictures);
}

QString SkydroidLink::_doubtMessage() const
{
    return _check.kind() == skydroid::find::Kind::Vehicle
               ? tr("Locked, but the app sees no vehicle in the box any more. The camera waits.")
               : tr("Locked, but the app sees no person in the box any more. The camera waits.");
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

bool SkydroidLink::_videoHasStopped(const QImage &pixels, int left, int top, int width, int height, int channels)
{
    // A stopped video gives the very same bytes; a live one never does (its noise alone
    // changes them). FNV-1a over the picture, without the bars beside it.
    quint64 signature = 1469598103934665603ULL;
    const int x0 = std::clamp(left, 0, pixels.width());
    const int y0 = std::clamp(top, 0, pixels.height());
    const int x1 = std::clamp(left + width, x0, pixels.width());
    const int y1 = std::clamp(top + height, y0, pixels.height());
    const qsizetype count = static_cast<qsizetype>(x1 - x0) * channels;
    for (int y = y0; y < y1; ++y) {
        const uchar *row = pixels.constScanLine(y) + static_cast<qsizetype>(x0) * channels;
        for (qsizetype i = 0; i < count; ++i) {
            signature = (signature ^ row[i]) * 1099511628211ULL;
        }
    }
    if (!_pictureSame.isValid() || signature != _pictureSignature) {
        _pictureSignature = signature;
        _pictureSame.start();
        _pictureSameAngles = _attitudeFresh();
        _pictureSameYaw = _yaw;
        _pictureSamePitch = _pitch;
        return false;
    }
    if (_videoStopped) {
        return true;  // still the same picture
    }
    if (!_pictureSameAngles && _attitudeFresh()) {
        // The gimbal angles came after the picture: the turn counts from now.
        _pictureSameAngles = true;
        _pictureSameYaw = _yaw;
        _pictureSamePitch = _pitch;
    }
    if (_lockBusy || _pictureSame.elapsed() < kVideoStoppedMs) {
        return false;
    }
    if (_pictureSameAngles && _attitudeFresh()) {
        // The gimbal turned, and the picture did not change.
        return std::max(std::abs(_yaw - _pictureSameYaw), std::abs(_pitch - _pictureSamePitch)) >= kVideoStoppedTurnDeg;
    }
    // No gimbal angles: the app asks the camera to turn, and the picture does not change.
    return _followYawDps != 0.0 || _followPitchDps != 0.0;
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
    // While the finder does not see the object on the box (task AC), what the tracker found is
    // something else: not the object.
    const bool seen = result.found && !_lockDoubt;
    _lockSeen = seen;
    _lockBoxValid = true;
    _lockBoxU = (result.box.cx - left) / width;
    _lockBoxV = (result.box.cy - top) / height;
    _lockBoxW = result.box.w / width;
    _lockBoxH = result.box.h / height;
    // The box in this picture, for the finder's looks at it.
    const skydroid::check::Box box{_lockBoxU, _lockBoxV, _lockBoxW, _lockBoxH};
    _lockBoxes.push_back(LockBoxAt{_pictureAtMs, box, seen});
    while (!_lockBoxes.empty() && _pictureAtMs - _lockBoxes.front().atMs > kLockBoxesKeepMs) {
        _lockBoxes.pop_front();
    }
    if (seen) {
        _lastSeenBox = box;
        _lastSeenAtMs = _pictureAtMs;
    }

    QString message;
    if (seen) {
        _lockLost.invalidate();
        // The lens of the picture the object was found in: on the thermal picture the thermal
        // lens (the day lens made the camera turn 1.7 to 1.9 times too fast there).
        double hfov = 0.0;
        double vfov = 0.0;
        pictureFov(hfov, vfov);
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
        _placeObject(right - turnedYaw, up - turnedPitch, hfov, vfov);
        const bool turning = _lockOffDeg > kLockTurningDeg;
        if (_foundAgainAt.isValid() && _foundAgainAt.elapsed() < kFoundAgainNoteMs) {
            // The finder found it again (task AC): said for a while, so the operator sees why.
            message = (_check.kind() == skydroid::find::Kind::Vehicle ? tr("Found the vehicle again.")
                                                                       : tr("Found the person again.")) +
                      QLatin1Char(' ') +
                      (turning ? tr("Turning the camera to it...") : tr("The camera follows it."));
        } else {
            message = turning ? tr("Locked. Turning the camera to the object...") : tr("Locked. The camera follows the object.");
        }
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
        message = _lockDoubt ? _doubtMessage() : tr("Locked, but the object is not seen right now.");
    }
    _lockNumbers = QStringLiteral("off %1 deg, speed %2 / %3 deg/s, match %4, colours %5, %6 pictures/s")
                       .arg(_lockOffDeg, 0, 'f', 1)
                       .arg(_followYawDps, 0, 'f', 1)
                       .arg(_followPitchDps, 0, 'f', 1)
                       .arg(result.psr, 0, 'f', 0)
                       .arg(result.alike, 0, 'f', 2)
                       .arg(_pictureRate, 0, 'f', 0) +
                   // The first pictures of a lock, with the tracker's rules of before task Q.
                   (result.starting ? QStringLiteral(", lock start") : QString());
    if (_check.kind()) {
        // The finder's check, for a field video (task AC).
        _lockNumbers += _lockDoubt           ? QStringLiteral(", check: not there")
                        : _check.misses() > 0 ? QStringLiteral(", check: missed %1").arg(_check.misses())
                                              : QStringLiteral(", check: ok");
    }
    emit lockBoxChanged();
    if (message != _lockMessage || wasSeen != _lockSeen) {
        _lockMessage = message;
        emit lockChanged();
    }
}

void SkydroidLink::_placeObject(double rightDeg, double upDeg, double hfovDeg, double vfovDeg)
{
    if (_placedAt.isValid() && _placedAt.elapsed() < kPlaceEveryMs) {
        return;
    }
    // Never on old gimbal angles, and not without the drone's own place and height.
    skydroid::geo::LaserInput view;
    QString why;
    const std::optional<double> height = _sampleHeight();
    if (!_attitudeFresh() || !height || !_sampleVehiclePose(view, why)) {
        return;
    }
    // The object's direction as a point of the picture the camera would give now. (What the
    // camera has turned since the picture was taken is off already, as for its speeds.)
    view.gimbalYawDeg = _yaw;
    view.gimbalYawLeftPositive = _yawLeftPositive();
    view.gimbalPitchDeg = _pitch;
    view.videoXNorm = std::clamp(0.5 + rightDeg / hfovDeg, 0.0, 1.0);
    view.videoYNorm = std::clamp(0.5 - upDeg / vfovDeg, 0.0, 1.0);
    view.cameraHfovDeg = hfovDeg;
    view.cameraVfovDeg = vfovDeg;
    // The ground at the object: level with the take-off point, or as much higher as the laser found it.
    const bool byLaser = _groundKnown && _groundAt.isValid() && _groundAt.elapsed() < kGroundFreshMs;
    skydroid::geo::PictureInput input;
    input.view = view;
    input.heightAboveGroundM = *height - (byLaser ? std::max(0.0, _groundRiseM) : 0.0);
    const skydroid::geo::PictureResult result = skydroid::geo::computePictureTarget(input);
    if (result.why != skydroid::geo::PictureWhy::None) {
        return;  // too flat a look, or too low: no place is better than a wrong one
    }
    // How far the place can be off: along the look, from the angle and from the height.
    const double angleSigma = byLaser ? kPlaceAngleSigmaLaserDeg : kPlaceAngleSigmaDeg;
    const double heightSigma = byLaser ? kPlaceHeightSigmaLaserM : kPlaceHeightSigmaM;
    const double lookDown = std::max(1.0, result.lookDownDeg) * kPi / 180.0;
    const double sigma = std::max(kPlaceLeastSigmaM, std::hypot(result.metresPerDegree * angleSigma, heightSigma / std::tan(lookDown)));
    _placedAt.start();
    emit objectPlaced(result.point.targetLatDeg, result.point.targetLonDeg, sigma,
                      byLaser ? *height - _groundRiseM : std::nan(""), static_cast<int>(kFollowPictureAgeS * 1000.0));
}

void SkydroidLink::_learnGround()
{
    // The object is this far below the drone: the range along the look, and how steep the look is.
    const double below = _laserRangeM * std::sin(_target.depressionDeg * kPi / 180.0);
    _groundRises.push_back(*_shotHeightM - below);
    while (_groundRises.size() > static_cast<size_t>(kGroundShots)) {
        _groundRises.pop_front();
    }
    std::vector<double> sorted(_groundRises.begin(), _groundRises.end());
    std::sort(sorted.begin(), sorted.end());
    // The middle one. Of two, the lower: the one that brings the object less near.
    _groundRiseM = sorted[(sorted.size() - 1) / 2];
    _groundKnown = true;
    _groundAt.start();
}

void SkydroidLink::_forgetGround()
{
    _groundRises.clear();
    _groundKnown = false;
    _groundRiseM = 0.0;
    _groundAt.invalidate();
    _placedAt.invalidate();
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
    if (_pointMode != QStringLiteral("picture") && _camera.laser) {
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
    view.gimbalYawLeftPositive = _yawLeftPositive();
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
    // No lock, no finder's check (task AC).
    _check.stop();
    _lockDoubt = false;
    _lockBoxes.clear();
    _lastSeenAtMs = -1;
    _foundAgainAt.invalidate();
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
    // What was found, and the pictures kept, show the scene at the old size.
    _kept.clear();
    _finderKept.clear();
    emit zoomAsked();
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
        // The wheel's range, inside what the camera can do.
        const double pitchMin = std::max(_camera.pitchMinDeg, kWheelPitchMinDeg);
        const double pitchMax = std::min(_camera.pitchMaxDeg, kWheelPitchMaxDeg);
        const double yawRange = std::min(_camera.yawLimitDeg, kWheelYawRangeDeg);
        const double angle = (a == PitchAxis)
            ? pitchMin + t * (pitchMax - pitchMin)
            : -yawRange + t * 2.0 * yawRange;
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
    std::optional<double> other[2];  // a Viewpro or SIYI: both in one frame
    for (int a = 0; a < 2; ++a) {
        if (!_wheelTarget[a]) {
            continue;
        }
        if (_wheelSent[a] && std::abs(*_wheelSent[a] - *_wheelTarget[a]) < kWheelAngleStepDeg) {
            continue;
        }
        if (_top()) {
            _sendAngle(tags[a], *_wheelTarget[a], kWheelApproachDps);
        } else {
            other[a] = _wheelTarget[a];
        }
        _wheelSent[a] = _wheelTarget[a];
    }
    if (other[YawAxis] || other[PitchAxis]) {
        _sendAnglesOther(other[YawAxis], other[PitchAxis]);
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
    if (!_camera.photo || !_socket) {
        return;
    }
    if (!_top()) {
        // Neither says that it took one (VGCS does not ask either).
        if (_camera.protocol == camera::Protocol::Viewpro) {
            _sendBytes(_viewpro.photo());
        } else {
            _sendAllBytes(_siyi.photo());
        }
        emit cameraNote(tr("The camera was told to take a photo. It saves it on its own SD card."), false);
        return;
    }
    _send(top::buildPhoto(_options));
    _photoWaiting = true;
    _photoVerdictTimer.start();
}

void SkydroidLink::_photoVerdict()
{
    if (!_photoWaiting) {
        return;
    }
    _photoWaiting = false;
    emit cameraNote(joined(tr("The camera did not answer the photo command."), _sdCardProblem()), true);
}

void SkydroidLink::toggleRecord()
{
    // Start or stop, by what the screen shows. (Until test build 13 every press sent REC 01, so
    // the second press never stopped the camera.)
    setRecord(!_recording);
}

void SkydroidLink::setRecord(bool start)
{
    if (!_camera.record || !_socket) {
        return;
    }
    if (_camera.protocol == camera::Protocol::Viewpro) {
        // Nothing when it already does what is asked (it says so in its status, which answers the press).
        _sendBytes(_viewpro.setRecording(start));
    } else if (_camera.protocol == camera::Protocol::Siyi) {
        // A toggle only, and it does not say whether it records: by what the screen shows.
        if (start != _recording) {
            _sendAllBytes(_siyi.toggleRecording());
        }
    } else {
        _send(top::buildRecord(start, _options));
    }
    if (start == _recording) {
        return;  // nothing changes on the screen: the frame makes sure, no note
    }
    _recordWanted = start;
    _recordChecking = true;
    _recordPressAt.start();
    _recordVerdictTimer.start();
    const quint64 press = ++_recordPresses;
    for (const int ms : kRecordAskAfterMs) {
        QTimer::singleShot(ms, this, [this, press]() {
            if (press == _recordPresses && _recordChecking) {
                _send(top::buildRecordQuery(_options));
            }
        });
    }
    _setRecording(start);
}

void SkydroidLink::_recordAnswered(bool on)
{
    if (_recordChecking && _recordPressAt.elapsed() < kRecordSettleMs) {
        return;  // it may answer a question asked before the press
    }
    _recordReported = on;
    _recordReportAt.start();
    if (!_recordChecking) {
        _setRecording(on);  // as the camera says: the RC's own button can start it too
        return;
    }
    if (on != _recordWanted) {
        _setRecording(_recording);  // the camera may still be starting: the verdict decides
        return;
    }
    _recordChecking = false;
    _recordVerdictTimer.stop();
    _setRecording(on);
    if (on) {
        const QString problem = _sdCardProblem();
        emit cameraNote(joined(tr("Recording on the camera's SD card."), problem), !problem.isEmpty());
    } else {
        emit cameraNote(tr("Recording stopped. The video is on the camera's SD card."), false);
        QTimer::singleShot(kSdCardAskAgainMs, this, &SkydroidLink::_askSdCard);
    }
}

void SkydroidLink::_recordVerdict()
{
    if (!_recordChecking) {
        return;
    }
    _recordChecking = false;
    // Only an answer after the press counts (one right after it was not taken).
    const bool answered = _recordReported && _recordReportAt.isValid() && _recordReportAt.elapsed() < _recordPressAt.elapsed();
    if (!answered) {
        emit cameraNote(_recordWanted ? joined(tr("The camera was told to record. It does not say whether it does."), _sdCardProblem())
                                      : tr("The camera was told to stop recording. It does not say whether it did."),
                        true);
        return;
    }
    // It said the other thing: an answer that matched would have ended the check.
    _setRecording(*_recordReported);
    emit cameraNote(_recordWanted ? joined(tr("Recording did not start."), _sdCardProblem())
                                  : tr("Recording did not stop. Press the button again."),
                    true);
}

void SkydroidLink::_setRecording(bool on)
{
    const bool confirmed = _recordReported && *_recordReported == on;
    if (on == _recording && confirmed == _recordConfirmed) {
        return;
    }
    _recording = on;
    _recordConfirmed = confirmed;
    emit recordingChanged();
}

void SkydroidLink::_askSdCard()
{
    _send(top::buildSdCardQuery(_options));
    // A camera that never answers: the camera settings say so.
    if (!_sdCard && ++_sdCardAsked == kSdCardSilentAsks) {
        emit sdCardChanged();
    }
}

void SkydroidLink::_sdCardAnswered(const top::SdCard &card)
{
    const bool changed = !_sdCard || _sdCard->freeMb != card.freeMb || _sdCard->totalMb != card.totalMb;
    _sdCard = card;
    _sdCardAt.start();
    _sdCardAsked = 0;
    if (changed) {
        emit sdCardChanged();
    }
}

QString SkydroidLink::_sdCardProblem() const
{
    if (!_sdCard) {
        return QString();
    }
    if (_sdCard->totalMb == 0 && _sdCard->freeMb == 0) {
        return tr("The camera has no SD card.");
    }
    if (_sdCard->freeMb < kSdCardLowMb) {
        return tr("The camera's SD card is almost full: %1 free.").arg(sizeText(_sdCard->freeMb));
    }
    return QString();
}

QString SkydroidLink::sdCardWarning() const
{
    if (!_sdCard) {
        return QString();
    }
    if (_sdCard->totalMb == 0 && _sdCard->freeMb == 0) {
        return tr("No SD card");
    }
    return _sdCard->freeMb < kSdCardLowMb ? tr("SD card almost full") : QString();
}

QString SkydroidLink::sdCardText() const
{
    if (!_sdCard) {
        return _sdCardAsked >= kSdCardSilentAsks ? tr("The camera does not say how much room its SD card has.") : QString();
    }
    if (_sdCard->totalMb == 0 && _sdCard->freeMb == 0) {
        return tr("The camera says it has no SD card.");
    }
    return tr("The camera's SD card has %1 free of %2.").arg(sizeText(_sdCard->freeMb), sizeText(_sdCard->totalMb));
}

void SkydroidLink::_forgetCameraReports()
{
    _recordChecking = false;
    _recordVerdictTimer.stop();
    _photoWaiting = false;
    _photoVerdictTimer.stop();
    _recordReported.reset();
    _setRecording(_recording);  // no longer confirmed
    const bool sdShown = _sdCard.has_value() || _sdCardAsked >= kSdCardSilentAsks;
    _sdCard.reset();
    _sdCardAsked = 0;
    if (sdShown) {
        emit sdCardChanged();
    }
}

void SkydroidLink::zoom(int direction)
{
    if (direction == 0 || _camera.zoom == camera::Zoom::None) {
        return;
    }
    const int d = direction > 0 ? 1 : -1;
    _endAppLockForZoom();
    if (!_top()) {
        // One click: the rules send the stop a moment later, the next click takes that back
        // (so a button held down zooms on). A tap ends a "back to 1x" run.
        _zoomOutLeft = 0;
        _zoomOutTimer.stop();
        if (_camera.protocol == camera::Protocol::Viewpro) {
            _sendBytes(_viewpro.zoomStep(d, _nowMs()));
        } else {
            _sendAllBytes(_siyi.zoomStep(d, _nowMs()));
            if (d > 0 && !_zoomUnknown) {
                _zoomUnknown = true;  // zoomed in: how far, the SIYI does not say
                emit zoomChanged();
            }
        }
        return;
    }
    if (_camera.zoom == camera::Zoom::Pulse) {
        _lensZoom(d);
        return;
    }
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
    if (_camera.zoom == camera::Zoom::None) {
        return;
    }
    _endAppLockForZoom();
    if (!_top()) {
        // Neither has a "1x" command in VGCS: zoom out, until a Viewpro says 1x, a SIYI for a while.
        _zoomOutLeft = kZoomHomeOutMs / kZoomOutIntervalMs;
        _zoomOutTimer.start();
        _zoomOutTick();
        return;
    }
    if (_camera.zoom == camera::Zoom::Pulse) {
        // The C20: zoom out as long as a SIYI's "back to 1x", then stop. It is at 1x then.
        _lensStopTimer.stop();
        _send(top::buildLensZoom("out", _options));
        _lensHomeTimer.start();
        return;
    }
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
    if (!_top()) {
        const bool viewpro = _camera.protocol == camera::Protocol::Viewpro;
        const bool atOne = viewpro && _liveZoomX > 0.0 && _liveZoomX <= 1.05;
        if (_zoomOutLeft <= 0 || !_socket || atOne) {
            const bool ranOut = _zoomOutLeft <= 0;
            _zoomOutLeft = 0;
            _zoomOutTimer.stop();
            if (viewpro) {
                _sendBytes(_viewpro.zoomStep(0, _nowMs()));  // stop now
            } else if (_socket && ranOut && _zoomUnknown) {
                _zoomUnknown = false;  // zoomed out long enough: at 1x (it does not say)
                emit zoomChanged();
            }
            return;
        }
        if (viewpro) {
            _sendBytes(_viewpro.zoomStep(-1, _nowMs()));
        } else {
            _sendAllBytes(_siyi.zoomStep(-1, _nowMs()));
        }
        --_zoomOutLeft;
        return;
    }
    if (_zoomOutLeft <= 0 || !_socket) {
        _zoomOutLeft = 0;
        _zoomOutTimer.stop();
        QTimer::singleShot(kZoomQueryAfterCmdMs, this, [this]() { _send(top::buildDzmQuery(_options)); });
        return;
    }
    _sendZoomStepFrames(-1);
    --_zoomOutLeft;
}

void SkydroidLink::_lensZoom(int direction)
{
    // The C20 (task AN): its lens moves while it is told to and does not say how far, like a
    // SIYI. Each click tells it again and puts the stop off: a held button zooms on, a tap moves
    // the lens a little. A click ends a "back to 1x" run. (In build 14 it got the V13's step,
    // whose stop came at once: the client's C20 did not zoom.)
    _lensHomeTimer.stop();
    _send(top::buildLensZoom(direction > 0 ? "in" : "out", _options));
    _lensStopTimer.setInterval(_lensStopTimer.isActive() ? kLensZoomHeldMs : kLensZoomPulseMs);
    _lensStopTimer.start();
    if (direction > 0 && !_zoomUnknown) {
        _zoomUnknown = true;  // zoomed in: how far, the C20 does not say
        emit zoomChanged();
    }
}

void SkydroidLink::_lensHomeDone()
{
    _send(top::buildLensZoom("stop", _options));
    if (_socket && _zoomUnknown) {
        _zoomUnknown = false;  // zoomed out long enough: at 1x (it does not say)
        emit zoomChanged();
    }
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
    return camera::countedStepsMax(_camera);
}

QString SkydroidLink::zoomLabel() const
{
    // A Viewpro says its zoom; a SIYI does not, so its 1x only after "back to 1x".
    if (_camera.zoom == camera::Zoom::Live) {
        return _liveZoomX > 0.0 ? QStringLiteral("%1x").arg(_liveZoomX, 0, 'f', 1) : QString();
    }
    if (_camera.zoom == camera::Zoom::Pulse) {
        return _zoomUnknown ? QString() : QStringLiteral("1.0x");
    }
    // A camera that reports its step: until it does, nothing rather than a guess.
    return camera::zoomLabel(_camera, _zoomCount, _zoomStep);
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
    if (!_socket || !_camera.laser) {
        _laserValid = false;
        _laserMessage = !_socket ? tr("Camera link is off. Turn it on in the camera settings.")
                                 : tr("This camera has no laser.");
        emit laserChanged();
        return;
    }
    _laserBusy = true;
    _laserFromE.reset();
    _laserFromD.reset();

    // The lock's own shot goes to the object: it is under the cross and seen there.
    _shotAtLockedObject = keepLastResult && _lockActive && _lockByApp && _lockSeen && _lockOffDeg <= kLockOnCrossDeg;
    // Pose and gimbal angles at the moment of the shot, not when the reply lands.
    _shotPose = skydroid::geo::LaserInput{};
    _shotPoseValid = _sampleVehiclePose(_shotPose, _shotPoseWhy);
    _shotGimbalValid = _attitudeValid;
    _shotPose.gimbalYawDeg = _yaw;
    // Which way that yaw counts: the same switch as the tap aiming, because it
    // is one camera. Without this the target landed on the wrong side of the
    // drone's nose whenever the camera was turned (field video, 2026-10-06).
    _shotPose.gimbalYawLeftPositive = _yawLeftPositive();
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
        if (_target.ok && !_target.nearHorizon && _shotAtLockedObject && _lockActive && _lockByApp && _shotHeightM) {
            _learnGround();
        }
    }
    emit targetChanged();
}
