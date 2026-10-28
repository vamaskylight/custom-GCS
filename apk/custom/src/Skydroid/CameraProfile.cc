#include "CameraProfile.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QSaveFile>
#include <QtNetwork/QHostAddress>

#include <algorithm>
#include <cmath>

namespace skydroid::camera {

namespace {

constexpr const char kFormat[] = "vama-camera-1";
constexpr const char kProtocolTop[] = "skydroid-top";
constexpr const char kProtocolViewpro[] = "viewpro";
constexpr const char kProtocolSiyi[] = "siyi";
constexpr double kPi = 3.14159265358979323846;
constexpr qint64 kMaxFileBytes = 64 * 1024;
constexpr int kMaxLenses = 4;
constexpr int kMaxNameLength = 40;

// The addresses of Skydroid's cameras: day picture on RTSP port 554, thermal on 555
// (DOCS/SKYDROID-C13-OBSERVATION-SETUP.md; the C14 Pro assumed the same).
constexpr const char kSkydroidHost[] = "192.168.144.108";
constexpr int kSkydroidPort = 5000;
constexpr const char kSkydroidDayUrl[] = "rtsp://192.168.144.108:554/stream=1";
constexpr const char kSkydroidThermalUrl[] = "rtsp://192.168.144.108:555/stream=2";

// Viewpro and SIYI as VGCS has them (vgcs/video/camera_control.py, vgcs/skydroid/camera_files.py).
// The client's Viewpro sits on the Skydroid link's network (rtsp://192.168.144.119:554, the client
// 2026-10-28); Viewpro's factory address, printed on the unit, is 192.168.2.119.
constexpr const char kViewproHost[] = "192.168.144.119";
constexpr int kViewproPort = 2000;
constexpr const char kSiyiHost[] = "192.168.144.25";
constexpr int kSiyiPort = 37260;

QString text(const char *sentence)
{
    return QCoreApplication::translate("CameraProfile", sentence);
}

double narrow(double fovDeg, double ratio)
{
    return 2.0 * std::atan(std::tan(fovDeg * kPi / 360.0) * ratio) * 180.0 / kPi;
}

Profile skydroid(const char *id, const char *name)
{
    Profile p;
    p.id = QString::fromLatin1(id);
    p.name = QString::fromLatin1(name);
    p.builtIn = true;
    p.host = QString::fromLatin1(kSkydroidHost);
    p.port = kSkydroidPort;
    p.dayVideoUrl = QString::fromLatin1(kSkydroidDayUrl);
    p.thermalVideoUrl = QString::fromLatin1(kSkydroidThermalUrl);
    return p;
}

/// The lens a reported step picks: the last one that starts at or before it.
int lensIndex(const Profile &p, int step)
{
    int index = 0;
    for (int i = 0; i < static_cast<int>(p.lenses.size()); ++i) {
        if (step >= p.lenses[i].firstStep) {
            index = i;
        }
    }
    return index;
}

/// Steps cropped on that lens.
int cropSteps(const Lens &lens, int step)
{
    return std::clamp(step - lens.firstStep, 0, lens.lastStep - lens.firstStep);
}

QString zoomName(Zoom zoom)
{
    switch (zoom) {
    case Zoom::None:
        return QStringLiteral("none");
    case Zoom::Reported:
        return QStringLiteral("reported");
    case Zoom::Live:
        return QStringLiteral("live");
    case Zoom::Pulse:
        return QStringLiteral("pulse");
    case Zoom::Counted:
        break;
    }
    return QStringLiteral("counted");
}

QString protocolName(Protocol protocol)
{
    switch (protocol) {
    case Protocol::Viewpro:
        return QString::fromLatin1(kProtocolViewpro);
    case Protocol::Siyi:
        return QString::fromLatin1(kProtocolSiyi);
    case Protocol::Top:
        break;
    }
    return QString::fromLatin1(kProtocolTop);
}

bool isRtsp(const QString &url)
{
    return url.startsWith(QStringLiteral("rtsp://"), Qt::CaseInsensitive) && url.size() > 7;
}

bool inRange(double value, double low, double high)
{
    return std::isfinite(value) && value >= low && value <= high;
}

bool isBuiltInId(const QString &id)
{
    for (const Profile &p : builtInProfiles()) {
        if (p.id.compare(id, Qt::CaseInsensitive) == 0) {
            return true;
        }
    }
    return false;
}

}  // namespace

QList<Profile> builtInProfiles()
{
    // The C12: the C13's calibrated view (VGCS uses it for the C12 too, aim_fov_from_profile
    // off), 4x zoom (DOCS/SKYDROID-CAMERA-SPECS.md), up to +30 degrees. Its thermal lens from
    // the sensor (data sheet: 7 mm, 384 x 288 points of 12 um): 36.4 x 27.7.
    Profile c12 = skydroid("C12", "V12");
    c12.maxZoom = 4.0;
    c12.thermalFovHDeg = 36.4;
    c12.thermalFovVDeg = 27.7;
    c12.pitchMaxDeg = 30.0;

    // The C13: 83.4 x 46.9, VGCS's calibrated view (not the data sheet's 77.4 x 48.8), 30x
    // digital zoom counted at 0.1x a step as VGCS does (not measured on the camera: the
    // settings show the camera's own zoom report to check it), -90 to +10 degrees. Its thermal
    // stream is 640 x 512 ("hevc, 640x512" in the client's VGCS log of 2026-10-08). Its data
    // sheet gives the thermal lens in two rows: "48.7 x 38.6" (used here) and "24.8 x 18.7",
    // which is the lens of a 256 x 192 sensor and stands on the C12's sheet too. Its 9.1 mm
    // lens and 640 x 512 points of 12 um give 45.8 x 37.3. Good enough to turn the camera (the
    // laser measures where the cross ends up) and to follow a locked object, not to place a
    // point from the picture: on the thermal picture that is the cross only.
    Profile c13 = skydroid("C13", "V13");

    // The C14 Pro (VGCS c14pro_default): it reports its zoom step. Short lens steps 0 to 85
    // (61.4 x 47.9), long lens 86 to 184 (14.7 x 11.1); each step crops 36 x 20 points of a
    // 3840 x 2160 readout, starting again at the lens change (Skydroid, 2026-09-19). Thermal
    // lens 32.84 x 26.35 from the maker. Up to +60 degrees, 120 either way. The laser reaches
    // 1200 m on a building (1500 m taken). It wants "#TP" on every gimbal frame.
    Profile c14 = skydroid("C14 Pro", "V14 Pro");
    c14.zoom = Zoom::Reported;
    c14.fovHDeg = 61.4;
    c14.fovVDeg = 47.9;
    c14.lenses = {Lens{61.4, 47.9, 0, 85}, Lens{14.7, 11.1, 86, 184}};
    c14.thermalFovHDeg = 32.84;
    c14.thermalFovVDeg = 26.35;
    c14.thermalFovSure = true;
    c14.pitchMaxDeg = 60.0;
    c14.yawLimitDeg = 120.0;
    c14.laserMaxM = 1500.0;
    c14.upperGimbalHeader = true;

    // The C20 (task AJ): TOP over TCP port 5000 only, "#TP" on every gimbal frame (TOP document
    // V1.1.6, sections 2.1.1 and 2.2.2). 22x optical zoom, 58.3 degrees wide at 1x, 1080p
    // (34.9 high), -90 to +45 degrees, 150 either way, no laser, no thermal picture. Its day
    // video as the client's own camera file had it (2026-10-22, the picture worked). TOP lists
    // no photo or recording command for the C20: they are on, to be tried on the camera. Its
    // zoom is the lens's (task AN): ZMC to the lens address M, held while it zooms, and no zoom
    // report, so the "pulse" zoom. The V13's counted step did not zoom it (client 2026-10-27).
    Profile c20 = skydroid("C20", "V20");
    c20.tcp = true;
    c20.zoom = Zoom::Pulse;
    c20.dayVideoUrl = QStringLiteral("rtsp://192.168.144.108:554/main");
    c20.thermalVideoUrl.clear();
    c20.fovHDeg = 58.3;
    c20.fovVDeg = 34.9;
    c20.maxZoom = 22.0;
    c20.pitchMaxDeg = 45.0;
    c20.yawLimitDeg = 150.0;
    c20.laser = false;
    c20.ownTracker = false;
    c20.upperGimbalHeader = true;

    // The Viewpro A20KTR (task AK), named "VPro" by the client (2026-10-28): Viewpro's protocol over
    // TCP 2000, its one video at the client's address. 70.2 x 43.1 is the view its own screen showed
    // at 1x, used until the camera says its own (it says its zoom and its view in every status). Its
    // tilt and turn are estimates, not from a data sheet. Its thermal picture comes in the same video
    // after a sensor switch. Its thermal lens is Viewpro's: 35 mm on 640 x 512 at 12 um, 12.5 x 10.0
    // degrees (also by the sensor's size). Not "sure": how the 5:4 thermal picture sits in the 16:9
    // video is not known, so a point on the thermal picture is placed at the cross only. No laser in
    // the app yet. Its own tracker is not used (as in VGCS).
    Profile a20;
    a20.id = QStringLiteral("A20KTR");
    a20.name = QStringLiteral("VPro");
    a20.builtIn = true;
    a20.protocol = Protocol::Viewpro;
    a20.host = QString::fromLatin1(kViewproHost);
    a20.port = kViewproPort;
    a20.dayVideoUrl = QStringLiteral("rtsp://192.168.144.119:554");
    a20.thermalBySwitch = true;
    a20.thermalFovHDeg = 12.5;
    a20.thermalFovVDeg = 10.0;
    a20.fovHDeg = 70.2;
    a20.fovVDeg = 43.1;
    a20.zoom = Zoom::Live;
    a20.pitchMaxDeg = 30.0;
    a20.yawLimitDeg = 150.0;
    a20.laser = false;
    a20.ownTracker = false;

    // The SIYI ZR10 (task AK), named "VR10" by the client (2026-10-28): SIYI's protocol over UDP
    // 37260. 71.5 degrees wide at 1x (ZR10 manual), 45.2 high (from its diagonal). It zooms while
    // it is told to and does not say how far, so its view holds at 1x only. -90 to +25 degrees, 45
    // either way: the range it can be told to go (DOCS/SIYI-ZR10-HARDWARE-REFERENCE.md). No laser,
    // no thermal picture.
    Profile zr10;
    zr10.id = QStringLiteral("ZR10");
    zr10.name = QStringLiteral("VR10");
    zr10.builtIn = true;
    zr10.protocol = Protocol::Siyi;
    zr10.host = QString::fromLatin1(kSiyiHost);
    zr10.port = kSiyiPort;
    zr10.dayVideoUrl = QStringLiteral("rtsp://192.168.144.25:8554/main.264");
    zr10.fovHDeg = 71.5;
    zr10.fovVDeg = 45.2;
    zr10.zoom = Zoom::Pulse;
    zr10.pitchMaxDeg = 25.0;
    zr10.yawLimitDeg = 45.0;
    zr10.laser = false;
    zr10.ownTracker = false;

    return {c12, c13, c14, c20, a20, zr10};
}

QString defaultId()
{
    return QStringLiteral("C13");
}

int countedStepsMax(const Profile &p)
{
    if (p.zoom != Zoom::Counted) {
        return 0;
    }
    return static_cast<int>(std::lround((p.maxZoom - 1.0) / kCountedStepZoom));
}

void dayFov(const Profile &p, int countedSteps, int reportedStep, double &horizontalDeg, double &verticalDeg)
{
    if (p.zoom == Zoom::Reported && !p.lenses.empty()) {
        const int step = reportedStep >= 0 ? reportedStep : p.lenses.front().firstStep;
        const Lens &lens = p.lenses[lensIndex(p, step)];
        const int m = cropSteps(lens, step);
        horizontalDeg = narrow(lens.fovHDeg, static_cast<double>(p.sensorW - p.cropStepW * m) / p.sensorW);
        verticalDeg = narrow(lens.fovVDeg, static_cast<double>(p.sensorH - p.cropStepH * m) / p.sensorH);
        return;
    }
    if (p.zoom == Zoom::Counted) {
        // Narrowed by the zoom counted, as VGCS does (tan(fov/2) / zoom).
        const double zoomX = 1.0 + countedSteps * kCountedStepZoom;
        horizontalDeg = narrow(p.fovHDeg, 1.0 / zoomX);
        verticalDeg = narrow(p.fovVDeg, 1.0 / zoomX);
        return;
    }
    horizontalDeg = p.fovHDeg;
    verticalDeg = p.fovVDeg;
}

QString zoomLabel(const Profile &p, int countedSteps, int reportedStep)
{
    if (p.zoom == Zoom::Counted) {
        return QStringLiteral("%1x").arg(1.0 + countedSteps * kCountedStepZoom, 0, 'f', 1);
    }
    if (p.zoom != Zoom::Reported || p.lenses.empty() || reportedStep < 0) {
        // A reported zoom: until the camera reports its step, nothing rather than a guess.
        return QString();
    }
    // As VGCS labels it ("W2.9x", "T5.3x"): the crop, times how much narrower the lens is
    // than the widest one.
    const int index = lensIndex(p, reportedStep);
    const Lens &lens = p.lenses[index];
    const int m = cropSteps(lens, reportedStep);
    const double crop = static_cast<double>(p.sensorW) / std::max(1, p.sensorW - p.cropStepW * m);
    const double lensFactor = index == 0 ? 1.0
                                         : std::tan(p.lenses.front().fovHDeg * kPi / 360.0) / std::tan(lens.fovHDeg * kPi / 360.0);
    const QString prefix = p.lenses.size() > 1 ? (index == 0 ? QStringLiteral("W") : QStringLiteral("T")) : QString();
    return QStringLiteral("%1%2x").arg(prefix).arg(crop * lensFactor, 0, 'f', 1);
}

QString problem(const Profile &p)
{
    const QString name = p.name.trimmed();
    if (name.isEmpty()) {
        return text("Give the camera a name.");
    }
    if (name.size() > kMaxNameLength) {
        return text("The name is too long: %1 signs at the most.").arg(kMaxNameLength);
    }
    if (QHostAddress(p.host.trimmed()).isNull()) {
        return text("The camera IP address is not an IP address (for example 192.168.144.108).");
    }
    if (p.port < 1 || p.port > 65535) {
        return text("The control port must be 1 to 65535.");
    }
    if (!isRtsp(p.dayVideoUrl.trimmed())) {
        return text("The day video must be an RTSP address (rtsp://...).");
    }
    if (!p.thermalVideoUrl.trimmed().isEmpty() && !isRtsp(p.thermalVideoUrl.trimmed())) {
        return text("The thermal video must be an RTSP address (rtsp://...), or empty for a camera without one.");
    }
    // Each protocol has its zooms, and the app uses no laser or tracker of a Viewpro or SIYI camera.
    switch (p.protocol) {
    case Protocol::Top:
        if (p.zoom != Zoom::None && p.zoom != Zoom::Counted && p.zoom != Zoom::Reported && p.zoom != Zoom::Pulse) {
            return text("A Skydroid TOP camera's zoom is none, counted, reported or pulse.");
        }
        break;
    case Protocol::Viewpro:
        if (p.zoom != Zoom::None && p.zoom != Zoom::Live) {
            return text("A VPro camera's zoom is none or live.");
        }
        break;
    case Protocol::Siyi:
        if (p.zoom != Zoom::None && p.zoom != Zoom::Pulse) {
            return text("A SIYI camera's zoom is none or pulse.");
        }
        break;
    default:
        return text("The protocol must be \"skydroid-top\", \"viewpro\" or \"siyi\".");
    }
    if (p.protocol != Protocol::Top) {
        if (p.laser) {
            return text("The app has no laser for a VPro or SIYI camera yet. Untick Laser.");
        }
        if (p.ownTracker) {
            return text("The app uses no tracker of a VPro or SIYI camera. Untick \"Its own object tracker\".");
        }
    }
    if (p.thermalBySwitch) {
        if (p.protocol != Protocol::Viewpro) {
            return text("Only a VPro camera switches its day video to the thermal picture.");
        }
        if (!p.thermalVideoUrl.trimmed().isEmpty()) {
            return text("A camera that switches to its thermal picture has no thermal video of its own: leave it empty.");
        }
    }
    if (p.zoom == Zoom::Reported) {
        if (p.lenses.empty() || p.lenses.size() > static_cast<size_t>(kMaxLenses)) {
            return text("A camera that reports its zoom step needs 1 to %1 lenses.").arg(kMaxLenses);
        }
        if (!inRange(p.sensorW, 16, 100000) || !inRange(p.sensorH, 16, 100000) || p.cropStepW < 0 || p.cropStepH < 0) {
            return text("The sensor size must be 16 to 100000 points, and the crop of a step 0 or more.");
        }
        for (size_t i = 0; i < p.lenses.size(); ++i) {
            const Lens &lens = p.lenses[i];
            const int n = static_cast<int>(i) + 1;
            if (!inRange(lens.fovHDeg, 0.5, 179.0) || !inRange(lens.fovVDeg, 0.5, 179.0)) {
                return text("Lens %1: its view must be 0.5 to 179 degrees.").arg(n);
            }
            if (lens.firstStep < 0 || lens.lastStep < lens.firstStep || lens.lastStep > 9999) {
                return text("Lens %1: its steps must run from a first to a last step, 0 to 9999.").arg(n);
            }
            if (i > 0 && lens.firstStep <= p.lenses[i - 1].lastStep) {
                return text("Lens %1 must start after the last step of lens %2.").arg(n).arg(n - 1);
            }
            const int m = lens.lastStep - lens.firstStep;
            if (p.sensorW - p.cropStepW * m < 1 || p.sensorH - p.cropStepH * m < 1) {
                return text("Lens %1: its last step crops more than the whole sensor.").arg(n);
            }
        }
    } else {
        if (!inRange(p.fovHDeg, 1.0, 179.0) || !inRange(p.fovVDeg, 1.0, 179.0)) {
            return text("The day lens's view must be 1 to 179 degrees.");
        }
        if (p.zoom == Zoom::Counted && !inRange(p.maxZoom, 1.1, 100.0)) {
            return text("The highest zoom must be 1.1x to 100x.");
        }
    }
    if ((!p.thermalVideoUrl.trimmed().isEmpty() || p.thermalBySwitch) &&
        (!inRange(p.thermalFovHDeg, 1.0, 179.0) || !inRange(p.thermalFovVDeg, 1.0, 179.0))) {
        return text("The thermal lens's view must be 1 to 179 degrees.");
    }
    if (!inRange(p.pitchMinDeg, -180.0, 0.0) || !inRange(p.pitchMaxDeg, 0.0, 180.0) || p.pitchMaxDeg <= p.pitchMinDeg) {
        return text("The tilt must go from -180 to 0 degrees (down) up to 0 to 180 degrees.");
    }
    if (!inRange(p.yawLimitDeg, 1.0, 360.0)) {
        return text("The turn either way must be 1 to 360 degrees.");
    }
    // The laser's reading has 4 hex digits of decimetres (SkydroidTop, slrMaxDmForRange).
    if (p.laser && !inRange(p.laserMaxM, 5.0, 6553.0)) {
        return text("The laser's longest reading must be 5 to 6553 m.");
    }
    return QString();
}

QJsonObject toJson(const Profile &p)
{
    QJsonObject o;
    o[QStringLiteral("format")] = QString::fromLatin1(kFormat);
    o[QStringLiteral("name")] = p.name;
    o[QStringLiteral("protocol")] = protocolName(p.protocol);
    o[QStringLiteral("host")] = p.host;
    o[QStringLiteral("port")] = p.port;
    o[QStringLiteral("transport")] = p.controlOverTcp() ? QStringLiteral("tcp") : QStringLiteral("udp");
    o[QStringLiteral("dayVideoUrl")] = p.dayVideoUrl;
    o[QStringLiteral("thermalVideoUrl")] = p.thermalVideoUrl;
    o[QStringLiteral("thermalBySwitch")] = p.thermalBySwitch;
    o[QStringLiteral("zoom")] = zoomName(p.zoom);
    o[QStringLiteral("dayFovH")] = p.fovHDeg;
    o[QStringLiteral("dayFovV")] = p.fovVDeg;
    o[QStringLiteral("maxZoom")] = p.maxZoom;
    QJsonArray lenses;
    for (const Lens &lens : p.lenses) {
        QJsonObject l;
        l[QStringLiteral("fovH")] = lens.fovHDeg;
        l[QStringLiteral("fovV")] = lens.fovVDeg;
        l[QStringLiteral("firstStep")] = lens.firstStep;
        l[QStringLiteral("lastStep")] = lens.lastStep;
        lenses.append(l);
    }
    o[QStringLiteral("lenses")] = lenses;
    o[QStringLiteral("sensorW")] = p.sensorW;
    o[QStringLiteral("sensorH")] = p.sensorH;
    o[QStringLiteral("cropStepW")] = p.cropStepW;
    o[QStringLiteral("cropStepH")] = p.cropStepH;
    o[QStringLiteral("thermalFovH")] = p.thermalFovHDeg;
    o[QStringLiteral("thermalFovV")] = p.thermalFovVDeg;
    o[QStringLiteral("thermalFovSure")] = p.thermalFovSure;
    o[QStringLiteral("pitchMin")] = p.pitchMinDeg;
    o[QStringLiteral("pitchMax")] = p.pitchMaxDeg;
    o[QStringLiteral("yawLimit")] = p.yawLimitDeg;
    o[QStringLiteral("laser")] = p.laser;
    o[QStringLiteral("laserMaxM")] = p.laserMaxM;
    o[QStringLiteral("photo")] = p.photo;
    o[QStringLiteral("record")] = p.record;
    o[QStringLiteral("ownTracker")] = p.ownTracker;
    o[QStringLiteral("upperGimbalHeader")] = p.upperGimbalHeader;
    return o;
}

std::optional<Profile> fromJson(const QJsonObject &json, const QString &id, QString *why)
{
    QString missing;
    auto value = [&](const char *key) {
        const QJsonValue v = json.value(QLatin1String(key));
        if (v.isUndefined() && missing.isEmpty()) {
            missing = QString::fromLatin1(key);
        }
        return v;
    };
    auto number = [&](const char *key) {
        const QJsonValue v = value(key);
        return v.isDouble() ? v.toDouble() : std::nan("");
    };
    auto integer = [&](const char *key) {
        const double d = number(key);
        return (std::isfinite(d) && d == std::floor(d) && std::abs(d) < 1e9) ? static_cast<int>(d) : -1;
    };
    auto flag = [&](const char *key, bool otherwise) {
        const QJsonValue v = json.value(QLatin1String(key));
        return v.isBool() ? v.toBool() : otherwise;
    };
    auto fail = [&](const QString &reason) -> std::optional<Profile> {
        if (why) {
            *why = reason;
        }
        return std::nullopt;
    };

    if (json.value(QStringLiteral("format")).toString() != QLatin1String(kFormat)) {
        return fail(text("it is not a camera file of this app (no \"format\": \"%1\").").arg(QLatin1String(kFormat)));
    }
    Profile p;
    const QJsonValue protocol = json.value(QStringLiteral("protocol"));
    if (protocol == QJsonValue(QLatin1String(kProtocolTop))) {
        p.protocol = Protocol::Top;
    } else if (protocol == QJsonValue(QLatin1String(kProtocolViewpro))) {
        p.protocol = Protocol::Viewpro;
    } else if (protocol == QJsonValue(QLatin1String(kProtocolSiyi))) {
        p.protocol = Protocol::Siyi;
    } else {
        return fail(text("its protocol is not \"skydroid-top\", \"viewpro\" or \"siyi\"."));
    }
    // Files from before the C20 have no transport: UDP, as every camera had then.
    const QJsonValue transport = json.value(QStringLiteral("transport"));
    if (!transport.isUndefined() && transport != QJsonValue(QStringLiteral("udp"))
        && transport != QJsonValue(QStringLiteral("tcp"))) {
        return fail(text("its transport is not \"udp\" or \"tcp\"."));
    }
    // The file's own; it counts for TOP only (controlOverTcp: Viewpro takes TCP, SIYI UDP).
    p.tcp = transport == QJsonValue(QStringLiteral("tcp"));
    p.id = id;
    p.builtIn = false;
    p.name = value("name").toString().trimmed();
    p.host = value("host").toString().trimmed();
    p.port = integer("port");
    p.dayVideoUrl = value("dayVideoUrl").toString().trimmed();
    p.thermalVideoUrl = json.value(QStringLiteral("thermalVideoUrl")).toString().trimmed();
    p.thermalBySwitch = flag("thermalBySwitch", false);  // files from before task AK have none: no switch
    const QString zoom = value("zoom").toString();
    if (zoom == QLatin1String("none")) {
        p.zoom = Zoom::None;
    } else if (zoom == QLatin1String("counted")) {
        p.zoom = Zoom::Counted;
    } else if (zoom == QLatin1String("reported")) {
        p.zoom = Zoom::Reported;
    } else if (zoom == QLatin1String("live")) {
        p.zoom = Zoom::Live;
    } else if (zoom == QLatin1String("pulse")) {
        p.zoom = Zoom::Pulse;
    } else if (missing.isEmpty()) {
        return fail(text("its zoom is not one of none, counted, reported, live or pulse."));
    }
    if (p.zoom == Zoom::Reported) {
        p.lenses.clear();
        const QJsonArray lenses = value("lenses").toArray();
        for (const QJsonValue &v : lenses) {
            const QJsonObject l = v.toObject();
            const double first = l.value(QStringLiteral("firstStep")).toDouble(-1.0);
            const double last = l.value(QStringLiteral("lastStep")).toDouble(-1.0);
            Lens lens;
            lens.fovHDeg = l.value(QStringLiteral("fovH")).toDouble(std::nan(""));
            lens.fovVDeg = l.value(QStringLiteral("fovV")).toDouble(std::nan(""));
            lens.firstStep = (first == std::floor(first) && std::abs(first) < 1e9) ? static_cast<int>(first) : -1;
            lens.lastStep = (last == std::floor(last) && std::abs(last) < 1e9) ? static_cast<int>(last) : -1;
            p.lenses.push_back(lens);
        }
        p.sensorW = integer("sensorW");
        p.sensorH = integer("sensorH");
        p.cropStepW = integer("cropStepW");
        p.cropStepH = integer("cropStepH");
        // The widest view, for the editor.
        if (!p.lenses.empty()) {
            p.fovHDeg = p.lenses.front().fovHDeg;
            p.fovVDeg = p.lenses.front().fovVDeg;
        }
    } else {
        p.fovHDeg = number("dayFovH");
        p.fovVDeg = number("dayFovV");
        // A pulse zoom keeps its lens's range too, when its file has one (the V20's 22x), though
        // nothing counts it.
        if (p.zoom == Zoom::Counted || (p.zoom == Zoom::Pulse && json.contains(QStringLiteral("maxZoom")))) {
            p.maxZoom = number("maxZoom");
        }
    }
    if (p.hasThermal()) {
        p.thermalFovHDeg = number("thermalFovH");
        p.thermalFovVDeg = number("thermalFovV");
    }
    p.thermalFovSure = flag("thermalFovSure", false);
    p.pitchMinDeg = number("pitchMin");
    p.pitchMaxDeg = number("pitchMax");
    p.yawLimitDeg = number("yawLimit");
    p.laser = flag("laser", true);
    if (p.laser) {
        p.laserMaxM = number("laserMaxM");
    }
    p.photo = flag("photo", true);
    p.record = flag("record", true);
    p.ownTracker = flag("ownTracker", true);
    p.upperGimbalHeader = flag("upperGimbalHeader", false);
    if (!missing.isEmpty()) {
        return fail(text("it has no \"%1\".").arg(missing));
    }
    const QString wrong = problem(p);
    if (!wrong.isEmpty()) {
        return fail(wrong);
    }
    return p;
}

std::optional<Profile> fromFields(const QJsonObject &fields, QString *why)
{
    QJsonObject json = fields;
    json[QStringLiteral("format")] = QString::fromLatin1(kFormat);
    if (!json.contains(QStringLiteral("protocol"))) {
        json[QStringLiteral("protocol")] = QString::fromLatin1(kProtocolTop);
    }
    return fromJson(json, fields.value(QStringLiteral("id")).toString(), why);
}

QString idForName(const QString &name)
{
    QString id;
    for (const QChar c : name.toLower()) {
        if (c.unicode() < 128 && c.isLetterOrNumber()) {
            id += c;
        } else if (!id.isEmpty() && !id.endsWith(QLatin1Char('-'))) {
            id += QLatin1Char('-');
        }
    }
    while (id.endsWith(QLatin1Char('-'))) {
        id.chop(1);
    }
    return id.isEmpty() ? QStringLiteral("camera") : id;
}

void Store::load()
{
    _profiles = builtInProfiles();
    _notes.clear();
    if (_folder.isEmpty()) {
        return;
    }
    QDir dir(_folder);
    if (!dir.exists() && !QDir().mkpath(_folder)) {
        _notes.append(text("The folder for cameras cannot be made: %1").arg(_folder));
        return;
    }
    QList<Profile> added;
    const QFileInfoList files = dir.entryInfoList({QStringLiteral("*.json")}, QDir::Files, QDir::Name);
    for (const QFileInfo &file : files) {
        const QString id = file.completeBaseName();
        QString why;
        if (isBuiltInId(id)) {
            why = text("it has the file name of a built-in camera.");
        } else if (file.size() > kMaxFileBytes) {
            why = text("it is too big for a camera file.");
        } else {
            QFile f(file.absoluteFilePath());
            if (!f.open(QIODevice::ReadOnly)) {
                why = text("it cannot be read.");
            } else {
                QJsonParseError error;
                const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &error);
                if (error.error != QJsonParseError::NoError || !doc.isObject()) {
                    why = text("it is not a camera file (%1).").arg(error.errorString());
                } else if (const std::optional<Profile> p = fromJson(doc.object(), id, &why)) {
                    const QList<Profile> all = _profiles + added;
                    for (const Profile &other : all) {
                        if (other.name.compare(p->name, Qt::CaseInsensitive) == 0) {
                            why = text("a camera named \"%1\" is there already.").arg(other.name);
                            break;
                        }
                    }
                    if (why.isEmpty()) {
                        added.append(*p);
                    }
                }
            }
        }
        if (!why.isEmpty()) {
            _notes.append(QStringLiteral("%1: %2").arg(file.fileName(), why));
        }
    }
    std::sort(added.begin(), added.end(), [](const Profile &a, const Profile &b) {
        return a.name.compare(b.name, Qt::CaseInsensitive) < 0;
    });
    _profiles += added;
}

const Profile *Store::find(const QString &id) const
{
    for (const Profile &p : _profiles) {
        if (p.id == id) {
            return &p;
        }
    }
    return nullptr;
}

QString Store::_nameProblem(const Profile &profile) const
{
    for (const Profile &other : _profiles) {
        if (other.id != profile.id && other.name.compare(profile.name.trimmed(), Qt::CaseInsensitive) == 0) {
            return text("A camera named \"%1\" is there already. Give this one another name.").arg(other.name);
        }
    }
    return QString();
}

QString Store::save(Profile profile, QString *savedId)
{
    profile.builtIn = false;
    profile.name = profile.name.trimmed();
    profile.host = profile.host.trimmed();
    profile.dayVideoUrl = profile.dayVideoUrl.trimmed();
    profile.thermalVideoUrl = profile.thermalVideoUrl.trimmed();
    // An added camera keeps its file. Anything else (a copy of a built-in one, a new one) gets a
    // new file, named after the camera.
    const Profile *known = find(profile.id);
    if (!known || known->builtIn) {
        const QString base = idForName(profile.name);
        QString id = base;
        for (int n = 2; isBuiltInId(id) || find(id) || QFile::exists(QDir(_folder).filePath(id + QStringLiteral(".json"))); ++n) {
            id = QStringLiteral("%1-%2").arg(base).arg(n);
        }
        profile.id = id;
    }
    QString why = problem(profile);
    if (why.isEmpty()) {
        why = _nameProblem(profile);
    }
    if (!why.isEmpty()) {
        return why;
    }
    if (_folder.isEmpty() || !QDir().mkpath(_folder)) {
        return text("The folder for cameras cannot be made: %1").arg(_folder);
    }
    const QString path = QDir(_folder).filePath(profile.id + QStringLiteral(".json"));
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        return text("The camera's file cannot be written: %1").arg(path);
    }
    (void) file.write(QJsonDocument(toJson(profile)).toJson(QJsonDocument::Indented));
    if (!file.commit()) {
        return text("The camera's file cannot be written: %1").arg(path);
    }
    load();
    if (savedId) {
        *savedId = profile.id;
    }
    return QString();
}

QString Store::remove(const QString &id)
{
    const Profile *p = find(id);
    if (!p) {
        return text("There is no such camera.");
    }
    if (p->builtIn) {
        return text("A built-in camera cannot be deleted.");
    }
    const QString path = QDir(_folder).filePath(id + QStringLiteral(".json"));
    if (!QFile::remove(path) && QFile::exists(path)) {
        return text("The camera's file cannot be deleted: %1").arg(path);
    }
    load();
    return QString();
}

}  // namespace skydroid::camera
