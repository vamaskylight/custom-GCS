// Host test for the camera profiles (task AA, CameraProfile.h): the built-in cameras are the
// ones the app was tested with (their numbers, and the old formulas of SkydroidLink.cc for
// their lens and zoom, copied here as they were), a camera's file gives the same camera back,
// wrong values are refused with a sentence the editor can show, and the folder of added
// cameras (save, read again, skip files that are not cameras, delete).
// Run with apk/custom/test/run_link_test.ps1.

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QTemporaryDir>
#include <QtTest/QTest>

#include <algorithm>
#include <cmath>
#include <functional>

#include "CameraProfile.h"

namespace camera = skydroid::camera;
using camera::Profile;
using camera::Protocol;
using camera::Zoom;

namespace {

constexpr double kPi = 3.14159265358979323846;

// --- The formulas of SkydroidLink.cc before the camera profiles (2026-10-19), as they were. ---

double oldNarrow(double fovDeg, double ratio)
{
    return 2.0 * std::atan(std::tan(fovDeg * kPi / 360.0) * ratio) * 180.0 / kPi;
}

// currentFov for the C14 Pro (VGCS c14pro_default).
void oldC14Fov(int zoomStep, double &h, double &v)
{
    const int step = zoomStep >= 0 ? zoomStep : 0;
    const bool longLens = step >= 86;
    const int m = longLens ? std::clamp(step - 86, 0, 98) : std::clamp(step, 0, 85);
    h = oldNarrow(longLens ? 14.7 : 61.4, (3840.0 - 36.0 * m) / 3840.0);
    v = oldNarrow(longLens ? 11.1 : 47.9, (2160.0 - 20.0 * m) / 2160.0);
}

// currentFov for the C12 and the C13: 83.4 x 46.9 narrowed by the zoom counted.
void oldCountedFov(int zoomCount, double &h, double &v)
{
    const double zoomX = 1.0 + zoomCount * 0.1;
    h = oldNarrow(83.4, 1.0 / zoomX);
    v = oldNarrow(46.9, 1.0 / zoomX);
}

QString oldC14Label(int step)
{
    const bool longLens = step >= 86;
    const int m = longLens ? std::clamp(step - 86, 0, 98) : std::clamp(step, 0, 85);
    const double crop = 3840.0 / std::max(1, 3840 - 36 * m);
    const double lensFactor = longLens ? std::tan(61.4 * kPi / 360.0) / std::tan(14.7 * kPi / 360.0) : 1.0;
    return QStringLiteral("%1%2x").arg(longLens ? QStringLiteral("T") : QStringLiteral("W")).arg(crop * lensFactor, 0, 'f', 1);
}

Profile builtIn(const QString &id)
{
    for (const Profile &p : camera::builtInProfiles()) {
        if (p.id == id) {
            return p;
        }
    }
    return Profile{};
}

/// A camera that is fine: a copy of the V13 under another name.
Profile added(const QString &name)
{
    Profile p = builtIn(QStringLiteral("C13"));
    p.id.clear();
    p.builtIn = false;
    p.name = name;
    return p;
}

void writeFile(const QString &path, const QByteArray &bytes)
{
    QFile file(path);
    QVERIFY(file.open(QIODevice::WriteOnly));
    file.write(bytes);
}

QByteArray cameraFile(const Profile &p)
{
    return QJsonDocument(camera::toJson(p)).toJson();
}

// The camera files of the check that the app and VGCS read each other's files (VAMA_CAMERA_FILES):
// vgcs/ written by VGCS (apk/custom/test/gen_camera_files.py), apk/ by this test.
QString sampleFolder(const char *who)
{
    return QStringLiteral(VAMA_CAMERA_FILES "/") + QString::fromLatin1(who);
}

/// Both the same, numbers by their value (VGCS writes 30.0 where Qt writes 30). where: the first difference.
bool sameJson(const QJsonValue &a, const QJsonValue &b, const QString &at, QString *where)
{
    if (a.isDouble() && b.isDouble()) {
        if (a.toDouble() == b.toDouble()) {
            return true;
        }
    } else if (a.isArray() && b.isArray()) {
        const QJsonArray x = a.toArray();
        const QJsonArray y = b.toArray();
        if (x.size() == y.size()) {
            for (qsizetype i = 0; i < x.size(); ++i) {
                if (!sameJson(x.at(i), y.at(i), QStringLiteral("%1[%2]").arg(at).arg(i), where)) {
                    return false;
                }
            }
            return true;
        }
    } else if (a.isObject() && b.isObject()) {
        const QJsonObject x = a.toObject();
        const QJsonObject y = b.toObject();
        if (x.keys() == y.keys()) {
            for (const QString &key : x.keys()) {
                if (!sameJson(x.value(key), y.value(key), at + QLatin1Char('.') + key, where)) {
                    return false;
                }
            }
            return true;
        }
    } else if (a == b) {
        return true;
    }
    *where = at;
    return false;
}

}  // namespace

class CameraProfileTest : public QObject
{
    Q_OBJECT

private slots:
    void theBuiltInCamerasAreTheTestedOnes();
    void theDayLensIsTheOldFormula();
    void theZoomIsTheOldOne();
    void aFileGivesTheSameCameraBack();
    void theTransportIsInTheFile();
    void theEditorsFieldsNeedNoFormat();
    void wrongValuesAreRefusedWithASentence();
    void eachProtocolHasItsZoomsAndFeatures();
    void namesBecomeFileNames();
    void savingMakesAFileThatIsReadAgain();
    void namesAreNotTwice();
    void builtInCamerasStay();
    void filesThatAreNoCamerasAreSkippedWithANote();
    void deletingRemovesTheFile();
    void theFilesVgcsWritesAreRead();
    void theAppWritesTheFilesVgcsReads();
};

void CameraProfileTest::theBuiltInCamerasAreTheTestedOnes()
{
    const QList<Profile> all = camera::builtInProfiles();
    QCOMPARE(all.size(), 6);
    QCOMPARE(all.at(0).id, QStringLiteral("C12"));
    QCOMPARE(all.at(1).id, QStringLiteral("C13"));
    QCOMPARE(all.at(2).id, QStringLiteral("C14 Pro"));
    QCOMPARE(all.at(3).id, QStringLiteral("C20"));
    QCOMPARE(all.at(4).id, QStringLiteral("A20KTR"));
    QCOMPARE(all.at(5).id, QStringLiteral("ZR10"));
    QCOMPARE(all.at(0).name, QStringLiteral("V12"));
    QCOMPARE(all.at(1).name, QStringLiteral("V13"));
    QCOMPARE(all.at(2).name, QStringLiteral("V14 Pro"));
    QCOMPARE(all.at(3).name, QStringLiteral("V20"));
    QCOMPARE(all.at(4).name, QStringLiteral("Viewpro A20KTR"));
    QCOMPARE(all.at(5).name, QStringLiteral("SIYI ZR10"));
    QCOMPARE(camera::defaultId(), QStringLiteral("C13"));
    for (const Profile &p : all.mid(0, 4)) {
        QVERIFY(p.protocol == Protocol::Top);
    }
    // The Viewpro A20KTR and the SIYI ZR10 (task AK), with VGCS's numbers
    // (camera_files.apk_built_in_cameras, held equal through the cross-app files).
    const Profile a20 = all.at(4);
    QVERIFY(a20.builtIn && a20.protocol == Protocol::Viewpro && a20.controlOverTcp());
    QCOMPARE(a20.host, QStringLiteral("192.168.2.119"));
    QCOMPARE(a20.port, 2000);
    QCOMPARE(a20.dayVideoUrl, QStringLiteral("rtsp://192.168.2.119:554/stream0"));
    QVERIFY(a20.zoom == Zoom::Live);
    QCOMPARE(a20.fovHDeg, 70.2);
    QCOMPARE(a20.fovVDeg, 43.1);
    QCOMPARE(a20.pitchMaxDeg, 30.0);
    QCOMPARE(a20.yawLimitDeg, 150.0);
    const Profile zr10 = all.at(5);
    QVERIFY(zr10.builtIn && zr10.protocol == Protocol::Siyi && !zr10.controlOverTcp());
    QCOMPARE(zr10.host, QStringLiteral("192.168.144.25"));
    QCOMPARE(zr10.port, 37260);
    QCOMPARE(zr10.dayVideoUrl, QStringLiteral("rtsp://192.168.144.25:8554/main.264"));
    QVERIFY(zr10.zoom == Zoom::Pulse);
    QCOMPARE(zr10.fovHDeg, 71.5);
    QCOMPARE(zr10.fovVDeg, 45.2);
    QCOMPARE(zr10.pitchMaxDeg, 25.0);
    QCOMPARE(zr10.yawLimitDeg, 45.0);
    for (const Profile &p : {a20, zr10}) {
        QVERIFY(!p.laser && !p.ownTracker && p.photo && p.record && !p.upperGimbalHeader);
        QVERIFY(p.thermalVideoUrl.isEmpty() && !p.thermalFovSure);
        QCOMPARE(p.pitchMinDeg, -90.0);
        QVERIFY2(camera::problem(p).isEmpty(), qPrintable(camera::problem(p)));
    }
    // The A20KTR's thermal picture comes in its day video after a switch; the ZR10 has none.
    QVERIFY(a20.thermalBySwitch && a20.hasThermal());
    QVERIFY(!zr10.thermalBySwitch && !zr10.hasThermal());
    // The C20 (task AJ): TOP over TCP, its own video address, no thermal picture or laser.
    const Profile c20 = all.at(3);
    QVERIFY(c20.builtIn && c20.tcp && c20.controlOverTcp());
    QCOMPARE(c20.host, QStringLiteral("192.168.144.108"));
    QCOMPARE(c20.port, 5000);
    QCOMPARE(c20.dayVideoUrl, QStringLiteral("rtsp://192.168.144.108:554/main"));
    QVERIFY(!c20.hasThermal() && !c20.laser && !c20.ownTracker && c20.photo && c20.record);
    QVERIFY(c20.zoom == Zoom::Pulse);  // its lens (task AN, client 2026-10-27)
    QCOMPARE(c20.maxZoom, 22.0);
    QCOMPARE(c20.fovHDeg, 58.3);
    QCOMPARE(c20.fovVDeg, 34.9);
    QCOMPARE(c20.pitchMinDeg, -90.0);
    QCOMPARE(c20.pitchMaxDeg, 45.0);
    QCOMPARE(c20.yawLimitDeg, 150.0);
    QVERIFY(c20.upperGimbalHeader);
    QVERIFY2(camera::problem(c20).isEmpty(), qPrintable(camera::problem(c20)));
    for (const Profile &p : all.mid(0, 3)) {
        QVERIFY(p.builtIn && !p.tcp && !p.controlOverTcp());
        QCOMPARE(p.host, QStringLiteral("192.168.144.108"));
        QCOMPARE(p.port, 5000);
        QCOMPARE(p.dayVideoUrl, QStringLiteral("rtsp://192.168.144.108:554/stream=1"));
        QCOMPARE(p.thermalVideoUrl, QStringLiteral("rtsp://192.168.144.108:555/stream=2"));
        QVERIFY(p.laser && p.photo && p.record && p.ownTracker && p.hasThermal());
        QCOMPARE(p.pitchMinDeg, -90.0);
        QVERIFY2(camera::problem(p).isEmpty(), qPrintable(camera::problem(p)));
    }
    // The limits the link had: the aim's pitch top (C13 +10, C14 Pro +60, else +30) and yaw
    // (C14 Pro 120, else 90), the laser (C14 Pro 1500 m, else the TOP default of 1000 m),
    // the "#TP" header (C14 Pro), and the thermal lens (known for the C14 Pro only).
    const Profile c12 = all.at(0);
    QVERIFY(c12.zoom == Zoom::Counted);
    QCOMPARE(c12.maxZoom, 4.0);
    QCOMPARE(c12.pitchMaxDeg, 30.0);
    QCOMPARE(c12.yawLimitDeg, 90.0);
    QCOMPARE(c12.laserMaxM, 1000.0);
    QVERIFY(!c12.upperGimbalHeader);
    QCOMPARE(c12.thermalFovHDeg, 36.4);
    QCOMPARE(c12.thermalFovVDeg, 27.7);
    QVERIFY(!c12.thermalFovSure);
    const Profile c13 = all.at(1);
    QVERIFY(c13.zoom == Zoom::Counted);
    QCOMPARE(c13.maxZoom, 30.0);
    QCOMPARE(c13.pitchMaxDeg, 10.0);
    QCOMPARE(c13.yawLimitDeg, 90.0);
    QCOMPARE(c13.laserMaxM, 1000.0);
    QVERIFY(!c13.upperGimbalHeader);
    QCOMPARE(c13.thermalFovHDeg, 48.7);
    QCOMPARE(c13.thermalFovVDeg, 38.6);
    QVERIFY(!c13.thermalFovSure);
    const Profile c14 = all.at(2);
    QVERIFY(c14.zoom == Zoom::Reported);
    QCOMPARE(c14.pitchMaxDeg, 60.0);
    QCOMPARE(c14.yawLimitDeg, 120.0);
    QCOMPARE(c14.laserMaxM, 1500.0);
    QVERIFY(c14.upperGimbalHeader);
    QCOMPARE(c14.thermalFovHDeg, 32.84);
    QCOMPARE(c14.thermalFovVDeg, 26.35);
    QVERIFY(c14.thermalFovSure);
    QCOMPARE(int(c14.lenses.size()), 2);
    QCOMPARE(c14.lenses[0].firstStep, 0);
    QCOMPARE(c14.lenses[0].lastStep, 85);
    QCOMPARE(c14.lenses[1].firstStep, 86);
    QCOMPARE(c14.lenses[1].lastStep, 184);
}

void CameraProfileTest::theDayLensIsTheOldFormula()
{
    double h = 0.0;
    double v = 0.0;
    double oldH = 0.0;
    double oldV = 0.0;
    for (const QString &id : {QStringLiteral("C12"), QStringLiteral("C13")}) {
        for (int count : {0, 1, 10, 29, 30, 100, 290}) {
            camera::dayFov(builtIn(id), count, -1, h, v);
            oldCountedFov(count, oldH, oldV);
            QCOMPARE(h, oldH);
            QCOMPARE(v, oldV);
        }
    }
    for (int step : {-1, 0, 1, 50, 85, 86, 87, 120, 184, 185, 300}) {
        camera::dayFov(builtIn(QStringLiteral("C14 Pro")), 0, step, h, v);
        oldC14Fov(step, oldH, oldV);
        QCOMPARE(h, oldH);
        QCOMPARE(v, oldV);
    }
    // No zoom: the lens as it is.
    Profile fixed = added(QStringLiteral("Fixed"));
    fixed.zoom = Zoom::None;
    fixed.fovHDeg = 50.0;
    fixed.fovVDeg = 30.0;
    camera::dayFov(fixed, 12, 40, h, v);
    QCOMPARE(h, 50.0);
    QCOMPARE(v, 30.0);
    // A Viewpro says its own view (the link takes that): this is its 1x view until it does. A SIYI
    // does not say how far it zoomed: its 1x view, right only at 1x.
    camera::dayFov(builtIn(QStringLiteral("A20KTR")), 12, 40, h, v);
    QCOMPARE(h, 70.2);
    QCOMPARE(v, 43.1);
    camera::dayFov(builtIn(QStringLiteral("ZR10")), 12, 40, h, v);
    QCOMPARE(h, 71.5);
    QCOMPARE(v, 45.2);
}

void CameraProfileTest::theZoomIsTheOldOne()
{
    QCOMPARE(camera::countedStepsMax(builtIn(QStringLiteral("C12"))), 30);
    QCOMPARE(camera::countedStepsMax(builtIn(QStringLiteral("C13"))), 290);
    QCOMPARE(camera::countedStepsMax(builtIn(QStringLiteral("C14 Pro"))), 0);
    QCOMPARE(camera::zoomLabel(builtIn(QStringLiteral("C13")), 0, -1), QStringLiteral("1.0x"));
    QCOMPARE(camera::zoomLabel(builtIn(QStringLiteral("C13")), 10, -1), QStringLiteral("2.0x"));
    QCOMPARE(camera::zoomLabel(builtIn(QStringLiteral("C12")), 30, -1), QStringLiteral("4.0x"));
    // The C14 Pro before its first report: nothing, rather than a guess.
    QCOMPARE(camera::zoomLabel(builtIn(QStringLiteral("C14 Pro")), 0, -1), QString());
    for (int step : {0, 1, 50, 85, 86, 87, 120, 184, 185, 300}) {
        QCOMPARE(camera::zoomLabel(builtIn(QStringLiteral("C14 Pro")), 0, step), oldC14Label(step));
    }
    Profile fixed = added(QStringLiteral("Fixed"));
    fixed.zoom = Zoom::None;
    QCOMPARE(camera::zoomLabel(fixed, 5, 5), QString());
    QCOMPARE(camera::countedStepsMax(fixed), 0);
    // One lens: no W or T.
    Profile one = builtIn(QStringLiteral("C14 Pro"));
    one.lenses.resize(1);
    QCOMPARE(camera::zoomLabel(one, 0, 0), QStringLiteral("1.0x"));
    // Nothing counted for a Viewpro (the link shows the zoom it says) or a SIYI (no one knows it).
    for (const QString &id : {QStringLiteral("A20KTR"), QStringLiteral("ZR10")}) {
        QCOMPARE(camera::countedStepsMax(builtIn(id)), 0);
        QCOMPARE(camera::zoomLabel(builtIn(id), 5, 5), QString());
    }
}

void CameraProfileTest::aFileGivesTheSameCameraBack()
{
    for (Profile p : camera::builtInProfiles()) {
        p.builtIn = false;
        QString why;
        const std::optional<Profile> back = camera::fromJson(camera::toJson(p), QStringLiteral("x"), &why);
        QVERIFY2(back.has_value(), qPrintable(why));
        QCOMPARE(back->id, QStringLiteral("x"));
        QVERIFY(!back->builtIn);
        QCOMPARE(camera::toJson(*back), camera::toJson(p));
    }
    // A camera without a thermal picture, laser, photo, record or tracker of its own.
    Profile bare = added(QStringLiteral("Bare"));
    bare.thermalVideoUrl.clear();
    bare.laser = false;
    bare.photo = false;
    bare.record = false;
    bare.ownTracker = false;
    bare.zoom = Zoom::None;
    QString why;
    const std::optional<Profile> back = camera::fromJson(camera::toJson(bare), QStringLiteral("bare"), &why);
    QVERIFY2(back.has_value(), qPrintable(why));
    QVERIFY(!back->hasThermal() && !back->laser && !back->photo && !back->record && !back->ownTracker);
    QVERIFY(back->zoom == Zoom::None);
}

void CameraProfileTest::theTransportIsInTheFile()
{
    const QList<Profile> all = camera::builtInProfiles();
    QCOMPARE(camera::toJson(all.at(1)).value(QStringLiteral("transport")).toString(), QStringLiteral("udp"));
    QCOMPARE(camera::toJson(all.at(3)).value(QStringLiteral("transport")).toString(), QStringLiteral("tcp"));
    // A file from before the C20 has no transport: UDP.
    QJsonObject old = camera::toJson(all.at(3));
    old.remove(QStringLiteral("transport"));
    QString why;
    std::optional<Profile> p = camera::fromJson(old, QStringLiteral("old"), &why);
    QVERIFY2(p.has_value(), qPrintable(why));
    QVERIFY(!p->tcp);
    // Anything else is refused, with VGCS's sentence.
    for (const QJsonValue &wrong : {QJsonValue(QStringLiteral("serial")), QJsonValue(QJsonValue::Null), QJsonValue(true),
                                    QJsonValue(QStringLiteral("TCP"))}) {
        QJsonObject bad = camera::toJson(all.at(3));
        bad[QStringLiteral("transport")] = wrong;
        why.clear();
        QVERIFY(!camera::fromJson(bad, QStringLiteral("bad"), &why).has_value());
        QCOMPARE(why, QStringLiteral("its transport is not \"udp\" or \"tcp\"."));
    }
    // The editor's fields carry it.
    QJsonObject fields = camera::toJson(all.at(1));
    fields[QStringLiteral("transport")] = QStringLiteral("tcp");
    fields[QStringLiteral("id")] = QString();
    p = camera::fromFields(fields, &why);
    QVERIFY2(p.has_value(), qPrintable(why));
    QVERIFY(p->tcp);
    // Only a TOP camera chooses: Viewpro takes its commands over TCP, SIYI over UDP (task AK),
    // whatever the file says. A wrong value is still wrong.
    Profile a20 = all.at(4);
    Profile zr10 = all.at(5);
    a20.tcp = false;
    zr10.tcp = true;
    QCOMPARE(camera::toJson(a20).value(QStringLiteral("transport")).toString(), QStringLiteral("tcp"));
    QCOMPARE(camera::toJson(zr10).value(QStringLiteral("transport")).toString(), QStringLiteral("udp"));
    QJsonObject viewpro = camera::toJson(all.at(4));
    viewpro[QStringLiteral("transport")] = QStringLiteral("udp");
    p = camera::fromJson(viewpro, QStringLiteral("v"), &why);
    QVERIFY2(p.has_value(), qPrintable(why));
    QVERIFY(p->controlOverTcp());
    QJsonObject siyi = camera::toJson(all.at(5));
    siyi[QStringLiteral("transport")] = QStringLiteral("tcp");
    p = camera::fromJson(siyi, QStringLiteral("s"), &why);
    QVERIFY2(p.has_value(), qPrintable(why));
    QVERIFY(!p->controlOverTcp());
    siyi.remove(QStringLiteral("transport"));
    p = camera::fromJson(siyi, QStringLiteral("s"), &why);
    QVERIFY2(p.has_value() && !p->controlOverTcp(), qPrintable(why));
    viewpro[QStringLiteral("transport")] = QStringLiteral("serial");
    QVERIFY(!camera::fromJson(viewpro, QStringLiteral("v"), &why).has_value());
    QCOMPARE(why, QStringLiteral("its transport is not \"udp\" or \"tcp\"."));
}

void CameraProfileTest::theEditorsFieldsNeedNoFormat()
{
    QJsonObject fields = camera::toJson(added(QStringLiteral("Gimbal X")));
    fields.remove(QStringLiteral("format"));
    fields.remove(QStringLiteral("protocol"));
    fields[QStringLiteral("id")] = QStringLiteral("gimbal-x");
    QString why;
    std::optional<Profile> p = camera::fromFields(fields, &why);
    QVERIFY2(p.has_value(), qPrintable(why));
    QCOMPARE(p->id, QStringLiteral("gimbal-x"));
    QVERIFY(p->protocol == Protocol::Top);  // fields without a protocol: Skydroid TOP
    // The editor's protocol is taken (task AK), and one the app does not speak is refused.
    QJsonObject viewpro = camera::toJson(builtIn(QStringLiteral("A20KTR")));
    viewpro.remove(QStringLiteral("format"));
    viewpro[QStringLiteral("name")] = QStringLiteral("Roof Viewpro");
    viewpro[QStringLiteral("id")] = QString();
    p = camera::fromFields(viewpro, &why);
    QVERIFY2(p.has_value(), qPrintable(why));
    QVERIFY(p->protocol == Protocol::Viewpro && p->zoom == Zoom::Live && p->controlOverTcp());
    fields[QStringLiteral("protocol")] = QStringLiteral("anything");
    QVERIFY(!camera::fromFields(fields, &why));
    QCOMPARE(why, QStringLiteral("its protocol is not \"skydroid-top\", \"viewpro\" or \"siyi\"."));
    // A file is not taken without its format.
    fields[QStringLiteral("protocol")] = QStringLiteral("skydroid-top");
    QVERIFY(!camera::fromJson(fields, QStringLiteral("gimbal-x"), &why));
    QVERIFY2(why.contains(QStringLiteral("not a camera file")), qPrintable(why));
}

// The same sentences as VGCS's (tests/test_camera_files.py, test_each_protocol_has_its_zooms_and_features).
void CameraProfileTest::eachProtocolHasItsZoomsAndFeatures()
{
    const Profile a20 = builtIn(QStringLiteral("A20KTR"));
    const Profile zr10 = builtIn(QStringLiteral("ZR10"));
    const Profile top = added(QStringLiteral("Gimbal X"));
    struct Case
    {
        Profile camera;
        std::function<void(Profile &)> change;
        const char *says;
    };
    const QList<Case> cases = {
        {top, [](Profile &p) { p.zoom = Zoom::Live; }, "A Skydroid TOP camera's zoom is none, counted, reported or pulse."},
        {a20, [](Profile &p) { p.zoom = Zoom::Counted; }, "A Viewpro camera's zoom is none or live."},
        {a20, [](Profile &p) { p.zoom = Zoom::Pulse; }, "A Viewpro camera's zoom is none or live."},
        {zr10, [](Profile &p) { p.zoom = Zoom::Live; }, "A SIYI camera's zoom is none or pulse."},
        {zr10, [](Profile &p) { p.zoom = Zoom::Reported; }, "A SIYI camera's zoom is none or pulse."},
        {a20, [](Profile &p) { p.laser = true; }, "The app has no laser for a Viewpro or SIYI camera yet. Untick Laser."},
        {zr10, [](Profile &p) { p.laser = true; }, "The app has no laser for a Viewpro or SIYI camera yet. Untick Laser."},
        {a20, [](Profile &p) { p.ownTracker = true; },
         "The app uses no tracker of a Viewpro or SIYI camera. Untick \"Its own object tracker\"."},
        {top, [](Profile &p) { p.protocol = static_cast<Protocol>(9); },
         "The protocol must be \"skydroid-top\", \"viewpro\" or \"siyi\"."},
        {zr10, [](Profile &p) { p.thermalBySwitch = true; }, "Only a Viewpro camera switches its day video to the thermal picture."},
        {top, [](Profile &p) { p.thermalBySwitch = true; }, "Only a Viewpro camera switches its day video to the thermal picture."},
        {a20, [](Profile &p) { p.thermalVideoUrl = QStringLiteral("rtsp://192.168.2.119:554/stream1"); },
         "A camera that switches to its thermal picture has no thermal video of its own: leave it empty."},
        {a20, [](Profile &p) { p.thermalFovHDeg = 0.5; }, "The thermal lens's view must be 1 to 179 degrees."},
    };
    for (const Case &c : cases) {
        Profile p = c.camera;
        c.change(p);
        QCOMPARE(camera::problem(p), QString::fromLatin1(c.says));
    }
    for (Profile p : {a20, zr10, top}) {
        p.zoom = Zoom::None;
        p.maxZoom = 0.0;
        QVERIFY2(camera::problem(p).isEmpty(), qPrintable(camera::problem(p)));
    }
    // A TOP camera with a lens that zooms while told to and says nothing, like the C20 (task AN).
    {
        Profile p = top;
        p.zoom = Zoom::Pulse;
        QVERIFY2(camera::problem(p).isEmpty(), qPrintable(camera::problem(p)));
        QCOMPARE(builtIn(QStringLiteral("C20")).zoom, Zoom::Pulse);
        QString why;
        const std::optional<Profile> back = camera::fromJson(camera::toJson(p), QStringLiteral("lens-cam"), &why);
        QVERIFY2(back, qPrintable(why));
        QVERIFY(back->zoom == Zoom::Pulse);
    }
    // A file's zoom is checked against its protocol once it is read.
    QJsonObject json = camera::toJson(a20);
    json[QStringLiteral("zoom")] = QStringLiteral("counted");
    QString why;
    QVERIFY(!camera::fromJson(json, QStringLiteral("x"), &why));
    QCOMPARE(why, QStringLiteral("A Viewpro camera's zoom is none or live."));
    // A live or pulse zoom has the 1x view of the day lens, as none and counted do.
    json = camera::toJson(a20);
    json.remove(QStringLiteral("dayFovH"));
    QVERIFY(!camera::fromJson(json, QStringLiteral("x"), &why));
    QCOMPARE(why, QStringLiteral("it has no \"dayFovH\"."));
    json = camera::toJson(zr10);
    json.remove(QStringLiteral("maxZoom"));
    QVERIFY2(camera::fromJson(json, QStringLiteral("x"), &why), qPrintable(why));
    // The switch is read, with the thermal lens it needs; a file from before has none.
    json = camera::toJson(a20);
    QCOMPARE(json.value(QStringLiteral("thermalBySwitch")), QJsonValue(true));
    json.remove(QStringLiteral("thermalFovH"));
    QVERIFY(!camera::fromJson(json, QStringLiteral("x"), &why));
    QCOMPARE(why, QStringLiteral("it has no \"thermalFovH\"."));
    json = camera::toJson(top);
    json.remove(QStringLiteral("thermalBySwitch"));
    const std::optional<Profile> old = camera::fromJson(json, QStringLiteral("x"), &why);
    QVERIFY2(old && !old->thermalBySwitch, qPrintable(why));
}

void CameraProfileTest::wrongValuesAreRefusedWithASentence()
{
    struct Case
    {
        const char *what;
        std::function<void(Profile &)> change;
        const char *says;
    };
    const QList<Case> cases = {
        {"no name", [](Profile &p) { p.name = QStringLiteral("  "); }, "name"},
        {"a long name", [](Profile &p) { p.name = QString(41, QLatin1Char('a')); }, "too long"},
        {"a host name", [](Profile &p) { p.host = QStringLiteral("camera.local"); }, "IP address"},
        {"a wrong address", [](Profile &p) { p.host = QStringLiteral("300.1.1.1"); }, "IP address"},
        {"port 0", [](Profile &p) { p.port = 0; }, "port"},
        {"port 70000", [](Profile &p) { p.port = 70000; }, "port"},
        {"no day video", [](Profile &p) { p.dayVideoUrl.clear(); }, "day video"},
        {"day video not RTSP", [](Profile &p) { p.dayVideoUrl = QStringLiteral("http://1.2.3.4/v"); }, "day video"},
        {"thermal video not RTSP", [](Profile &p) { p.thermalVideoUrl = QStringLiteral("udp://1.2.3.4:5600"); }, "thermal video"},
        {"a day lens of 0", [](Profile &p) { p.fovHDeg = 0.0; }, "day lens"},
        {"a day lens of 180", [](Profile &p) { p.fovVDeg = 180.0; }, "day lens"},
        {"a highest zoom of 1x", [](Profile &p) { p.maxZoom = 1.0; }, "highest zoom"},
        {"a highest zoom of 101x", [](Profile &p) { p.maxZoom = 101.0; }, "highest zoom"},
        {"no lens", [](Profile &p) { p.zoom = Zoom::Reported; p.lenses.clear(); }, "lenses"},
        {"five lenses", [](Profile &p) {
             p.zoom = Zoom::Reported;
             p.lenses = {camera::Lens{60, 40, 0, 9}, camera::Lens{50, 30, 10, 19}, camera::Lens{40, 20, 20, 29},
                         camera::Lens{30, 15, 30, 39}, camera::Lens{20, 10, 40, 49}};
         }, "lenses"},
        {"lenses that overlap", [](Profile &p) {
             p.zoom = Zoom::Reported;
             p.lenses = {camera::Lens{60, 40, 0, 50}, camera::Lens{20, 10, 50, 90}};
         }, "must start after"},
        {"a lens's steps backwards", [](Profile &p) {
             p.zoom = Zoom::Reported;
             p.lenses = {camera::Lens{60, 40, 10, 5}};
         }, "steps"},
        {"a crop past the sensor", [](Profile &p) {
             p.zoom = Zoom::Reported;
             p.lenses = {camera::Lens{60, 40, 0, 200}};
         }, "whole sensor"},
        {"a thermal lens of 0", [](Profile &p) { p.thermalFovHDeg = 0.0; }, "thermal lens"},
        {"a tilt down that is up", [](Profile &p) { p.pitchMinDeg = 10.0; }, "tilt"},
        {"a tilt up that is down", [](Profile &p) { p.pitchMaxDeg = -5.0; }, "tilt"},
        {"a turn of 0", [](Profile &p) { p.yawLimitDeg = 0.0; }, "turn"},
        {"a turn of 400", [](Profile &p) { p.yawLimitDeg = 400.0; }, "turn"},
        {"a laser of 2 m", [](Profile &p) { p.laserMaxM = 2.0; }, "laser"},
        {"a laser of 7000 m", [](Profile &p) { p.laserMaxM = 7000.0; }, "laser"},
        {"a number that is no number", [](Profile &p) { p.fovHDeg = std::nan(""); }, "day lens"},
    };
    for (const Case &c : cases) {
        Profile p = added(QStringLiteral("Gimbal X"));
        c.change(p);
        const QString says = camera::problem(p);
        QVERIFY2(says.contains(QLatin1String(c.says), Qt::CaseInsensitive),
                 qPrintable(QStringLiteral("%1: \"%2\"").arg(QLatin1String(c.what), says)));
    }
    // What a feature that is off does not need.
    Profile p = added(QStringLiteral("Gimbal X"));
    p.laser = false;
    p.laserMaxM = 0.0;
    p.thermalVideoUrl.clear();
    p.thermalFovHDeg = 0.0;
    p.zoom = Zoom::None;
    p.maxZoom = 0.0;
    QVERIFY2(camera::problem(p).isEmpty(), qPrintable(camera::problem(p)));
}

void CameraProfileTest::namesBecomeFileNames()
{
    QCOMPARE(camera::idForName(QStringLiteral("Gimbal X 30x")), QStringLiteral("gimbal-x-30x"));
    QCOMPARE(camera::idForName(QStringLiteral("  Gimbal -- X  ")), QStringLiteral("gimbal-x"));
    QCOMPARE(camera::idForName(QStringLiteral("../../evil")), QStringLiteral("evil"));
    QCOMPARE(camera::idForName(QStringLiteral(" --- ")), QStringLiteral("camera"));
}

void CameraProfileTest::savingMakesAFileThatIsReadAgain()
{
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    const QString folder = dir.filePath(QStringLiteral("Cameras"));
    camera::Store store;
    store.setFolder(folder);
    store.load();
    QVERIFY(QDir(folder).exists());  // made, so files can be copied in
    QCOMPARE(store.profiles().size(), 6);

    QString id;
    QCOMPARE(store.save(added(QStringLiteral("Gimbal X")), &id), QString());
    QCOMPARE(id, QStringLiteral("gimbal-x"));
    QVERIFY(QFile::exists(QDir(folder).filePath(QStringLiteral("gimbal-x.json"))));
    QCOMPARE(store.profiles().size(), 7);
    QVERIFY(store.find(id) && !store.find(id)->builtIn);

    // Another store reads the file as it was saved.
    camera::Store again;
    again.setFolder(folder);
    again.load();
    QVERIFY(again.find(id));
    QCOMPARE(camera::toJson(*again.find(id)), camera::toJson(*store.find(id)));

    // Saved again with its id: the same file, changed.
    Profile changed = *store.find(id);
    changed.pitchMaxDeg = 25.0;
    changed.name = QStringLiteral("Gimbal X2");
    QCOMPARE(store.save(changed, &id), QString());
    QCOMPARE(id, QStringLiteral("gimbal-x"));
    QCOMPARE(store.profiles().size(), 7);
    QCOMPARE(store.find(id)->pitchMaxDeg, 25.0);
    QCOMPARE(store.find(id)->name, QStringLiteral("Gimbal X2"));

    // Another camera whose name gives the same file name: a number is added.
    QCOMPARE(store.save(added(QStringLiteral("Gimbal X!")), &id), QString());
    QCOMPARE(id, QStringLiteral("gimbal-x-2"));
    // A camera named like a built-in camera's id: the file is never "c13.json".
    QCOMPARE(store.save(added(QStringLiteral("C13")), &id), QString());
    QCOMPARE(id, QStringLiteral("c13-2"));

    // The list: the built-in ones first, then the added ones by name.
    QStringList names;
    for (const Profile &p : store.profiles()) {
        names.append(p.name);
    }
    QCOMPARE(names, QStringList({QStringLiteral("V12"), QStringLiteral("V13"), QStringLiteral("V14 Pro"),
                                 QStringLiteral("V20"), QStringLiteral("Viewpro A20KTR"), QStringLiteral("SIYI ZR10"),
                                 QStringLiteral("C13"), QStringLiteral("Gimbal X!"), QStringLiteral("Gimbal X2")}));
    // A camera named like the ZR10's id: the file is never "zr10.json" either.
    QCOMPARE(store.save(added(QStringLiteral("ZR10")), &id), QString());
    QCOMPARE(id, QStringLiteral("zr10-2"));

    // Wrong values: refused, nothing written.
    Profile wrong = added(QStringLiteral("Wrong"));
    wrong.port = 0;
    QVERIFY(store.save(wrong, &id).contains(QStringLiteral("port")));
    QVERIFY(!QFile::exists(QDir(folder).filePath(QStringLiteral("wrong.json"))));
}

void CameraProfileTest::namesAreNotTwice()
{
    QTemporaryDir dir;
    camera::Store store;
    store.setFolder(dir.path());
    store.load();
    QString id;
    QCOMPARE(store.save(added(QStringLiteral("Gimbal X")), &id), QString());
    QVERIFY(store.save(added(QStringLiteral("gimbal x")), &id).contains(QStringLiteral("there already")));
    QVERIFY(store.save(added(QStringLiteral("V13")), &id).contains(QStringLiteral("there already")));
    QVERIFY(store.save(added(QStringLiteral("siyi zr10")), &id).contains(QStringLiteral("there already")));
    QCOMPARE(store.profiles().size(), 7);
}

void CameraProfileTest::builtInCamerasStay()
{
    QTemporaryDir dir;
    camera::Store store;
    store.setFolder(dir.path());
    store.load();
    // A built-in camera saved under another name is a copy: a new file, and the V13 as it was.
    Profile copy = builtIn(QStringLiteral("C13"));
    copy.name = QStringLiteral("My V13");
    copy.pitchMaxDeg = 20.0;
    QString id;
    QCOMPARE(store.save(copy, &id), QString());
    QCOMPARE(id, QStringLiteral("my-v13"));
    QCOMPARE(store.find(QStringLiteral("C13"))->pitchMaxDeg, 10.0);
    QVERIFY(store.find(QStringLiteral("C13"))->builtIn);
    QVERIFY(store.remove(QStringLiteral("C13")).contains(QStringLiteral("built-in")));
    QVERIFY(store.remove(QStringLiteral("A20KTR")).contains(QStringLiteral("built-in")));
    QCOMPARE(store.profiles().size(), 7);
}

void CameraProfileTest::filesThatAreNoCamerasAreSkippedWithANote()
{
    QTemporaryDir dir;
    const QDir folder(dir.path());
    writeFile(folder.filePath(QStringLiteral("good.json")), cameraFile(added(QStringLiteral("Good"))));
    writeFile(folder.filePath(QStringLiteral("broken.json")), QByteArrayLiteral("{ \"name\": "));
    QJsonObject other = camera::toJson(added(QStringLiteral("Other")));
    other[QStringLiteral("format")] = QStringLiteral("someone-else-1");
    writeFile(folder.filePath(QStringLiteral("other.json")), QJsonDocument(other).toJson());
    QJsonObject visca = camera::toJson(added(QStringLiteral("Visca")));
    visca[QStringLiteral("protocol")] = QStringLiteral("visca");
    writeFile(folder.filePath(QStringLiteral("visca.json")), QJsonDocument(visca).toJson());
    // A V13's file that says SIYI: its counted zoom is no SIYI zoom.
    QJsonObject siyi = camera::toJson(added(QStringLiteral("Siyi")));
    siyi[QStringLiteral("protocol")] = QStringLiteral("siyi");
    writeFile(folder.filePath(QStringLiteral("siyi.json")), QJsonDocument(siyi).toJson());
    QJsonObject noPort = camera::toJson(added(QStringLiteral("No port")));
    noPort.remove(QStringLiteral("port"));
    writeFile(folder.filePath(QStringLiteral("noport.json")), QJsonDocument(noPort).toJson());
    QJsonObject badZoom = camera::toJson(added(QStringLiteral("Bad zoom")));
    badZoom[QStringLiteral("zoom")] = QStringLiteral("optical");
    writeFile(folder.filePath(QStringLiteral("badzoom.json")), QJsonDocument(badZoom).toJson());
    writeFile(folder.filePath(QStringLiteral("C13.json")), cameraFile(added(QStringLiteral("Fake V13"))));
    writeFile(folder.filePath(QStringLiteral("twin.json")), cameraFile(added(QStringLiteral("good"))));
    Profile wrong = added(QStringLiteral("Wrong"));
    wrong.yawLimitDeg = 0.0;
    writeFile(folder.filePath(QStringLiteral("wrong.json")), cameraFile(wrong));
    writeFile(folder.filePath(QStringLiteral("big.json")), QByteArray(70 * 1024, ' '));
    writeFile(folder.filePath(QStringLiteral("readme.txt")), QByteArrayLiteral("not a camera"));

    camera::Store store;
    store.setFolder(dir.path());
    store.load();
    QCOMPARE(store.profiles().size(), 7);
    QVERIFY(store.find(QStringLiteral("good")));
    const QString notes = store.notes().join(QLatin1Char('\n'));
    QCOMPARE(store.notes().size(), 10);
    for (const char *says : {"broken.json: it is not a camera file", "other.json: it is not a camera file",
                             "visca.json: its protocol is not \"skydroid-top\", \"viewpro\" or \"siyi\".",
                             "siyi.json: A SIYI camera's zoom is none or pulse.", "noport.json: it has no \"port\"",
                             "badzoom.json: its zoom is not one of none, counted, reported, live or pulse.",
                             "C13.json: it has the file name of a built-in camera",
                             "twin.json: a camera named \"Good\" is there already", "wrong.json: The turn either way",
                             "big.json: it is too big"}) {
        QVERIFY2(notes.contains(QLatin1String(says)), qPrintable(QStringLiteral("%1 not in:\n%2").arg(QLatin1String(says), notes)));
    }
}

void CameraProfileTest::deletingRemovesTheFile()
{
    QTemporaryDir dir;
    camera::Store store;
    store.setFolder(dir.path());
    store.load();
    QString id;
    QCOMPARE(store.save(added(QStringLiteral("Gimbal X")), &id), QString());
    QCOMPARE(store.remove(id), QString());
    QVERIFY(!QFile::exists(QDir(dir.path()).filePath(id + QStringLiteral(".json"))));
    QVERIFY(!store.find(id));
    QCOMPARE(store.profiles().size(), 6);
    QVERIFY(store.remove(id).contains(QStringLiteral("no such camera")));
}

// A camera added in VGCS works in the app once its file is copied over: the app reads every
// file VGCS wrote, and its own file of that camera has the same values.
void CameraProfileTest::theFilesVgcsWritesAreRead()
{
    camera::Store store;
    store.setFolder(sampleFolder("vgcs"));
    store.load();
    QVERIFY2(store.notes().isEmpty(), qPrintable(store.notes().join(QLatin1Char('\n'))));
    const QFileInfoList files = QDir(sampleFolder("vgcs")).entryInfoList({QStringLiteral("*.json")}, QDir::Files, QDir::Name);
    QCOMPARE(files.size(), 10);  // with a Viewpro and a SIYI camera, which VGCS writes for the app
    for (const QFileInfo &file : files) {
        const Profile *p = store.find(file.completeBaseName());
        QVERIFY2(p && !p->builtIn, qPrintable(file.fileName()));
        QFile f(file.filePath());
        QVERIFY(f.open(QIODevice::ReadOnly));
        QString where;
        QVERIFY2(sameJson(camera::toJson(*p), QJsonDocument::fromJson(f.readAll()).object(), file.fileName(), &where),
                 qPrintable(where));
    }
}

// The app's own files of the same cameras, as it saves them: VGCS's test reads these
// (tests/test_camera_files_cross_app.py). VAMA_WRITE_CAMERA_FILES=1 writes them again.
void CameraProfileTest::theAppWritesTheFilesVgcsReads()
{
    camera::Store store;
    store.setFolder(sampleFolder("vgcs"));
    store.load();
    const bool write = qEnvironmentVariableIntValue("VAMA_WRITE_CAMERA_FILES") == 1;
    QDir apk(sampleFolder("apk"));
    if (write) {
        QVERIFY(QDir().mkpath(apk.path()));
    }
    int added = 0;
    for (const Profile &p : store.profiles()) {
        if (p.builtIn) {
            continue;
        }
        ++added;
        const QString path = apk.filePath(p.id + QStringLiteral(".json"));
        const QByteArray bytes = QJsonDocument(camera::toJson(p)).toJson(QJsonDocument::Indented);
        if (write) {
            writeFile(path, bytes);
            continue;
        }
        QFile f(path);
        QVERIFY2(f.open(QIODevice::ReadOnly), qPrintable(path + QStringLiteral(" is missing: run once with VAMA_WRITE_CAMERA_FILES=1")));
        QCOMPARE(f.readAll(), bytes);
    }
    QCOMPARE(added, 10);
    QCOMPARE(apk.entryList({QStringLiteral("*.json")}, QDir::Files).size(), added);
}

QTEST_GUILESS_MAIN(CameraProfileTest)
#include "camera_profile_test.moc"
