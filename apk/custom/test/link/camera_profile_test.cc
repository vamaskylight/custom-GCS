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

}  // namespace

class CameraProfileTest : public QObject
{
    Q_OBJECT

private slots:
    void theBuiltInCamerasAreTheTestedOnes();
    void theDayLensIsTheOldFormula();
    void theZoomIsTheOldOne();
    void aFileGivesTheSameCameraBack();
    void theEditorsFieldsNeedNoFormat();
    void wrongValuesAreRefusedWithASentence();
    void namesBecomeFileNames();
    void savingMakesAFileThatIsReadAgain();
    void namesAreNotTwice();
    void builtInCamerasStay();
    void filesThatAreNoCamerasAreSkippedWithANote();
    void deletingRemovesTheFile();
};

void CameraProfileTest::theBuiltInCamerasAreTheTestedOnes()
{
    const QList<Profile> all = camera::builtInProfiles();
    QCOMPARE(all.size(), 3);
    QCOMPARE(all.at(0).id, QStringLiteral("C12"));
    QCOMPARE(all.at(1).id, QStringLiteral("C13"));
    QCOMPARE(all.at(2).id, QStringLiteral("C14 Pro"));
    QCOMPARE(all.at(0).name, QStringLiteral("V12"));
    QCOMPARE(all.at(1).name, QStringLiteral("V13"));
    QCOMPARE(all.at(2).name, QStringLiteral("V14 Pro"));
    QCOMPARE(camera::defaultId(), QStringLiteral("C13"));
    for (const Profile &p : all) {
        QVERIFY(p.builtIn);
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

void CameraProfileTest::theEditorsFieldsNeedNoFormat()
{
    QJsonObject fields = camera::toJson(added(QStringLiteral("Gimbal X")));
    fields.remove(QStringLiteral("format"));
    fields[QStringLiteral("protocol")] = QStringLiteral("anything");
    fields[QStringLiteral("id")] = QStringLiteral("gimbal-x");
    QString why;
    const std::optional<Profile> p = camera::fromFields(fields, &why);
    QVERIFY2(p.has_value(), qPrintable(why));
    QCOMPARE(p->id, QStringLiteral("gimbal-x"));
    // A file is not taken without them.
    QVERIFY(!camera::fromJson(fields, QStringLiteral("gimbal-x"), &why));
    QVERIFY2(why.contains(QStringLiteral("not a camera file")), qPrintable(why));
    fields[QStringLiteral("format")] = QStringLiteral("vama-camera-1");
    QVERIFY(!camera::fromJson(fields, QStringLiteral("gimbal-x"), &why));
    QVERIFY2(why.contains(QStringLiteral("Skydroid TOP")), qPrintable(why));
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
    QCOMPARE(store.profiles().size(), 3);

    QString id;
    QCOMPARE(store.save(added(QStringLiteral("Gimbal X")), &id), QString());
    QCOMPARE(id, QStringLiteral("gimbal-x"));
    QVERIFY(QFile::exists(QDir(folder).filePath(QStringLiteral("gimbal-x.json"))));
    QCOMPARE(store.profiles().size(), 4);
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
    QCOMPARE(store.profiles().size(), 4);
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
                                 QStringLiteral("C13"), QStringLiteral("Gimbal X!"), QStringLiteral("Gimbal X2")}));

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
    QCOMPARE(store.profiles().size(), 4);
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
    QCOMPARE(store.profiles().size(), 4);
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
    QCOMPARE(store.profiles().size(), 4);
    QVERIFY(store.find(QStringLiteral("good")));
    const QString notes = store.notes().join(QLatin1Char('\n'));
    QCOMPARE(store.notes().size(), 9);
    for (const char *says : {"broken.json: it is not a camera file", "other.json: it is not a camera file",
                             "siyi.json: its protocol is not Skydroid TOP", "noport.json: it has no \"port\"",
                             "badzoom.json: its zoom is not one of", "C13.json: it has the file name of a built-in camera",
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
    QCOMPARE(store.profiles().size(), 3);
    QVERIFY(store.remove(id).contains(QStringLiteral("no such camera")));
}

QTEST_GUILESS_MAIN(CameraProfileTest)
#include "camera_profile_test.moc"
