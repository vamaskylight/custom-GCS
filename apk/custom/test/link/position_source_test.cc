// Host test for the app's position source (PositionSource.h and PositionSourceStore.h, A8 step 4,
// 2026-10-30): where the drone's position comes from, GPS 1 or the down camera as GPS 2. The rules
// are a port of VGCS's vgcs/app/position_source.py: ../position_source_vectors.inc holds scripts
// and what VGCS's own class said after each step (gen_position_source_vectors.py), and the port must
// say the same. Then the store the screen reads: one source for each drone, its values, the refresh
// once a second (readings expire) and the note on a change in the air.
// Run with apk/custom/test/run_link_test.ps1.

#include <QtQml/QJSEngine>
#include <QtTest/QSignalSpy>
#include <QtTest/QTest>

#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

#include "PositionSource.h"
#include "PositionSourceStore.h"

namespace {

constexpr double kAbsent = -1.0;
const double kNaN = std::numeric_limits<double>::quiet_NaN();

/// One step of a script and what VGCS said after it (see gen_position_source_vectors.py).
struct Step
{
    double t;
    const char *what;
    int fix;           // gps1, gps2: the fix type; armed: 1 or 0
    double accM;       // gps2: the accuracy in metres, 0: not given
    int satellites;    // gps2
    double gps2Type;   // settings, kAbsent: not in them
    double gpsType2;
    double autoSwitch;
    const char *source;
    const char *text;
    const char *level;
    const char *tag;
    const char *tooltip;
    const char *note;  // nullptr: no note
    bool warning;
};

struct Script
{
    const char *name;
    const Step *steps;
    size_t count;
};

std::vector<Script> &scripts()
{
    static std::vector<Script> all;
    return all;
}

bool addScript(const char *name, const Step *steps, size_t count)
{
    scripts().push_back({name, steps, count});
    return true;
}

#define CHECK_SCRIPT(name) const bool kAdded_##name = addScript(#name, kSteps_##name, sizeof(kSteps_##name) / sizeof(Step))
#include "../position_source_vectors.inc"
#undef CHECK_SCRIPT

const char *sourceName(vama::PositionSource::Source source)
{
    switch (source) {
    case vama::PositionSource::Source::Gps1:
        return "gps1";
    case vama::PositionSource::Source::Camera:
        return "camera";
    case vama::PositionSource::Source::SecondGps:
        return "gps2";
    case vama::PositionSource::Source::None:
        return "none";
    case vama::PositionSource::Source::Unknown:
        return "unknown";
    }
    return "?";
}

/// Plays a script through the port. Empty: it said what VGCS said after every step.
QString play(const Script &script)
{
    vama::PositionSource rules;
    for (size_t i = 0; i < script.count; ++i) {
        const Step &s = script.steps[i];
        const std::string what = s.what;
        if (what == "gps1") {
            rules.updateGps1(s.fix, s.t);
        } else if (what == "gps2") {
            rules.updateGps2(s.fix, s.accM, s.satellites, s.t);
        } else if (what == "armed") {
            rules.setArmed(s.fix != 0);
        } else if (what == "settings") {
            std::map<std::string, double> params;
            if (s.gps2Type != kAbsent) {
                params["GPS2_TYPE"] = s.gps2Type;
            }
            if (s.gpsType2 != kAbsent) {
                params["GPS_TYPE2"] = s.gpsType2;
            }
            if (s.autoSwitch != kAbsent) {
                params["GPS_AUTO_SWITCH"] = s.autoSwitch;
            }
            rules.updateSettings(params);
        } else if (what == "reset") {
            rules.reset();
        }
        const QString at = QStringLiteral("%1, step %2 (%3 at %4 s): ").arg(script.name).arg(i).arg(s.what).arg(s.t);
        const auto differs = [&at](const char *field, const std::string &got, const char *want) {
            return got == want ? QString()
                               : at + QStringLiteral("%1 is \"%2\", VGCS: \"%3\"").arg(field, QString::fromStdString(got),
                                                                                      QString::fromUtf8(want));
        };
        for (const QString &difference : {differs("the source", sourceName(rules.source(s.t)), s.source),
                                          differs("the line", rules.fieldText(s.t), s.text),
                                          differs("the colour", rules.level(s.t), s.level),
                                          differs("the tag", rules.headerTag(s.t), s.tag),
                                          differs("the tooltip", rules.tooltip(), s.tooltip)}) {
            if (!difference.isEmpty()) {
                return difference;
            }
        }
        // VGCS takes the note where it shows the source: after a GPS message and once a second.
        const bool shown = what == "gps1" || what == "gps2" || what == "tick";
        const std::optional<vama::PositionSource::Event> note = shown ? rules.takeEvent(s.t) : std::nullopt;
        if (note.has_value() != (s.note != nullptr)) {
            return at + (note ? QStringLiteral("a note \"%1\", VGCS none").arg(QString::fromStdString(note->text))
                              : QStringLiteral("no note, VGCS \"%1\"").arg(QString::fromUtf8(s.note)));
        }
        if (note && (note->text != s.note || note->warning != s.warning)) {
            return at + QStringLiteral("the note \"%1\" (warning %2), VGCS \"%3\" (warning %4)")
                            .arg(QString::fromStdString(note->text)).arg(note->warning)
                            .arg(QString::fromUtf8(s.note)).arg(s.warning);
        }
    }
    return QString();
}

} // namespace

class PositionSourceTest : public QObject
{
    Q_OBJECT

private slots:
    void everyScriptSaysWhatVgcsSays();
    void theScriptsSeeEveryNote();
    void aSettingThatIsNoNumberIsLeftAlone();
    void theStoreShowsTheDronesSource();
    void theNoteComesOnceItLasted2s();
    void theAccuracyComesInMillimetres();
    void readingsExpireOnTheRefresh();
    void changedOnlyWhenSomethingChanged();
    void eachDroneHasItsOwn();
    void aDroneThatWentStartsAgain();
    void theStoreRefreshesOnItsTimer();
    void qmlGetsTheOneStore();
    void qmlNeverOwnsTheStoreOrADrone();
};

void PositionSourceTest::everyScriptSaysWhatVgcsSays()
{
    QVERIFY2(scripts().size() >= 14, "the vectors were read");
    for (const Script &script : scripts()) {
        const QString difference = play(script);
        QVERIFY2(difference.isEmpty(), qPrintable(difference));
    }
}

void PositionSourceTest::theScriptsSeeEveryNote()
{
    // A check that the scripts reach what matters: each kind of note, both colours of the tag, and
    // the words of every source.
    QStringList notes;
    QStringList tags;
    QStringList levels;
    for (const Script &script : scripts()) {
        for (size_t i = 0; i < script.count; ++i) {
            if (script.steps[i].note) {
                notes << QString::fromUtf8(script.steps[i].note);
            }
            tags << QString::fromUtf8(script.steps[i].tag);
            levels << QString::fromUtf8(script.steps[i].level);
        }
    }
    QVERIFY(notes.contains("GPS 1 has no 3D fix: the position comes from the camera (GPS 2)"));
    QVERIFY(notes.contains("GPS 1 has no 3D fix: the position comes from GPS 2"));
    QVERIFY(notes.contains("The position comes from GPS 1 again"));
    QVERIFY(notes.contains("No position: GPS 1 and the camera (GPS 2) have no 3D fix"));
    QVERIFY(tags.contains("CAM") && tags.contains("GPS2"));
    for (const char *level : {"na", "ok", "warn", "bad"}) {
        QVERIFY2(levels.contains(level), level);
    }
}

void PositionSourceTest::aSettingThatIsNoNumberIsLeftAlone()
{
    // VGCS skips a value int(float(...)) cannot read: here not a number, or one no int holds.
    vama::PositionSource rules;
    rules.updateSettings({{"GPS2_TYPE", 14.0}, {"GPS_AUTO_SWITCH", 4.0}});
    rules.updateSettings({{"GPS2_TYPE", kNaN}, {"GPS_AUTO_SWITCH", std::numeric_limits<double>::infinity()}});
    QVERIFY(rules.isCamera());
    rules.updateSettings({{"GPS2_TYPE", 1.0e12}});
    QVERIFY(rules.isCamera());
    rules.updateSettings({{"GPS_AUTO_SWITCH", 1.0}});
    QVERIFY(QString::fromStdString(rules.tooltip()).contains("GPS_AUTO_SWITCH is 1"));
}

void PositionSourceTest::theStoreShowsTheDronesSource()
{
    PositionSourceStore store(nullptr);
    qint64 now = 1000;
    store.setClock([&now] { return now; });
    VehiclePositionSource *s = store.forVehicle(1);
    QVERIFY(!s->hasGps2());
    QCOMPARE(s->text(), QStringLiteral("N/A"));
    QCOMPARE(s->level(), QStringLiteral("na"));

    store.setSettings(1, {{"GPS2_TYPE", 14.0}, {"GPS_AUTO_SWITCH", 4.0}});
    store.hearGps1(1, 3);
    QVERIFY2(!s->hasGps2(), "GPS 1 alone shows nothing new");
    store.hearGps2(1, 3, 1500, 10);
    QVERIFY(s->hasGps2());
    QCOMPARE(s->text(), QStringLiteral("GPS 1 (GPS 2 repeats GPS 1 until the drone is armed)"));
    QCOMPARE(s->level(), QStringLiteral("ok"));
    QCOMPARE(s->tag(), QString());
    QVERIFY(s->details().contains("GPS 2 is the down camera's position"));

    store.setArmed(1, true);
    now = 1500;
    store.hearGps1(1, 1);
    QCOMPARE(s->tag(), QStringLiteral("CAM"));
    QCOMPARE(s->level(), QStringLiteral("warn"));
    QCOMPARE(s->text(), QStringLiteral("The camera (GPS 2), 3D, about 1.5 m (GPS 1 has no 3D fix)"));
}

void PositionSourceTest::theNoteComesOnceItLasted2s()
{
    PositionSourceStore store(nullptr);
    qint64 now = 1000;
    store.setClock([&now] { return now; });
    VehiclePositionSource *s = store.forVehicle(1);
    QSignalSpy said(s, &VehiclePositionSource::said);
    store.setSettings(1, {{"GPS2_TYPE", 14.0}, {"GPS_AUTO_SWITCH", 4.0}});
    store.setArmed(1, true);
    store.hearGps2(1, 3, 1000, 0);
    store.hearGps1(1, 3);
    now = 3100;
    store.publishAll();
    QCOMPARE(said.count(), 0);  // GPS 1 at the start is no news

    store.hearGps1(1, 0);  // the camera from now
    store.hearGps2(1, 3, 1000, 0);
    now = 4500;
    store.publishAll();
    QCOMPARE(said.count(), 0);
    now = 5300;  // (not 5100: 5.1 - 3.1 is a hair under 2 in binary)
    store.hearGps2(1, 3, 1000, 0);
    QCOMPARE(said.count(), 1);
    QCOMPARE(said.at(0).at(0).toString(), QStringLiteral("GPS 1 has no 3D fix: the position comes from the camera (GPS 2)"));
    QCOMPARE(said.at(0).at(1).toBool(), true);
    now = 6300;
    store.publishAll();
    store.hearGps2(1, 3, 1000, 0);
    QCOMPARE(said.count(), 1);  // once
}

void PositionSourceTest::theAccuracyComesInMillimetres()
{
    PositionSourceStore store(nullptr);
    qint64 now = 1000;
    store.setClock([&now] { return now; });
    store.setSettings(1, {{"GPS2_TYPE", 14.0}});
    store.setArmed(1, true);
    store.hearGps1(1, 0);
    store.hearGps2(1, 3, 4250, 0);
    VehiclePositionSource *s = store.forVehicle(1);
    QCOMPARE(s->text(), QStringLiteral("The camera (GPS 2), 3D, about 4.2 m (GPS 1 has no 3D fix)"));
    store.hearGps2(1, 3, 0, 0);
    QCOMPARE(s->text(), QStringLiteral("The camera (GPS 2), 3D (GPS 1 has no 3D fix)"));  // 0: none given
    store.hearGps2(1, 3, 0xFFFFFFFFu, 0);
    QCOMPARE(s->text(), QStringLiteral("The camera (GPS 2), 3D, about 4294967.3 m (GPS 1 has no 3D fix)"));
}

void PositionSourceTest::readingsExpireOnTheRefresh()
{
    PositionSourceStore store(nullptr);
    qint64 now = 1000;
    store.setClock([&now] { return now; });
    store.setSettings(1, {{"GPS2_TYPE", 14.0}});
    store.setArmed(1, true);
    store.hearGps1(1, 0);
    store.hearGps2(1, 3, 2000, 0);
    VehiclePositionSource *s = store.forVehicle(1);
    QCOMPARE(s->tag(), QStringLiteral("CAM"));
    now = 6000;
    store.publishAll();
    QCOMPARE(s->tag(), QStringLiteral("CAM"));  // 5 s old still counts
    now = 6100;
    store.publishAll();
    QCOMPARE(s->tag(), QString());
    QCOMPARE(s->text(), QStringLiteral("N/A"));
    QCOMPARE(s->level(), QStringLiteral("na"));
    QVERIFY2(s->hasGps2(), "the drone has a GPS 2, it only stopped reporting");
}

void PositionSourceTest::changedOnlyWhenSomethingChanged()
{
    PositionSourceStore store(nullptr);
    qint64 now = 1000;
    store.setClock([&now] { return now; });
    store.setSettings(1, {{"GPS2_TYPE", 14.0}});
    store.hearGps1(1, 3);
    VehiclePositionSource *s = store.forVehicle(1);
    QSignalSpy changed(s, &VehiclePositionSource::changed);
    store.hearGps2(1, 3, 1000, 0);
    QCOMPARE(changed.count(), 1);
    now = 1500;
    store.hearGps2(1, 3, 1000, 0);
    store.hearGps1(1, 3);
    store.publishAll();
    QCOMPARE(changed.count(), 1);
    store.setSettings(1, {{"GPS_AUTO_SWITCH", 1.0}});
    store.publishAll();
    QCOMPARE(changed.count(), 2);  // the details now name the setting
    QVERIFY(s->details().contains("GPS_AUTO_SWITCH is 1"));
}

void PositionSourceTest::eachDroneHasItsOwn()
{
    PositionSourceStore store(nullptr);
    qint64 now = 1000;
    store.setClock([&now] { return now; });
    store.setSettings(1, {{"GPS2_TYPE", 14.0}});
    store.setArmed(1, true);
    store.hearGps1(1, 0);
    store.hearGps2(1, 3, 1000, 0);
    store.hearGps1(2, 3);
    QCOMPARE(store.forVehicle(1)->tag(), QStringLiteral("CAM"));
    QCOMPARE(store.forVehicle(2)->tag(), QString());
    QVERIFY(!store.forVehicle(2)->hasGps2());
    QCOMPARE(store.forVehicle(1), store.forVehicle(1));
    QVERIFY(store.forVehicle(1) != store.forVehicle(2));
}

void PositionSourceTest::aDroneThatWentStartsAgain()
{
    PositionSourceStore store(nullptr);
    qint64 now = 1000;
    store.setClock([&now] { return now; });
    store.setSettings(1, {{"GPS2_TYPE", 14.0}});
    store.setArmed(1, true);
    store.hearGps1(1, 0);
    store.hearGps2(1, 3, 1000, 0);
    VehiclePositionSource *s = store.forVehicle(1);
    QSignalSpy changed(s, &VehiclePositionSource::changed);
    store.reset(1);
    QCOMPARE(changed.count(), 1);
    QVERIFY(!s->hasGps2());
    QCOMPARE(s->tag(), QString());
    QCOMPARE(s->text(), QStringLiteral("N/A"));
    QCOMPARE(store.forVehicle(1), s);  // the same object: QML may hold it
    store.hearGps2(1, 3, 1000, 0);
    // Not the camera yet: the settings come again with the next GPS2_RAW (CustomPlugin).
    QCOMPARE(s->text(), QStringLiteral("GPS 2, 3D, about 1.0 m (GPS 1 has no 3D fix)"));
    store.reset(9);  // never seen: nothing is made
    QCOMPARE(changed.count(), 2);
}

void PositionSourceTest::theStoreRefreshesOnItsTimer()
{
    PositionSourceStore store(nullptr);
    qint64 now = 1000;
    store.setClock([&now] { return now; });
    store.setSettings(1, {{"GPS2_TYPE", 14.0}});
    store.hearGps1(1, 0);
    store.hearGps2(1, 3, 1000, 0);
    VehiclePositionSource *s = store.forVehicle(1);
    QCOMPARE(s->tag(), QStringLiteral("CAM"));
    now = 7000;
    QSignalSpy changed(s, &VehiclePositionSource::changed);
    QVERIFY2(changed.wait(PositionSourceStore::kPublishMs + 500), "the tag goes within a second after 5 s");
    QCOMPARE(s->tag(), QString());
}

void PositionSourceTest::qmlGetsTheOneStore()
{
    static_assert(!std::is_default_constructible_v<PositionSourceStore>, "QML would make its own store");
    PositionSourceStore *store = PositionSourceStore::create(nullptr, nullptr);
    QCOMPARE(store, PositionSourceStore::instance());
    QCOMPARE(QJSEngine::objectOwnership(store), QJSEngine::CppOwnership);
}

void PositionSourceTest::qmlNeverOwnsTheStoreOrADrone()
{
    // The QML engine makes an object it gets its own unless C++ set the owner: then it could delete it.
    QJSEngine engine;
    PositionSourceStore *store = PositionSourceStore::create(nullptr, &engine);
    engine.globalObject().setProperty("store", engine.newQObject(store));
    QCOMPARE(QJSEngine::objectOwnership(store), QJSEngine::CppOwnership);
    QObject *source = engine.evaluate("store.forVehicle(7)").toQObject();
    QVERIFY(source != nullptr);
    QCOMPARE(source, store->forVehicle(7));
    QCOMPARE(QJSEngine::objectOwnership(source), QJSEngine::CppOwnership);
}

QTEST_GUILESS_MAIN(PositionSourceTest)
#include "position_source_test.moc"
