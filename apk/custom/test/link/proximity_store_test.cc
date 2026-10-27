// Host test for the app's obstacle sensor store (ProximityStore.h, A7, 2026-10-27): only the proximity
// sensor's own DISTANCE_SENSOR messages count (ids 10 to 17, as ArduPilot sends them), each with its
// own range, an empty direction has no value, and a direction not heard for 2 s has none either. QGC's
// own values (VehicleDistanceSensorFactGroup) froze when the sensor stopped, took the range of any
// sensor, and showed an empty direction of the RPLidar A3 (range 0) as "0.00".
// Run with apk/custom/test/run_link_test.ps1.

#include <QtQml/QJSEngine>
#include <QtTest/QSignalSpy>
#include <QtTest/QTest>

#include <cmath>
#include <type_traits>

#include "ProximityStore.h"

namespace {

// What ArduPilot 4.7.0 sends (GCS_Common.cpp send_proximity): id 10 + direction, orientation = direction.
constexpr int kAhead = 10;
constexpr int kRight = 12;

bool none(double v)
{
    return std::isnan(v);
}

} // namespace

class ProximityStoreTest : public QObject
{
    Q_OBJECT

private slots:
    void theEightDirectionsInTheirPlaces();
    void anEmptyDirectionHasNoValue();
    void theA3sRangeIsZero();
    void onlyTheProximitySensorCounts();
    void theRangeIsTheSensorsOwn();
    void aDirectionNotHeardFor2sHasNoValue();
    void changedOnlyWhenSomethingChanged();
    void theFirstMessageShowsTheRadar();
    void anOddOrientationFallsBackToTheId();
    void eachDroneHasItsOwn();
    void theStoreExpiresOnItsTimer();
    void qmlGetsTheOneStore();
    void qmlNeverOwnsTheStoreOrADrone();
};

void ProximityStoreTest::theEightDirectionsInTheirPlaces()
{
    ProximitySectors s;
    QVERIFY(!s.telemetryAvailable());
    const int cm[8] = {280, 4000, 900, 4000, 4000, 4000, 1500, 4000};
    for (int i = 0; i < 8; ++i) {
        QVERIFY(s.hear(kAhead + i, i, cm[i], 4000, 1000));
    }
    QVERIFY(s.telemetryAvailable());
    QCOMPARE(s.value(0), 2.8);
    QCOMPARE(s.value(2), 9.0);
    QCOMPARE(s.value(6), 15.0);
    for (int i : {1, 3, 4, 5, 7}) {
        QVERIFY2(none(s.value(i)), "a direction at the sensor's range has nothing in it");
    }
    const QVariantList values = s.values();
    QCOMPARE(values.size(), 8);
    QCOMPARE(values.at(0).toDouble(), 2.8);
    QVERIFY(none(values.at(1).toDouble()));
    QVERIFY(none(s.value(8)) && none(s.value(-1)));
}

void ProximityStoreTest::anEmptyDirectionHasNoValue()
{
    ProximitySectors s;
    s.hear(kAhead, 0, 0, 4000, 1000);
    QVERIFY(none(s.value(0)));
    s.hear(kAhead, 0, 0xFFFF, 4000, 1000);
    QVERIFY2(none(s.value(0)), "MAVLink's no reading");
    s.hear(kAhead, 0, 4100, 4000, 1000);
    QVERIFY(none(s.value(0)));
    s.hear(kAhead, 0, 3999, 4000, 1000);
    QCOMPARE(s.value(0), 39.99);
}

void ProximityStoreTest::theA3sRangeIsZero()
{
    // ArduPilot knows no range for the A3 ("RPLidar UNKNOWN"): its range is 0, and an empty direction comes as 0.
    ProximitySectors s;
    s.hear(kAhead, 0, 500, 0, 1000);
    s.hear(kAhead + 1, 1, 0, 0, 1000);
    QCOMPARE(s.value(0), 5.0);
    QVERIFY2(none(s.value(1)), "QGC showed this as 0.00 in the middle of the video");
    QCOMPARE(s.maxDistance(), 0.0);
    s.hear(kAhead + 1, 1, 0xFFFF, 0, 1000);
    QVERIFY(none(s.value(1)));
}

void ProximityStoreTest::onlyTheProximitySensorCounts()
{
    ProximitySectors s;
    // A forward rangefinder (id 0, orientation 0), a downward one (id 1, orientation 25), the upward reading (id 19).
    QVERIFY(!s.hear(0, 0, 350, 4000, 1000));
    QVERIFY(!s.hear(1, 25, 1200, 2200, 1000));
    QVERIFY(!s.hear(19, 24, 600, 4000, 1000));
    QVERIFY(!s.hear(9, 0, 350, 4000, 1000));
    QVERIFY(!s.hear(18, 0, 350, 4000, 1000));
    QVERIFY(!s.telemetryAvailable());
    QVERIFY(none(s.value(0)));
}

void ProximityStoreTest::theRangeIsTheSensorsOwn()
{
    ProximityStore store(nullptr);
    qint64 now = 1000;
    store.setClock([&now] { return now; });
    store.hear(1, kAhead, 0, 280, 0);          // the A3: range 0
    store.hear(1, 1, 25, 900, 2200);           // a downward rangefinder: its range is not the A3's
    ProximitySectors *s = store.forVehicle(1);
    QCOMPARE(s->maxDistance(), 0.0);
    QCOMPARE(s->value(0), 2.8);
    store.hear(1, kRight, 2, 4000, 4000);      // a TF02 Pro beside it: ArduPilot sends the longest range
    QCOMPARE(s->maxDistance(), 40.0);
    QVERIFY(none(s->value(2)));
}

void ProximityStoreTest::aDirectionNotHeardFor2sHasNoValue()
{
    // ArduPilot sends no direction while the sensor is not good: the arcs must not freeze.
    ProximitySectors s;
    s.hear(kAhead, 0, 500, 4000, 1000);
    s.hear(kRight, 2, 700, 4000, 2500);
    s.expire(3000);
    QCOMPARE(s.value(0), 5.0);
    s.expire(3001);
    QVERIFY(none(s.value(0)));
    QCOMPARE(s.value(2), 7.0);
    s.expire(4501);
    QVERIFY(none(s.value(2)));
    QVERIFY2(s.telemetryAvailable(), "the radar stays, empty");
    s.hear(kAhead, 0, 450, 4000, 5000);
    QCOMPARE(s.value(0), 4.5);
}

void ProximityStoreTest::changedOnlyWhenSomethingChanged()
{
    ProximitySectors s;
    QSignalSpy spy(&s, &ProximitySectors::changed);
    s.hear(kAhead, 0, 500, 4000, 1000);
    QCOMPARE(spy.count(), 1);
    s.hear(kAhead, 0, 500, 4000, 1200);
    s.hear(kAhead + 1, 1, 4000, 4000, 1200);   // empty, as it was
    QCOMPARE(spy.count(), 1);
    s.hear(kAhead, 0, 480, 4000, 1400);
    QCOMPARE(spy.count(), 2);
    s.hear(kAhead, 0, 480, 2200, 1450);        // the same reading, another range: the range circle changes
    QCOMPARE(spy.count(), 3);
    s.expire(1500);
    QCOMPARE(spy.count(), 3);
    s.expire(9000);
    QCOMPARE(spy.count(), 4);
    s.expire(9500);
    QCOMPARE(spy.count(), 4);
}

void ProximityStoreTest::theFirstMessageShowsTheRadar()
{
    // The A3 with nothing in range: no value and range 0, as before the first message, but the radar is shown now.
    ProximitySectors s;
    QSignalSpy spy(&s, &ProximitySectors::changed);
    s.hear(kAhead, 0, 0, 0, 1000);
    QCOMPARE(spy.count(), 1);
    QVERIFY(s.telemetryAvailable());
}

void ProximityStoreTest::anOddOrientationFallsBackToTheId()
{
    ProximitySectors s;
    s.hear(kAhead + 3, 99, 420, 4000, 1000);
    QCOMPARE(s.value(3), 4.2);
}

void ProximityStoreTest::eachDroneHasItsOwn()
{
    ProximityStore store(nullptr);
    store.setClock([] { return 1000; });
    store.hear(1, kAhead, 0, 300, 4000);
    store.hear(2, kAhead, 0, 700, 4000);
    QCOMPARE(store.forVehicle(1)->value(0), 3.0);
    QCOMPARE(store.forVehicle(2)->value(0), 7.0);
    QCOMPARE(store.forVehicle(1), store.forVehicle(1));
    QVERIFY(!store.forVehicle(3)->telemetryAvailable());
    QCOMPARE(QJSEngine::objectOwnership(store.forVehicle(1)), QJSEngine::CppOwnership);
}

void ProximityStoreTest::theStoreExpiresOnItsTimer()
{
    ProximityStore store(nullptr);
    qint64 now = 1000;
    store.setClock([&now] { return now; });
    store.hear(1, kAhead, 0, 300, 4000);
    ProximitySectors *s = store.forVehicle(1);
    now = 4000;
    QSignalSpy spy(s, &ProximitySectors::changed);
    QVERIFY2(spy.wait(1000), "the arcs go within a second of their 2 s");
    QVERIFY(none(s->value(0)));
}

void ProximityStoreTest::qmlGetsTheOneStore()
{
    static_assert(!std::is_default_constructible_v<ProximityStore>, "QML would make its own store");
    ProximityStore *store = ProximityStore::create(nullptr, nullptr);
    QCOMPARE(store, ProximityStore::instance());
    QCOMPARE(QJSEngine::objectOwnership(store), QJSEngine::CppOwnership);
}

void ProximityStoreTest::qmlNeverOwnsTheStoreOrADrone()
{
    // The QML engine makes an object it gets its own unless C++ set the owner: then it could delete it.
    QJSEngine engine;
    ProximityStore *store = ProximityStore::create(nullptr, &engine);
    engine.globalObject().setProperty("store", engine.newQObject(store));
    QCOMPARE(QJSEngine::objectOwnership(store), QJSEngine::CppOwnership);
    QObject *sectors = engine.evaluate("store.forVehicle(7)").toQObject();
    QVERIFY(sectors != nullptr);
    QCOMPARE(sectors, store->forVehicle(7));
    QCOMPARE(QJSEngine::objectOwnership(sectors), QJSEngine::CppOwnership);
}

QTEST_GUILESS_MAIN(ProximityStoreTest)
#include "proximity_store_test.moc"
