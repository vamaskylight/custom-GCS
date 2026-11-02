#include "ProximityStore.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QSettings>
#include <QtQml/QJSEngine>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <type_traits>

namespace {

constexpr int kNoReadingCm = 0xFFFF;
const double kNone = std::numeric_limits<double>::quiet_NaN();
// The same list as VGCS's PROXIMITY card (vgcs/app/obstacle_watch.py BAR_RANGE_CHOICES_M).
constexpr double kBarRanges[] = {1, 2, 3, 4, 5, 6, 8, 10, 15, 20, 30, 40};

bool isBarRange(double metres)
{
    return std::find(std::begin(kBarRanges), std::end(kBarRanges), metres) != std::end(kBarRanges);
}

/// The distance in metres, or NaN for "nothing in range": no reading, or the sensor's own range
/// (an empty direction comes as the range; 0 for a sensor of unknown range).
double reading(int currentCm, int maxCm)
{
    if (currentCm <= 0 || currentCm >= kNoReadingCm) {
        return kNone;
    }
    if (maxCm > 0 && currentCm >= maxCm) {
        return kNone;
    }
    return currentCm / 100.0;
}

bool sameValue(double a, double b)
{
    return (std::isnan(a) && std::isnan(b)) || a == b;
}

} // namespace

ProximitySectors::ProximitySectors(QObject *parent)
    : QObject(parent)
{
    for (int i = 0; i < kDirections; ++i) {
        _metres[i] = kNone;
        _heardMs[i] = -1;
    }
}

QVariantList ProximitySectors::values() const
{
    QVariantList out;
    out.reserve(kDirections);
    for (int i = 0; i < kDirections; ++i) {
        out.append(_metres[i]);
    }
    return out;
}

double ProximitySectors::value(int direction) const
{
    return direction >= 0 && direction < kDirections ? _metres[direction] : kNone;
}

bool ProximitySectors::hear(int sensorId, int orientation, int currentCm, int maxCm, qint64 nowMs)
{
    if (sensorId < kFirstId || sensorId >= kFirstId + kDirections) {
        return false;   // a rangefinder (ids 0 to 9), or the proximity sensor's upward reading (id 19)
    }
    // ArduPilot sends direction i with orientation i; the id says the same.
    const int direction = (orientation >= 0 && orientation < kDirections) ? orientation : sensorId - kFirstId;
    const double metres = reading(currentCm, maxCm);
    const double range = maxCm > 0 ? maxCm / 100.0 : 0.0;
    const bool newValue = !sameValue(_metres[direction], metres) || range != _maxM || !_heard;
    _metres[direction] = metres;
    _heardMs[direction] = nowMs;
    _maxM = range;
    _heard = true;
    if (newValue) {
        emit changed();
    }
    return true;
}

QVariantList ProximitySectors::sides() const
{
    QVariantList out;
    out.reserve(kSides);
    for (int i = 0; i < kSides; ++i) {
        out.append(side(i));
    }
    return out;
}

double ProximitySectors::side(int index) const
{
    if (index < 0 || index >= kSides) {
        return kNone;
    }
    // Side 0 faces direction 0, side 1 direction 2, and so on, with the diagonal on each hand of it.
    const int middle = index * 2;
    double nearest = kNone;
    for (const int direction : {middle + kDirections - 1, middle, middle + 1}) {
        const double metres = _metres[direction % kDirections];
        if (!std::isnan(metres) && (std::isnan(nearest) || metres < nearest)) {
            nearest = metres;
        }
    }
    return nearest <= _barRangeM ? nearest : kNone;
}

void ProximitySectors::setBarRange(double metres)
{
    if (metres != _barRangeM) {
        _barRangeM = metres;
        emit changed();
    }
}

void ProximitySectors::expire(qint64 nowMs)
{
    bool cleared = false;
    for (int i = 0; i < kDirections; ++i) {
        if (_heardMs[i] >= 0 && nowMs - _heardMs[i] > kMaxAgeMs && !std::isnan(_metres[i])) {
            _metres[i] = kNone;
            cleared = true;
        }
    }
    if (cleared) {
        emit changed();
    }
}

/*===========================================================================*/

ProximityStore::ProximityStore(QObject *parent)
    : QObject(parent)
{
    const double saved = QSettings().value(QLatin1String(kBarRangeKey), kDefaultBarRange).toDouble();
    _barRangeM = isBarRange(saved) ? saved : kDefaultBarRange;
    _elapsed.start();
    _expiry.setInterval(kExpiryCheckMs);
    connect(&_expiry, &QTimer::timeout, this, &ProximityStore::expireAll);
    _expiry.start();
}

ProximityStore *ProximityStore::instance()
{
    static ProximityStore *store = new ProximityStore(QCoreApplication::instance());
    return store;
}

ProximityStore *ProximityStore::create(QQmlEngine *qmlEngine, QJSEngine *jsEngine)
{
    Q_UNUSED(qmlEngine);
    Q_UNUSED(jsEngine);
    ProximityStore *store = instance();
    QJSEngine::setObjectOwnership(store, QJSEngine::CppOwnership);
    return store;
}

// QML calls create() only for a singleton that cannot be made without an argument (see SkydroidLink.cc).
static_assert(!std::is_default_constructible_v<ProximityStore>,
              "QML would make its own ProximityStore instead of using ProximityStore::create()");

ProximitySectors *ProximityStore::forVehicle(int vehicleId)
{
    ProximitySectors *sectors = _vehicles.value(vehicleId, nullptr);
    if (sectors == nullptr) {
        sectors = new ProximitySectors(this);
        sectors->setBarRange(_barRangeM);
        QJSEngine::setObjectOwnership(sectors, QJSEngine::CppOwnership);
        _vehicles.insert(vehicleId, sectors);
    }
    return sectors;
}

void ProximityStore::setBarRange(double metres)
{
    if (!isBarRange(metres) || metres == _barRangeM) {
        return;
    }
    _barRangeM = metres;
    QSettings().setValue(QLatin1String(kBarRangeKey), metres);
    for (ProximitySectors *sectors : std::as_const(_vehicles)) {
        sectors->setBarRange(metres);
    }
    emit barRangeChanged();
}

QVariantList ProximityStore::barRangeChoices() const
{
    QVariantList out;
    for (const double metres : kBarRanges) {
        out.append(metres);
    }
    return out;
}

void ProximityStore::hear(int vehicleId, int sensorId, int orientation, int currentCm, int maxCm)
{
    forVehicle(vehicleId)->hear(sensorId, orientation, currentCm, maxCm, _now());
}

void ProximityStore::expireAll()
{
    const qint64 now = _now();
    for (ProximitySectors *sectors : std::as_const(_vehicles)) {
        sectors->expire(now);
    }
}

qint64 ProximityStore::_now() const
{
    return _clock ? _clock() : _elapsed.elapsed();
}
