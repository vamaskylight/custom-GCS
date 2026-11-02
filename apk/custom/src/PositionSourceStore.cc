#include "PositionSourceStore.h"

#include <QtCore/QCoreApplication>
#include <QtQml/QJSEngine>

#include <type_traits>

VehiclePositionSource::VehiclePositionSource(QObject *parent)
    : QObject(parent)
{
}

void VehiclePositionSource::publish(double nowS)
{
    const bool hasGps2 = _rules.hasGps2();
    const QString tag = QString::fromStdString(_rules.headerTag(nowS));
    const QString text = QString::fromStdString(_rules.fieldText(nowS));
    const QString level = QString::fromStdString(_rules.level(nowS));
    const QString details = QString::fromStdString(_rules.tooltip());
    if (hasGps2 != _hasGps2 || tag != _tag || text != _text || level != _level || details != _details) {
        _hasGps2 = hasGps2;
        _tag = tag;
        _text = text;
        _level = level;
        _details = details;
        emit changed();
    }
    if (const std::optional<vama::PositionSource::Event> event = _rules.takeEvent(nowS)) {
        emit said(QString::fromStdString(event->text), event->warning);
    }
}

/*===========================================================================*/

PositionSourceStore::PositionSourceStore(QObject *parent)
    : QObject(parent)
{
    _elapsed.start();
    _publish.setInterval(kPublishMs);
    connect(&_publish, &QTimer::timeout, this, &PositionSourceStore::publishAll);
    _publish.start();
}

PositionSourceStore *PositionSourceStore::instance()
{
    static PositionSourceStore *store = new PositionSourceStore(QCoreApplication::instance());
    return store;
}

PositionSourceStore *PositionSourceStore::create(QQmlEngine *qmlEngine, QJSEngine *jsEngine)
{
    Q_UNUSED(qmlEngine);
    Q_UNUSED(jsEngine);
    PositionSourceStore *store = instance();
    QJSEngine::setObjectOwnership(store, QJSEngine::CppOwnership);
    return store;
}

// QML calls create() only for a singleton that cannot be made without an argument (see SkydroidLink.cc).
static_assert(!std::is_default_constructible_v<PositionSourceStore>,
              "QML would make its own PositionSourceStore instead of using PositionSourceStore::create()");

VehiclePositionSource *PositionSourceStore::forVehicle(int vehicleId)
{
    VehiclePositionSource *source = _vehicles.value(vehicleId, nullptr);
    if (source == nullptr) {
        source = new VehiclePositionSource(this);
        QJSEngine::setObjectOwnership(source, QJSEngine::CppOwnership);
        _vehicles.insert(vehicleId, source);
    }
    return source;
}

void PositionSourceStore::hearGps1(int vehicleId, int fixType)
{
    const double now = _nowS();
    VehiclePositionSource *source = forVehicle(vehicleId);
    source->rules().updateGps1(fixType, now);
    source->publish(now);
}

void PositionSourceStore::hearGps2(int vehicleId, int fixType, quint32 hAccMm, int satellites)
{
    const double now = _nowS();
    VehiclePositionSource *source = forVehicle(vehicleId);
    // VGCS's _mm_to_m_or_none: 0 is "not given", not "0 m accurate".
    source->rules().updateGps2(fixType, hAccMm > 0 ? hAccMm / 1000.0 : 0.0, satellites, now);
    source->publish(now);
}

void PositionSourceStore::setArmed(int vehicleId, bool armed)
{
    forVehicle(vehicleId)->rules().setArmed(armed);
}

void PositionSourceStore::setSettings(int vehicleId, const std::map<std::string, double> &params)
{
    forVehicle(vehicleId)->rules().updateSettings(params);
}

void PositionSourceStore::reset(int vehicleId)
{
    VehiclePositionSource *source = _vehicles.value(vehicleId, nullptr);
    if (source != nullptr) {
        source->rules().reset();
        source->publish(_nowS());
    }
}

void PositionSourceStore::publishAll()
{
    const double now = _nowS();
    for (VehiclePositionSource *source : std::as_const(_vehicles)) {
        source->publish(now);
    }
}

double PositionSourceStore::_nowS() const
{
    return (_clock ? _clock() : _elapsed.elapsed()) / 1000.0;
}
