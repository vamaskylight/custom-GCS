#pragma once

#include <QtCore/QElapsedTimer>
#include <QtCore/QHash>
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QTimer>
#include <QtQml/qqmlregistration.h>

#include <functional>
#include <map>
#include <string>

#include "PositionSource.h"

class QJSEngine;
class QQmlEngine;

/// One drone's position source as the screen shows it (A8 step 4, 2026-10-30): GPS 1, or the down
/// camera as GPS 2 (the GPS-denied navigation, drone/gps_denied/vama_nav). "CAM" beside the
/// satellites in the toolbar while the camera gives the position, the "Position source" part of
/// the GPS page (src/VehicleGPSIndicator.qml), and a note when the source changes in the air
/// (src/FlyViewCustomLayer.qml). The rules and the words are VGCS's (PositionSource.h).
class VehiclePositionSource : public QObject
{
    Q_OBJECT
    QML_ANONYMOUS

    /// True once the drone reported a GPS 2 (GPS2_RAW). A drone without one shows nothing new.
    Q_PROPERTY(bool hasGps2 READ hasGps2 NOTIFY changed)
    /// "CAM" while the camera gives the position, "GPS2" for a second real GPS, else "".
    Q_PROPERTY(QString tag READ tag NOTIFY changed)
    /// Where the position comes from, in VGCS's words ("N/A" without GPS 2).
    Q_PROPERTY(QString text READ text NOTIFY changed)
    /// "na", "ok", "warn" or "bad", for the colour.
    Q_PROPERTY(QString level READ level NOTIFY changed)
    /// What GPS 2 is, and a wrong setting: lines (VGCS's tooltip).
    Q_PROPERTY(QString details READ details NOTIFY changed)

public:
    explicit VehiclePositionSource(QObject *parent = nullptr);

    bool hasGps2() const { return _hasGps2; }
    QString tag() const { return _tag; }
    QString text() const { return _text; }
    QString level() const { return _level; }
    QString details() const { return _details; }

    vama::PositionSource &rules() { return _rules; }
    /// The values again for this moment (the readings expire), and the note when one is due (said).
    void publish(double nowS);

signals:
    void changed();
    /// A new position source in the air, once it has lasted 2 s. warning: the drone now flies on
    /// the camera's position, or on none.
    void said(const QString &text, bool warning);

private:
    vama::PositionSource _rules;
    bool _hasGps2 = false;
    QString _tag;
    QString _text = QStringLiteral("N/A");
    QString _level = QStringLiteral("na");
    QString _details;
};

/// The app's position sources, one for each drone, fed by CustomPlugin::mavlinkMessage (GPS_RAW_INT,
/// GPS2_RAW, and the armed state and GPS settings QGC holds). QML:
/// PositionSourceStore.forVehicle(vehicle.id).
class PositionSourceStore : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON

public:
    /// VGCS shows the source again once a second too: the readings expire.
    static constexpr int kPublishMs = 1000;

    /// No default for the parent, on purpose: QML makes a singleton with a constructor that needs no
    /// argument when there is one, instead of using create() (see SkydroidLink.h, tasks X and V).
    explicit PositionSourceStore(QObject *parent);

    /// The app's one store. QML gets it too (create()).
    static PositionSourceStore *instance();
    static PositionSourceStore *create(QQmlEngine *qmlEngine, QJSEngine *jsEngine);

    /// The drone with this MAVLink system id (QGC's Vehicle::id()), made at the first ask.
    Q_INVOKABLE VehiclePositionSource *forVehicle(int vehicleId);

    /// GPS_RAW_INT: GPS 1's fix type.
    void hearGps1(int vehicleId, int fixType);
    /// GPS2_RAW: GPS 2's fix type, its accuracy (h_acc, mm, 0: none given) and its satellites.
    void hearGps2(int vehicleId, int fixType, quint32 hAccMm, int satellites);
    void setArmed(int vehicleId, bool armed);
    /// GPS2_TYPE (GPS_TYPE2 before ArduPilot 4.6) and GPS_AUTO_SWITCH, as the drone has them.
    void setSettings(int vehicleId, const std::map<std::string, double> &params);
    /// The drone went away: when it comes back nothing is known (VGCS: a new link).
    void reset(int vehicleId);
    /// Every drone's values again. The store's timer calls it every kPublishMs.
    void publishAll();
    /// For tests: the clock, in milliseconds.
    void setClock(std::function<qint64()> clock) { _clock = std::move(clock); }

private:
    double _nowS() const;

    QHash<int, VehiclePositionSource *> _vehicles;
    QTimer _publish;
    QElapsedTimer _elapsed;
    std::function<qint64()> _clock;
};
