#pragma once

// Loads the map packs that VGCS made (MapPack.h) into QGC's map store by itself when
// the app starts, so the map shows the place without the internet (task U, client
// 2026-10-15: "end user won't know how to import offline map").
//
// Where a pack goes, copied from the PC as the folder it is (pack.json with the picture folders):
//  - The app's own folder, Android/data/com.vama.gcs/files/Map packs. The app may always read
//    it, and makes it at its start, so it is there to copy into over USB.
//  - Download/VAMA maps. Easier to find, but Android 11 and newer let the app read it only
//    with "All files access" for the app (QGC asks for it at the first start).
//
// How it goes:
//  - At start, a thread of its own looks for packs and reads them (MapPack::findPacks).
//  - A pack is loaded when it is new or changed (its signature is kept in the settings), and
//    again when its pictures are no longer in QGC's map store (the store was cleared). That
//    check asks QGC's map worker for three of the pack's pictures.
//  - Loading: the thread writes the pack as a QGC tile file (MapPack::writeTileFile), then
//    QGC's map worker merges that file into its store, as the import in Offline Maps does.
//    So each pack is a tile set of its own, "Map pack <folder>": it is listed in Offline
//    Maps and can be deleted there, and QGC does not trim it (QGC trims only its default
//    set, the pictures it keeps from the map seen online).
//  - Then the map is switched to the pack's map (Esri World Satellite for VGCS's satellite
//    pictures): QGC keeps pictures per map, so only that map shows them.
//  - One message says what was loaded, and what was not and why. Nothing is said when
//    there was nothing new.
//
// The pack folders are only read, never changed.

#include <QtCore/QHash>
#include <QtCore/QList>
#include <QtCore/QObject>
#include <QtCore/QPointer>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QTimer>

#include <atomic>
#include <functional>

#include "MapPack.h"

class QThread;
class QGCMapTask;

class MapPackLoader : public QObject
{
    Q_OBJECT

public:
    explicit MapPackLoader(QObject *parent = nullptr);
    /// For tests: looks in `roots` (the app's own folder first) and keeps its go-between
    /// files in `workFolder`, in place of folders() and the app's cache.
    MapPackLoader(const QStringList &roots, const QString &workFolder, QObject *parent = nullptr);
    ~MapPackLoader() override;

    /// Looks for packs and loads the new ones. Once, when the app starts.
    void start();

    /// The folders looked in for packs. The first is the app's own.
    static QStringList folders();

signals:
    /// All packs were looked at, and the message (if any) was shown.
    void done();

private:
    // What the settings keep of a pack, by its folder.
    struct Kept
    {
        QString signature;
        QStringList probes;  // "z/x/y" of three pictures the pack put in the store
        QString reported;    // the signature and problem last reported
    };

    void _scanned(const QList<MapPack::Pack> &packs);
    void _next();
    void _check(const MapPack::Pack &pack);
    void _checked(const MapPack::Pack &pack, bool allThere);
    void _write(const MapPack::Pack &pack);
    void _written(const MapPack::Pack &pack, const QString &file, const MapPack::Written &written);
    void _import();
    void _imported();
    void _importFailed(const QString &error);
    void _finish();
    void _report(const MapPack::Pack &pack, const QString &problem);
    void _runInThread(std::function<void()> work);
    bool _addTask(QGCMapTask *task);
    void _retryLater();

    static QString _keyOf(const QString &folder);
    Kept _kept(const QString &key) const;
    void _keep(const QString &key, const Kept &kept);

    QList<MapPack::Pack> _toCheck;
    QList<MapPack::Pack> _toLoad;
    // The pack whose file waits for QGC's map worker.
    MapPack::Pack _importing;
    QString _importFile;
    MapPack::Written _importWritten;
    bool _importWaiting = false;
    // Probes of the pack being checked.
    int _probesAsked = 0;
    int _probesFound = 0;
    int _probesAnswered = 0;

    QStringList _roots;      // folders(), as they were at the start
    QString _workFolder;     // the go-between files
    QStringList _lines;      // the message, line by line
    QString _loadedMap;      // the map of the last pack loaded
    bool _downloadHint = false;
    int _retries = 0;
    QTimer _retryTimer;
    QPointer<QThread> _thread;
    std::atomic<bool> _stop{false};
    bool _started = false;
    bool _adding = false;    // a task is being handed to QGC's map worker
    bool _broken = false;    // QGC's map worker refused a task: its store does not work
};
