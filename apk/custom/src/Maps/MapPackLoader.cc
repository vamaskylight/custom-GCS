#include "MapPackLoader.h"

#include <QtCore/QCryptographicHash>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QSettings>
#include <QtCore/QStandardPaths>
#include <QtCore/QStorageInfo>
#include <QtCore/QThread>

#ifdef Q_OS_ANDROID
#include <QtCore/QJniEnvironment>
#include <QtCore/QJniObject>
#include <QtCore/qcoreapplication_platform.h>
#endif

#include "FlightMapSettings.h"
#include "QGCApplication.h"
#include "QGCCacheTile.h"
#include "QGCLoggingCategory.h"
#include "QGCMapEngine.h"
#include "QGCMapEngineManager.h"
#include "QGCMapTasks.h"
#include "QGCMapUrlEngine.h"
#include "SettingsManager.h"

QGC_LOGGING_CATEGORY_ON(MapPackLog, "Custom.MapPack")

namespace {

constexpr const char *kSettingsGroup = "VamaMapPacks";
constexpr const char *kOwnFolder = "Map packs";
constexpr const char *kSharedFolder = "Download/VAMA maps";
// QGC's map worker starts with the first map on the screen. Until then a task is
// tried again this often, this many times (5 minutes in all).
constexpr int kRetryMs = 2000;
constexpr int kMaxRetries = 150;
// Free room needed besides twice the pack (the go-between file, and the store growing by the pack).
constexpr qint64 kSpareBytes = 64LL * 1024 * 1024;

// The go-between files: in the app's cache, on the same storage as QGC's map store.
QString tempFolder()
{
    QString cache = QStandardPaths::writableLocation(QStandardPaths::CacheLocation);
    if (cache.isEmpty()) {
        cache = QDir::tempPath();
    }
    return cache + QStringLiteral("/VamaMapPacks");
}

QString probeText(const MapPack::Tile &tile)
{
    return QStringLiteral("%1/%2/%3").arg(tile.z).arg(tile.x).arg(tile.y);
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

}  // namespace

MapPackLoader::MapPackLoader(QObject *parent)
    : MapPackLoader(QStringList(), QString(), parent)
{
}

MapPackLoader::MapPackLoader(const QStringList &roots, const QString &workFolder, QObject *parent)
    : QObject(parent)
    , _roots(roots)
    , _workFolder(workFolder)
{
    _retryTimer.setSingleShot(true);
    (void) connect(&_retryTimer, &QTimer::timeout, this, &MapPackLoader::_next);
}

MapPackLoader::~MapPackLoader()
{
    // A pack being written stops at its next look at _stop.
    _stop = true;
    if (_thread) {
        (void) _thread->wait();
    }
}

QStringList MapPackLoader::folders()
{
    QStringList list;
#ifdef Q_OS_ANDROID
    QJniEnvironment env;
    QString own;
    const QJniObject context = QNativeInterface::QAndroidApplication::context();
    if (context.isValid()) {
        own = pathOf(context.callObjectMethod("getExternalFilesDir", "(Ljava/lang/String;)Ljava/io/File;", static_cast<jobject>(nullptr)));
    }
    (void) env.checkAndClearExceptions();
    if (own.isEmpty()) {
        own = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    }
    list.append(own + QLatin1Char('/') + QLatin1String(kOwnFolder));
    const QString shared = pathOf(QJniObject::callStaticObjectMethod("android/os/Environment", "getExternalStorageDirectory", "()Ljava/io/File;"));
    (void) env.checkAndClearExceptions();
    if (!shared.isEmpty()) {
        list.append(shared + QLatin1Char('/') + QLatin1String(kSharedFolder));
    }
#else
    list.append(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + QLatin1Char('/') + QLatin1String(kOwnFolder));
#endif
    return list;
}

void MapPackLoader::start()
{
    if (_started) {
        return;
    }
    _started = true;

    if (_roots.isEmpty()) {
        _roots = folders();
    }
    if (_workFolder.isEmpty()) {
        _workFolder = tempFolder();
    }
    const QStringList roots = _roots;
    // The app's own folder is made, so it is there to copy packs into.
    if (!QDir().mkpath(roots.constFirst())) {
        qCWarning(MapPackLog) << "Cannot make" << roots.constFirst();
    }
    // Go-between files of a start that was cut off.
    (void) QDir(_workFolder).removeRecursively();

    qCInfo(MapPackLog) << "Looking for map packs in" << roots;
    _runInThread([this, roots]() {
        QList<MapPack::Pack> packs;
        for (const QString &root : roots) {
            if (_stop) {
                return;
            }
            packs += MapPack::findPacks(root);
        }
        (void) QMetaObject::invokeMethod(this, [this, packs]() { _scanned(packs); }, Qt::QueuedConnection);
    });
}

void MapPackLoader::_scanned(const QList<MapPack::Pack> &packs)
{
    for (const MapPack::Pack &pack : packs) {
        if (!pack.problem.isEmpty()) {
            _report(pack, pack.problem);
            continue;
        }
        if (UrlFactory::getQtMapIdFromProviderType(pack.mapName) < 0) {
            _report(pack, tr("the app has no map for its source (%1)").arg(pack.source));
            continue;
        }
        const Kept kept = _kept(_keyOf(pack.folder));
        if (kept.signature == pack.signature && !kept.probes.isEmpty()) {
            _toCheck.append(pack);
        } else {
            _toLoad.append(pack);
        }
    }
    qCInfo(MapPackLog) << packs.size() << "map packs found," << _toCheck.size() << "loaded before," << _toLoad.size() << "new or changed";
    _next();
}

void MapPackLoader::_next()
{
    if (_stop) {
        return;
    }
    if (_importWaiting) {
        _import();
    } else if (!_toCheck.isEmpty()) {
        _check(_toCheck.takeFirst());
    } else if (!_toLoad.isEmpty()) {
        _write(_toLoad.takeFirst());
    } else {
        _finish();
    }
}

// Asks QGC's map worker for some pictures the pack had put in the store.
void MapPackLoader::_check(const MapPack::Pack &pack)
{
    const QStringList probes = _kept(_keyOf(pack.folder)).probes;
    _probesAsked = int(probes.size());
    _probesFound = 0;
    _probesAnswered = 0;
    const auto answered = [this, pack](bool found) {
        _probesFound += found ? 1 : 0;
        if (++_probesAnswered == _probesAsked) {
            _checked(pack, _probesFound == _probesAsked);
        }
    };
    for (qsizetype i = 0; i < probes.size(); ++i) {
        const QStringList zxy = probes.at(i).split(QLatin1Char('/'));
        const int z = zxy.value(0).toInt();
        const int x = zxy.value(1).toInt();
        const int y = zxy.value(2).toInt();
        auto *task = new QGCFetchTileTask(UrlFactory::getTileHash(pack.mapName, x, y, z));
        (void) connect(task, &QGCFetchTileTask::tileFetched, this, [answered](QGCCacheTile *tile) {
            delete tile;
            answered(true);
        });
        (void) connect(task, &QGCMapTask::error, this, [this, answered](QGCMapTask::TaskType, const QString &) {
            if (!_adding) {
                answered(false);
            }
        });
        if (!_addTask(task)) {
            // The map worker has not started yet (it starts with the first map): ask again later.
            // Once started it takes every task, so this is the first picture asked for.
            _toCheck.prepend(pack);
            _retryLater();
            return;
        }
    }
}

void MapPackLoader::_checked(const MapPack::Pack &pack, bool allThere)
{
    if (allThere) {
        qCInfo(MapPackLog) << "Already in the map store:" << pack.folder;
    } else {
        qCInfo(MapPackLog) << "Its pictures are no longer in the map store, loading it again:" << pack.folder;
        _toLoad.append(pack);
    }
    _next();
}

void MapPackLoader::_write(const MapPack::Pack &pack)
{
    const QString folder = _workFolder;
    (void) QDir().mkpath(folder);
    const qint64 free = QStorageInfo(folder).bytesAvailable();
    const qint64 needed = 2 * pack.bytes + kSpareBytes;
    if (free >= 0 && free < needed) {
        _report(pack, tr("the remote control has %1 MB free, and loading it needs %2 MB").arg(free >> 20).arg(needed >> 20));
        _next();
        return;
    }

    const QString file = folder + QStringLiteral("/pack-%1.db").arg(_keyOf(pack.folder));
    const QString setName = tr("Map pack %1").arg(pack.name);
    const int mapId = UrlFactory::getQtMapIdFromProviderType(pack.mapName);
    const QString mapName = pack.mapName;
    qCInfo(MapPackLog) << "Loading" << pack.folder << "with" << pack.tiles.size() << "pictures," << (pack.bytes >> 10) << "KB";
    _runInThread([this, pack, file, setName, mapId, mapName]() {
        const MapPack::Written written = MapPack::writeTileFile(
            pack, file, setName, mapId,
            [mapName](int x, int y, int z) { return UrlFactory::getTileHash(mapName, x, y, z); },
            [this]() { return _stop.load(); });
        (void) QMetaObject::invokeMethod(this, [this, pack, file, written]() { _written(pack, file, written); }, Qt::QueuedConnection);
    });
}

void MapPackLoader::_written(const MapPack::Pack &pack, const QString &file, const MapPack::Written &written)
{
    if (!written.error.isEmpty()) {
        _report(pack, written.error);
        _next();
        return;
    }
    _importing = pack;
    _importFile = file;
    _importWritten = written;
    _importWaiting = true;
    _import();
}

// QGC's map worker merges the file into its store, as the import in Offline Maps does.
void MapPackLoader::_import()
{
    auto *task = new QGCImportTileTask(_importFile, /*replace=*/false);
    (void) connect(task, &QGCImportTileTask::actionCompleted, this, &MapPackLoader::_imported);
    (void) connect(task, &QGCMapTask::error, this, [this](QGCMapTask::TaskType, const QString &error) {
        if (!_adding) {
            _importFailed(error);
        }
    });
    if (!_addTask(task)) {
        _retryLater();
        return;
    }
    _importWaiting = false;
}

void MapPackLoader::_imported()
{
    (void) QFile::remove(_importFile);
    const QString key = _keyOf(_importing.folder);
    Kept kept;
    kept.signature = _importing.signature;
    for (const MapPack::Tile &tile : std::as_const(_importWritten.probes)) {
        kept.probes.append(probeText(tile));
    }
    _keep(key, kept);
    _loadedMap = _importing.mapName;
    QString line = tr("Map pack \"%1\" is loaded: %2 map pictures, zoom %3 to %4. The map shows this place without the internet.")
                       .arg(_importing.name).arg(_importWritten.tiles).arg(_importing.minZoom).arg(_importing.maxZoom);
    if (_importWritten.skipped > 0) {
        line += QLatin1Char(' ') + tr("%1 files in it were no map pictures and were left out.").arg(_importWritten.skipped);
    }
    _lines.append(line);
    qCInfo(MapPackLog) << "Loaded" << _importing.folder << _importWritten.tiles << "pictures," << _importWritten.skipped << "left out";
    // Offline Maps lists the new set when it is open.
    QGCMapEngineManager::instance()->loadTileSets();
    _next();
}

void MapPackLoader::_importFailed(const QString &error)
{
    (void) QFile::remove(_importFile);
    _report(_importing, tr("the map store did not take it (%1)").arg(error));
    _next();
}

void MapPackLoader::_finish()
{
    if (!_loadedMap.isEmpty()) {
        // QGC keeps pictures per map, so the map that shows the pack's pictures is chosen.
        FlightMapSettings *const settings = SettingsManager::instance()->flightMapSettings();
        const QString current = settings->mapProvider()->rawValue().toString() + QLatin1Char(' ') + settings->mapType()->rawValue().toString();
        if (current != _loadedMap) {
            // QGC's map names are "<provider> <type>" (QGCMapEngineManager::mapProviderList).
            const qsizetype space = _loadedMap.indexOf(QLatin1Char(' '));
            settings->mapProvider()->setRawValue(_loadedMap.left(space));
            settings->mapType()->setRawValue(_loadedMap.mid(space + 1));
            _lines.append(tr("The map is now \"%1\", the map of the pack. The pack's pictures show on that map only.").arg(_loadedMap));
            qCInfo(MapPackLog) << "Map changed from" << current << "to" << _loadedMap;
        }
        _loadedMap.clear();
    }
    if (_downloadHint) {
        _lines.append(tr("Android lets the app read the Download folder only when \"All files access\" is on for VAMA GCS. "
                         "The app's own folder works always: %1").arg(_roots.constFirst()));
        _downloadHint = false;
    }
    if (!_lines.isEmpty()) {
        qgcApp()->showAppMessage(_lines.join(QStringLiteral("\n\n")), tr("Offline maps"));
        _lines.clear();
    }
    emit done();
}

void MapPackLoader::_report(const MapPack::Pack &pack, const QString &problem)
{
    qCWarning(MapPackLog) << "Map pack not loaded:" << pack.folder << problem;
    // Each problem of a pack is told once, not at every start.
    const QString key = _keyOf(pack.folder);
    Kept kept = _kept(key);
    const QString reported = pack.signature + QLatin1Char(' ') + problem;
    if (kept.reported == reported) {
        return;
    }
    kept.reported = reported;
    _keep(key, kept);
    _lines.append(tr("Map pack \"%1\" was not loaded: %2.").arg(pack.name, problem));
    if (_roots.size() > 1 && pack.folder.startsWith(_roots.at(1))) {
        _downloadHint = true;
    }
}

void MapPackLoader::_runInThread(std::function<void()> work)
{
    QThread *const thread = QThread::create(std::move(work));
    thread->setObjectName(QStringLiteral("VamaMapPacks"));
    (void) connect(thread, &QThread::finished, thread, &QObject::deleteLater);
    _thread = thread;
    // Below the threads that show the video and answer the operator.
    thread->start(QThread::LowPriority);
}

// Hands a task to QGC's map worker. False when the worker has not started yet (it starts
// with the first map on the screen), or when it refused the task. A refusal sends the task's
// error at once, from within this call: the error handlers ignore that and _broken is set.
bool MapPackLoader::_addTask(QGCMapTask *task)
{
    _adding = true;
    (void) connect(task, &QGCMapTask::error, this, [this]() {
        if (_adding) {
            _broken = true;
        }
    });
    const bool added = getQGCMapEngine()->addTask(task);
    _adding = false;
    if (!added) {
        task->deleteLater();
    }
    return added;
}

void MapPackLoader::_retryLater()
{
    if (_broken || ++_retries > kMaxRetries) {
        if (_broken) {
            qCWarning(MapPackLog) << "QGC's map worker refused a task, so no map pack was loaded";
            _lines.append(tr("No map pack was loaded: the map store of the app does not work."));
        } else {
            qCWarning(MapPackLog) << "QGC's map worker did not start, so no map pack was loaded";
        }
        _toCheck.clear();
        _toLoad.clear();
        if (_importWaiting) {
            (void) QFile::remove(_importFile);
            _importWaiting = false;
        }
        _finish();
        return;
    }
    _retryTimer.start(kRetryMs);
}

QString MapPackLoader::_keyOf(const QString &folder)
{
    return QString::fromLatin1(QCryptographicHash::hash(folder.toUtf8(), QCryptographicHash::Sha1).toHex().left(16));
}

MapPackLoader::Kept MapPackLoader::_kept(const QString &key) const
{
    QSettings settings;
    settings.beginGroup(QLatin1String(kSettingsGroup));
    settings.beginGroup(key);
    Kept kept;
    kept.signature = settings.value(QStringLiteral("signature")).toString();
    kept.probes = settings.value(QStringLiteral("probes")).toStringList();
    kept.reported = settings.value(QStringLiteral("reported")).toString();
    return kept;
}

void MapPackLoader::_keep(const QString &key, const Kept &kept)
{
    QSettings settings;
    settings.beginGroup(QLatin1String(kSettingsGroup));
    settings.beginGroup(key);
    settings.setValue(QStringLiteral("signature"), kept.signature);
    settings.setValue(QStringLiteral("probes"), kept.probes);
    settings.setValue(QStringLiteral("reported"), kept.reported);
}
