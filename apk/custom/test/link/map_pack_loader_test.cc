// Host test for the map pack loader (task U): what the app does at its start with the packs
// it finds (src/Maps/MapPackLoader.cc). Stand-ins replace the QGC parts it talks to
// (map_stubs): the map engine runs its tasks against a real QGC tile store
// (QGCTileCacheDatabase.cpp, the code the app runs), the application keeps the messages,
// the settings keep the map choice. The packs come from ../map_packs (gen_map_packs.py).

#include <QtCore/QDir>
#include <QtCore/QDirIterator>
#include <QtCore/QElapsedTimer>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QSettings>
#include <QtCore/QTemporaryDir>
#include <QtSql/QSqlQuery>
#include <QtTest/QSignalSpy>
#include <QtTest/QtTest>

#include <memory>

#include "MapPackLoader.h"
#include "QGCApplication.h"
#include "QGCCacheTile.h"
#include "QGCMapEngine.h"
#include "QGCMapEngineManager.h"
#include "QGCMapUrlEngine.h"
#include "QGCTileCacheDatabase.h"
#include "SettingsManager.h"

namespace {

const QString kSatellite = QStringLiteral("Esri World Satellite");
const QString kGoogle = QStringLiteral("http://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}");

QString packFolder(const QString &name)
{
    return QStringLiteral(VAMA_MAP_PACKS "/") + name;
}

QByteArray bytesOf(const QString &path)
{
    QFile file(path);
    return file.open(QIODevice::ReadOnly) ? file.readAll() : QByteArray();
}

bool writeFile(const QString &path, const QByteArray &bytes)
{
    QDir().mkpath(QFileInfo(path).absolutePath());
    QFile file(path);
    return file.open(QIODevice::WriteOnly) && file.write(bytes) == bytes.size();
}

bool copyFolder(const QString &from, const QString &to)
{
    QDirIterator it(from, QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
        const QString file = it.next();
        const QString target = to + file.mid(from.size());
        QDir().mkpath(QFileInfo(target).absolutePath());
        if (!QFile::copy(file, target)) {
            return false;
        }
    }
    return true;
}

QString loadedLine(const QString &name, int pictures, int minZoom, int maxZoom)
{
    return QStringLiteral("Map pack \"%1\" is loaded: %2 map pictures, zoom %3 to %4. The map shows this place without the internet.")
        .arg(name).arg(pictures).arg(minZoom).arg(maxZoom);
}

const QString kSwitchedLine = QStringLiteral("The map is now \"Esri World Satellite\", the map of the pack. The pack's pictures show on that map only.");

}  // namespace

class MapPackLoaderTest : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase()
    {
        QVERIFY(_temp.isValid());
        // The loader keeps what it loaded in the settings: a file of this test, never the developer's own.
        QCoreApplication::setOrganizationName(QStringLiteral("VamaTest"));
        QCoreApplication::setApplicationName(QStringLiteral("map_pack_loader_test"));
        QSettings::setDefaultFormat(QSettings::IniFormat);
        QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, _temp.filePath(QStringLiteral("settings")));
        QVERIFY2(QFileInfo::exists(packFolder(QStringLiteral("esri_satellite/pack.json"))),
                 "the test packs are missing: run apk/custom/test/gen_map_packs.py");
    }

    // Each test: a fresh app (store, folders, settings, map choice) on its own folders.
    void init()
    {
        const QString base = _temp.filePath(QString::fromLatin1(QTest::currentTestFunction()));
        _own = base + QStringLiteral("/own/Map packs");
        _shared = base + QStringLiteral("/shared/Download/VAMA maps");
        _work = base + QStringLiteral("/cache/VamaMapPacks");
        // QGC makes the store's folder first (QGeoFileTileCacheQGC::_initCache).
        QVERIFY(QDir().mkpath(base + QStringLiteral("/QGCMapCache")));
        _store = std::make_unique<QGCTileCacheDatabase>(base + QStringLiteral("/QGCMapCache/qgcMapCache.db"));
        QVERIFY(_store->init() && _store->connectDB());
        QSettings().remove(QStringLiteral("VamaMapPacks"));
        QGCMapEngine *const engine = getQGCMapEngine();
        engine->state = QGCMapEngine::State::Working;
        engine->notStartedFor = 0;
        engine->store = _store.get();
        setMap(QStringLiteral("Google"), QStringLiteral("Hybrid"));
    }

    void cleanup()
    {
        getQGCMapEngine()->store = nullptr;
        _store.reset();
    }

    void aNewPackIsLoadedAndTheMapChosen()
    {
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), _own + QStringLiteral("/kalol")));
        const int lists = QGCMapEngineManager::instance()->loads;

        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(qgcApp()->messages.constFirst().first, QStringLiteral("Offline maps"));
        QCOMPARE(qgcApp()->messages.constFirst().second, loadedLine(QStringLiteral("kalol"), 17, 14, 16) + QStringLiteral("\n\n") + kSwitchedLine);
        QCOMPARE(map(), kSatellite);
        QCOMPARE(_imports, 1);
        QCOMPARE(_fetches, 0);
        QCOMPARE(QGCMapEngineManager::instance()->loads, lists + 1);

        const QList<TileSetRecord> sets = namedSets();
        QCOMPARE(sets.size(), 1);
        QCOMPARE(sets.constFirst().name, QStringLiteral("Map pack kalol"));
        QCOMPARE(sets.constFirst().mapTypeStr, kSatellite);
        QCOMPARE(int(sets.constFirst().numTiles), 17);
        // The picture under the client's place, at zoom 16, as QGC's map asks for it.
        const std::unique_ptr<QGCCacheTile> tile = _store->getTile(UrlFactory::getTileHash(kSatellite, 46032, 28964, 16));
        QVERIFY(tile);
        QCOMPARE(tile->img, bytesOf(packFolder(QStringLiteral("esri_satellite/16/46032/28964.png"))));
        // The go-between file is gone, and the pack is as it was.
        QCOMPARE(QDir(_work).entryList(QDir::Files).size(), 0);
        QCOMPARE(QDir(_own + QStringLiteral("/kalol/16")).entryList(QDir::Dirs | QDir::NoDotAndDotDot).size(), 3);
    }

    void aPackSaysWhileItLoadsAndWhereItIs()
    {
        // Task AB (field test of build 11): a big pack took minutes, the app said nothing, and the
        // map stayed on another place.
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), _own + QStringLiteral("/kalol")));
        QVERIFY(run());
        QCOMPARE(_loadingSeen, QStringList({QStringLiteral("Loading map pack \"kalol\": 17 map pictures, zoom 14 to 16. "
                                                          "The map shows them when this is done."),
                                           QString()}));
        QCOMPARE(_loadingAtEnd, QString());
        // The middle of the area its pack.json gives (VGCS's bbox), once.
        QCOMPARE(_loadedAt.size(), 1);
        QCOMPARE(_loadedAt.constFirst().x(), (20.442206755300035 + 20.449393244699962) / 2.0);
        QCOMPARE(_loadedAt.constFirst().y(), (72.85956517290967 + 72.86723482709033) / 2.0);
    }

    void aPackWithTheOverviewIsLoadedAndPlacedByItsDetail()
    {
        // VGCS's export since task AB: zoom 10 to 13 over the area and 5 km round it, then the detail.
        // Every picture is loaded, and the map goes to the middle of the detail (pack.json's area), not
        // to the middle of the wider pictures.
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_overview")), _own + QStringLiteral("/kalol")));
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(qgcApp()->messages.constFirst().second, loadedLine(QStringLiteral("kalol"), 23, 10, 15) + QStringLiteral("\n\n") + kSwitchedLine);
        QCOMPARE(_loadingSeen.value(0), QStringLiteral("Loading map pack \"kalol\": 23 map pictures, zoom 10 to 15. "
                                                     "The map shows them when this is done."));
        QCOMPARE(int(namedSets().constFirst().numTiles), 23);
        // The zoom 10 picture of the client's place, as QGC's map asks for it.
        const std::unique_ptr<QGCCacheTile> tile = _store->getTile(UrlFactory::getTileHash(kSatellite, 719, 452, 10));
        QVERIFY(tile);
        QCOMPARE(tile->img, bytesOf(packFolder(QStringLiteral("esri_overview/10/719/452.png"))));
        QCOMPARE(_loadedAt.size(), 1);
        QCOMPARE(_loadedAt.constFirst().x(), (20.442206755300035 + 20.449393244699962) / 2.0);
        QCOMPARE(_loadedAt.constFirst().y(), (72.85956517290967 + 72.86723482709033) / 2.0);
    }

    void aLoadedPackIsOnlyChecked()
    {
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), _own + QStringLiteral("/kalol")));
        QVERIFY(run());
        QVERIFY(run());
        // Three of its pictures were asked for, all there: nothing loaded, nothing said.
        QCOMPARE(qgcApp()->messages.size(), 0);
        QCOMPARE(_fetches, 3);
        QCOMPARE(_imports, 0);
        QCOMPARE(namedSets().size(), 1);
        // Nothing loading, and the map is not moved.
        QVERIFY(_loadingSeen.isEmpty());
        QVERIFY(_loadedAt.isEmpty());
    }

    void aPackWhosePicturesWereClearedIsLoadedAgain()
    {
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), _own + QStringLiteral("/kalol")));
        QVERIFY(run());
        // Someone deleted the set in Offline Maps (QGC's own delete).
        QVERIFY(_store->deleteTileSet(namedSets().constFirst().setID));
        QCOMPARE(namedSets().size(), 0);
        QVERIFY(!_store->getTile(UrlFactory::getTileHash(kSatellite, 46032, 28964, 16)));

        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        // The map is still the pack's map: no word about it.
        QCOMPARE(qgcApp()->messages.constFirst().second, loadedLine(QStringLiteral("kalol"), 17, 14, 16));
        QVERIFY(_fetches >= 1);
        QCOMPARE(_imports, 1);
        QCOMPARE(namedSets().size(), 1);
        QVERIFY(_store->getTile(UrlFactory::getTileHash(kSatellite, 46032, 28964, 16)));
    }

    void aPackMissingOnePictureIsLoadedAgain()
    {
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), _own + QStringLiteral("/kalol")));
        QVERIFY(run());
        // One of the pictures it asks for (the pack's last) is gone from the store, the others are there.
        const QString last = UrlFactory::getTileHash(kSatellite, 46033, 28965, 16);
        QSqlQuery query(_store->database());
        QVERIFY(query.prepare(QStringLiteral("DELETE FROM Tiles WHERE hash = ?")));
        query.addBindValue(last);
        QVERIFY(query.exec());
        QVERIFY(!_store->getTile(last));
        QVERIFY(_store->getTile(UrlFactory::getTileHash(kSatellite, 11507, 7240, 14)));

        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(qgcApp()->messages.constFirst().second, loadedLine(QStringLiteral("kalol"), 17, 14, 16));
        QCOMPARE(_fetches, 3);
        QCOMPARE(_imports, 1);
        QVERIFY(_store->getTile(last));
    }

    void aChangedPackIsLoadedAgain()
    {
        const QString pack = _own + QStringLiteral("/kalol");
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), pack));
        QVERIFY(run());
        QVERIFY(writeFile(pack + QStringLiteral("/16/46032/28966.png"), bytesOf(pack + QStringLiteral("/16/46032/28964.png"))));

        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(qgcApp()->messages.constFirst().second, loadedLine(QStringLiteral("kalol"), 18, 14, 16));
        QCOMPARE(_fetches, 0);
        QCOMPARE(_imports, 1);
        // QGC gives the second set a name of its own. The pictures are kept once.
        const QList<TileSetRecord> sets = namedSets();
        QCOMPARE(sets.size(), 2);
        QCOMPARE(sets.at(0).name, QStringLiteral("Map pack kalol"));
        QCOMPARE(sets.at(1).name, QStringLiteral("Map pack kalol 0001"));
        QCOMPARE(int(sets.at(1).numTiles), 18);
        QCOMPARE(int(_store->computeTotals().totalCount), 18);
    }

    void theMapChosenLaterStays()
    {
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), _own + QStringLiteral("/kalol")));
        QVERIFY(run());
        QCOMPARE(map(), kSatellite);
        // The operator chooses another map. With nothing new to load, it stays.
        setMap(QStringLiteral("Google"), QStringLiteral("Hybrid"));
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 0);
        QCOMPARE(map(), QStringLiteral("Google Hybrid"));
    }

    void otherMapsArePackMapsToo()
    {
        QVERIFY(copyFolder(packFolder(QStringLiteral("osm_streets")), _own + QStringLiteral("/streets")));
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(qgcApp()->messages.constFirst().second,
                 loadedLine(QStringLiteral("streets"), 4, 15, 15) + QStringLiteral("\n\n")
                     + QStringLiteral("The map is now \"Street Map\", the map of the pack. The pack's pictures show on that map only."));
        // QGC's names are "<provider> <type>".
        QCOMPARE(SettingsManager::instance()->flightMapSettings()->mapProvider()->rawValue().toString(), QStringLiteral("Street"));
        QCOMPARE(SettingsManager::instance()->flightMapSettings()->mapType()->rawValue().toString(), QStringLiteral("Map"));
    }

    void leftOutFilesAreTold()
    {
        const QString pack = _own + QStringLiteral("/kalol");
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), pack));
        QVERIFY(writeFile(pack + QStringLiteral("/16/46034/28963.png"), QByteArray()));
        QVERIFY(writeFile(pack + QStringLiteral("/16/46034/28964.png"), "Map data not yet available"));
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(qgcApp()->messages.constFirst().second,
                 loadedLine(QStringLiteral("kalol"), 17, 14, 16) + QStringLiteral(" 2 files in it were no map pictures and were left out.\n\n") + kSwitchedLine);
        // Its probes are pictures that are in the store, so the next start finds it loaded.
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 0);
        QCOMPARE(_imports, 0);
    }

    void aProblemIsToldOnce()
    {
        const QString pack = _own + QStringLiteral("/google");
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), pack));
        QVERIFY(writeFile(pack + QStringLiteral("/pack.json"), QByteArrayLiteral("{\"format\": \"vgcs-tile-pack/1\", \"template\": \"") + kGoogle.toLatin1()
                          + QByteArrayLiteral("\"}")));
        const QString told = QStringLiteral("Map pack \"google\" was not loaded: the app has no map for its source (%1).").arg(kGoogle);

        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(qgcApp()->messages.constFirst().second, told);
        QCOMPARE(map(), QStringLiteral("Google Hybrid"));
        QCOMPARE(_imports, 0);
        // Not at every start.
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 0);
        // Again when the pack changed.
        QVERIFY(writeFile(pack + QStringLiteral("/16/46032/28966.png"), bytesOf(pack + QStringLiteral("/16/46032/28964.png"))));
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(qgcApp()->messages.constFirst().second, told);
    }

    void aPackInDownloadThatCannotBeReadGivesTheHint()
    {
        // Android 11 and newer hide pack.json and the pictures of Download from an app without
        // "All files access": the folders are seen, their files are not.
        QVERIFY(QDir().mkpath(_shared + QStringLiteral("/kalol/16/46032")));
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), _own + QStringLiteral("/kalol")));
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        const QStringList lines = qgcApp()->messages.constFirst().second.split(QStringLiteral("\n\n"));
        QCOMPARE(lines.size(), 4);
        QCOMPARE(lines.at(0), QStringLiteral("Map pack \"kalol\" was not loaded: it has no pack.json."));
        QCOMPARE(lines.at(1), loadedLine(QStringLiteral("kalol"), 17, 14, 16));
        QCOMPARE(lines.at(2), kSwitchedLine);
        QCOMPARE(lines.at(3), QStringLiteral("Android lets the app read the Download folder only when \"All files access\" is on for VAMA GCS. "
                                             "The app's own folder works always: %1").arg(_own));
    }

    void aProblemInTheOwnFolderGivesNoHint()
    {
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite/16")), _own + QStringLiteral("/bare/16")));
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(qgcApp()->messages.constFirst().second, QStringLiteral("Map pack \"bare\" was not loaded: it has no pack.json."));
    }

    void theMapWorkerIsWaitedFor()
    {
        // QGC's map worker starts with the first map on the screen, maybe after the loader asks.
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), _own + QStringLiteral("/kalol")));
        getQGCMapEngine()->notStartedFor = 2;
        QElapsedTimer took;
        took.start();
        QVERIFY(run(30000));
        QVERIFY(took.elapsed() >= 3500);  // asked again after 2 s, twice
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(qgcApp()->messages.constFirst().second, loadedLine(QStringLiteral("kalol"), 17, 14, 16) + QStringLiteral("\n\n") + kSwitchedLine);
        QCOMPARE(_imports, 1);
    }

    void aStoreThatDoesNotWorkIsToldOnce()
    {
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), _own + QStringLiteral("/kalol")));
        getQGCMapEngine()->state = QGCMapEngine::State::Broken;
        QVERIFY(run());
        // One line: the refusal's own error is not told as the pack's problem too.
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(qgcApp()->messages.constFirst().second, QStringLiteral("No map pack was loaded: the map store of the app does not work."));
        QCOMPARE(map(), QStringLiteral("Google Hybrid"));
        QCOMPARE(QDir(_work).entryList(QDir::Files).size(), 0);
        // It said it was loading, and that ended. The map is not moved.
        QCOMPARE(_loadingSeen.size(), 2);
        QCOMPARE(_loadingAtEnd, QString());
        QVERIFY(_loadedAt.isEmpty());
        // When the store works again, the pack loads.
        getQGCMapEngine()->state = QGCMapEngine::State::Working;
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(qgcApp()->messages.constFirst().second, loadedLine(QStringLiteral("kalol"), 17, 14, 16) + QStringLiteral("\n\n") + kSwitchedLine);
    }

    void aCheckRefusedByTheStoreEndsOnce()
    {
        // A pack of one picture keeps one probe. The refusal of its check must not count as the
        // check's answer as well (that would load the pack while the loader ends).
        const QString pack = _own + QStringLiteral("/one");
        QVERIFY(writeFile(pack + QStringLiteral("/pack.json"), bytesOf(packFolder(QStringLiteral("esri_satellite/pack.json")))));
        QVERIFY(writeFile(pack + QStringLiteral("/16/46032/28964.png"), bytesOf(packFolder(QStringLiteral("esri_satellite/16/46032/28964.png")))));
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(_imports, 1);
        getQGCMapEngine()->state = QGCMapEngine::State::Broken;
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(qgcApp()->messages.constFirst().second, QStringLiteral("No map pack was loaded: the map store of the app does not work."));
        QCOMPARE(_imports, 0);
        QCOMPARE(QDir(_work).entryList(QDir::Files).size(), 0);
    }

    void aFailedImportIsTold()
    {
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), _own + QStringLiteral("/kalol")));
        getQGCMapEngine()->state = QGCMapEngine::State::FailsImports;
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(qgcApp()->messages.constFirst().second,
                 QStringLiteral("Map pack \"kalol\" was not loaded: the map store did not take it (Error opening import database)."));
        QCOMPARE(_imports, 1);
        QCOMPARE(map(), QStringLiteral("Google Hybrid"));
        QCOMPARE(_loadingAtEnd, QString());
        QVERIFY(_loadedAt.isEmpty());
        QCOMPARE(QDir(_work).entryList(QDir::Files).size(), 0);
        // Not kept as loaded: the next start loads it.
        getQGCMapEngine()->state = QGCMapEngine::State::Working;
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(qgcApp()->messages.constFirst().second, loadedLine(QStringLiteral("kalol"), 17, 14, 16) + QStringLiteral("\n\n") + kSwitchedLine);
    }

    void noPacksSayNothing()
    {
        QVERIFY(!QFileInfo::exists(_own));
        // A go-between file of a start that was cut off.
        QVERIFY(writeFile(_work + QStringLiteral("/pack-0123456789abcdef.db"), "half written"));
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 0);
        QCOMPARE(_fetches, 0);
        QCOMPARE(_imports, 0);
        QVERIFY(!QFileInfo::exists(_work + QStringLiteral("/pack-0123456789abcdef.db")));
        // The app's own folder is made, to copy packs into.
        QVERIFY(QFileInfo(_own).isDir());
        QVERIFY(!QFileInfo::exists(_shared));
    }

    void aLoaderEndedAtOnceLeavesNoTrace()
    {
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), _own + QStringLiteral("/kalol")));
        {
            // The app is closed right after its start.
            MapPackLoader loader({_own, _shared}, _work);
            loader.start();
        }
        QCoreApplication::processEvents();
        QCOMPARE(namedSets().size(), 0);
        QCOMPARE(QDir(_work).entryList(QDir::Files).size(), 0);
        // The next start loads the pack.
        QVERIFY(run());
        QCOMPARE(qgcApp()->messages.size(), 1);
        QCOMPARE(namedSets().size(), 1);
    }

private:
    // One start of the app: a loader on this test's folders, until it is done.
    bool run(int timeoutMs = 20000)
    {
        qgcApp()->messages.clear();
        QGCMapEngine *const engine = getQGCMapEngine();
        const int fetches = engine->fetches;
        const int imports = engine->imports;
        MapPackLoader loader({_own, _shared}, _work);
        QSignalSpy done(&loader, &MapPackLoader::done);
        // What the fly view is told: each loading line, and where a loaded pack is.
        _loadingSeen.clear();
        _loadedAt.clear();
        (void) connect(&loader, &MapPackLoader::loadingChanged, &loader, [this, &loader]() { _loadingSeen.append(loader.loading()); });
        (void) connect(&loader, &MapPackLoader::packLoaded, &loader, [this](double latitude, double longitude) {
            _loadedAt.append(QPointF(latitude, longitude));
        });
        loader.start();
        const bool finished = done.count() > 0 || done.wait(timeoutMs);
        // Nothing more may happen once it is done: work still on its way would show here.
        QTest::qWait(200);
        _fetches = engine->fetches - fetches;
        _imports = engine->imports - imports;
        _loadingAtEnd = loader.loading();
        return finished && done.count() == 1;
    }

    QList<TileSetRecord> namedSets()
    {
        QList<TileSetRecord> sets = _store->getTileSets();
        sets.removeIf([](const TileSetRecord &set) { return set.defaultSet; });
        return sets;
    }

    static QString map()
    {
        FlightMapSettings *const settings = SettingsManager::instance()->flightMapSettings();
        return settings->mapProvider()->rawValue().toString() + QLatin1Char(' ') + settings->mapType()->rawValue().toString();
    }

    static void setMap(const QString &provider, const QString &type)
    {
        FlightMapSettings *const settings = SettingsManager::instance()->flightMapSettings();
        settings->mapProvider()->setRawValue(provider);
        settings->mapType()->setRawValue(type);
    }

    QTemporaryDir _temp;
    QString _own;
    QString _shared;
    QString _work;
    std::unique_ptr<QGCTileCacheDatabase> _store;
    int _fetches = 0;
    int _imports = 0;
    QStringList _loadingSeen;  // loading() at each change, in the last run
    QString _loadingAtEnd;     // loading() when it was done
    QList<QPointF> _loadedAt;  // packLoaded(): latitude, longitude
};

QTEST_GUILESS_MAIN(MapPackLoaderTest)
#include "map_pack_loader_test.moc"
