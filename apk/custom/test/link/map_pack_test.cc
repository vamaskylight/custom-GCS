// Host test for the map packs (task U). The app reads a pack that VGCS made and writes it
// as a QGC tile file (src/Maps/MapPack.cc). QGC's own tile store code merges that file, as
// Offline Maps' import does: QGCTileCacheDatabase.cpp from apk/qgc-src, the code the app
// runs. The packs in ../map_packs were made by VGCS's own pack code (gen_map_packs.py).

#include <QtCore/QDir>
#include <QtCore/QDirIterator>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QSettings>
#include <QtCore/QTemporaryDir>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>
#include <QtTest/QtTest>

#include <algorithm>
#include <cmath>
#include <memory>
#include <tuple>

#include "MapPack.h"
#include "QGCCacheTile.h"
#include "QGCMapUrlEngine.h"
#include "QGCTileCacheDatabase.h"

namespace {

const QString kSatellite = QStringLiteral("Esri World Satellite");
const QString kEsriImagery = QStringLiteral("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}");

// gen_map_packs.py made the packs around the client's place.
constexpr double kLat = 20.4458;
constexpr double kLon = 72.8634;

QString packFolder(const QString &name)
{
    return QStringLiteral(VAMA_MAP_PACKS "/") + name;
}

MapPack::TileKey keyFor(const QString &mapName)
{
    return [mapName](int x, int y, int z) { return UrlFactory::getTileHash(mapName, x, y, z); };
}

// The tile under a place, by QGC's own sums (MapProvider::long2tileX and lat2tileY).
int tileX(double lon, int z)
{
    return static_cast<int>(floor((lon + 180.0) / 360.0 * pow(2.0, z)));
}

int tileY(double lat, int z)
{
    return static_cast<int>(floor((1.0 - log(tan(lat * M_PI / 180.0) + 1.0 / cos(lat * M_PI / 180.0)) / M_PI) / 2.0 * pow(2.0, z)));
}

QByteArray bytesOf(const QString &path)
{
    QFile file(path);
    return file.open(QIODevice::ReadOnly) ? file.readAll() : QByteArray();
}

// Source text without spaces and line ends (the QGC copy here has Windows line ends).
QByteArray squeezed(QByteArray text)
{
    text.removeIf([](char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; });
    return text;
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

bool hasTile(const MapPack::Pack &pack, int z, int x, int y)
{
    return std::any_of(pack.tiles.cbegin(), pack.tiles.cend(),
                       [&](const MapPack::Tile &t) { return t.z == z && t.x == x && t.y == y; });
}

// A QGC tile store, made and opened as the app's map worker does it (QGCCacheWorker::run).
std::unique_ptr<QGCTileCacheDatabase> openStore(const QString &path)
{
    auto store = std::make_unique<QGCTileCacheDatabase>(path);
    if (!store->init() || !store->connectDB()) {
        return nullptr;
    }
    return store;
}

QList<TileSetRecord> namedSets(QGCTileCacheDatabase &store)
{
    QList<TileSetRecord> sets = store.getTileSets();
    sets.removeIf([](const TileSetRecord &set) { return set.defaultSet; });
    return sets;
}

}  // namespace

class MapPackTest : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase()
    {
        QVERIFY(_temp.isValid());
        // QGC's store code may use QSettings: never the developer's own.
        QSettings::setDefaultFormat(QSettings::IniFormat);
        QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, _temp.filePath(QStringLiteral("settings")));
        QVERIFY2(QFileInfo::exists(packFolder(QStringLiteral("esri_satellite/pack.json"))),
                 "the test packs are missing: run apk/custom/test/gen_map_packs.py");
    }

    void sourcesHaveTheirQgcMaps()
    {
        QCOMPARE(MapPack::mapNameForSource(kEsriImagery), kSatellite);
        QCOMPARE(MapPack::mapNameForSource(QStringLiteral(" http://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x} ")), kSatellite);
        QCOMPARE(MapPack::mapNameForSource(QStringLiteral("https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}")),
                 QStringLiteral("Esri World Street"));
        QCOMPARE(MapPack::mapNameForSource(QStringLiteral("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png")), QStringLiteral("Street Map"));
        QCOMPARE(MapPack::mapNameForSource(QStringLiteral("https://tile.openstreetmap.org/{z}/{x}/{y}.png")), QStringLiteral("Street Map"));
        // Sources the app has no map of its own for.
        QCOMPARE(MapPack::mapNameForSource(QStringLiteral("http://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}")), QString());
        QCOMPARE(MapPack::mapNameForSource(QStringLiteral("https://server.arcgisonline.com/ArcGIS/rest/services/World_Terrain_Base/MapServer/tile/{z}/{y}/{x}")), QString());
        QCOMPARE(MapPack::mapNameForSource(QStringLiteral("{local}")), QString());
        QCOMPARE(MapPack::mapNameForSource(QString()), QString());

        // The names are QGC's own: they are in the QGC source the app is built from.
        const QByteArray esri = squeezed(bytesOf(QStringLiteral(VAMA_QGC_SRC "/src/QtLocationPlugin/Providers/EsriMapProvider.h")));
        const QByteArray generic = squeezed(bytesOf(QStringLiteral(VAMA_QGC_SRC "/src/QtLocationPlugin/Providers/GenericMapProvider.h")));
        QVERIFY(esri.contains(R"(QStringLiteral("EsriWorldSatellite"),QStringLiteral("World_Imagery"))"));
        QVERIFY(esri.contains(R"(QStringLiteral("EsriWorldStreet"),QStringLiteral("World_Street_Map"))"));
        QVERIFY(generic.contains(R"(OpenStreetMapProvider():MapProvider(QStringLiteral("StreetMap"))"));
        QVERIFY(generic.contains("http://tile.openstreetmap.org/%1/%2/%3.png"));
    }

    void picturesAreToldByTheirBytes()
    {
        QCOMPARE(MapPack::pictureFormat(QByteArray("\x89PNG\r\n\x1a\n rest", 13)), QStringLiteral("png"));
        QCOMPARE(MapPack::pictureFormat(QByteArray("\xFF\xD8\xFF\xE0 rest", 8)), QStringLiteral("jpg"));
        QCOMPARE(MapPack::pictureFormat(QByteArray("GIF89a rest")), QStringLiteral("gif"));
        QCOMPARE(MapPack::pictureFormat(QByteArray()), QString());
        QCOMPARE(MapPack::pictureFormat(QByteArray("\xFF\xD8", 2)), QString());
        QCOMPARE(MapPack::pictureFormat(QByteArray("Map data not yet available")), QString());
        QCOMPARE(MapPack::pictureFormat(QByteArray("<html><body>blocked</body></html>")), QString());
        // VGCS names every picture .png, and Esri's are JPEG inside.
        QCOMPARE(MapPack::pictureFormat(bytesOf(packFolder(QStringLiteral("esri_satellite/16/46032/28964.png")))), QStringLiteral("jpg"));
        QCOMPARE(MapPack::pictureFormat(bytesOf(packFolder(QStringLiteral("esri_streets/15/23015/14481.png")))), QStringLiteral("png"));
    }

    void aVgcsPackIsRead()
    {
        const MapPack::Pack pack = MapPack::readPack(packFolder(QStringLiteral("esri_satellite")));
        QCOMPARE(pack.problem, QString());
        QCOMPARE(pack.name, QStringLiteral("esri_satellite"));
        QCOMPARE(pack.source, kEsriImagery);
        QCOMPARE(pack.mapName, kSatellite);
        QCOMPARE(pack.tiles.size(), 17);  // pack.json says 17
        QCOMPARE(pack.minZoom, 14);
        QCOMPARE(pack.maxZoom, 16);
        qint64 bytes = 0;
        for (qsizetype i = 0; i < pack.tiles.size(); ++i) {
            const MapPack::Tile &t = pack.tiles.at(i);
            QCOMPARE(t.bytes, QFileInfo(t.path).size());
            QCOMPARE(QFileInfo(t.path).fileName(), QStringLiteral("%1.png").arg(t.y));
            bytes += t.bytes;
            if (i > 0) {
                const MapPack::Tile &u = pack.tiles.at(i - 1);
                QVERIFY(std::tie(u.z, u.x, u.y) < std::tie(t.z, t.x, t.y));
            }
        }
        QCOMPARE(pack.bytes, bytes);
        // The area is pack.json's.
        QCOMPARE(pack.south, 20.442206755300035);
        QCOMPARE(pack.west, 72.85956517290967);
        QCOMPARE(pack.north, 20.449393244699962);
        QCOMPARE(pack.east, 72.86723482709033);
        // The picture under the place, at each zoom, is the one QGC's map asks for there.
        for (int z = 14; z <= 16; ++z) {
            QVERIFY2(hasTile(pack, z, tileX(kLon, z), tileY(kLat, z)), qPrintable(QStringLiteral("zoom %1").arg(z)));
        }
        QCOMPARE(pack.signature.size(), 40);
    }

    void tilesAreInNumberOrder()
    {
        // Folder names sort as text (10 before 9). The pictures go by zoom, x and y as numbers.
        const QString folder = _temp.filePath(QStringLiteral("order"));
        QVERIFY(writeFile(folder + QStringLiteral("/pack.json"), bytesOf(packFolder(QStringLiteral("esri_satellite/pack.json")))));
        const QByteArray jpeg = bytesOf(packFolder(QStringLiteral("esri_satellite/16/46032/28964.png")));
        for (const char *name : {"10/1/1.png", "9/1/1.png", "5/10/10.png", "5/10/9.png", "5/10/1.png", "5/9/1.png"}) {
            QVERIFY(writeFile(folder + QLatin1Char('/') + QLatin1String(name), jpeg));
        }
        const MapPack::Pack pack = MapPack::readPack(folder);
        QStringList order;
        for (const MapPack::Tile &t : pack.tiles) {
            order.append(QStringLiteral("%1/%2/%3").arg(t.z).arg(t.x).arg(t.y));
        }
        QCOMPARE(order, QStringList({QStringLiteral("5/9/1"), QStringLiteral("5/10/1"), QStringLiteral("5/10/9"),
                                     QStringLiteral("5/10/10"), QStringLiteral("9/1/1"), QStringLiteral("10/1/1")}));
        QCOMPARE(pack.minZoom, 5);
        QCOMPARE(pack.maxZoom, 10);
    }

    void anAreaComesFromThePicturesWithoutOneInPackJson()
    {
        const QString folder = _temp.filePath(QStringLiteral("noarea"));
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), folder));
        QVERIFY(writeFile(folder + QStringLiteral("/pack.json"), QByteArrayLiteral("{\"format\": \"vgcs-tile-pack/1\", \"template\": \"")
                          + kEsriImagery.toLatin1() + QByteArrayLiteral("\"}")));
        const MapPack::Pack pack = MapPack::readPack(folder);
        QCOMPARE(pack.problem, QString());
        // The zoom 14 pictures: x 11507 to 11508, y 7240 to 7241, a square of 2 by 2 pictures around the place.
        QVERIFY(pack.west < kLon && kLon < pack.east && pack.south < kLat && kLat < pack.north);
        QCOMPARE(tileX(pack.west + 1e-9, 14), 11507);
        QCOMPARE(tileX(pack.east - 1e-9, 14), 11508);
        QCOMPARE(tileY(pack.north - 1e-9, 14), 7240);
        QCOMPARE(tileY(pack.south + 1e-9, 14), 7241);
    }

    void packsAreFoundInFolders()
    {
        const QString root = _temp.filePath(QStringLiteral("found"));
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), root + QStringLiteral("/A")));
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_streets")), root + QStringLiteral("/B/C")));
        QVERIFY(copyFolder(packFolder(QStringLiteral("osm_streets")), root + QStringLiteral("/D/E/F")));  // too deep
        QVERIFY(QDir().mkpath(root + QStringLiteral("/G")));
        QVERIFY(writeFile(root + QStringLiteral("/H/readme.txt"), "a note"));
        // A pack inside a pack is not looked for.
        QVERIFY(copyFolder(packFolder(QStringLiteral("osm_streets")), root + QStringLiteral("/A/extra")));

        QList<MapPack::Pack> packs = MapPack::findPacks(root);
        QCOMPARE(packs.size(), 2);
        QCOMPARE(packs.at(0).name, QStringLiteral("A"));
        QCOMPARE(packs.at(0).problem, QString());
        QCOMPARE(packs.at(0).tiles.size(), 17);
        QCOMPARE(packs.at(1).name, QStringLiteral("C"));
        QCOMPARE(packs.at(1).mapName, QStringLiteral("Esri World Street"));

        // A root that is a pack itself.
        packs = MapPack::findPacks(root + QStringLiteral("/A"));
        QCOMPARE(packs.size(), 1);
        QCOMPARE(packs.at(0).name, QStringLiteral("A"));

        QCOMPARE(MapPack::findPacks(root + QStringLiteral("/G")).size(), 0);
        QCOMPARE(MapPack::findPacks(_temp.filePath(QStringLiteral("no such folder"))).size(), 0);

        // Map pictures without pack.json are found, with the reason they cannot be loaded.
        const QString bare = _temp.filePath(QStringLiteral("bare"));
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite/16")), bare + QStringLiteral("/X/16")));
        packs = MapPack::findPacks(bare);
        QCOMPARE(packs.size(), 1);
        QCOMPARE(packs.at(0).name, QStringLiteral("X"));
        QCOMPARE(packs.at(0).problem, QStringLiteral("it has no pack.json"));
        QCOMPARE(packs.at(0).tiles.size(), 9);
    }

    void foldersThatCannotBeLoadedSayWhy_data()
    {
        QTest::addColumn<QByteArray>("manifest");
        QTest::addColumn<bool>("pictures");
        QTest::addColumn<QString>("problem");
        const QByteArray esri = kEsriImagery.toLatin1();
        QTest::newRow("no JSON") << QByteArray("this is not JSON") << true << QStringLiteral("its pack.json cannot be read");
        QTest::newRow("a list") << QByteArray("[1, 2]") << true << QStringLiteral("its pack.json cannot be read");
        QTest::newRow("other kind") << QByteArray("{\"format\": \"vgcs-tile-pack/2\", \"template\": \"" + esri + "\"}") << true
                                    << QStringLiteral("it is a pack of another kind (vgcs-tile-pack/2)");
        QTest::newRow("no source") << QByteArray("{\"format\": \"vgcs-tile-pack/1\"}") << true << QStringLiteral("its pack.json names no map source");
        QTest::newRow("Google") << QByteArray("{\"template\": \"http://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}\"}") << true
                                << QStringLiteral("the app has no map for its source (http://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z})");
        QTest::newRow("no pictures") << QByteArray("{\"format\": \"vgcs-tile-pack/1\", \"template\": \"" + esri + "\"}") << false
                                     << QStringLiteral("it has no map pictures");
        QTest::newRow("too big") << QByteArray("{\"template\": \"" + esri + "\"}" + QByteArray(70 * 1024, ' ')) << true
                                 << QStringLiteral("its pack.json cannot be read");
    }

    void foldersThatCannotBeLoadedSayWhy()
    {
        QFETCH(QByteArray, manifest);
        QFETCH(bool, pictures);
        QFETCH(QString, problem);
        const QString folder = _temp.filePath(QStringLiteral("why/") + QString::fromLatin1(QTest::currentDataTag()));
        if (pictures) {
            QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), folder));
        }
        QVERIFY(writeFile(folder + QStringLiteral("/pack.json"), manifest));
        const QList<MapPack::Pack> packs = MapPack::findPacks(folder);
        QCOMPARE(packs.size(), 1);
        QCOMPARE(packs.at(0).problem, problem);
        QCOMPARE(packs.at(0).signature.size(), 40);
    }

    void filesThatAreNoPicturesAreLeftOut()
    {
        const QString folder = _temp.filePath(QStringLiteral("junk"));
        QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), folder));
        const QByteArray jpeg = bytesOf(packFolder(QStringLiteral("esri_satellite/16/46032/28964.png")));
        // Listed, and written: a capital suffix.
        QVERIFY(writeFile(folder + QStringLiteral("/16/46032/28966.PNG"), jpeg));
        // Listed, then left out when written: no picture inside.
        QVERIFY(writeFile(folder + QStringLiteral("/16/46034/28963.png"), QByteArray()));
        QVERIFY(writeFile(folder + QStringLiteral("/16/46034/28964.png"), "Map data not yet available"));
        QVERIFY(writeFile(folder + QStringLiteral("/16/46034/28965.png"), jpeg + QByteArray(4 * 1024 * 1024, '\0')));
        // Not listed: not <number>/<number>/<number>.png, or outside the zoom's own pictures.
        QVERIFY(writeFile(folder + QStringLiteral("/16/46032/notes.txt"), jpeg));
        QVERIFY(writeFile(folder + QStringLiteral("/16/46032/28967.jpg"), jpeg));
        QVERIFY(writeFile(folder + QStringLiteral("/16/46032/1.2.png"), jpeg));
        QVERIFY(writeFile(folder + QStringLiteral("/16/abc/28964.png"), jpeg));
        QVERIFY(writeFile(folder + QStringLiteral("/16/46032/65536.png"), jpeg));
        QVERIFY(writeFile(folder + QStringLiteral("/16/65536/1.png"), jpeg));
        QVERIFY(writeFile(folder + QStringLiteral("/24/0/0.png"), jpeg));
        QVERIFY(writeFile(folder + QStringLiteral("/0/0/0.png"), jpeg));
        QVERIFY(writeFile(folder + QStringLiteral("/maps/16/46032/28964.png"), jpeg));

        const MapPack::Pack pack = MapPack::readPack(folder);
        QCOMPARE(pack.problem, QString());
        QCOMPARE(pack.tiles.size(), 17 + 4);

        const QString file = _temp.filePath(QStringLiteral("junk.db"));
        const MapPack::Written written = MapPack::writeTileFile(pack, file, QStringLiteral("Map pack junk"),
                                                                UrlFactory::getQtMapIdFromProviderType(kSatellite), keyFor(kSatellite));
        QCOMPARE(written.error, QString());
        QCOMPARE(written.tiles, 18);
        QCOMPARE(written.skipped, 3);
        auto store = openStore(_temp.filePath(QStringLiteral("junk-store.db")));
        QVERIFY(store);
        QVERIFY(store->importSetsMerge(file, nullptr).success);
        QVERIFY(store->getTile(UrlFactory::getTileHash(kSatellite, 46032, 28966, 16)));
        QVERIFY(!store->getTile(UrlFactory::getTileHash(kSatellite, 46034, 28963, 16)));
        QVERIFY(!store->getTile(UrlFactory::getTileHash(kSatellite, 46034, 28964, 16)));
        QVERIFY(!store->getTile(UrlFactory::getTileHash(kSatellite, 46034, 28965, 16)));
    }

    void theSignatureFollowsThePack()
    {
        const QString original = MapPack::readPack(packFolder(QStringLiteral("esri_satellite"))).signature;
        const QString a = _temp.filePath(QStringLiteral("sig/a"));
        const QString b = _temp.filePath(QStringLiteral("sig/b"));
        const QString c = _temp.filePath(QStringLiteral("sig/c"));
        const QString d = _temp.filePath(QStringLiteral("sig/d"));
        for (const QString &copy : {a, b, c, d}) {
            QVERIFY(copyFolder(packFolder(QStringLiteral("esri_satellite")), copy));
        }
        // The same pack in another folder: the same.
        QCOMPARE(MapPack::readPack(a).signature, original);
        // One more picture.
        QVERIFY(writeFile(b + QStringLiteral("/16/46032/28966.png"), bytesOf(a + QStringLiteral("/16/46032/28964.png"))));
        QVERIFY(MapPack::readPack(b).signature != original);
        // A picture of another size.
        QVERIFY(writeFile(c + QStringLiteral("/16/46032/28964.png"), bytesOf(a + QStringLiteral("/16/46032/28964.png")) + "x"));
        QVERIFY(MapPack::readPack(c).signature != original);
        // Another pack.json.
        QVERIFY(writeFile(d + QStringLiteral("/pack.json"), bytesOf(a + QStringLiteral("/pack.json")) + "\n"));
        QVERIFY(MapPack::readPack(d).signature != original);
    }

    void qgcImportsThePackAsANamedSet_data()
    {
        QTest::addColumn<QString>("name");
        QTest::addColumn<QString>("map");
        QTest::addColumn<QString>("format");
        QTest::addColumn<int>("count");
        QTest::addColumn<int>("minZoom");
        QTest::addColumn<int>("maxZoom");
        QTest::newRow("Esri satellite") << QStringLiteral("esri_satellite") << kSatellite << QStringLiteral("jpg") << 17 << 14 << 16;
        QTest::newRow("Esri streets") << QStringLiteral("esri_streets") << QStringLiteral("Esri World Street") << QStringLiteral("png") << 4 << 15 << 15;
        QTest::newRow("OpenStreetMap") << QStringLiteral("osm_streets") << QStringLiteral("Street Map") << QStringLiteral("png") << 4 << 15 << 15;
        // VGCS's export since task AB: the overview, zoom 10 to 13, before the detail.
        QTest::newRow("Esri satellite with the overview") << QStringLiteral("esri_overview") << kSatellite << QStringLiteral("jpg") << 23 << 10 << 15;
    }

    void qgcImportsThePackAsANamedSet()
    {
        QFETCH(QString, name);
        QFETCH(QString, map);
        QFETCH(QString, format);
        QFETCH(int, count);
        QFETCH(int, minZoom);
        QFETCH(int, maxZoom);

        const MapPack::Pack pack = MapPack::readPack(packFolder(name));
        QCOMPARE(pack.problem, QString());
        QCOMPARE(pack.mapName, map);
        const int mapId = UrlFactory::getQtMapIdFromProviderType(map);
        QVERIFY(mapId > 0);

        const QString file = _temp.filePath(name + QStringLiteral(".db"));
        const QString setName = QStringLiteral("Map pack ") + name;
        const MapPack::Written written = MapPack::writeTileFile(pack, file, setName, mapId, keyFor(map));
        QCOMPARE(written.error, QString());
        QCOMPARE(written.tiles, count);
        QCOMPARE(written.skipped, 0);
        QCOMPARE(written.probes.size(), 3);
        QCOMPARE(written.probes.at(0).path, pack.tiles.constFirst().path);
        QCOMPARE(written.probes.at(1).path, pack.tiles.at(count / 2).path);
        QCOMPARE(written.probes.at(2).path, pack.tiles.constLast().path);
        {
            // The file itself: QGC's tables, one tile set (not a default one), each picture once in it.
            QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), QStringLiteral("look"));
            db.setDatabaseName(file);
            db.setConnectOptions(QStringLiteral("QSQLITE_OPEN_READONLY"));
            QVERIFY(db.open());
            QSqlQuery q(db);
            QVERIFY(q.exec(QStringLiteral("PRAGMA user_version")) && q.next());
            QCOMPARE(q.value(0).toInt(), 1);
            QVERIFY(q.exec(QStringLiteral("SELECT name, typeStr, type, numTiles, defaultSet, minZoom, maxZoom FROM TileSets")) && q.next());
            QCOMPARE(q.value(0).toString(), setName);
            QCOMPARE(q.value(1).toString(), map);
            QCOMPARE(q.value(2).toInt(), mapId);
            QCOMPARE(q.value(3).toInt(), count);
            QCOMPARE(q.value(4).toInt(), 0);
            QCOMPARE(q.value(5).toInt(), minZoom);
            QCOMPARE(q.value(6).toInt(), maxZoom);
            QVERIFY(!q.next());
            QVERIFY(q.exec(QStringLiteral("SELECT COUNT(*), COUNT(DISTINCT tileID) FROM SetTiles")) && q.next());
            QCOMPARE(q.value(0).toInt(), count);
            QCOMPARE(q.value(1).toInt(), count);
            QVERIFY(q.exec(QStringLiteral("SELECT COUNT(*) FROM Tiles WHERE type = %1").arg(mapId)) && q.next());
            QCOMPARE(q.value(0).toInt(), count);
        }
        QSqlDatabase::removeDatabase(QStringLiteral("look"));

        // QGC's own import, as Offline Maps does it (merge, not replace).
        auto store = openStore(_temp.filePath(name + QStringLiteral("-store.db")));
        QVERIFY(store);
        int progress = -1;
        const DatabaseResult result = store->importSetsMerge(file, [&progress](int p) { progress = p; });
        QVERIFY2(result.success, qPrintable(result.errorString));
        QCOMPARE(progress, 100);

        const QList<TileSetRecord> sets = namedSets(*store);
        QCOMPARE(sets.size(), 1);
        const TileSetRecord &set = sets.constFirst();
        QCOMPARE(set.name, setName);
        QCOMPARE(set.mapTypeStr, map);
        QCOMPARE(set.type, mapId);
        QCOMPARE(int(set.numTiles), count);
        QCOMPARE(set.minZoom, minZoom);
        QCOMPARE(set.maxZoom, maxZoom);
        QCOMPARE(set.topleftLat, pack.north);
        QCOMPARE(set.topleftLon, pack.west);
        QCOMPARE(set.bottomRightLat, pack.south);
        QCOMPARE(set.bottomRightLon, pack.east);

        // Each picture is there under the key QGC's map asks for, as it is in the pack.
        for (const MapPack::Tile &tile : pack.tiles) {
            const std::unique_ptr<QGCCacheTile> stored = store->getTile(UrlFactory::getTileHash(map, tile.x, tile.y, tile.z));
            QVERIFY2(stored, qPrintable(tile.path));
            QCOMPARE(stored->img, bytesOf(tile.path));
            QCOMPARE(stored->format, format);
            QCOMPARE(stored->type, map);
        }
        // Not under another map.
        const MapPack::Tile &first = pack.tiles.constFirst();
        QVERIFY(!store->getTile(UrlFactory::getTileHash(QStringLiteral("Google Hybrid"), first.x, first.y, first.z)));
        // Nothing went into the default set, which QGC trims when it grows past its size (1 GB).
        const TotalsResult totals = store->computeTotals();
        QCOMPARE(int(totals.totalCount), count);
        QCOMPARE(int(totals.defaultCount), 0);
        QVERIFY(store->pruneCache(quint64(1) << 40));
        for (const MapPack::Tile &tile : pack.tiles) {
            QVERIFY(store->getTile(UrlFactory::getTileHash(map, tile.x, tile.y, tile.z)));
        }
    }

    void picturesTheStoreHadJoinTheSet()
    {
        // The app showed some of these pictures online before: they are in QGC's default set.
        const MapPack::Pack pack = MapPack::readPack(packFolder(QStringLiteral("esri_satellite")));
        auto store = openStore(_temp.filePath(QStringLiteral("had-store.db")));
        QVERIFY(store);
        for (int i = 0; i < 5; ++i) {
            const MapPack::Tile &tile = pack.tiles.at(i);
            QVERIFY(store->saveTile(UrlFactory::getTileHash(kSatellite, tile.x, tile.y, tile.z), QStringLiteral("jpg"), bytesOf(tile.path),
                                    kSatellite, QGCTileCacheDatabase::kInvalidTileSet));
        }
        QCOMPARE(int(store->computeTotals().defaultCount), 5);

        const QString file = _temp.filePath(QStringLiteral("had.db"));
        QCOMPARE(MapPack::writeTileFile(pack, file, QStringLiteral("Map pack had"), UrlFactory::getQtMapIdFromProviderType(kSatellite),
                                        keyFor(kSatellite)).error, QString());
        QVERIFY(store->importSetsMerge(file, nullptr).success);
        const QList<TileSetRecord> sets = namedSets(*store);
        QCOMPARE(sets.size(), 1);
        QCOMPARE(int(sets.constFirst().numTiles), 17);
        // So trimming the default set keeps them too.
        QVERIFY(store->pruneCache(quint64(1) << 40));
        QCOMPARE(int(store->computeTotals().totalCount), 17);
    }

    void aPackLoadedAgainGetsASecondName()
    {
        // What QGC does when the loader loads a pack again (a changed pack, or a cleared store
        // that was not cleared after all): a second set with a name of its own, no picture twice.
        const MapPack::Pack pack = MapPack::readPack(packFolder(QStringLiteral("esri_satellite")));
        const QString file = _temp.filePath(QStringLiteral("again.db"));
        QCOMPARE(MapPack::writeTileFile(pack, file, QStringLiteral("Map pack again"), UrlFactory::getQtMapIdFromProviderType(kSatellite),
                                        keyFor(kSatellite)).error, QString());
        auto store = openStore(_temp.filePath(QStringLiteral("again-store.db")));
        QVERIFY(store);
        QVERIFY(store->importSetsMerge(file, nullptr).success);
        QVERIFY(store->importSetsMerge(file, nullptr).success);
        const QList<TileSetRecord> sets = namedSets(*store);
        QCOMPARE(sets.size(), 2);
        QCOMPARE(sets.at(0).name, QStringLiteral("Map pack again"));
        QCOMPARE(sets.at(1).name, QStringLiteral("Map pack again 0001"));
        QCOMPARE(int(sets.at(0).numTiles), 17);
        QCOMPARE(int(sets.at(1).numTiles), 17);
        QCOMPARE(int(store->computeTotals().totalCount), 17);
    }

    void writingCanBeStopped()
    {
        const MapPack::Pack pack = MapPack::readPack(packFolder(QStringLiteral("esri_satellite")));
        const QString file = _temp.filePath(QStringLiteral("stopped.db"));
        int asked = 0;
        const MapPack::Written written = MapPack::writeTileFile(pack, file, QStringLiteral("Map pack stopped"),
                                                                UrlFactory::getQtMapIdFromProviderType(kSatellite), keyFor(kSatellite),
                                                                [&asked]() { return ++asked >= 1; });
        QCOMPARE(written.error, QStringLiteral("the app was closed"));
        QCOMPARE(asked, 1);
        QVERIFY(!QFileInfo::exists(file));
    }

    void aPackWithoutGoodPicturesWritesNoFile()
    {
        const QString folder = _temp.filePath(QStringLiteral("empty"));
        QVERIFY(writeFile(folder + QStringLiteral("/pack.json"), bytesOf(packFolder(QStringLiteral("esri_satellite/pack.json")))));
        QVERIFY(writeFile(folder + QStringLiteral("/16/46032/28964.png"), QByteArray()));
        QVERIFY(writeFile(folder + QStringLiteral("/16/46032/28965.png"), "<html>blocked</html>"));
        const MapPack::Pack pack = MapPack::readPack(folder);
        QCOMPARE(pack.problem, QString());
        QCOMPARE(pack.tiles.size(), 2);
        const QString file = _temp.filePath(QStringLiteral("empty.db"));
        const MapPack::Written written = MapPack::writeTileFile(pack, file, QStringLiteral("Map pack empty"),
                                                                UrlFactory::getQtMapIdFromProviderType(kSatellite), keyFor(kSatellite));
        QCOMPARE(written.error, QStringLiteral("none of its map pictures could be read"));
        QCOMPARE(written.skipped, 2);
        QVERIFY(!QFileInfo::exists(file));

        // A pack of a source the app has no map for writes nothing either.
        MapPack::Pack other = MapPack::readPack(packFolder(QStringLiteral("esri_satellite")));
        other.mapName.clear();
        QVERIFY(!MapPack::writeTileFile(other, file, QStringLiteral("Map pack other"), -1, keyFor(kSatellite)).error.isEmpty());
        QVERIFY(!QFileInfo::exists(file));
    }

private:
    QTemporaryDir _temp;
};

QTEST_GUILESS_MAIN(MapPackTest)
#include "map_pack_test.moc"
