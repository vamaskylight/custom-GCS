#include "MapPack.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QCryptographicHash>
#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <tuple>
#include <utility>

namespace MapPack {

namespace {

constexpr const char *kManifest = "pack.json";
constexpr const char *kFormat = "vgcs-tile-pack/1";
// VGCS writes a pack.json of a few hundred bytes. A much bigger file is not one.
constexpr qint64 kManifestMaxBytes = 64 * 1024;
// A map picture is some kilobytes. A much bigger file is something else.
constexpr qint64 kPictureMaxBytes = 4 * 1024 * 1024;
// QGC shows zoom 1 to 23 (QGC_MAX_MAP_ZOOM).
constexpr int kMaxZoom = 23;
// Folder levels searched below a root (a pack copied inside a folder of its own is still found).
constexpr int kSearchDepth = 2;
// The writing asks whether to stop after this many pictures.
constexpr int kCancelEvery = 64;
constexpr double kPi = 3.14159265358979323846;

std::atomic<int> s_connections{0};

QString text(const char *words)
{
    return QCoreApplication::translate("MapPack", words);
}

// A name of digits only, as VGCS gives zoom, x and y. Short enough for an int.
bool isNumber(const QString &name)
{
    if (name.isEmpty() || name.size() > 9) {
        return false;
    }
    return std::all_of(name.cbegin(), name.cend(), [](QChar c) { return c >= u'0' && c <= u'9'; });
}

bool hasZoomFolder(const QDir &dir)
{
    const QStringList subs = dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
    return std::any_of(subs.cbegin(), subs.cend(), isNumber);
}

// Every <zoom>/<x>/<y>.png, as vgcs/map/tile_pack.py iter_pack_tiles finds them. Anything else is left alone.
void listTiles(const QDir &dir, Pack &pack)
{
    const QFileInfoList zooms = dir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
    for (const QFileInfo &zoomInfo : zooms) {
        if (!isNumber(zoomInfo.fileName())) {
            continue;
        }
        const int z = zoomInfo.fileName().toInt();
        if (z < 1 || z > kMaxZoom) {
            continue;
        }
        const qint64 side = qint64(1) << z;
        const QFileInfoList xs = QDir(zoomInfo.absoluteFilePath()).entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
        for (const QFileInfo &xInfo : xs) {
            if (!isNumber(xInfo.fileName())) {
                continue;
            }
            const int x = xInfo.fileName().toInt();
            if (x >= side) {
                continue;
            }
            const QFileInfoList files = QDir(xInfo.absoluteFilePath()).entryInfoList(QDir::Files | QDir::NoSymLinks);
            for (const QFileInfo &file : files) {
                if (file.suffix().compare(QStringLiteral("png"), Qt::CaseInsensitive) != 0 || !isNumber(file.completeBaseName())) {
                    continue;
                }
                const int y = file.completeBaseName().toInt();
                if (y >= side) {
                    continue;
                }
                pack.tiles.append(Tile{z, x, y, file.absoluteFilePath(), file.size()});
                pack.bytes += file.size();
            }
        }
    }
    std::sort(pack.tiles.begin(), pack.tiles.end(), [](const Tile &a, const Tile &b) {
        return std::tie(a.z, a.x, a.y) < std::tie(b.z, b.x, b.y);
    });
    if (!pack.tiles.isEmpty()) {
        pack.minZoom = pack.tiles.constFirst().z;
        pack.maxZoom = pack.tiles.constLast().z;
    }
}

double tileLon(qint64 x, int z)
{
    return double(x) / double(qint64(1) << z) * 360.0 - 180.0;
}

double tileLat(qint64 y, int z)
{
    const double n = kPi * (1.0 - 2.0 * double(y) / double(qint64(1) << z));
    return std::atan(std::sinh(n)) * 180.0 / kPi;
}

// The area pack.json gives: [south, west, north, east] (VGCS's plan.bbox). False when it gives none that is sound.
bool areaFromManifest(const QJsonObject &manifest, Pack &pack)
{
    const QJsonArray box = manifest.value(QStringLiteral("bbox")).toArray();
    if (box.size() != 4) {
        return false;
    }
    double v[4];
    for (int i = 0; i < 4; ++i) {
        if (!box.at(i).isDouble() || !std::isfinite(box.at(i).toDouble())) {
            return false;
        }
        v[i] = box.at(i).toDouble();
    }
    const double south = std::min(v[0], v[2]);
    const double north = std::max(v[0], v[2]);
    const double west = std::min(v[1], v[3]);
    const double east = std::max(v[1], v[3]);
    if (south < -90.0 || north > 90.0 || west < -180.0 || east > 180.0 || south == north || west == east) {
        return false;
    }
    pack.south = south;
    pack.north = north;
    pack.west = west;
    pack.east = east;
    return true;
}

// The area the pictures of the lowest zoom cover.
void areaFromTiles(Pack &pack)
{
    qint64 minX = 0, maxX = 0, minY = 0, maxY = 0;
    bool first = true;
    for (const Tile &tile : std::as_const(pack.tiles)) {
        if (tile.z != pack.minZoom) {
            break;
        }
        if (first) {
            minX = maxX = tile.x;
            minY = maxY = tile.y;
            first = false;
        }
        minX = std::min<qint64>(minX, tile.x);
        maxX = std::max<qint64>(maxX, tile.x);
        minY = std::min<qint64>(minY, tile.y);
        maxY = std::max<qint64>(maxY, tile.y);
    }
    if (first) {
        return;
    }
    pack.west = tileLon(minX, pack.minZoom);
    pack.east = tileLon(maxX + 1, pack.minZoom);
    pack.north = tileLat(minY, pack.minZoom);
    pack.south = tileLat(maxY + 1, pack.minZoom);
}

QString signatureOf(const QByteArray &manifest, const Pack &pack)
{
    QCryptographicHash hash(QCryptographicHash::Sha1);
    hash.addData(manifest);
    for (const Tile &tile : pack.tiles) {
        hash.addData(QStringLiteral("\n%1/%2/%3 %4").arg(tile.z).arg(tile.x).arg(tile.y).arg(tile.bytes).toLatin1());
    }
    return QString::fromLatin1(hash.result().toHex());
}

void search(const QDir &dir, int depth, QList<Pack> &packs)
{
    if (QFileInfo::exists(dir.filePath(QLatin1String(kManifest))) || hasZoomFolder(dir)) {
        packs.append(readPack(dir.absolutePath()));
        return;
    }
    if (depth >= kSearchDepth) {
        return;
    }
    const QFileInfoList subs = dir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot | QDir::NoSymLinks, QDir::Name);
    for (const QFileInfo &sub : subs) {
        search(QDir(sub.absoluteFilePath()), depth + 1, packs);
    }
}

// The tables and indexes of QGC's tile file (QGCTileCacheDatabase::_createDB, schema 1).
const char *const kSchema[] = {
    "CREATE TABLE IF NOT EXISTS Tiles ("
    "tileID INTEGER PRIMARY KEY NOT NULL, "
    "hash TEXT NOT NULL UNIQUE, "
    "format TEXT NOT NULL, "
    "tile BLOB NULL, "
    "size INTEGER, "
    "type INTEGER, "
    "date INTEGER DEFAULT 0)",
    "CREATE TABLE IF NOT EXISTS TileSets ("
    "setID INTEGER PRIMARY KEY NOT NULL, "
    "name TEXT NOT NULL UNIQUE, "
    "typeStr TEXT, "
    "topleftLat REAL DEFAULT 0.0, "
    "topleftLon REAL DEFAULT 0.0, "
    "bottomRightLat REAL DEFAULT 0.0, "
    "bottomRightLon REAL DEFAULT 0.0, "
    "minZoom INTEGER DEFAULT 3, "
    "maxZoom INTEGER DEFAULT 3, "
    "type INTEGER DEFAULT -1, "
    "numTiles INTEGER DEFAULT 0, "
    "defaultSet INTEGER DEFAULT 0, "
    "date INTEGER DEFAULT 0)",
    "CREATE TABLE IF NOT EXISTS SetTiles ("
    "setID INTEGER NOT NULL REFERENCES TileSets(setID) ON DELETE CASCADE, "
    "tileID INTEGER NOT NULL REFERENCES Tiles(tileID) ON DELETE CASCADE)",
    "CREATE TABLE IF NOT EXISTS TilesDownload ("
    "setID INTEGER NOT NULL REFERENCES TileSets(setID) ON DELETE CASCADE, "
    "hash TEXT NOT NULL, "
    "type INTEGER, "
    "x INTEGER, "
    "y INTEGER, "
    "z INTEGER, "
    "state INTEGER DEFAULT 0)",
    "CREATE UNIQUE INDEX IF NOT EXISTS idx_settiles_unique ON SetTiles(tileID, setID)",
    "CREATE INDEX IF NOT EXISTS idx_settiles_setid ON SetTiles(setID)",
    "CREATE INDEX IF NOT EXISTS idx_settiles_tileid ON SetTiles(tileID)",
    "CREATE UNIQUE INDEX IF NOT EXISTS idx_tilesdownload_setid_hash ON TilesDownload(setID, hash)",
    "CREATE INDEX IF NOT EXISTS idx_tilesdownload_setid_state ON TilesDownload(setID, state)",
    "CREATE INDEX IF NOT EXISTS idx_tiles_date ON Tiles(date)",
    "PRAGMA user_version = 1",
};

Written writeInto(QSqlDatabase &db, const Pack &pack, const QString &setName, int mapId,
                  const TileKey &tileKey, const std::function<bool()> &cancel)
{
    Written result;
    QSqlQuery query(db);
    // The file is a go-between that is thrown away, so it needs no safety against a power cut.
    (void) query.exec(QStringLiteral("PRAGMA synchronous=OFF"));
    (void) query.exec(QStringLiteral("PRAGMA journal_mode=MEMORY"));
    for (const char *statement : kSchema) {
        if (!query.exec(QLatin1String(statement))) {
            result.error = text("the map file could not be made");
            return result;
        }
    }
    if (!db.transaction()) {
        result.error = text("the map file could not be made");
        return result;
    }

    const qint64 now = QDateTime::currentSecsSinceEpoch();
    if (!query.prepare(QStringLiteral(
            "INSERT INTO TileSets(name, typeStr, topleftLat, topleftLon, bottomRightLat, bottomRightLon, "
            "minZoom, maxZoom, type, numTiles, defaultSet, date) VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, 0, 0, ?)"))) {
        result.error = text("the map file could not be made");
        return result;
    }
    query.addBindValue(setName);
    query.addBindValue(pack.mapName);
    query.addBindValue(pack.north);
    query.addBindValue(pack.west);
    query.addBindValue(pack.south);
    query.addBindValue(pack.east);
    query.addBindValue(pack.minZoom);
    query.addBindValue(pack.maxZoom);
    query.addBindValue(mapId);
    query.addBindValue(now);
    if (!query.exec()) {
        result.error = text("the map file could not be made");
        return result;
    }
    const qint64 setId = query.lastInsertId().toLongLong();

    QSqlQuery addTile(db);
    QSqlQuery addLink(db);
    if (!addTile.prepare(QStringLiteral("INSERT INTO Tiles(hash, format, tile, size, type, date) VALUES(?, ?, ?, ?, ?, ?)"))
        || !addLink.prepare(QStringLiteral("INSERT INTO SetTiles(setID, tileID) VALUES(?, ?)"))) {
        result.error = text("the map file could not be made");
        return result;
    }

    QList<qsizetype> written;
    for (qsizetype i = 0; i < pack.tiles.size(); ++i) {
        if (cancel && (i % kCancelEvery) == 0 && cancel()) {
            result.error = text("the app was closed");
            return result;
        }
        const Tile &tile = pack.tiles.at(i);
        QFile file(tile.path);
        if (!file.open(QIODevice::ReadOnly)) {
            ++result.skipped;
            continue;
        }
        const QByteArray picture = file.read(kPictureMaxBytes + 1);
        const QString format = pictureFormat(picture);
        if (picture.size() > kPictureMaxBytes || format.isEmpty()) {
            ++result.skipped;
            continue;
        }
        addTile.addBindValue(tileKey(tile.x, tile.y, tile.z));
        addTile.addBindValue(format);
        addTile.addBindValue(picture);
        addTile.addBindValue(picture.size());
        addTile.addBindValue(mapId);
        addTile.addBindValue(now);
        if (!addTile.exec()) {
            ++result.skipped;
            continue;
        }
        addLink.addBindValue(setId);
        addLink.addBindValue(addTile.lastInsertId());
        if (!addLink.exec()) {
            result.error = text("the map file could not be made");
            return result;
        }
        written.append(i);
    }
    result.tiles = int(written.size());
    if (written.isEmpty()) {
        result.error = text("none of its map pictures could be read");
        return result;
    }

    if (!query.prepare(QStringLiteral("UPDATE TileSets SET numTiles = ? WHERE setID = ?"))) {
        result.error = text("the map file could not be made");
        return result;
    }
    query.addBindValue(result.tiles);
    query.addBindValue(setId);
    if (!query.exec() || !db.commit()) {
        result.error = text("the map file could not be made");
        return result;
    }
    qsizetype last = -1;
    for (const qsizetype at : {qsizetype(0), written.size() / 2, written.size() - 1}) {
        if (at != last) {
            result.probes.append(pack.tiles.at(written.at(at)));
            last = at;
        }
    }
    return result;
}

}  // namespace

QString mapNameForSource(const QString &source)
{
    // The three sources VGCS can make a pack from (vgcs/map/surface/map_tiles_mixin.py), and the QGC map
    // that shows the same pictures. The pack keeps x and y as the map does, so the pictures fit as they are.
    const QString s = source.trimmed().toLower();
    if (s.contains(QLatin1String("arcgisonline.com/arcgis/rest/services/world_imagery/mapserver/tile/"))) {
        return QStringLiteral("Esri World Satellite");
    }
    if (s.contains(QLatin1String("arcgisonline.com/arcgis/rest/services/world_street_map/mapserver/tile/"))) {
        return QStringLiteral("Esri World Street");
    }
    if (s.contains(QLatin1String("tile.openstreetmap.org/{z}/{x}/{y}.png"))) {
        return QStringLiteral("Street Map");
    }
    return QString();
}

QString pictureFormat(const QByteArray &picture)
{
    if (picture.startsWith(QByteArrayLiteral("\x89\x50\x4E\x47\x0D\x0A\x1A\x0A"))) {
        return QStringLiteral("png");
    }
    if (picture.startsWith(QByteArrayLiteral("\xFF\xD8\xFF"))) {
        return QStringLiteral("jpg");
    }
    if (picture.startsWith(QByteArrayLiteral("GIF8"))) {
        return QStringLiteral("gif");
    }
    return QString();
}

Pack readPack(const QString &folder)
{
    Pack pack;
    const QDir dir(folder);
    pack.folder = dir.absolutePath();
    pack.name = dir.dirName();

    QByteArray manifestBytes;
    QJsonObject manifest;
    QFile manifestFile(dir.filePath(QLatin1String(kManifest)));
    if (!manifestFile.exists()) {
        pack.problem = text("it has no pack.json");
    } else if (manifestFile.size() > kManifestMaxBytes || !manifestFile.open(QIODevice::ReadOnly)) {
        pack.problem = text("its pack.json cannot be read");
    } else {
        manifestBytes = manifestFile.readAll();
        const QJsonDocument doc = QJsonDocument::fromJson(manifestBytes);
        if (!doc.isObject()) {
            pack.problem = text("its pack.json cannot be read");
        } else {
            manifest = doc.object();
        }
    }

    listTiles(dir, pack);
    pack.signature = signatureOf(manifestBytes, pack);
    if (!pack.problem.isEmpty()) {
        return pack;
    }

    const QJsonValue format = manifest.value(QStringLiteral("format"));
    pack.source = manifest.value(QStringLiteral("template")).toString().trimmed();
    pack.mapName = mapNameForSource(pack.source);
    if (!format.isUndefined() && format.toString() != QLatin1String(kFormat)) {
        pack.problem = text("it is a pack of another kind (%1)").arg(format.toVariant().toString());
    } else if (pack.source.isEmpty()) {
        pack.problem = text("its pack.json names no map source");
    } else if (pack.mapName.isEmpty()) {
        pack.problem = text("the app has no map for its source (%1)").arg(pack.source);
    } else if (pack.tiles.isEmpty()) {
        pack.problem = text("it has no map pictures");
    }
    if (!areaFromManifest(manifest, pack)) {
        areaFromTiles(pack);
    }
    return pack;
}

QList<Pack> findPacks(const QString &root)
{
    QList<Pack> packs;
    const QDir dir(root);
    if (dir.exists()) {
        search(dir, 0, packs);
    }
    return packs;
}

Written writeTileFile(const Pack &pack, const QString &path, const QString &setName, int mapId,
                      const TileKey &tileKey, const std::function<bool()> &cancel)
{
    Written result;
    if (pack.mapName.isEmpty() || mapId < 0 || !tileKey) {
        result.error = text("the app has no map for its source (%1)").arg(pack.source);
        return result;
    }
    (void) QFile::remove(path);
    const QString connection = QStringLiteral("VamaMapPack_%1").arg(s_connections.fetch_add(1));
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), connection);
        db.setDatabaseName(path);
        if (!db.open()) {
            result.error = text("the map file could not be made");
        } else {
            result = writeInto(db, pack, setName, mapId, tileKey, cancel);
            if (!result.error.isEmpty()) {
                db.rollback();
            }
            db.close();
        }
    }
    QSqlDatabase::removeDatabase(connection);
    if (!result.error.isEmpty()) {
        (void) QFile::remove(path);
    }
    return result;
}

}  // namespace MapPack
