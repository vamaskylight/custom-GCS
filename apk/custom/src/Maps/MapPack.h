#pragma once

// Map packs made by VGCS, loaded by the app by itself (task U, client 2026-10-15:
// "end user won't know how to import offline map", and the remote controls are
// never online).
//
// A pack is a folder from VGCS ("Export tile pack...", vgcs/map/tile_pack.py):
// pack.json, which names the tile source, and the map pictures as
// <zoom>/<x>/<y>.png. Esri's pictures are JPEG inside, despite the name.
//
// This part reads a pack and writes it as a QGC tile file: QGC's own tables
// with one named tile set, the kind of file that Offline Maps imports. It has
// no QGC code in it. QGC's number for a map and its key for a tile are made
// when the app starts, so the caller passes them in (MapPackLoader.h).

#include <QtCore/QList>
#include <QtCore/QString>

#include <functional>

namespace MapPack {

/// One map picture of a pack.
struct Tile
{
    int z = 0;
    int x = 0;
    int y = 0;
    QString path;
    qint64 bytes = 0;
};

/// What a pack folder holds.
struct Pack
{
    QString folder;     ///< the pack's folder
    QString name;       ///< the folder's name
    QString source;     ///< the tile address that pack.json names
    QString mapName;    ///< QGC's map for that source. Empty when the app has none.
    QList<Tile> tiles;  ///< every <zoom>/<x>/<y>.png, by zoom, then x, then y
    qint64 bytes = 0;   ///< all pictures together
    int minZoom = 0;
    int maxZoom = 0;
    // The area: the one pack.json gives, else the one the pictures cover.
    double north = 0.0;
    double west = 0.0;
    double south = 0.0;
    double east = 0.0;
    QString signature;  ///< changes when pack.json or any picture's name or size changes
    QString problem;    ///< why it cannot be loaded. Empty when it can.
};

/// The QGC map that shows the pictures of a VGCS tile address. Empty when there is none.
QString mapNameForSource(const QString &source);

/// "png", "jpg" or "gif" by a picture's first bytes, as QGC's MapProvider::getImageFormat
/// tells them. Empty for anything else.
QString pictureFormat(const QByteArray &picture);

/// The pack in `folder`. A folder without pack.json is no pack: its problem says so.
Pack readPack(const QString &folder);

/// The packs in `root` and in the two folder levels below it. A folder of map pictures
/// without pack.json comes back too, with its problem. Zoom folders are not searched.
QList<Pack> findPacks(const QString &root);

/// QGC's key of the map picture at x, y and zoom z, in the pack's map.
using TileKey = std::function<QString(int x, int y, int z)>;

struct Written
{
    int tiles = 0;      ///< pictures in the file
    int skipped = 0;    ///< files that are no picture: empty, too big, other bytes, or unreadable
    QList<Tile> probes; ///< some pictures that are in the file: the first, one in the middle, the last
    QString error;      ///< empty when the file was written
};

/// Writes `pack` as a new QGC tile file at `path`, with one tile set named `setName`.
/// `mapId` is QGC's number for pack.mapName and `tileKey` gives QGC's key of each picture.
/// `cancel`, when given, is asked now and then, and true stops the writing (an error then).
Written writeTileFile(const Pack &pack, const QString &path, const QString &setName, int mapId,
                      const TileKey &tileKey, const std::function<bool()> &cancel = {});

}  // namespace MapPack
