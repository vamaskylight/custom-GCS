// Stand-in for the bodies of QGC's UrlFactory in map_pack_test, with what QGC's tile
// store code (QGCTileCacheDatabase.cpp) and the test use. The header is QGC's own.
// QGC numbers its maps as the app starts (MapProvider::_mapIdIndex), so here a few maps
// have numbers of their own. The key of a tile is made as QGC makes it (getTileHash).

#include "QGCMapUrlEngine.h"

#include <QtCore/QPair>

namespace {

// A few of QGC's maps, by their names in QGC 5.1.5 (QGCMapUrlEngine.cpp, _providers).
const QList<QPair<QString, int>> kMaps = {
    {QStringLiteral("Google Hybrid"), 4},
    {QStringLiteral("Esri World Street"), 15},
    {QStringLiteral("Esri World Satellite"), 16},
    {QStringLiteral("Street Map"), 44},
};

}  // namespace

quint32 UrlFactory::averageSizeForType(QStringView)
{
    return 13652;  // QGC_AVERAGE_TILE_SIZE
}

QGCTileSet UrlFactory::getTileCount(int, double, double, double, double, QStringView)
{
    return QGCTileSet();
}

int UrlFactory::getQtMapIdFromProviderType(QStringView type)
{
    for (const auto &map : kMaps) {
        if (map.first == type) {
            return map.second;
        }
    }
    return -1;
}

QString UrlFactory::getProviderTypeFromQtMapId(int qtMapId)
{
    for (const auto &map : kMaps) {
        if (map.second == qtMapId) {
            return map.first;
        }
    }
    return QString();
}

int UrlFactory::hashFromProviderType(QStringView type)
{
    return getQtMapIdFromProviderType(type);
}

QString UrlFactory::getTileHash(QStringView type, int x, int y, int z)
{
    return QString::asprintf("%010d%08d%08d%03d", hashFromProviderType(type), x, y, z);
}
