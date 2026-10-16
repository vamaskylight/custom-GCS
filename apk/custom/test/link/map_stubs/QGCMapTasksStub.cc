// Stand-in for QGCMapTasks.cpp in the map pack tests: the two destructors QGC defines there
// (moc's code for QGCMapTasks.h needs them). QGC's own file needs QGCCachedTileSet, which
// needs much more of QGC. No test makes a tile set task, so a stand-in tile set does.

class QGCCachedTileSet
{
};

#include "QGCCacheTile.h"
#include "QGCMapTasks.h"

QGCCreateTileSetTask::~QGCCreateTileSetTask()
{
    if (!m_saved) {
        delete m_tileSet;
    }
}

QGCSaveTileTask::~QGCSaveTileTask()
{
    delete m_tile;
}
