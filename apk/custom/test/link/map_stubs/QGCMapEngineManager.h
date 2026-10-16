#pragma once

// Stand-in for QGC's map engine manager in map_pack_loader_test: it counts the asks to
// list the tile sets again (Offline Maps).

class QGCMapEngineManager
{
public:
    static QGCMapEngineManager *instance()
    {
        static QGCMapEngineManager manager;
        return &manager;
    }

    void loadTileSets() { ++loads; }

    int loads = 0;
};
