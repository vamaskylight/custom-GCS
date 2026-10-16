#pragma once

// Stand-in for QGC's map engine in map_pack_loader_test. It runs each task as QGC's map
// worker does (QGCCacheWorker), on a later turn of the event loop, against a real QGC tile
// store (QGCTileCacheDatabase). It can also act as a worker that has not started yet (no
// map shown so far), or as one whose store does not work.

#include <QtCore/QObject>

class QGCMapTask;
class QGCTileCacheDatabase;

class QGCMapEngine : public QObject
{
public:
    // FailsImports: works, but each import fails as QGC's does on a file it cannot read.
    enum class State { NotStarted, Broken, Working, FailsImports };

    bool addTask(QGCMapTask *task);
    static QGCMapEngine *instance();

    // Set by the test.
    State state = State::Working;
    int notStartedFor = 0;  // this many tasks first find no worker
    QGCTileCacheDatabase *store = nullptr;
    // Counted for the test.
    int fetches = 0;
    int imports = 0;

private:
    void _run(QGCMapTask *task);
};

QGCMapEngine *getQGCMapEngine();
