#include "QGCMapEngine.h"

#include <QtCore/QTimer>

#include "QGCCacheTile.h"
#include "QGCMapTasks.h"
#include "QGCTileCacheDatabase.h"

QGCMapEngine *QGCMapEngine::instance()
{
    static QGCMapEngine engine;
    return &engine;
}

QGCMapEngine *getQGCMapEngine()
{
    return QGCMapEngine::instance();
}

bool QGCMapEngine::addTask(QGCMapTask *task)
{
    if (notStartedFor > 0) {
        // QGCMapEngine::addTask before the first map: no worker yet.
        --notStartedFor;
        return false;
    }
    switch (state) {
    case State::NotStarted:
        return false;
    case State::Broken:
        // QGCCacheWorker::enqueueTask when its store could not be opened: the error at once, then false.
        task->setError(QStringLiteral("Database Not Initialized"));
        task->deleteLater();
        return false;
    case State::Working:
    case State::FailsImports:
        break;
    }
    QTimer::singleShot(0, this, [this, task]() { _run(task); });
    return true;
}

// As QGCCacheWorker::_runTask, _getTile and _importSets.
void QGCMapEngine::_run(QGCMapTask *task)
{
    switch (task->type()) {
    case QGCMapTask::TaskType::taskFetchTile: {
        ++fetches;
        auto *fetch = static_cast<QGCFetchTileTask *>(task);
        std::unique_ptr<QGCCacheTile> tile = store->getTile(fetch->hash());
        if (tile) {
            fetch->setTileFetched(tile.release());
        } else {
            fetch->setError(QStringLiteral("Tile not in cache database"));
        }
        break;
    }
    case QGCMapTask::TaskType::taskImport: {
        ++imports;
        auto *import = static_cast<QGCImportTileTask *>(task);
        const auto progress = [import](int percentage) { import->setProgress(percentage); };
        DatabaseResult result;
        if (state == State::FailsImports) {
            result.errorString = QStringLiteral("Error opening import database");
        } else {
            result = import->replace() ? store->importSetsReplace(import->path(), progress)
                                       : store->importSetsMerge(import->path(), progress);
        }
        if (result.success) {
            import->setImportCompleted();
        } else {
            import->setError(result.errorString);
        }
        break;
    }
    default:
        task->setError(QStringLiteral("not done by this stand-in"));
        break;
    }
    task->deleteLater();
}
