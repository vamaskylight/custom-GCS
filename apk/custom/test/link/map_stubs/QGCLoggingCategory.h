#pragma once

// Stand-in for QGC's logging categories in map_pack_test: plain Qt categories.

#include <QtCore/QLoggingCategory>

#define QGC_LOGGING_CATEGORY(name, categoryStr) Q_LOGGING_CATEGORY(name, categoryStr, QtWarningMsg)
#define QGC_LOGGING_CATEGORY_ON(name, categoryStr) Q_LOGGING_CATEGORY(name, categoryStr, QtInfoMsg)
