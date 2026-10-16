#pragma once

// Stand-in for QGC's application in map_pack_loader_test: it keeps the messages shown.

#include <QtCore/QList>
#include <QtCore/QPair>
#include <QtCore/QString>

class QGCApplication
{
public:
    void showAppMessage(const QString &message, const QString &title = QString()) { messages.append({title, message}); }

    QList<QPair<QString, QString>> messages;  // title, text
};

inline QGCApplication *qgcApp()
{
    static QGCApplication app;
    return &app;
}
