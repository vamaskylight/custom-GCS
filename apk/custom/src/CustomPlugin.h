#pragma once

#include <QtCore/QElapsedTimer>
#include <QtCore/QTranslator>
#include <QtQml/QQmlAbstractUrlInterceptor>

#include "AdminPassword.h"
#include "QGCCorePlugin.h"
#include "QGCOptions.h"

class CustomOptions;
class CustomPlugin;
class CustomSettings;
class MapPackLoader;
class QQmlApplicationEngine;

Q_DECLARE_LOGGING_CATEGORY(CustomLog)

class CustomFlyViewOptions : public QGCFlyViewOptions
{
    Q_OBJECT

public:
    explicit CustomFlyViewOptions(CustomOptions *options, QObject *parent = nullptr);

    // Overrides from QGCFlyViewOptions

    /// This app flies one vehicle at a time.
    /// This in turn simplifies various parts of the QGC ui.
    bool showMultiVehicleList() const final { return false; }
};

/*===========================================================================*/

class CustomOptions : public QGCOptions
{
    Q_OBJECT

public:
    explicit CustomOptions(CustomPlugin *plugin, QObject *parent = nullptr);

    // Overrides from QGCOptions

    /// Firmware upgrade page is only shown in Advanced Mode.
    bool showFirmwareUpgrade() const final { return _plugin->showAdvancedUI(); }
    QGCFlyViewOptions *flyViewOptions() const final { return _flyViewOptions; }

private:
    QGCCorePlugin *_plugin = nullptr;
    CustomFlyViewOptions *_flyViewOptions = nullptr;
};

/*===========================================================================*/

class CustomPlugin : public QGCCorePlugin
{
    Q_OBJECT

public:
    explicit CustomPlugin(QObject *parent = nullptr);

    static QGCCorePlugin *instance();

    // Overrides from QGCCorePlugin

    QGCOptions *options() final { return _options; }
    /// Runs once the settings exist: shows QGC's 3D View settings page again on Android,
    /// and loads the map packs from VGCS (MapPackLoader.h)
    void init() final;
    /// This allows you to override/hide QGC Application settings
    void adjustSettingMetaData(const QString &settingsGroup, FactMetaData &metaData, bool &userVisible) final;
    /// This modifies QGC colors palette to match possible custom corporate branding
    void paletteOverride(const QString &colorName, QGCPalette::PaletteColorInfo_t &colorInfo) final;
    /// We override this so we can get access to QQmlApplicationEngine and use it to register our qml module
    QQmlApplicationEngine *createQmlApplicationEngine(QObject *parent) final;
    /// Releases the url interceptor attached in createQmlApplicationEngine before the engine is destroyed
    void destroyQmlApplicationEngine(QQmlApplicationEngine *qmlEngine) final;

    /// Advanced Mode (with Parameters) needs the admin password (task Z, AdminPassword.h).
    /// True: the password was right, and Advanced Mode is on.
    Q_INVOKABLE bool unlockAdvanced(const QString &password);
    /// Seconds until the next password may be tried (after a wrong one). 0: now.
    Q_INVOKABLE int adminWaitSeconds() const;

private slots:
    void _advancedChanged(bool advanced);

private:
    void _addSettingsEntry(const QString& title, const char* qmlFile, const char* iconFile = nullptr);

    CustomOptions *_options = nullptr;
    QQmlApplicationEngine *_qmlEngine = nullptr;
    MapPackLoader *_mapPacks = nullptr;
    vama::AdminPassword _adminPassword{vama::builtInAdminSecret()};
    QElapsedTimer _adminClock;
    bool _adminUnlocked = false;  // Advanced Mode was turned on with the password
    class CustomOverrideInterceptor *_urlInterceptor = nullptr;
    QVariantList _customSettingsList; // Not to be mixed up with QGCCorePlugin implementation
};

/*===========================================================================*/

class CustomOverrideInterceptor : public QQmlAbstractUrlInterceptor
{
public:
    CustomOverrideInterceptor();

    QUrl intercept(const QUrl &url, QQmlAbstractUrlInterceptor::DataType type) final;
};
