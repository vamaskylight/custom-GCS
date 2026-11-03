#include "VamaLicense.h"

#include <QtCore/QLoggingCategory>
#include <QtCore/QSettings>
#include <QtCore/QUrl>
#include <QtCore/QVariantMap>
#include <QtGui/QClipboard>
#include <QtGui/QGuiApplication>
#include <QtQml/QQmlComponent>
#include <QtQml/QQmlEngine>
#include <QtQuick/QQuickItem>
#include <QtQuick/QQuickWindow>

#ifdef Q_OS_ANDROID
#include <QtCore/QJniObject>
#include <QtCore/qcoreapplication_platform.h>
#endif

Q_LOGGING_CATEGORY(VamaLicenseLog, "Vama.License")

namespace {

const char kSettingsGroup[] = "VamaLicense";
const char kLockScreen[] = "qrc:/Custom/qml/Vama/VamaLicenseLock.qml";

} // namespace

VamaLicense::VamaLicense(QObject *parent)
    : QObject(parent)
    , _publicKey(vama::license::builtInPublicKey())
{
}

bool VamaLicense::required()
{
#ifdef Q_OS_ANDROID
    return true;  // test builds and customer builds alike (the developer, 2026-11-04)
#else
    return qEnvironmentVariable("VAMA_REQUIRE_LICENSE") == QLatin1String("1");
#endif
}

void VamaLicense::setAndroidId(const QString &androidId)
{
    _androidId = androidId;
}

void VamaLicense::setClipboard(std::function<QString()> read, std::function<void(const QString &)> write)
{
    _readClipboard = std::move(read);
    _writeClipboard = std::move(write);
}

QString VamaLicense::_readAndroidId()
{
#ifdef Q_OS_ANDROID
    // Settings.Secure.ANDROID_ID ("android_id"). On Android 8 and later there is one for each RC,
    // user and signing key of the app.
    const QJniObject context = QNativeInterface::QAndroidApplication::context();
    if (!context.isValid()) {
        return {};
    }
    const QJniObject resolver = context.callObjectMethod("getContentResolver", "()Landroid/content/ContentResolver;");
    if (!resolver.isValid()) {
        return {};
    }
    const QJniObject name = QJniObject::fromString(QStringLiteral("android_id"));
    const QJniObject id = QJniObject::callStaticObjectMethod(
        "android/provider/Settings$Secure", "getString",
        "(Landroid/content/ContentResolver;Ljava/lang/String;)Ljava/lang/String;", resolver.object<jobject>(),
        name.object<jstring>());
    return id.isValid() ? id.toString() : QString();
#else
    return {};
#endif
}

QDate VamaLicense::_todayOrClock() const
{
    return _today.isValid() ? _today : QDate::currentDate();
}

void VamaLicense::load()
{
    _fingerprint = vama::license::fingerprintFromAndroidId(_androidId ? *_androidId : _readAndroidId());
    QSettings settings;
    settings.beginGroup(QString::fromLatin1(kSettingsGroup));
    const QString saved = settings.value(QStringLiteral("key")).toString();
    const vama::license::Result result = vama::license::checkKey(saved, _fingerprint, _todayOrClock(), _publicKey);
    const bool wasLocked = _locked;
    _locked = !result.ok();
    _info = result.ok() ? result.info : std::nullopt;
    // A key that stopped fitting says why (VGCS's window does the same). No key at all needs no words.
    _savedKeyProblem = result.status == vama::license::Status::Missing ? QString() : vama::license::statusText(result);
    if (_locked) {
        qCInfo(VamaLicenseLog) << "locked, device code" << deviceCode() << "-"
                               << (_savedKeyProblem.isEmpty() ? QStringLiteral("no key saved") : _savedKeyProblem);
    } else {
        qCInfo(VamaLicenseLog) << "license OK:" << keySummary();
    }
    if (wasLocked != _locked) {
        emit lockedChanged();
    }
}

QString VamaLicense::deviceCode() const
{
    return vama::license::deviceCode(_fingerprint);
}

QString VamaLicense::keySummary() const
{
    return _info ? vama::license::describe(*_info) : QString();
}

QString VamaLicense::activate(const QString &key)
{
    const vama::license::Result result = vama::license::checkKey(key, _fingerprint, _todayOrClock(), _publicKey);
    if (!result.ok()) {
        qCInfo(VamaLicenseLog) << "key not accepted:" << vama::license::statusText(result);
        return vama::license::statusText(result);
    }
    QSettings settings;
    settings.beginGroup(QString::fromLatin1(kSettingsGroup));
    settings.setValue(QStringLiteral("key"), key.trimmed());
    settings.endGroup();
    settings.sync();
    _info = result.info;
    qCInfo(VamaLicenseLog) << "license activated:" << keySummary();
    if (_locked) {
        _locked = false;
        emit lockedChanged();
        emit unlocked();
    }
    return {};
}

void VamaLicense::copyDeviceCode() const
{
    if (_writeClipboard) {
        _writeClipboard(deviceCode());
    } else if (QClipboard *clipboard = QGuiApplication::clipboard()) {
        clipboard->setText(deviceCode());
    }
}

QString VamaLicense::clipboardText() const
{
    if (_readClipboard) {
        return _readClipboard();
    }
    QClipboard *clipboard = QGuiApplication::clipboard();
    return clipboard ? clipboard->text() : QString();
}

QObject *VamaLicense::showLock(QQmlEngine *engine, QQuickWindow *window)
{
    if (!engine || !window || !window->contentItem()) {
        qCWarning(VamaLicenseLog) << "no window for the lock screen";
        return nullptr;
    }
    QQmlComponent component(engine, QUrl(QString::fromLatin1(kLockScreen)));
    QObject *lock = component.createWithInitialProperties(QVariantMap{
        {QStringLiteral("license"), QVariant::fromValue<QObject *>(this)},
        {QStringLiteral("parent"), QVariant::fromValue<QQuickItem *>(window->contentItem())},
    });
    if (!lock) {
        qCWarning(VamaLicenseLog) << "the lock screen could not be made:" << component.errorString();
        return nullptr;
    }
    lock->setParent(window);
    (void) QMetaObject::invokeMethod(lock, "open");
    (void) connect(this, &VamaLicense::unlocked, lock, &QObject::deleteLater);
    return lock;
}
