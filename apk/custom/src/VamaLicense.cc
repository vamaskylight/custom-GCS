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

// The screen that covers the app when the lock screen cannot be made (row A9, 2026-11-06). Only
// QtQuick, so it can be made where the lock screen could not. It takes no key: it says why and
// shows the device code, for a photo to VAMA. Every tap and key, the RC's back key too, stays here.
const char kPlainLock[] = R"QML(
import QtQuick

Rectangle {
    id: plain
    objectName: "vamaLicensePlainLock"
    required property string deviceCode
    required property string problem
    anchors.fill: parent
    z: 1000000
    color: "#101418"
    focus: true
    Component.onCompleted: forceActiveFocus()
    Keys.onPressed: (event) => { event.accepted = true }
    Keys.onReleased: (event) => { event.accepted = true }

    MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.AllButtons
        hoverEnabled: true
        preventStealing: true
        onWheel: (wheel) => { wheel.accepted = true }
    }

    Text {
        objectName: "vamaLicensePlainLockText"
        anchors.centerIn: parent
        width: parent.width * 0.85
        wrapMode: Text.WordWrap
        horizontalAlignment: Text.AlignHCenter
        color: "#ffffff"
        font.pixelSize: Math.max(16, Math.min(plain.width, plain.height) / 26)
        text: qsTr("This copy of Vama GCS needs a license key for this RC, and its lock screen could not open.")
              + "\n\n" + (plain.deviceCode !== "" ? qsTr("Device code: %1").arg(plain.deviceCode) + "\n\n" : "")
              + qsTr("Please send a photo of this screen to VAMA.") + "\n\n" + plain.problem
    }
}
)QML";

} // namespace

VamaLicense::VamaLicense(QObject *parent)
    : QObject(parent)
    , _publicKey(vama::license::builtInPublicKey())
    , _lockScreenUrl(QString::fromLatin1(kLockScreen))
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
        _lockScreenProblem = QStringLiteral("no window for the lock screen");
        qCWarning(VamaLicenseLog) << _lockScreenProblem;
        return nullptr;
    }
    QQmlComponent component(engine, _lockScreenUrl);
    QObject *lock = component.isReady() ? component.createWithInitialProperties(QVariantMap{
                                              {QStringLiteral("license"), QVariant::fromValue<QObject *>(this)},
                                              {QStringLiteral("parent"), QVariant::fromValue<QQuickItem *>(window->contentItem())},
                                          })
                                        : nullptr;
    if (!lock) {
        _lockScreenProblem = component.errorString().trimmed();
        qCWarning(VamaLicenseLog) << "the lock screen could not be made:" << _lockScreenProblem;
        return nullptr;
    }
    _lockScreenProblem.clear();
    lock->setParent(window);
    (void) QMetaObject::invokeMethod(lock, "open");
    (void) connect(this, &VamaLicense::unlocked, lock, &QObject::deleteLater);
    return lock;
}

QObject *VamaLicense::lockWindow(QQmlEngine *engine, QQuickWindow *window)
{
    if (QObject *lock = showLock(engine, window)) {
        return lock;
    }
    return _showPlainLock(engine, window);
}

QObject *VamaLicense::_showPlainLock(QQmlEngine *engine, QQuickWindow *window)
{
    if (!engine || !window || !window->contentItem()) {
        return nullptr;
    }
    QQmlComponent component(engine);
    component.setData(QByteArray(kPlainLock), QUrl(QStringLiteral("qrc:/Custom/qml/Vama/VamaLicensePlainLock.qml")));
    QObject *plain = component.isReady() ? component.createWithInitialProperties(QVariantMap{
                                               {QStringLiteral("deviceCode"), deviceCode()},
                                               {QStringLiteral("problem"), _lockScreenProblem},
                                               {QStringLiteral("parent"), QVariant::fromValue<QQuickItem *>(window->contentItem())},
                                           })
                                         : nullptr;
    if (!plain) {
        qCWarning(VamaLicenseLog) << "the plain lock screen could not be made either:" << component.errorString();
        return nullptr;
    }
    plain->setParent(window);
    qCWarning(VamaLicenseLog) << "a plain screen covers the app in place of the lock screen";
    return plain;
}

void VamaLicense::holdWhileLocked(std::function<void(bool)> hold)
{
    if (!hold || !_locked) {
        return;
    }
    hold(true);
    (void) connect(this, &VamaLicense::unlocked, this, [hold]() { hold(false); });
}
