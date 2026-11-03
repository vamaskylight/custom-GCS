#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QDate>
#include <QtCore/QObject>
#include <QtCore/QString>

#include <functional>
#include <optional>

#include "LicenseKey.h"

class QQmlEngine;
class QQuickWindow;

/// VAMA (row A9, the client 2026-11-03): the app opens only on an RC with a license key made for
/// it (LicenseKey.h). At the start it reads the RC's ID and the saved key. Without a key that fits,
/// it is locked: CustomPlugin holds the drone links back, and showLock covers the whole window with
/// the lock screen (VamaLicenseLock.qml), which shows the device code and takes the key VAMA sends.
/// A key that fits is saved, and the app is unlocked for this and every later start.
class VamaLicense : public QObject
{
    Q_OBJECT
    /// True while the app has no key for this RC.
    Q_PROPERTY(bool locked READ locked NOTIFY lockedChanged)
    /// This RC's device code, for the user to send to VAMA. Empty when its ID cannot be read.
    Q_PROPERTY(QString deviceCode READ deviceCode CONSTANT)
    /// False when the RC's ID cannot be read: then no key can open the app here.
    Q_PROPERTY(bool canActivate READ canActivate CONSTANT)
    /// Why the saved key no longer opens the app (for another RC, or past its end date), or empty.
    Q_PROPERTY(QString savedKeyProblem READ savedKeyProblem CONSTANT)

public:
    explicit VamaLicense(QObject *parent = nullptr);

    /// True on Android, the only place the app runs. Elsewhere only with VAMA_REQUIRE_LICENSE=1.
    static bool required();

    /// Reads this RC's ID and the saved key (once, at the start).
    void load();

    bool locked() const { return _locked; }
    QString deviceCode() const;
    bool canActivate() const { return !_fingerprint.isEmpty(); }
    QString savedKeyProblem() const { return _savedKeyProblem; }

    /// The key the user pasted. Empty when it opens the app (then it is saved, and the app unlocked),
    /// otherwise why it does not.
    Q_INVOKABLE QString activate(const QString &key);
    /// The device code to the clipboard, for a message to VAMA.
    Q_INVOKABLE void copyDeviceCode() const;
    /// The clipboard's text, for the Paste button.
    Q_INVOKABLE QString clipboardText() const;

    /// Covers the window with the lock screen until a key opens the app. Null when it could not be
    /// made (a fault in the app; the drone links stay held back all the same).
    QObject *showLock(QQmlEngine *engine, QQuickWindow *window);

    /// The key in use, for the log: "serial 12, no end date". Empty while locked.
    QString keySummary() const;

    // For the tests and the screen preview, in place of Android's ID, the clock, VAMA's key and the
    // clipboard (the PC's clipboard is the developer's, who works on it while the preview runs).
    void setAndroidId(const QString &androidId);
    void setToday(const QDate &today) { _today = today; }
    void setPublicKey(const QByteArray &publicKey) { _publicKey = publicKey; }
    void setClipboard(std::function<QString()> read, std::function<void(const QString &)> write);

signals:
    void lockedChanged();
    /// A key opened the app: the drone links may start.
    void unlocked();

private:
    QDate _todayOrClock() const;
    static QString _readAndroidId();

    QByteArray _fingerprint;
    QByteArray _publicKey;
    QDate _today;
    std::optional<QString> _androidId;  // set by a test, else read from Android
    bool _locked = true;
    QString _savedKeyProblem;
    std::optional<vama::license::Info> _info;
    std::function<QString()> _readClipboard;
    std::function<void(const QString &)> _writeClipboard;
};
