#pragma once

// The admin password that guards Advanced Mode (task Z, client 2026-10-16: "we want
// authentication so the authenticated person only modify those parameters").
//
// Parameters, and QGC's other advanced pages, show only in Advanced Mode, and turning it
// on asks for this password (in the app menu: VamaAdminPasswordPanel.qml, then
// CustomPlugin::unlockAdvanced).
// The app keeps only a salted hash of it: PBKDF2 with HMAC-SHA256.
//
// A customer build gets the team's password: apk/make_admin_password.ps1 makes its salt and
// hash once, outside the repo, and apk/build_android.ps1 writes them into admin_password.inc
// beside this file for that build (not in git). A test build has the test password.
//
// The hash can be read out of an APK, and a guess at it tried away from the app. So the
// password must be long. And a password in the app does not stop another app (Mission
// Planner) from changing parameters over the radio: MAVLink signing does (task T).

#include <QtCore/QByteArray>
#include <QtCore/QString>

namespace vama {

/// A password's salt and PBKDF2 hash, and the number of rounds.
struct AdminSecret
{
    QByteArray salt;
    QByteArray hash;
    int iterations = 0;
};

/// The password of the test builds (it is in their test steps).
inline constexpr const char kTestAdminPassword[] = "vama1234";

/// The salt and hash of the test password.
AdminSecret testAdminSecret();

/// The secret in this app: a customer build's (admin_password.inc), else the test one.
AdminSecret builtInAdminSecret();

/// PBKDF2 with HMAC-SHA256 over the password's UTF-8 bytes: 32 bytes.
QByteArray deriveAdminHash(const QString &password, const QByteArray &salt, int iterations);

/// Checks passwords, and makes the next try wait after a wrong one.
class AdminPassword
{
public:
    enum class Result { Right, Wrong, Wait };

    explicit AdminPassword(const AdminSecret &secret);

    /// nowMs: a clock in milliseconds. A try during a wait is not checked (Result::Wait).
    Result check(const QString &password, qint64 nowMs);
    /// Seconds until the next try may be made. 0: now.
    int waitSeconds(qint64 nowMs) const;

    /// After a wrong password the next try waits this long, and after this many wrong ones in a row longer.
    static constexpr int kWrongWaitMs = 5000;
    static constexpr int kManyWrong = 5;
    static constexpr int kManyWrongWaitMs = 60000;

private:
    AdminSecret _secret;
    int _wrongInARow = 0;
    qint64 _nextTryMs = 0;
    bool _anyTry = false;
};

}  // namespace vama
