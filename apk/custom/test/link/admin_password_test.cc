// Host test for the admin password of Advanced Mode (task Z, AdminPassword.h): the hash is
// PBKDF2-HMAC-SHA256 as Python's hashlib and .NET make it (apk/make_admin_password.ps1), a
// test build opens with the test password, a customer build with the team's (from the file
// that apk/build_android.ps1 writes), and the waits after wrong passwords.
// Built twice (CMakeLists.txt): admin_password_test as a test build, admin_password_team_test
// as a customer build. Run with apk/custom/test/run_link_test.ps1.

#include <QtTest/QTest>

#include <initializer_list>

#include "AdminPassword.h"

using vama::AdminPassword;
using vama::AdminSecret;
using Result = vama::AdminPassword::Result;

namespace {

// Few rounds, so the rules run fast. The app's secrets have 100000.
AdminSecret quickSecret(const QString &password)
{
    const QByteArray salt = QByteArray::fromHex("0102030405060708090a0b0c0d0e0f10");
    return {salt, vama::deriveAdminHash(password, salt, 1000), 1000};
}

const QString kRight = QStringLiteral("right one");
const QString kWrong = QStringLiteral("wrong");

}  // namespace

class AdminPasswordTest : public QObject
{
    Q_OBJECT

private slots:
    void theHashIsPbkdf2HmacSha256();
    void theTestSecretIsTheTestPassword();
    void theBuildHasTheRightPassword();
    void theRightPasswordIsRight();
    void aWrongPasswordWaitsFiveSeconds();
    void duringTheWaitNothingIsChecked();
    void fiveWrongInARowWaitAMinute();
    void theRightPasswordStartsTheCountAgain();
    void anEmptyPasswordIsWrong();
    void aBrokenSecretOpensWithNothing();
    void aHashThatDiffersInOneByteDoesNotOpen();
    void theWaitIsInWholeSecondsRoundedUp();
};

void AdminPasswordTest::theHashIsPbkdf2HmacSha256()
{
    // RFC 7914, section 11 (its first 32 bytes, the length the app keeps).
    QCOMPARE(vama::deriveAdminHash(QStringLiteral("passwd"), QByteArrayLiteral("salt"), 1).toHex(),
             QByteArrayLiteral("55ac046e56e3089fec1691c22544b605f94185216dde0465e68b9d57c20dacbc"));
    QCOMPARE(vama::deriveAdminHash(QStringLiteral("Password"), QByteArrayLiteral("NaCl"), 80000).toHex(),
             QByteArrayLiteral("4ddcd8f60b98be21830cee5ef22701f9641a4418d04c0414aeff08876b34ab56"));
    // Made by Python's hashlib.pbkdf2_hmac and by .NET's Rfc2898DeriveBytes (the way
    // apk/make_admin_password.ps1 makes the team's), with the rounds of the app's secrets.
    QCOMPARE(vama::deriveAdminHash(QStringLiteral("vama1234"), QByteArray::fromHex("00112233445566778899aabbccddeeff"), 100000).toHex(),
             QByteArrayLiteral("7bb9413a594504cafc80aecc6a34e5d59a99362f98473c9160e709737cd2426b"));
}

void AdminPasswordTest::theTestSecretIsTheTestPassword()
{
    const AdminSecret test = vama::testAdminSecret();
    QCOMPARE(test.iterations, 100000);
    QCOMPARE(test.salt.size(), 16);
    QCOMPARE(vama::deriveAdminHash(QString::fromLatin1(vama::kTestAdminPassword), test.salt, test.iterations), test.hash);
}

void AdminPasswordTest::theBuildHasTheRightPassword()
{
    const AdminSecret built = vama::builtInAdminSecret();
#ifdef VAMA_TEST_TEAM_PASSWORD
    // A customer build: the team's password, from the build script's file. The test password
    // does not open it.
    QVERIFY2(built.hash != vama::testAdminSecret().hash, "a customer build has the test password");
    QCOMPARE(built.iterations, 100000);
    AdminPassword team(built);
    QCOMPARE(team.check(QStringLiteral(VAMA_TEST_TEAM_PASSWORD), 0), Result::Right);
    QCOMPARE(team.check(QString::fromLatin1(vama::kTestAdminPassword), 0), Result::Wrong);
#else
    // A test build: the test password.
    QVERIFY2(built.hash == vama::testAdminSecret().hash,
             "not the test password: apk/custom/src/admin_password.inc is left from a customer build, remove it");
    QCOMPARE(AdminPassword(built).check(QString::fromLatin1(vama::kTestAdminPassword), 0), Result::Right);
#endif
}

void AdminPasswordTest::theRightPasswordIsRight()
{
    AdminPassword admin(quickSecret(kRight));
    QCOMPARE(admin.waitSeconds(0), 0);
    QCOMPARE(admin.check(kRight, 1000), Result::Right);
    QCOMPARE(admin.waitSeconds(1000), 0);
    QCOMPARE(admin.check(kRight, 1000), Result::Right);  // no wait after a right one
    QCOMPARE(admin.check(QStringLiteral("Right one"), 1000), Result::Wrong);  // capitals count
}

void AdminPasswordTest::aWrongPasswordWaitsFiveSeconds()
{
    QCOMPARE(AdminPassword::kWrongWaitMs, 5000);
    AdminPassword admin(quickSecret(kRight));
    QCOMPARE(admin.check(kWrong, 10000), Result::Wrong);
    QCOMPARE(admin.waitSeconds(10000), 5);
    QCOMPARE(admin.check(kRight, 14999), Result::Wait);
    QCOMPARE(admin.waitSeconds(15000), 0);
    QCOMPARE(admin.check(kRight, 15000), Result::Right);
}

void AdminPasswordTest::duringTheWaitNothingIsChecked()
{
    AdminPassword admin(quickSecret(kRight));
    QCOMPARE(admin.check(kWrong, 0), Result::Wrong);
    QCOMPARE(admin.check(kRight, 4999), Result::Wait);  // not even the right one
    QCOMPARE(admin.check(kWrong, 4999), Result::Wait);  // and a try in the wait does not make it longer
    QCOMPARE(admin.waitSeconds(4999), 1);
    QCOMPARE(admin.check(kRight, 5000), Result::Right);
}

void AdminPasswordTest::fiveWrongInARowWaitAMinute()
{
    QCOMPARE(AdminPassword::kManyWrong, 5);
    QCOMPARE(AdminPassword::kManyWrongWaitMs, 60000);
    AdminPassword admin(quickSecret(kRight));
    qint64 now = 0;
    for (int i = 1; i <= 4; ++i) {
        QCOMPARE(admin.check(kWrong, now), Result::Wrong);
        QCOMPARE(admin.waitSeconds(now), 5);
        now += 5000;
    }
    QCOMPARE(admin.check(kWrong, now), Result::Wrong);  // the fifth in a row
    QCOMPARE(admin.waitSeconds(now), 60);
    QCOMPARE(admin.check(kRight, now + 59999), Result::Wait);
    QCOMPARE(admin.check(kWrong, now + 60000), Result::Wrong);  // the sixth: a minute again
    QCOMPARE(admin.waitSeconds(now + 60000), 60);
    QCOMPARE(admin.check(kRight, now + 120000), Result::Right);
}

void AdminPasswordTest::theRightPasswordStartsTheCountAgain()
{
    AdminPassword admin(quickSecret(kRight));
    qint64 now = 0;
    for (int i = 1; i <= 4; ++i) {
        QCOMPARE(admin.check(kWrong, now), Result::Wrong);
        now += 5000;
    }
    QCOMPARE(admin.check(kRight, now), Result::Right);
    QCOMPARE(admin.check(kWrong, now), Result::Wrong);  // the first wrong one again, not the fifth
    QCOMPARE(admin.waitSeconds(now), 5);
}

void AdminPasswordTest::anEmptyPasswordIsWrong()
{
    // Even where the hash would match (the build script makes no such secret).
    AdminPassword admin(quickSecret(QString()));
    QCOMPARE(admin.check(QString(), 0), Result::Wrong);
}

void AdminPasswordTest::aBrokenSecretOpensWithNothing()
{
    // Nothing at all: no rounds give no hash, and an empty hash must not match it.
    QCOMPARE(AdminPassword(AdminSecret{}).check(kRight, 0), Result::Wrong);
    const QByteArray salt = QByteArray::fromHex("0102030405060708090a0b0c0d0e0f10");
    QCOMPARE(AdminPassword(AdminSecret{salt, QByteArray(), 0}).check(kRight, 0), Result::Wrong);
    // No salt, or a hash of the wrong length: not a secret of the build script.
    QCOMPARE(AdminPassword(AdminSecret{QByteArray(), vama::deriveAdminHash(kRight, QByteArray(), 1000), 1000}).check(kRight, 0),
             Result::Wrong);
    QCOMPARE(AdminPassword(AdminSecret{salt, vama::deriveAdminHash(kRight, salt, 1000).left(16), 1000}).check(kRight, 0),
             Result::Wrong);
}

void AdminPasswordTest::aHashThatDiffersInOneByteDoesNotOpen()
{
    // Every byte counts, not only the last or the first.
    const AdminSecret good = quickSecret(kRight);
    for (int i = 0; i < good.hash.size(); ++i) {
        AdminSecret other = good;
        other.hash[i] = static_cast<char>(other.hash.at(i) ^ 0x01);
        QVERIFY2(AdminPassword(other).check(kRight, 0) == Result::Wrong, qPrintable(QStringLiteral("byte %1").arg(i)));
    }
}

void AdminPasswordTest::theWaitIsInWholeSecondsRoundedUp()
{
    AdminPassword admin(quickSecret(kRight));
    QCOMPARE(admin.waitSeconds(0), 0);  // before any try
    QCOMPARE(admin.check(kWrong, 1000), Result::Wrong);
    QCOMPARE(admin.waitSeconds(1000), 5);
    QCOMPARE(admin.waitSeconds(1001), 5);  // 4.999 s
    QCOMPARE(admin.waitSeconds(5000), 1);
    QCOMPARE(admin.waitSeconds(5999), 1);  // 1 ms: never 0 while it waits
    QCOMPARE(admin.waitSeconds(6000), 0);
}

QTEST_GUILESS_MAIN(AdminPasswordTest)
#include "admin_password_test.moc"
