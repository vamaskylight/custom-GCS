// Host test for the app's license key (row A9, 2026-11-04): Ed25519.h, LicenseKey.h and VamaLicense.h.
// The rules are a port of VGCS's vgcs/app/license_key.py and vgcs/app/ed25519.py:
// ../license_key_vectors.inc holds signatures, device codes and keys made by VGCS's own code, with
// what VGCS says of each (gen_license_vectors.py), and the port must say the same. Then the app's
// lock: locked without a key, the words for a key that does not fit, a key that fits saved and kept
// for the next start, and the clipboard. The lock screen itself is checked in the screen preview.
// Run with apk/custom/test/run_link_test.ps1.

#include <QtCore/QDir>
#include <QtCore/QSettings>
#include <QtTest/QSignalSpy>
#include <QtTest/QTest>

#include "Ed25519.h"
#include "LicenseKey.h"
#include "VamaLicense.h"

namespace {

#include "../license_key_vectors.inc"

QByteArray hex(const char *text)
{
    return QByteArray::fromHex(QByteArray(text));
}

QDate day(const char *iso)
{
    return QDate::fromString(QString::fromLatin1(iso), Qt::ISODate);
}

const char *const kHereId = kDevices[0].androidId;
const char *const kElsewhereId = kDevices[1].androidId;
const char *const kKeyForHere = kKeys[0].key;
const char *const kKeyWithAnEnd = kKeys[1].key;  // ends 2027-03-31
const char *const kKeyForElsewhere = kKeys[3].key;
const QDate kIssued(2026, 11, 4);

} // namespace

class LicenseKeyTest : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase();
    void init();

    void signaturesAsVgcs();
    void signaturesOfTheWrongSize();
    void deviceCodesAsVgcs();
    void typedCodesAsVgcs();
    void keysAsVgcs();
    void theBuiltInKeyIsVgcs();
    void aTestKeyNeverOpensARealBuild();
    void wordsForEachState();
    void describeSaysTheSerialAndTheEnd();

    void lockedWithoutAKey();
    void aKeyThatDoesNotFitIsExplained();
    void aKeyThatFitsUnlocksAndIsSaved();
    void theSavedKeyOpensTheNextStart();
    void savedSettingsOnAnotherRcStayLocked();
    void anEndedKeySaysWhen();
    void withoutAnIdNothingCanBeActivated();
    void theClipboardStandIn();
    void requiredOnlyOnAndroid();

private:
    static VamaLicense *makeLicense(const char *androidId, QObject *parent);
};

void LicenseKeyTest::initTestCase()
{
    QCoreApplication::setOrganizationName("VamaLicenseTest");
    QCoreApplication::setApplicationName("VamaLicenseTest");
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, QDir::tempPath() + "/vama-license-test");
}

void LicenseKeyTest::init()
{
    QSettings().remove("VamaLicense");
}

VamaLicense *LicenseKeyTest::makeLicense(const char *androidId, QObject *parent)
{
    auto *license = new VamaLicense(parent);
    license->setAndroidId(QString::fromUtf8(androidId));
    license->setPublicKey(hex(kTestPublicKeyHex));
    license->setToday(kIssued);
    license->setClipboard([]() { return QString(); }, [](const QString &) {});
    return license;
}

void LicenseKeyTest::signaturesAsVgcs()
{
    for (const SignatureCase &c : kSignatures) {
        QVERIFY2(vama::ed25519::verify(hex(c.publicKey), hex(c.message), hex(c.signature)) == c.ok, c.why);
    }
}

void LicenseKeyTest::signaturesOfTheWrongSize()
{
    const SignatureCase &good = kSignatures[0];
    QVERIFY(vama::ed25519::verify(hex(good.publicKey), hex(good.message), hex(good.signature)));
    QVERIFY(!vama::ed25519::verify(hex(good.publicKey).left(31), hex(good.message), hex(good.signature)));
    QVERIFY(!vama::ed25519::verify(hex(good.publicKey), hex(good.message), hex(good.signature) + '\0'));
    QVERIFY(!vama::ed25519::verify(QByteArray(), QByteArray(), QByteArray()));
}

void LicenseKeyTest::deviceCodesAsVgcs()
{
    for (const DeviceCase &c : kDevices) {
        const QByteArray fingerprint = vama::license::fingerprintFromAndroidId(QString::fromUtf8(c.androidId));
        QCOMPARE(fingerprint.toHex(), QByteArray(c.fingerprint));
        QCOMPARE(vama::license::deviceCode(fingerprint), QString::fromLatin1(c.code));
        if (!fingerprint.isEmpty()) {
            QCOMPARE(vama::license::parseDeviceCode(QString::fromLatin1(c.code)), fingerprint);
        }
    }
}

void LicenseKeyTest::typedCodesAsVgcs()
{
    for (const CodeCase &c : kCodes) {
        QVERIFY2(vama::license::parseDeviceCode(QString::fromUtf8(c.text)).toHex() == QByteArray(c.fingerprint), c.text);
    }
}

void LicenseKeyTest::keysAsVgcs()
{
    for (const KeyCase &c : kKeys) {
        const vama::license::Result result =
            vama::license::checkKey(QString::fromUtf8(c.key), hex(c.fingerprint), day(c.today), hex(kTestPublicKeyHex));
        QVERIFY2(result.status == c.status, c.why);
        QVERIFY2(result.info.has_value() == (c.serial != 0), c.why);
        if (result.info) {
            QVERIFY2(result.info->serial == c.serial, c.why);
            QVERIFY2(result.info->issued == day(c.issued), c.why);
            QVERIFY2(result.info->expires == (*c.expires ? day(c.expires) : QDate()), c.why);
            QVERIFY2(result.info->features == c.features, c.why);
            QVERIFY2(result.info->fingerprint.toHex() == QByteArray(c.forFingerprint), c.why);
        }
    }
}

void LicenseKeyTest::theBuiltInKeyIsVgcs()
{
    QCOMPARE(vama::license::builtInPublicKey().toHex(), QByteArray(kVgcsPublicKeyHex));
}

void LicenseKeyTest::aTestKeyNeverOpensARealBuild()
{
    // Without the test public key: VAMA's own. A key made with the test secret must fail there.
    const vama::license::Result result =
        vama::license::checkKey(QString::fromLatin1(kKeyForHere), hex(kDevices[0].fingerprint), kIssued);
    QCOMPARE(result.status, vama::license::Status::Invalid);
    QVERIFY(!result.info);
}

void LicenseKeyTest::wordsForEachState()
{
    using vama::license::Status;
    const auto words = [](Status status) {
        vama::license::Result result;
        result.status = status;
        return vama::license::statusText(result);
    };
    QCOMPARE(words(Status::Ok), QString());
    QCOMPARE(words(Status::Missing), QString("Paste the license key first."));
    QCOMPARE(words(Status::Damaged), QString("This key is not complete. Copy the whole key again."));
    QCOMPARE(words(Status::Invalid), QString("This is not a valid license key for this app."));
    QCOMPARE(words(Status::WrongDevice),
             QString("This key is for another RC. Send the device code above to VAMA for a key for this one."));
    QCOMPARE(words(Status::NoDeviceId),
             QString("The app cannot read this RC's ID, so it cannot be activated here. Please contact VAMA."));
    QCOMPARE(words(Status::OtherProduct),
             QString("This key is for VGCS on a laptop, not for this app. "
                     "Send the device code above to VAMA for a key for this RC."));
    const vama::license::Result ended = vama::license::checkKey(
        QString::fromLatin1(kKeyWithAnEnd), hex(kDevices[0].fingerprint), QDate(2027, 4, 1), hex(kTestPublicKeyHex));
    QCOMPARE(vama::license::statusText(ended), QString("This key ended on 2027-03-31. Ask VAMA for a new key."));
}

void LicenseKeyTest::describeSaysTheSerialAndTheEnd()
{
    vama::license::Info info;
    info.serial = 12;
    QCOMPARE(vama::license::describe(info), QString("serial 12, no end date"));
    info.expires = QDate(2027, 3, 31);
    QCOMPARE(vama::license::describe(info), QString("serial 12, ends 2027-03-31"));
}

void LicenseKeyTest::lockedWithoutAKey()
{
    QObject owner;
    VamaLicense *license = makeLicense(kHereId, &owner);
    license->load();
    QVERIFY(license->locked());
    QVERIFY(license->canActivate());
    QCOMPARE(license->deviceCode(), QString::fromLatin1(kDevices[0].code));
    QCOMPARE(license->savedKeyProblem(), QString());  // no key at all needs no words
    QCOMPARE(license->keySummary(), QString());
}

void LicenseKeyTest::aKeyThatDoesNotFitIsExplained()
{
    QObject owner;
    VamaLicense *license = makeLicense(kHereId, &owner);
    license->load();
    QSignalSpy unlocked(license, &VamaLicense::unlocked);
    QCOMPARE(license->activate("  "), QString("Paste the license key first."));
    QCOMPARE(license->activate(QString::fromLatin1(kKeyForElsewhere)),
             QString("This key is for another RC. Send the device code above to VAMA for a key for this one."));
    QCOMPARE(license->activate(QString::fromLatin1(kKeyForHere).chopped(6)),
             QString("This key is not complete. Copy the whole key again."));
    QVERIFY(license->locked());
    QCOMPARE(unlocked.count(), 0);
    QVERIFY(!QSettings().contains("VamaLicense/key"));
}

void LicenseKeyTest::aKeyThatFitsUnlocksAndIsSaved()
{
    QObject owner;
    VamaLicense *license = makeLicense(kHereId, &owner);
    license->load();
    QSignalSpy unlocked(license, &VamaLicense::unlocked);
    QSignalSpy lockedChanged(license, &VamaLicense::lockedChanged);
    const QString pasted = QStringLiteral("\n  ") + QString::fromLatin1(kKeyForHere) + QStringLiteral("  \n");
    QCOMPARE(license->activate(pasted), QString());
    QVERIFY(!license->locked());
    QCOMPARE(unlocked.count(), 1);
    QCOMPARE(lockedChanged.count(), 1);
    QCOMPARE(license->keySummary(), QString("serial 7, no end date"));
    QCOMPARE(QSettings().value("VamaLicense/key").toString(), QString::fromLatin1(kKeyForHere));
    // Once open, another key changes nothing about the lock (no second "unlocked").
    QCOMPARE(license->activate(QString::fromLatin1(kKeyForHere)), QString());
    QCOMPARE(unlocked.count(), 1);
}

void LicenseKeyTest::theSavedKeyOpensTheNextStart()
{
    {
        QObject owner;
        VamaLicense *license = makeLicense(kHereId, &owner);
        license->load();
        QCOMPARE(license->activate(QString::fromLatin1(kKeyForHere)), QString());
    }
    QObject owner;
    VamaLicense *license = makeLicense(kHereId, &owner);
    QSignalSpy unlocked(license, &VamaLicense::unlocked);
    license->load();
    QVERIFY(!license->locked());
    QCOMPARE(license->keySummary(), QString("serial 7, no end date"));
    QCOMPARE(unlocked.count(), 0);  // at the start the links are not held back, so nothing to let go
}

void LicenseKeyTest::savedSettingsOnAnotherRcStayLocked()
{
    // The app's settings copied to another RC with the APK: the key comes along, but does not fit.
    QSettings().setValue("VamaLicense/key", QString::fromLatin1(kKeyForHere));
    QObject owner;
    VamaLicense *license = makeLicense(kElsewhereId, &owner);
    license->load();
    QVERIFY(license->locked());
    QCOMPARE(license->savedKeyProblem(),
             QString("This key is for another RC. Send the device code above to VAMA for a key for this one."));
    QCOMPARE(license->deviceCode(), QString::fromLatin1(kDevices[1].code));
}

void LicenseKeyTest::anEndedKeySaysWhen()
{
    QSettings().setValue("VamaLicense/key", QString::fromLatin1(kKeyWithAnEnd));
    QObject owner;
    VamaLicense *license = makeLicense(kHereId, &owner);
    license->setToday(QDate(2027, 3, 31));
    license->load();
    QVERIFY(!license->locked());
    QCOMPARE(license->keySummary(), QString("serial 8, ends 2027-03-31"));
    VamaLicense *later = makeLicense(kHereId, &owner);
    later->setToday(QDate(2027, 4, 1));
    later->load();
    QVERIFY(later->locked());
    QCOMPARE(later->savedKeyProblem(), QString("This key ended on 2027-03-31. Ask VAMA for a new key."));
}

void LicenseKeyTest::withoutAnIdNothingCanBeActivated()
{
    QObject owner;
    VamaLicense *license = makeLicense("", &owner);
    license->load();
    QVERIFY(license->locked());
    QVERIFY(!license->canActivate());
    QCOMPARE(license->deviceCode(), QString());
    QCOMPARE(license->savedKeyProblem(),
             QString("The app cannot read this RC's ID, so it cannot be activated here. Please contact VAMA."));
    QCOMPARE(license->activate(QString::fromLatin1(kKeyForHere)), license->savedKeyProblem());
    QVERIFY(license->locked());
}

void LicenseKeyTest::theClipboardStandIn()
{
    QObject owner;
    VamaLicense *license = makeLicense(kHereId, &owner);
    QString clipboard = QStringLiteral("pasted text");
    license->setClipboard([&clipboard]() { return clipboard; }, [&clipboard](const QString &text) { clipboard = text; });
    license->load();
    QCOMPARE(license->clipboardText(), QString("pasted text"));
    license->copyDeviceCode();
    QCOMPARE(clipboard, QString::fromLatin1(kDevices[0].code));
}

void LicenseKeyTest::requiredOnlyOnAndroid()
{
    // This test runs on Windows: there the key is needed only when asked for.
    qunsetenv("VAMA_REQUIRE_LICENSE");
    QVERIFY(!VamaLicense::required());
    qputenv("VAMA_REQUIRE_LICENSE", "1");
    QVERIFY(VamaLicense::required());
    qputenv("VAMA_REQUIRE_LICENSE", "0");
    QVERIFY(!VamaLicense::required());
    qunsetenv("VAMA_REQUIRE_LICENSE");
}

QTEST_GUILESS_MAIN(LicenseKeyTest)
#include "license_key_test.moc"
