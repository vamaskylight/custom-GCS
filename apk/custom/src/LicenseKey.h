#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QDate>
#include <QtCore/QString>

#include <optional>

// VAMA (row A9, the client 2026-11-03: "i dont want my Apk or exe to be copied and transferred to
// another RC without permission"): the app opens only on an RC that has a license key made for it.
// These are VGCS's key rules (vgcs/app/license_key.py) for the app's product, checked against keys
// that VGCS's own code makes (test/gen_license_vectors.py):
//
// - Device code: 16 characters, for example ABCD-EFGH-IJKL-MNOP. A hash of the RC's ANDROID_ID,
//   which on Android 8 and later also depends on the app's signing key, so a test build and a
//   customer build have different codes. A factory reset changes it. The last two characters are a
//   check, so a mistyped code is refused before a key is made for it.
// - License key: that hash, a serial number, the issue date and an optional end date, signed with
//   VAMA's private key (Ed25519, Ed25519.h). The app holds only the public key, so it can check keys
//   but cannot make them. The key names its product: a VGCS key does not open the app.
//
// What it does not do: it stops a copied APK or a shared key from opening the app on another RC. It
// does not stop someone who changes the APK itself, and setting the RC's clock back gets past an end
// date.
namespace vama::license {

enum class Status {
    Ok,
    Missing,       // no key entered yet
    Damaged,       // not a complete key
    Invalid,       // not signed by VAMA
    WrongDevice,   // a real key, for another RC
    Expired,       // past its end date
    NoDeviceId,    // this RC's ID cannot be read
    OtherProduct,  // a real key, for VGCS on a laptop
};

struct Info {
    int serial = 0;
    QDate issued;
    QDate expires;  // not valid: no end date
    int features = 0;
    QByteArray fingerprint;
};

struct Result {
    Status status = Status::Missing;
    std::optional<Info> info;  // from a key VAMA signed, also when it is for another RC or has ended

    bool ok() const { return status == Status::Ok; }
};

/// VAMA's license public key, the same as VGCS's (LICENSE_PUBLIC_KEY in vgcs/app/license_key.py).
QByteArray builtInPublicKey();

/// The 9 bytes that tie a key to this RC, from Android's ANDROID_ID. Empty without an ID.
QByteArray fingerprintFromAndroidId(const QString &androidId);

/// The code the user sends to VAMA: 16 characters in four groups.
QString deviceCode(const QByteArray &fingerprint);

/// The fingerprint in a device code, or empty when it is mistyped. Case, spaces and dashes do not
/// matter, and 0, 1 and 8 count as O, I and B (the code never uses them).
QByteArray parseDeviceCode(const QString &text);

/// Whether a key opens the app on the RC with this fingerprint (empty: the ID could not be read).
/// The signature is checked first: a made-up key is never told which RC it is for.
Result checkKey(const QString &text, const QByteArray &fingerprint, const QDate &today,
                const QByteArray &publicKey = builtInPublicKey());

/// Why a key does not open the app, in words for the operator. Empty for a key that does.
QString statusText(const Result &result);

/// The key in a few words, for the log: "serial 12, no end date".
QString describe(const Info &info);

} // namespace vama::license
