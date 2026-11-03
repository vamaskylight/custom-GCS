#include "LicenseKey.h"

#include "Ed25519.h"

#include <QtCore/QCryptographicHash>
#include <QtCore/QStringList>

namespace vama::license {

namespace {

// VAMA's license public key, the same as VGCS's. The issuer's tool (packaging/vgcs_license_tool.py)
// reads it from this line to check that it signs with the matching private key: keep its form.
constexpr char kLicensePublicKeyHex[] = "d5bd5e5523238b32d6b885a4006f36c1dacef1893e2fd7e439e89dc43ba12798";

constexpr int kFingerprintBytes = 9;
constexpr int kPayloadBytes = 1 + kFingerprintBytes + 2 + 2 + 2 + 1;
constexpr int kKeyBytes = kPayloadBytes + 64;
constexpr unsigned char kAppKeyVersion = 0x11;   // product 1 (this app), key version 1
constexpr unsigned char kVgcsKeyVersion = 0x01;  // VGCS's keys

const char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567";  // base32, RFC 4648

QDate epoch()
{
    return QDate(2026, 1, 1);  // dates in a key are days after this
}

QByteArray withNul(const char *text)
{
    QByteArray bytes(text);
    bytes.append('\0');
    return bytes;
}

QByteArray codeCheck(const QByteArray &fingerprint)
{
    return QCryptographicHash::hash(withNul("VAMA app device code check") + fingerprint, QCryptographicHash::Sha256)
        .left(1);
}

QString base32Encode(const QByteArray &data)
{
    QString out;
    quint32 buffer = 0;
    int bits = 0;
    for (const char byte : data) {
        buffer = (buffer << 8) | quint32(uchar(byte));
        bits += 8;
        while (bits >= 5) {
            bits -= 5;
            out += QLatin1Char(kAlphabet[(buffer >> bits) & 31]);
        }
        buffer &= (1u << bits) - 1;
    }
    if (bits > 0) {
        out += QLatin1Char(kAlphabet[(buffer << (5 - bits)) & 31]);
    }
    return out;
}

// As Python's base64.b32decode of the text with its padding added (VGCS's _b32decode): nothing for a
// length base32 never has or a character outside the alphabet; the bits after the last whole byte
// are dropped.
std::optional<QByteArray> base32Decode(const QString &text)
{
    const qsizetype rest = text.size() % 8;
    if (rest == 1 || rest == 3 || rest == 6) {
        return std::nullopt;
    }
    QByteArray out;
    quint32 buffer = 0;
    int bits = 0;
    for (const QChar c : text) {
        const char16_t u = c.unicode();
        quint32 value = 0;
        if (u >= u'A' && u <= u'Z') {
            value = quint32(u - u'A');
        } else if (u >= u'2' && u <= u'7') {
            value = quint32(26 + (u - u'2'));
        } else {
            return std::nullopt;
        }
        buffer = (buffer << 5) | value;
        bits += 5;
        if (bits >= 8) {
            bits -= 8;
            out.append(char((buffer >> bits) & 0xff));
        }
        buffer &= (1u << bits) - 1;
    }
    return out;
}

// As VGCS's _normalise: letters and digits only, in upper case, with 0, 1 and 8 read as O, I and B.
// False for a letter or digit outside ASCII: VGCS keeps such a character, and the text then does
// not decode.
bool normalise(const QString &text, QString *out)
{
    QString kept;
    bool ascii = true;
    for (const QChar c : text) {
        const char16_t u = c.unicode();
        if (u >= u'a' && u <= u'z') {
            kept += QChar(char16_t(u - u'a' + u'A'));
        } else if ((u >= u'A' && u <= u'Z') || (u >= u'0' && u <= u'9')) {
            kept += c;
        } else if (c.isLetterOrNumber()) {
            ascii = false;
        }
    }
    for (QChar &c : kept) {
        if (c == u'0') {
            c = u'O';
        } else if (c == u'1') {
            c = u'I';
        } else if (c == u'8') {
            c = u'B';
        }
    }
    *out = kept;
    return ascii;
}

int bigEndian16(const QByteArray &data, int at)
{
    return (int(uchar(data.at(at))) << 8) | int(uchar(data.at(at + 1)));
}

Result makeResult(Status status, const std::optional<Info> &info = std::nullopt)
{
    Result r;
    r.status = status;
    r.info = info;
    return r;
}

} // namespace

QByteArray builtInPublicKey()
{
    return QByteArray::fromHex(kLicensePublicKeyHex);
}

QByteArray fingerprintFromAndroidId(const QString &androidId)
{
    const QString id = androidId.trimmed().toLower();
    if (id.isEmpty()) {
        return {};
    }
    const QByteArray source = withNul("VAMA app device code v1") + QByteArrayLiteral("android-id:") + id.toUtf8();
    return QCryptographicHash::hash(source, QCryptographicHash::Sha256).left(kFingerprintBytes);
}

QString deviceCode(const QByteArray &fingerprint)
{
    if (fingerprint.size() != kFingerprintBytes) {
        return {};
    }
    const QString text = base32Encode(fingerprint + codeCheck(fingerprint));
    return QStringList{text.mid(0, 4), text.mid(4, 4), text.mid(8, 4), text.mid(12, 4)}.join(QLatin1Char('-'));
}

QByteArray parseDeviceCode(const QString &text)
{
    QString kept;
    if (!normalise(text, &kept)) {
        return {};
    }
    const std::optional<QByteArray> raw = base32Decode(kept);
    if (!raw || raw->size() != kFingerprintBytes + 1) {
        return {};
    }
    const QByteArray fingerprint = raw->left(kFingerprintBytes);
    return raw->mid(kFingerprintBytes) == codeCheck(fingerprint) ? fingerprint : QByteArray();
}

Result checkKey(const QString &text, const QByteArray &fingerprint, const QDate &today, const QByteArray &publicKey)
{
    if (fingerprint.isEmpty()) {
        return makeResult(Status::NoDeviceId);
    }
    QString kept;
    const bool ascii = normalise(text, &kept);
    if (ascii && kept.isEmpty()) {
        return makeResult(Status::Missing);
    }
    const std::optional<QByteArray> raw = ascii ? base32Decode(kept) : std::nullopt;
    if (!raw || raw->size() != kKeyBytes) {
        return makeResult(Status::Damaged);
    }
    const QByteArray payload = raw->left(kPayloadBytes);
    const QByteArray signature = raw->mid(kPayloadBytes);
    const unsigned char version = uchar(raw->at(0));
    if (version != kAppKeyVersion) {
        if (version != kVgcsKeyVersion) {
            return makeResult(Status::Damaged);
        }
        // Said only of a key VAMA really made, and without what it is for.
        const bool vgcs = ed25519::verify(publicKey, withNul("VGCS license key v1") + payload, signature);
        return makeResult(vgcs ? Status::OtherProduct : Status::Invalid);
    }
    // The signature first: a made-up key must never be told which RC it is for.
    if (!ed25519::verify(publicKey, withNul("VAMA app license key v1") + payload, signature)) {
        return makeResult(Status::Invalid);
    }
    Info info;
    info.fingerprint = payload.mid(1, kFingerprintBytes);
    info.serial = bigEndian16(payload, 10);
    info.issued = epoch().addDays(bigEndian16(payload, 12));
    const int expiresDay = bigEndian16(payload, 14);
    info.expires = expiresDay ? epoch().addDays(expiresDay) : QDate();
    info.features = uchar(payload.at(16));
    if (info.fingerprint != fingerprint) {
        return makeResult(Status::WrongDevice, info);
    }
    if (info.expires.isValid() && today > info.expires) {
        return makeResult(Status::Expired, info);
    }
    return makeResult(Status::Ok, info);
}

QString statusText(const Result &result)
{
    switch (result.status) {
    case Status::Ok:
        return {};
    case Status::Missing:
        return QStringLiteral("Paste the license key first.");
    case Status::Damaged:
        return QStringLiteral("This key is not complete. Copy the whole key again.");
    case Status::Invalid:
        return QStringLiteral("This is not a valid license key for this app.");
    case Status::WrongDevice:
        return QStringLiteral("This key is for another RC. Send the device code above to VAMA for a key for this one.");
    case Status::Expired:
        if (result.info && result.info->expires.isValid()) {
            return QStringLiteral("This key ended on %1. Ask VAMA for a new key.")
                .arg(result.info->expires.toString(Qt::ISODate));
        }
        return QStringLiteral("This key has ended. Ask VAMA for a new key.");
    case Status::NoDeviceId:
        return QStringLiteral("The app cannot read this RC's ID, so it cannot be activated here. Please contact VAMA.");
    case Status::OtherProduct:
        return QStringLiteral("This key is for VGCS on a laptop, not for this app. "
                              "Send the device code above to VAMA for a key for this RC.");
    }
    return {};
}

QString describe(const Info &info)
{
    const QString end = info.expires.isValid() ? QStringLiteral("ends %1").arg(info.expires.toString(Qt::ISODate))
                                               : QStringLiteral("no end date");
    return QStringLiteral("serial %1, %2").arg(info.serial).arg(end);
}

} // namespace vama::license
