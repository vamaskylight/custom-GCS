#include "AdminPassword.h"

#include <QtNetwork/QPasswordDigestor>

#include <algorithm>

// A customer build's password: written by apk/build_android.ps1 (VAMA_ADMIN_SALT_HEX,
// VAMA_ADMIN_HASH_HEX, VAMA_ADMIN_ITERATIONS) and removed after that build. Not in git.
#if __has_include("admin_password.inc")
#include "admin_password.inc"
#endif

namespace vama {

namespace {

// kTestAdminPassword's salt and hash, made once (100000 rounds, as a customer build's).
constexpr const char kTestSaltHex[] = "c4196712152590a95a9e20e7573dff17";
constexpr const char kTestHashHex[] = "79483abd3d0626ff731367fc33bf9cecbeaa384176928a86528bd5d6ed2b39d2";
constexpr int kTestIterations = 100000;

constexpr int kHashBytes = 32;

// The same time for any password: no early end at the first byte that differs.
bool sameBytes(const QByteArray &a, const QByteArray &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    unsigned char diff = 0;
    for (qsizetype i = 0; i < a.size(); ++i) {
        diff |= static_cast<unsigned char>(a.at(i) ^ b.at(i));
    }
    return diff == 0;
}

}  // namespace

AdminSecret testAdminSecret()
{
    return {QByteArray::fromHex(kTestSaltHex), QByteArray::fromHex(kTestHashHex), kTestIterations};
}

AdminSecret builtInAdminSecret()
{
#if defined(VAMA_ADMIN_SALT_HEX) && defined(VAMA_ADMIN_HASH_HEX) && defined(VAMA_ADMIN_ITERATIONS)
    return {QByteArray::fromHex(VAMA_ADMIN_SALT_HEX), QByteArray::fromHex(VAMA_ADMIN_HASH_HEX), VAMA_ADMIN_ITERATIONS};
#else
    return testAdminSecret();
#endif
}

QByteArray deriveAdminHash(const QString &password, const QByteArray &salt, int iterations)
{
    return QPasswordDigestor::deriveKeyPbkdf2(QCryptographicHash::Sha256, password.toUtf8(), salt, iterations, kHashBytes);
}

AdminPassword::AdminPassword(const AdminSecret &secret)
    : _secret(secret)
{
}

AdminPassword::Result AdminPassword::check(const QString &password, qint64 nowMs)
{
    if (_anyTry && nowMs < _nextTryMs) {
        return Result::Wait;
    }
    _anyTry = true;
    const bool usable = _secret.iterations > 0 && !_secret.salt.isEmpty() && _secret.hash.size() == kHashBytes;
    if (usable && !password.isEmpty() && sameBytes(deriveAdminHash(password, _secret.salt, _secret.iterations), _secret.hash)) {
        _wrongInARow = 0;
        _nextTryMs = nowMs;
        return Result::Right;
    }
    ++_wrongInARow;
    _nextTryMs = nowMs + (_wrongInARow >= kManyWrong ? kManyWrongWaitMs : kWrongWaitMs);
    return Result::Wrong;
}

int AdminPassword::waitSeconds(qint64 nowMs) const
{
    if (!_anyTry || nowMs >= _nextTryMs) {
        return 0;
    }
    return static_cast<int>(std::max<qint64>(1, (_nextTryMs - nowMs + 999) / 1000));
}

}  // namespace vama
