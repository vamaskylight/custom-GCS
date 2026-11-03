#include "Ed25519.h"

#include <QtCore/QByteArrayView>
#include <QtCore/QCryptographicHash>

#include <cstdint>
#include <cstring>

// The field and curve arithmetic is TweetNaCl's (Bernstein, van Gastel, Janssen, Lange, Schwabe
// and Smetsers, public domain, https://tweetnacl.cr.yp.to/), its checking half only, in C++. The
// SHA-512 is Qt's. Changed from TweetNaCl: a left shift of a number that can be negative is written
// as a multiplication (C++17 leaves that shift undefined), and the checks that VGCS's ed25519.py
// makes and TweetNaCl leaves out are added: a public key's y below p, no x of 0 with the sign bit
// set, and S below the group order. The constants were checked against VGCS's ed25519.py.

namespace vama::ed25519 {

namespace {

using i64 = std::int64_t;
using u8 = std::uint8_t;
using gf = i64[16];  // a number modulo p = 2^255 - 19, in 16 limbs of 16 bits, lowest first

const gf kZero = {};
const gf kOne = {1};
// d = -121665 / 121666, 2d, the base point's x and y, and the square root of -1.
const gf kD = {0x78a3, 0x1359, 0x4dca, 0x75eb, 0xd8ab, 0x4141, 0x0a4d, 0x0070,
               0xe898, 0x7779, 0x4079, 0x8cc7, 0xfe73, 0x2b6f, 0x6cee, 0x5203};
const gf kD2 = {0xf159, 0x26b2, 0x9b94, 0xebd6, 0xb156, 0x8283, 0x149a, 0x00e0,
                0xd130, 0xeef3, 0x80f2, 0x198e, 0xfce7, 0x56df, 0xd9dc, 0x2406};
const gf kX = {0xd51a, 0x8f25, 0x2d60, 0xc956, 0xa7b2, 0x9525, 0xc760, 0x692c,
               0xdc5c, 0xfdd6, 0xe231, 0xc0a4, 0x53fe, 0xcd6e, 0x36d3, 0x2169};
const gf kY = {0x6658, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666,
               0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666};
const gf kSqrtM1 = {0xa0b0, 0x4a0e, 0x1b27, 0xc4ee, 0xe478, 0xad2f, 0x1806, 0x2f43,
                    0xd7a7, 0x3dfb, 0x0099, 0x2b4d, 0xdf0b, 0x4fc1, 0x2480, 0x2b83};
// The group order L = 2^252 + 27742317777372353535851937790883648493, its bytes lowest first.
const i64 kL[32] = {0xed, 0xd3, 0xf5, 0x5c, 0x1a, 0x63, 0x12, 0x58, 0xd6, 0x9c, 0xf7, 0xa2, 0xde, 0xf9, 0xde, 0x14,
                    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0x10};

bool equal32(const u8 *x, const u8 *y)
{
    unsigned difference = 0;
    for (int i = 0; i < 32; ++i) {
        difference |= unsigned(x[i] ^ y[i]);
    }
    return difference == 0;
}

void set25519(gf r, const gf a)
{
    for (int i = 0; i < 16; ++i) {
        r[i] = a[i];
    }
}

void car25519(gf o)
{
    for (int i = 0; i < 16; ++i) {
        o[i] += i64(1) << 16;
        const i64 c = o[i] >> 16;  // the shift of a negative number is arithmetic (GCC, Clang, C++20)
        if (i < 15) {
            o[i + 1] += c - 1;
        } else {
            o[0] += 38 * (c - 1);
        }
        o[i] -= c * 65536;
    }
}

void sel25519(gf p, gf q, int b)
{
    const i64 c = ~(i64(b) - 1);  // all ones for b = 1, nothing for b = 0
    for (int i = 0; i < 16; ++i) {
        const i64 t = c & (p[i] ^ q[i]);
        p[i] ^= t;
        q[i] ^= t;
    }
}

void pack25519(u8 *o, const gf n)
{
    gf m = {};
    gf t = {};
    set25519(t, n);
    car25519(t);
    car25519(t);
    car25519(t);
    for (int j = 0; j < 2; ++j) {
        m[0] = t[0] - 0xffed;
        for (int i = 1; i < 15; ++i) {
            m[i] = t[i] - 0xffff - ((m[i - 1] >> 16) & 1);
            m[i - 1] &= 0xffff;
        }
        m[15] = t[15] - 0x7fff - ((m[14] >> 16) & 1);
        const int b = int((m[15] >> 16) & 1);
        m[14] &= 0xffff;
        sel25519(t, m, 1 - b);
    }
    for (int i = 0; i < 16; ++i) {
        o[2 * i] = u8(t[i] & 0xff);
        o[2 * i + 1] = u8((t[i] >> 8) & 0xff);
    }
}

bool neq25519(const gf a, const gf b)
{
    u8 c[32];
    u8 d[32];
    pack25519(c, a);
    pack25519(d, b);
    return !equal32(c, d);
}

u8 par25519(const gf a)
{
    u8 d[32];
    pack25519(d, a);
    return u8(d[0] & 1);
}

bool isZero25519(const gf a)
{
    u8 d[32];
    pack25519(d, a);
    const u8 zero[32] = {};
    return equal32(d, zero);
}

void unpack25519(gf o, const u8 *n)
{
    for (int i = 0; i < 16; ++i) {
        o[i] = i64(n[2 * i]) + i64(n[2 * i + 1]) * 256;
    }
    o[15] &= 0x7fff;
}

void add25519(gf o, const gf a, const gf b)
{
    for (int i = 0; i < 16; ++i) {
        o[i] = a[i] + b[i];
    }
}

void sub25519(gf o, const gf a, const gf b)
{
    for (int i = 0; i < 16; ++i) {
        o[i] = a[i] - b[i];
    }
}

void mul25519(gf o, const gf a, const gf b)
{
    i64 t[31] = {};
    for (int i = 0; i < 16; ++i) {
        for (int j = 0; j < 16; ++j) {
            t[i + j] += a[i] * b[j];
        }
    }
    for (int i = 0; i < 15; ++i) {
        t[i] += 38 * t[i + 16];
    }
    for (int i = 0; i < 16; ++i) {
        o[i] = t[i];
    }
    car25519(o);
    car25519(o);
}

void sq25519(gf o, const gf a)
{
    mul25519(o, a, a);
}

void inv25519(gf o, const gf in)
{
    gf c;
    set25519(c, in);
    for (int a = 253; a >= 0; --a) {
        sq25519(c, c);
        if (a != 2 && a != 4) {
            mul25519(c, c, in);
        }
    }
    set25519(o, c);
}

void pow2523(gf o, const gf in)
{
    gf c;
    set25519(c, in);
    for (int a = 250; a >= 0; --a) {
        sq25519(c, c);
        if (a != 1) {
            mul25519(c, c, in);
        }
    }
    set25519(o, c);
}

// Points in extended coordinates (X, Y, Z, T), as in VGCS's ed25519.py.
void add(gf p[4], const gf q[4])
{
    gf a, b, c, d, t, e, f, g, h;
    sub25519(a, p[1], p[0]);
    sub25519(t, q[1], q[0]);
    mul25519(a, a, t);
    add25519(b, p[0], p[1]);
    add25519(t, q[0], q[1]);
    mul25519(b, b, t);
    mul25519(c, p[3], q[3]);
    mul25519(c, c, kD2);
    mul25519(d, p[2], q[2]);
    add25519(d, d, d);
    sub25519(e, b, a);
    sub25519(f, d, c);
    add25519(g, d, c);
    add25519(h, b, a);

    mul25519(p[0], e, f);
    mul25519(p[1], h, g);
    mul25519(p[2], g, f);
    mul25519(p[3], e, h);
}

void cswap(gf p[4], gf q[4], u8 b)
{
    for (int i = 0; i < 4; ++i) {
        sel25519(p[i], q[i], b);
    }
}

void pack(u8 *r, gf p[4])
{
    gf tx, ty, zi;
    inv25519(zi, p[2]);
    mul25519(tx, p[0], zi);
    mul25519(ty, p[1], zi);
    pack25519(r, ty);
    r[31] = u8(r[31] ^ (par25519(tx) << 7));
}

void scalarmult(gf p[4], gf q[4], const u8 *s)
{
    set25519(p[0], kZero);
    set25519(p[1], kOne);
    set25519(p[2], kOne);
    set25519(p[3], kZero);
    for (int i = 255; i >= 0; --i) {
        const u8 b = u8((s[i / 8] >> (i & 7)) & 1);
        cswap(p, q, b);
        add(q, p);
        add(p, p);
        cswap(p, q, b);
    }
}

void scalarbase(gf p[4], const u8 *s)
{
    gf q[4];
    set25519(q[0], kX);
    set25519(q[1], kY);
    set25519(q[2], kOne);
    mul25519(q[3], kX, kY);
    scalarmult(p, q, s);
}

void modL(u8 *r, i64 x[64])
{
    i64 carry = 0;
    for (int i = 63; i >= 32; --i) {
        carry = 0;
        int j = i - 32;
        for (; j < i - 12; ++j) {
            x[j] += carry - 16 * x[i] * kL[j - (i - 32)];
            carry = (x[j] + 128) >> 8;
            x[j] -= carry * 256;
        }
        x[j] += carry;
        x[i] = 0;
    }
    carry = 0;
    for (int j = 0; j < 32; ++j) {
        x[j] += carry - (x[31] >> 4) * kL[j];
        carry = x[j] >> 8;
        x[j] &= 255;
    }
    for (int j = 0; j < 32; ++j) {
        x[j] -= carry * kL[j];
    }
    for (int i = 0; i < 32; ++i) {
        x[i + 1] += x[i] >> 8;
        r[i] = u8(x[i] & 255);
    }
}

// r: 64 bytes of a hash in, their number modulo L out (its first 32 bytes).
void reduce(u8 *r)
{
    i64 x[64];
    for (int i = 0; i < 64; ++i) {
        x[i] = i64(r[i]);
    }
    for (int i = 0; i < 64; ++i) {
        r[i] = 0;
    }
    modL(r, x);
}

// A point's y must be below p (VGCS's _recover_x refuses it otherwise).
bool yBelowP(const u8 *s)
{
    if ((s[31] & 0x7f) != 0x7f) {
        return true;
    }
    for (int i = 30; i >= 1; --i) {
        if (s[i] != 0xff) {
            return true;
        }
    }
    return s[0] < 0xed;
}

bool scalarBelowL(const u8 *s)
{
    for (int i = 31; i >= 0; --i) {
        if (i64(s[i]) < kL[i]) {
            return true;
        }
        if (i64(s[i]) > kL[i]) {
            return false;
        }
    }
    return false;  // S is L itself
}

// r = -A for the point A that the 32 bytes encode. False when they encode no point.
bool unpackneg(gf r[4], const u8 *encoded)
{
    if (!yBelowP(encoded)) {
        return false;
    }
    gf t, chk, num, den, den2, den4, den6;
    set25519(r[2], kOne);
    unpack25519(r[1], encoded);
    sq25519(num, r[1]);
    mul25519(den, num, kD);
    sub25519(num, num, r[2]);
    add25519(den, r[2], den);

    sq25519(den2, den);
    sq25519(den4, den2);
    mul25519(den6, den4, den2);
    mul25519(t, den6, num);
    mul25519(t, t, den);

    pow2523(t, t);
    mul25519(t, t, num);
    mul25519(t, t, den);
    mul25519(t, t, den);
    mul25519(r[0], t, den);

    sq25519(chk, r[0]);
    mul25519(chk, chk, den);
    if (neq25519(chk, num)) {
        mul25519(r[0], r[0], kSqrtM1);
    }

    sq25519(chk, r[0]);
    mul25519(chk, chk, den);
    if (neq25519(chk, num)) {
        return false;
    }

    const int sign = encoded[31] >> 7;
    if (sign == 1 && isZero25519(r[0])) {
        return false;  // x = 0 has no negative form
    }
    if (par25519(r[0]) == sign) {
        sub25519(r[0], kZero, r[0]);
    }
    mul25519(r[3], r[0], r[1]);
    return true;
}

} // namespace

bool verify(const QByteArray &publicKey, const QByteArray &message, const QByteArray &signature)
{
    if (publicKey.size() != 32 || signature.size() != 64) {
        return false;
    }
    const auto *pk = reinterpret_cast<const u8 *>(publicKey.constData());
    const auto *sig = reinterpret_cast<const u8 *>(signature.constData());
    if (!scalarBelowL(sig + 32)) {
        return false;
    }
    gf p[4];
    gf q[4];
    if (!unpackneg(q, pk)) {
        return false;
    }

    QCryptographicHash sha(QCryptographicHash::Sha512);
    sha.addData(QByteArrayView(signature.constData(), 32));
    sha.addData(publicKey);
    sha.addData(message);
    const QByteArray digest = sha.result();
    u8 h[64];
    std::memcpy(h, digest.constData(), sizeof(h));
    reduce(h);

    scalarmult(p, q, h);      // [h](-A)
    scalarbase(q, sig + 32);  // [S]B
    add(p, q);                // [S]B - [h]A, which is R for a right signature
    u8 t[32];
    pack(t, p);
    return equal32(sig, t);
}

} // namespace vama::ed25519
