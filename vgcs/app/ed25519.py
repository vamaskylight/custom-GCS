"""Ed25519 signatures in plain Python (RFC 8032, section 5.1).

Used by the license key check (vgcs/app/license_key.py) and by the vendor's
key tool (packaging/vgcs_license_tool.py). Plain Python, so the exe needs no
extra package. It follows the RFC's reference algorithm and is checked
against the RFC's own test vectors (tests/test_license_key.py).

It is not constant time. That matters only for signing, which happens on
the vendor's computer, never in the app: the app only verifies.
"""

from __future__ import annotations

import hashlib

_P = 2**255 - 19
_L = 2**252 + 27742317777372353535851937790883648493
_D = -121665 * pow(121666, _P - 2, _P) % _P
_SQRT_M1 = pow(2, (_P - 1) // 4, _P)

# Points are in extended coordinates (X, Y, Z, T) with x = X/Z, y = Y/Z, x*y = T/Z.
_Point = tuple[int, int, int, int]


def _sha512_mod_l(data: bytes) -> int:
    return int.from_bytes(hashlib.sha512(data).digest(), "little") % _L


def _point_add(p: _Point, q: _Point) -> _Point:
    a = (p[1] - p[0]) * (q[1] - q[0]) % _P
    b = (p[1] + p[0]) * (q[1] + q[0]) % _P
    c = 2 * p[3] * q[3] * _D % _P
    d = 2 * p[2] * q[2] % _P
    e, f, g, h = b - a, d - c, d + c, b + a
    return (e * f % _P, g * h % _P, f * g % _P, e * h % _P)


def _point_mul(scalar: int, point: _Point) -> _Point:
    result: _Point = (0, 1, 1, 0)  # neutral element
    while scalar > 0:
        if scalar & 1:
            result = _point_add(result, point)
        point = _point_add(point, point)
        scalar >>= 1
    return result


def _point_equal(p: _Point, q: _Point) -> bool:
    # x1 / z1 == x2 / z2  <=>  x1 * z2 == x2 * z1
    if (p[0] * q[2] - q[0] * p[2]) % _P != 0:
        return False
    return (p[1] * q[2] - q[1] * p[2]) % _P == 0


def _recover_x(y: int, sign: int) -> int | None:
    if y >= _P:
        return None
    x2 = (y * y - 1) * pow(_D * y * y + 1, _P - 2, _P)
    if x2 == 0:
        return None if sign else 0
    x = pow(x2, (_P + 3) // 8, _P)
    if (x * x - x2) % _P != 0:
        x = x * _SQRT_M1 % _P
    if (x * x - x2) % _P != 0:
        return None
    if (x & 1) != sign:
        x = _P - x
    return x


_G_Y = 4 * pow(5, _P - 2, _P) % _P
_G_X = _recover_x(_G_Y, 0)
_G: _Point = (_G_X, _G_Y, 1, _G_X * _G_Y % _P)


def _point_compress(point: _Point) -> bytes:
    z_inv = pow(point[2], _P - 2, _P)
    x = point[0] * z_inv % _P
    y = point[1] * z_inv % _P
    return int.to_bytes(y | ((x & 1) << 255), 32, "little")


def _point_decompress(data: bytes) -> _Point | None:
    if len(data) != 32:
        return None
    y = int.from_bytes(data, "little")
    sign = y >> 255
    y &= (1 << 255) - 1
    x = _recover_x(y, sign)
    if x is None:
        return None
    return (x, y, 1, x * y % _P)


def _expand_secret(secret: bytes) -> tuple[int, bytes]:
    if len(secret) != 32:
        raise ValueError("an Ed25519 secret key is 32 bytes")
    h = hashlib.sha512(secret).digest()
    a = int.from_bytes(h[:32], "little")
    a &= (1 << 254) - 8
    a |= 1 << 254
    return a, h[32:]


def public_key(secret: bytes) -> bytes:
    """The 32-byte public key for a 32-byte secret key."""
    a, _prefix = _expand_secret(secret)
    return _point_compress(_point_mul(a, _G))


def sign(secret: bytes, message: bytes) -> bytes:
    """A 64-byte signature of the message."""
    a, prefix = _expand_secret(secret)
    public = _point_compress(_point_mul(a, _G))
    r = _sha512_mod_l(prefix + message)
    r_point = _point_compress(_point_mul(r, _G))
    h = _sha512_mod_l(r_point + public + message)
    s = (r + h * a) % _L
    return r_point + int.to_bytes(s, 32, "little")


def verify(public: bytes, message: bytes, signature: bytes) -> bool:
    """True only when the signature was made by the secret key of this public key."""
    if len(public) != 32 or len(signature) != 64:
        return False
    a_point = _point_decompress(public)
    if a_point is None:
        return False
    r_bytes = signature[:32]
    r_point = _point_decompress(r_bytes)
    if r_point is None:
        return False
    s = int.from_bytes(signature[32:], "little")
    if s >= _L:
        return False
    h = _sha512_mod_l(r_bytes + public + message)
    return _point_equal(_point_mul(s, _G), _point_add(r_point, _point_mul(h, a_point)))
