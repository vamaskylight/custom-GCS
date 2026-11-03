"""Make license keys and device codes with VGCS's own key rules, and write what VGCS says of them,
as checks for the app's C++ port (src/LicenseKey.h, src/Ed25519.h, row A9).

Everything here is made by the real vgcs/app/license_key.py and vgcs/app/ed25519.py, with a test key
pair (RFC 8032 test 1's secret), never VAMA's private key. The C++ test
(test/link/license_key_test.cc) checks the same signatures, device codes and keys, and must give
the same answers.

Run from the repo root after changing either side:

    python apk/custom/test/gen_license_vectors.py

It writes apk/custom/test/license_key_vectors.inc.
"""

from __future__ import annotations

import base64
import re
import sys
from datetime import date
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))

from vgcs.app import ed25519, license_key  # noqa: E402
from vgcs.app.license_key import Product  # noqa: E402

OUT = Path(__file__).with_name("license_key_vectors.inc")
APP_SOURCE = REPO / "apk" / "custom" / "src" / "LicenseKey.cc"

# Test key pairs (not VAMA's): RFC 8032 test 1's and test 2's secrets.
TEST_SECRET = bytes.fromhex("9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60")
OTHER_SECRET = bytes.fromhex("4ccd089b28ff96da9db6c346ec114e0f5b8a319f35aba624da8cf6ed4fb8a6fb")
TEST_PUBLIC = ed25519.public_key(TEST_SECRET)
ISSUED = date(2026, 11, 4)

# RFC 8032 section 7.1, tests 1 to 3 (as in VGCS's tests).
RFC8032 = [
    ("d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a", "",
     "e5564300c360ac729086e2cc806e828a84877f1eb8e5d974d873e065224901555fb8821590a33bacc61e39701cf9b46bd25bf5f0595bbe24655141438e7a100b"),
    ("3d4017c3e843895a92b70aa74d1b7ebc9c982ccf2ec4968cc0cd55f12af4660c", "72",
     "92a009a9f0d4cab8720e820b5f642540a2b27b5416503f8fb3762223ebdb69da085ac1e43e15996e458f3613d0f11d8c387b2eaeb4302aeeb00d291612bb0c00"),
    ("fc51cd8e6218a1a38da47ed00230f0580816ed13ba3303ac5deb911548908025", "af82",
     "6291d657deec24024827e69c3abe01a30ce548a284743a445e3680d7db5ac3ac18ff9b538d16f290ae67f760984dc6594a7c15e9716ed28dc027beceea1ec40a"),
]

# Python's names of the states, and the C++ ones.
STATUS = {
    "ok": "Ok", "missing": "Missing", "damaged": "Damaged", "invalid": "Invalid",
    "wrong_machine": "WrongDevice", "expired": "Expired", "no_machine_id": "NoDeviceId",
    "other_product": "OtherProduct",
}


def cstr(text: str) -> str:
    """A C++ string literal; characters outside ASCII as octal escapes of their UTF-8 bytes."""
    out = []
    for byte in text.encode("utf-8"):
        ch = chr(byte)
        if ch == "\\":
            out.append("\\\\")
        elif ch == '"':
            out.append('\\"')
        elif ch == "\n":
            out.append("\\n")
        elif 32 <= byte < 127:
            out.append(ch)
        else:
            out.append(f"\\{byte:03o}")
    return '"' + "".join(out) + '"'


def signature_cases() -> list[tuple[str, str, str, bool, str]]:
    cases = []
    for i, (public, message, signature) in enumerate(RFC8032, 1):
        cases.append((public, message, signature, True, f"RFC 8032 test {i}"))
    message = b"VAMA app".hex()
    signature = ed25519.sign(TEST_SECRET, bytes.fromhex(message))
    pub = TEST_PUBLIC.hex()
    other = ed25519.public_key(OTHER_SECRET).hex()
    big_s = signature[:32] + (int.from_bytes(signature[32:], "little") + ed25519._L).to_bytes(32, "little")
    flipped_r = bytes([signature[0] ^ 1]) + signature[1:]
    flipped_s = signature[:40] + bytes([signature[40] ^ 4]) + signature[41:]
    y_is_p = "ed" + "ff" * 30 + "7f"           # y = p: not below p
    minus_zero = "01" + "00" * 30 + "80"       # y = 1, x = 0 with the sign bit set
    not_a_point = next(y.to_bytes(32, "little").hex() for y in range(2, 100)
                       if ed25519._point_decompress(y.to_bytes(32, "little")) is None)  # a y with no x
    raw = [
        (pub, message, signature.hex(), "made by VGCS's ed25519.py"),
        (pub, message + "21", signature.hex(), "the message changed"),
        (pub, message, flipped_r.hex(), "a bit of R changed"),
        (pub, message, flipped_s.hex(), "a bit of S changed"),
        (pub, message, big_s.hex(), "S plus the group order"),
        (other, message, signature.hex(), "another public key"),
        (y_is_p, message, signature.hex(), "a public key with y = p"),
        (minus_zero, message, signature.hex(), "a public key with x = 0 and the sign bit set"),
        (not_a_point, message, signature.hex(), "a public key that is no point"),
        (pub, message, signature.hex()[:-2], "a signature one byte short"),
    ]
    raw += crafted_signatures()
    for public, msg, sig, why in raw:
        ok = ed25519.verify(bytes.fromhex(public), bytes.fromhex(msg), bytes.fromhex(sig))
        cases.append((public, msg, sig, ok, why))
    assert [c[3] for c in cases] == [True] * 4 + [False] * 11, "VGCS's ed25519.py changed its answers"
    return cases


def crafted_signatures() -> list[tuple[str, str, str, str]]:
    """Two signatures that only a check without VGCS's two point rules would take.

    Without them the checks that refuse such public keys could be lost unseen: an ordinary
    signature fails on those keys anyway.
    """
    msg = b"VAMA app"
    base, p = ed25519._G, ed25519._P

    def neg(q):
        return ((-q[0]) % p, q[1], q[2], (-q[3]) % p)

    # x = 0 with the sign bit set, read as the neutral point: then R = [s]B passes for any message.
    minus_zero = bytes.fromhex("01" + "00" * 30 + "80")
    s = 12345
    r = ed25519._point_compress(ed25519._point_mul(s, base))
    first = (minus_zero.hex(), msg.hex(), (r + s.to_bytes(32, "little")).hex(),
             "R = [S]B with the neutral point's minus-zero form as the key")
    # A point of order 4 (y = 0) written with y = p: a signature made for those very bytes.
    small = ed25519._point_decompress(bytes(32))
    y_is_p = p.to_bytes(32, "little")
    for s in range(1, 1000):
        s_b = ed25519._point_mul(s, base)
        for k in range(4):
            r_point = ed25519._point_add(s_b, neg(ed25519._point_mul(k, small)))
            r = ed25519._point_compress(r_point)
            h = ed25519._sha512_mod_l(r + y_is_p + msg)
            if ed25519._point_equal(ed25519._point_mul(h, small), ed25519._point_mul(k, small)):
                second = (y_is_p.hex(), msg.hex(), (r + s.to_bytes(32, "little")).hex(),
                          "a signature for a key of order 4 written with y = p")
                for public, message, signature, _why in (first, second):
                    assert lenient_verify(bytes.fromhex(public), bytes.fromhex(message), bytes.fromhex(signature))
                return [first, second]
    raise SystemExit("no signature found for the key with y = p")


def lenient_verify(public: bytes, message: bytes, signature: bytes) -> bool:
    """The check without the two point rules (y taken modulo p, x = 0 kept with its sign bit)."""
    p = ed25519._P
    y = (int.from_bytes(public, "little") & ((1 << 255) - 1)) % p
    x2 = (y * y - 1) * pow(ed25519._D * y * y + 1, p - 2, p) % p
    x = pow(x2, (p + 3) // 8, p)
    if (x * x - x2) % p:
        x = x * ed25519._SQRT_M1 % p
    if (x * x - x2) % p:
        return False
    if x and (x & 1) != public[31] >> 7:
        x = p - x
    a_point = (x, y, 1, x * y % p)
    r_point = ed25519._point_decompress(signature[:32])
    s = int.from_bytes(signature[32:], "little")
    h = ed25519._sha512_mod_l(signature[:32] + public + message)
    return r_point is not None and ed25519._point_equal(
        ed25519._point_mul(s, ed25519._G), ed25519._point_add(r_point, ed25519._point_mul(h, a_point)))


ANDROID_IDS = ["9774d56d682e549c", "0123456789abcdef", "  A1B2C3D4E5F60718 \n", "f", ""]


def device_cases() -> list[tuple[str, str, str]]:
    cases = []
    for android_id in ANDROID_IDS:
        fingerprint = license_key.app_fingerprint(android_id)
        code = license_key.machine_code(fingerprint, Product.APP) if fingerprint else ""
        cases.append((android_id, fingerprint.hex() if fingerprint else "", code))
    return cases


HERE = license_key.app_fingerprint("9774d56d682e549c")
ELSEWHERE = license_key.app_fingerprint("0123456789abcdef")


def code_cases() -> list[tuple[str, str]]:
    code = license_key.machine_code(HERE, Product.APP)
    typed = code.lower().replace("-", " ").replace("o", "0").replace("i", "1").replace("b", "8")
    changed = ("B" if code[0] != "B" else "C") + code[1:]
    texts = [
        code,
        typed,
        "  " + code.replace("-", "") + "\t",
        changed,
        code[:-1],
        code + "A",
        "",
        "----",
        license_key.machine_code(HERE, Product.VGCS),  # VGCS's machine code of the same 9 bytes
        code + "é",                                 # a letter outside ASCII
        code.replace("A", "9", 1) if "A" in code else code + "9",  # 9 is no base32 character
    ]
    cases = []
    for text in texts:
        fingerprint = license_key.parse_machine_code(text, Product.APP)
        cases.append((text, fingerprint.hex() if fingerprint else ""))
    assert cases[0][1] == HERE.hex() and cases[1][1] == HERE.hex() and cases[3][1] == ""
    return cases


def key_cases() -> list[dict]:
    def app_key(fingerprint=HERE, secret=TEST_SECRET, **kwargs):
        kwargs.setdefault("serial", 7)
        kwargs.setdefault("issued", ISSUED)
        return license_key.make_key(secret, fingerprint, product=Product.APP, **kwargs)

    plain = app_key()
    ending = app_key(serial=8, expires=date(2027, 3, 31), features=0x35)
    vgcs = license_key.make_key(TEST_SECRET, HERE, serial=9, issued=ISSUED)
    i = plain.index("-") + 1
    changed = plain[:i] + ("A" if plain[i] != "A" else "B") + plain[i + 1:]
    raw = license_key._b32decode(license_key._normalise(plain))
    relabelled = base64.b32encode(bytes([license_key.KEY_VERSION]) + raw[1:]).decode().rstrip("=")
    unknown = base64.b32encode(bytes([0x22]) + raw[1:]).decode().rstrip("=")
    look_alikes = plain.replace("O", "0").replace("I", "1").replace("B", "8")
    rows = [
        ("a key for this RC", plain, HERE, ISSUED),
        ("with an end date, on its last day", ending, HERE, date(2027, 3, 31)),
        ("with an end date, the day after", ending, HERE, date(2027, 4, 1)),
        ("a key for another RC", app_key(ELSEWHERE, serial=3), HERE, ISSUED),
        ("signed by someone else", app_key(secret=OTHER_SECRET, serial=1), HERE, ISSUED),
        ("signed by someone else, for another RC", app_key(ELSEWHERE, secret=OTHER_SECRET, serial=1), HERE, ISSUED),
        ("a VGCS key", vgcs, HERE, ISSUED),
        ("a VGCS key signed by someone else",
         license_key.make_key(OTHER_SECRET, HERE, serial=9, issued=ISSUED), HERE, ISSUED),
        ("an app key with VGCS's first byte", relabelled, HERE, ISSUED),
        ("an unknown first byte", unknown, HERE, ISSUED),
        ("one character changed", changed, HERE, ISSUED),
        ("six characters short", plain[:-6], HERE, ISSUED),
        ("one character short", plain[:-1], HERE, ISSUED),
        ("not a key", "hello", HERE, ISSUED),
        ("nothing but spaces", "   ", HERE, ISSUED),
        ("this RC's ID unknown", plain, None, ISSUED),
        ("pasted with new lines, in lower case", "\n  " + plain.lower().replace("-", "\n") + "  \n", HERE, ISSUED),
        ("typed with 0, 1 and 8", look_alikes, HERE, ISSUED),
        ("with a letter outside ASCII", plain + "é", HERE, ISSUED),
    ]
    cases = []
    for why, text, fingerprint, today in rows:
        result = license_key.check_key(text, fingerprint, today=today, public_key=TEST_PUBLIC, product=Product.APP)
        info = result.info
        cases.append({
            "why": why,
            "key": text,
            "fingerprint": fingerprint.hex() if fingerprint else "",
            "today": today.isoformat(),
            "status": STATUS[result.status.value],
            "serial": info.serial if info else 0,
            "issued": info.issued.isoformat() if info else "",
            "expires": info.expires.isoformat() if info and info.expires else "",
            "features": info.features if info else 0,
            "for": info.fingerprint.hex() if info else "",
        })
    wanted = ["Ok", "Ok", "Expired", "WrongDevice", "Invalid", "Invalid", "OtherProduct", "Invalid", "Invalid",
              "Damaged"]
    assert [c["status"] for c in cases[:10]] == wanted, [c["status"] for c in cases[:10]]
    return cases


def check_app_public_key() -> None:
    text = APP_SOURCE.read_text(encoding="utf-8")
    found = re.search(r'kLicensePublicKeyHex\[\]\s*=\s*"([0-9a-f]{64})"', text)
    if not found or bytes.fromhex(found.group(1)) != license_key.LICENSE_PUBLIC_KEY:
        raise SystemExit(f"{APP_SOURCE} does not hold VGCS's LICENSE_PUBLIC_KEY")


def main() -> None:
    check_app_public_key()
    lines = [
        "// Generated by gen_license_vectors.py from vgcs/app/license_key.py and vgcs/app/ed25519.py. Do not edit.",
        "// The keys are made with a test key pair (RFC 8032 test 1's secret), never VAMA's.",
        "// Include LicenseKey.h first (test/link/license_key_test.cc, and the screen preview's lock steps).",
        "",
        "struct SignatureCase",
        "{",
        "    const char *publicKey;",
        "    const char *message;",
        "    const char *signature;",
        "    bool ok;",
        "    const char *why;",
        "};",
        "",
        "struct DeviceCase",
        "{",
        "    const char *androidId;",
        "    const char *fingerprint;",
        "    const char *code;",
        "};",
        "",
        "struct CodeCase",
        "{",
        "    const char *text;",
        "    const char *fingerprint;",
        "};",
        "",
        "struct KeyCase",
        "{",
        "    const char *why;",
        "    const char *key;",
        "    const char *fingerprint;  // this RC's, empty: its ID unknown",
        "    const char *today;",
        "    vama::license::Status status;",
        "    int serial;",
        "    const char *issued;",
        "    const char *expires;",
        "    int features;",
        "    const char *forFingerprint;  // the RC the key is for",
        "};",
        "",
        "// VGCS's public key, which the app holds too (LICENSE_PUBLIC_KEY), and the test public key.",
        f"const char kVgcsPublicKeyHex[] = {cstr(license_key.LICENSE_PUBLIC_KEY.hex())};",
        f"const char kTestPublicKeyHex[] = {cstr(TEST_PUBLIC.hex())};",
        "",
        "// Signatures: public key, message, signature (hex), whether VGCS's ed25519.py accepts it, and why.",
        "const SignatureCase kSignatures[] = {",
    ]
    for public, message, signature, ok, why in signature_cases():
        lines.append(f"    {{{cstr(public)}, {cstr(message)},")
        lines.append(f"     {cstr(signature)},")
        lines.append(f"     {'true' if ok else 'false'}, {cstr(why)}}},")
    lines += ["};", "", "// Android IDs: the fingerprint (hex) and the device code VGCS's rules give them.",
              "const DeviceCase kDevices[] = {"]
    for android_id, fingerprint, code in device_cases():
        lines.append(f"    {{{cstr(android_id)}, {cstr(fingerprint)}, {cstr(code)}}},")
    lines += ["};", "", "// Device codes as typed, and the fingerprint VGCS finds in them (empty: refused).",
              "const CodeCase kCodes[] = {"]
    for text, fingerprint in code_cases():
        lines.append(f"    {{{cstr(text)}, {cstr(fingerprint)}}},")
    lines += ["};", "",
              "// Keys checked by VGCS's check_key for the app, with the test public key: the key, this RC's",
              "// fingerprint (empty: no ID), today, and the answer (state, serial, issued, end, features, for).",
              "const KeyCase kKeys[] = {"]
    for c in key_cases():
        lines.append(f"    {{{cstr(c['why'])},")
        lines.append(f"     {cstr(c['key'])},")
        lines.append(f"     {cstr(c['fingerprint'])}, {cstr(c['today'])}, vama::license::Status::{c['status']},")
        lines.append(f"     {c['serial']}, {cstr(c['issued'])}, {cstr(c['expires'])}, {c['features']}, {cstr(c['for'])}}},")
    lines += ["};", ""]
    OUT.write_text("\n".join(lines), encoding="utf-8", newline="\n")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
