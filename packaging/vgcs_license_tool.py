"""Make license keys for VGCS and for the Vama GCS app. For whoever issues keys; never shipped.

From the repo root:

    py -3.14 packaging/vgcs_license_tool.py new-keypair
    py -3.14 packaging/vgcs_license_tool.py make-key ABCD-EFGH-IJKL-MNOP --note "Pune lab laptop"
    py -3.14 packaging/vgcs_license_tool.py make-key ABCD-EFGH-IJKL-MNOP --expires 2027-03-31
    py -3.14 packaging/vgcs_license_tool.py make-key ABCD-EFGH-IJKL-MNOP --product app --note "RC 1, test builds"
    py -3.14 packaging/vgcs_license_tool.py check-key KEY --machine ABCD-EFGH-IJKL-MNOP
    py -3.14 packaging/vgcs_license_tool.py this-machine

A key for VGCS is made from the laptop's machine code (VGCS shows it). A key for
the app (--product app) is made from the RC's device code, which the app shows
when it opens without a key. An RC has one device code for test builds and
another for customer builds, so it needs a key for each kind it runs. Each key
names its product: a VGCS key never opens the app, nor the other way round.

The private key is %USERPROFILE%\\.vama\\vgcs-license-private.key (or --key-file).
It makes the keys of both. Keep it outside the repo, with a backup: without it
no new keys can be made for the copies already given out, and anyone who has it
can make keys.

Every key made is added to %USERPROFILE%\\.vama\\vgcs-licenses-issued.csv
(date, product, serial, code, end date, note, key), and serials count up
from that list.

The key rules are in vgcs/app/license_key.py (the app's C++ copy:
apk/custom/src/LicenseKey.cc).
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import secrets
import sys
from datetime import date
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from vgcs.app import ed25519, license_key  # noqa: E402
from vgcs.app.license_key import Product  # noqa: E402

VAMA_DIR = Path.home() / ".vama"
DEFAULT_KEY_FILE = VAMA_DIR / "vgcs-license-private.key"
ISSUED_LOG = VAMA_DIR / "vgcs-licenses-issued.csv"
LOG_FIELDS = ["issued", "product", "serial", "machine_code", "expires", "note", "key"]
# The app's public key, in its C++ copy of the key rules.
APP_KEY_SOURCE = REPO / "apk" / "custom" / "src" / "LicenseKey.cc"

PRODUCT_WORDS = {
    Product.VGCS: "VGCS on a laptop (machine code)",
    Product.APP: "the Vama GCS app on an RC (device code)",
}


def app_public_key() -> bytes | None:
    """The public key built into the app, read from its source, or None when the source is not here."""
    try:
        text = APP_KEY_SOURCE.read_text(encoding="utf-8")
    except OSError:
        return None
    found = re.search(r'kLicensePublicKeyHex\[\]\s*=\s*"([0-9a-f]{64})"', text)
    return bytes.fromhex(found.group(1)) if found else None


def load_secret(path: Path) -> bytes:
    try:
        text = path.read_text(encoding="ascii").strip()
    except OSError:
        raise SystemExit(f"No private key at {path}. Make one with: new-keypair")
    secret = bytes.fromhex(text)
    if len(secret) != 32:
        raise SystemExit(f"{path} is not a 32-byte private key")
    return secret


def next_serial(log: Path) -> int:
    if not log.is_file():
        return 1
    with log.open(newline="", encoding="utf-8") as f:
        serials = [int(row["serial"]) for row in csv.DictReader(f) if str(row.get("serial", "")).isdigit()]
    return max(serials, default=0) + 1


def append_log(log: Path, row: dict) -> None:
    log.parent.mkdir(parents=True, exist_ok=True)
    if log.is_file():
        with log.open(newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            old_fields = reader.fieldnames or []
            old_rows = list(reader)
        if "product" not in old_fields:
            # A list from before the app's keys: every key in it is a VGCS key. Written anew
            # beside it first, so a failure leaves the old list as it was.
            fresh = log.with_name(log.name + ".new")
            with fresh.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=LOG_FIELDS, extrasaction="ignore")
                writer.writeheader()
                for old in old_rows:
                    writer.writerow({**old, "product": Product.VGCS.value})
            os.replace(fresh, log)
    new = not log.is_file()
    with log.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=LOG_FIELDS)
        if new:
            writer.writeheader()
        writer.writerow(row)


def cmd_new_keypair(args) -> int:
    path = Path(args.key_file)
    if path.exists():
        print(f"{path} already exists. A new key pair would make every key given out so far useless.")
        print("Delete or move that file first only if that is really wanted.")
        return 1
    path.parent.mkdir(parents=True, exist_ok=True)
    secret = secrets.token_bytes(32)
    path.write_text(secret.hex() + "\n", encoding="ascii")
    public = ed25519.public_key(secret)
    print(f"Private key written to {path}. Keep a backup somewhere safe, outside the repo.")
    print("Put this line in vgcs/app/license_key.py:")
    print(f'LICENSE_PUBLIC_KEY = bytes.fromhex("{public.hex()}")')
    return 0


def cmd_make_key(args) -> int:
    product = Product(args.product)
    secret = load_secret(Path(args.key_file))
    public = ed25519.public_key(secret)
    if public != license_key.LICENSE_PUBLIC_KEY and not args.force:
        print("This private key does not match LICENSE_PUBLIC_KEY in vgcs/app/license_key.py,")
        print("so VGCS and the app would refuse keys made with it. Nothing made.")
        return 1
    if product is Product.APP:
        in_app = app_public_key()
        if in_app is not None and in_app != public and not args.force:
            print(f"This private key does not match the app's public key in {APP_KEY_SOURCE},")
            print("so the app would refuse keys made with it. Nothing made.")
            return 1
    fingerprint = license_key.parse_machine_code(args.machine_code, product)
    if fingerprint is None:
        other = Product.APP if product is Product.VGCS else Product.VGCS
        print(f"{args.machine_code!r} is not a valid code for {PRODUCT_WORDS[product]} (a typing error?). Nothing made.")
        if license_key.parse_machine_code(args.machine_code, other) is not None:
            print(f"It is a valid code for {PRODUCT_WORDS[other]}: add --product {other.value}.")
        return 1
    log = Path(args.log)
    serial = args.serial if args.serial else next_serial(log)
    expires = date.fromisoformat(args.expires) if args.expires else None
    issued = date.today()
    key = license_key.make_key(secret, fingerprint, serial=serial, issued=issued, expires=expires, product=product)
    # Check the new key exactly as the app will.
    result = license_key.check_key(key, fingerprint, today=issued, public_key=public, product=product)
    if not result.ok:
        print(f"The new key failed its own check ({result.status.value}). Nothing made.")
        return 1
    code = license_key.machine_code(fingerprint, product)
    append_log(log, {
        "issued": issued.isoformat(),
        "product": product.value,
        "serial": serial,
        "machine_code": code,
        "expires": expires.isoformat() if expires else "",
        "note": args.note or "",
        "key": key,
    })
    print(f"For: {PRODUCT_WORDS[product]}")
    print(f"Code: {code}")
    print(f"Serial {serial}, {'ends ' + expires.isoformat() if expires else 'no end date'}")
    print()
    print(key)
    return 0


def cmd_check_key(args) -> int:
    product = license_key.key_product(args.key) or Product.VGCS
    fingerprint = license_key.parse_machine_code(args.machine, product)
    if fingerprint is None:
        print(f"{args.machine!r} is not a valid code for {PRODUCT_WORDS[product]}.")
        return 1
    result = license_key.check_key(args.key, fingerprint, product=product)
    print(f"For: {PRODUCT_WORDS[product]}")
    print(f"Result: {result.status.value}")
    if result.info is not None:
        print(f"Serial {result.info.serial}, issued {result.info.issued.isoformat()}, "
              f"{'ends ' + result.info.expires.isoformat() if result.info.expires else 'no end date'}")
    return 0 if result.ok else 1


def cmd_this_machine(_args) -> int:
    fingerprint = license_key.machine_fingerprint()
    if fingerprint is None:
        print("This computer's ID cannot be read.")
        return 1
    print(license_key.machine_code(fingerprint))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Make and check license keys for VGCS and the Vama GCS app.")
    parser.add_argument("--key-file", default=str(DEFAULT_KEY_FILE), help="private key file")
    parser.add_argument("--log", default=str(ISSUED_LOG), help="list of issued keys (CSV)")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("new-keypair", help="make the private key (once)").set_defaults(run=cmd_new_keypair)

    make = sub.add_parser("make-key", help="make a key for a laptop's machine code or an RC's device code")
    make.add_argument("machine_code", help="the machine code VGCS shows, or with --product app the device code the app shows")
    make.add_argument("--product", choices=[p.value for p in Product], default=Product.VGCS.value,
                      help="vgcs (default): VGCS on a laptop; app: the Vama GCS app on an RC")
    make.add_argument("--serial", type=int, default=0, help="default: the next number in the list")
    make.add_argument("--expires", default="", help="end date, YYYY-MM-DD (default: none)")
    make.add_argument("--note", default="", help="who, or which laptop or RC, for the list")
    make.add_argument("--force", action="store_true", help="make it even if the app's public key differs")
    make.set_defaults(run=cmd_make_key)

    check = sub.add_parser("check-key", help="check a key against a machine code or device code")
    check.add_argument("key")
    check.add_argument("--machine", required=True, help="the machine code or device code the key is for")
    check.set_defaults(run=cmd_check_key)

    sub.add_parser("this-machine", help="show this computer's machine code").set_defaults(run=cmd_this_machine)

    args = parser.parse_args(argv)
    return args.run(args)


if __name__ == "__main__":
    raise SystemExit(main())
