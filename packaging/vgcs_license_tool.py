"""Make VGCS license keys. For whoever issues keys; never shipped in the exe.

From the repo root:

    py -3.14 packaging/vgcs_license_tool.py new-keypair
    py -3.14 packaging/vgcs_license_tool.py make-key ABCD-EFGH-IJKL-MNOP --note "Pune lab laptop"
    py -3.14 packaging/vgcs_license_tool.py make-key ABCD-EFGH-IJKL-MNOP --expires 2027-03-31
    py -3.14 packaging/vgcs_license_tool.py check-key KEY --machine ABCD-EFGH-IJKL-MNOP
    py -3.14 packaging/vgcs_license_tool.py this-machine

The private key is %USERPROFILE%\\.vama\\vgcs-license-private.key (or --key-file).
Keep it outside the repo, with a backup: without it no new keys can be made
for the exes already given out, and anyone who has it can make keys.

Every key made is added to %USERPROFILE%\\.vama\\vgcs-licenses-issued.csv
(date, serial, machine code, end date, note, key), and serials count up
from that list.

The key rules are in vgcs/app/license_key.py.
"""

from __future__ import annotations

import argparse
import csv
import secrets
import sys
from datetime import date
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from vgcs.app import ed25519, license_key  # noqa: E402

VAMA_DIR = Path.home() / ".vama"
DEFAULT_KEY_FILE = VAMA_DIR / "vgcs-license-private.key"
ISSUED_LOG = VAMA_DIR / "vgcs-licenses-issued.csv"
LOG_FIELDS = ["issued", "serial", "machine_code", "expires", "note", "key"]


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
    secret = load_secret(Path(args.key_file))
    public = ed25519.public_key(secret)
    if public != license_key.LICENSE_PUBLIC_KEY and not args.force:
        print("This private key does not match LICENSE_PUBLIC_KEY in vgcs/app/license_key.py,")
        print("so the app would refuse keys made with it. Nothing made.")
        return 1
    fingerprint = license_key.parse_machine_code(args.machine_code)
    if fingerprint is None:
        print(f"{args.machine_code!r} is not a valid machine code (a typing error?). Nothing made.")
        return 1
    log = Path(args.log)
    serial = args.serial if args.serial else next_serial(log)
    expires = date.fromisoformat(args.expires) if args.expires else None
    issued = date.today()
    key = license_key.make_key(secret, fingerprint, serial=serial, issued=issued, expires=expires)
    # Check the new key exactly as the app will.
    result = license_key.check_key(key, fingerprint, today=issued, public_key=public)
    if not result.ok:
        print(f"The new key failed its own check ({result.status.value}). Nothing made.")
        return 1
    append_log(log, {
        "issued": issued.isoformat(),
        "serial": serial,
        "machine_code": license_key.machine_code(fingerprint),
        "expires": expires.isoformat() if expires else "",
        "note": args.note or "",
        "key": key,
    })
    print(f"Machine code: {license_key.machine_code(fingerprint)}")
    print(f"Serial {serial}, {'ends ' + expires.isoformat() if expires else 'no end date'}")
    print()
    print(key)
    return 0


def cmd_check_key(args) -> int:
    fingerprint = license_key.parse_machine_code(args.machine)
    if fingerprint is None:
        print(f"{args.machine!r} is not a valid machine code.")
        return 1
    result = license_key.check_key(args.key, fingerprint)
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
    parser = argparse.ArgumentParser(description="Make and check VGCS license keys.")
    parser.add_argument("--key-file", default=str(DEFAULT_KEY_FILE), help="private key file")
    parser.add_argument("--log", default=str(ISSUED_LOG), help="list of issued keys (CSV)")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("new-keypair", help="make the private key (once)").set_defaults(run=cmd_new_keypair)

    make = sub.add_parser("make-key", help="make a key for a machine code")
    make.add_argument("machine_code")
    make.add_argument("--serial", type=int, default=0, help="default: the next number in the list")
    make.add_argument("--expires", default="", help="end date, YYYY-MM-DD (default: none)")
    make.add_argument("--note", default="", help="who or which laptop, for the list")
    make.add_argument("--force", action="store_true", help="make it even if the app's public key differs")
    make.set_defaults(run=cmd_make_key)

    check = sub.add_parser("check-key", help="check a key against a machine code")
    check.add_argument("key")
    check.add_argument("--machine", required=True)
    check.set_defaults(run=cmd_check_key)

    sub.add_parser("this-machine", help="show this computer's machine code").set_defaults(run=cmd_this_machine)

    args = parser.parse_args(argv)
    return args.run(args)


if __name__ == "__main__":
    raise SystemExit(main())
