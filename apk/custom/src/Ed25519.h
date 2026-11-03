#pragma once

#include <QtCore/QByteArray>

// VAMA (row A9): Ed25519 signature checks (RFC 8032) for the app's license key (LicenseKey.h).
// Checking only: keys are made on the issuer's PC by packaging/vgcs_license_tool.py.
namespace vama::ed25519 {

/// True only when `signature` (64 bytes) was made for `message` by the secret key of `publicKey`
/// (32 bytes). It gives the same answers as VGCS's vgcs/app/ed25519.py: a public key or an R that
/// is not a point of the curve, and an S that is not below the group order, are refused.
bool verify(const QByteArray &publicKey, const QByteArray &message, const QByteArray &signature);

} // namespace vama::ed25519
