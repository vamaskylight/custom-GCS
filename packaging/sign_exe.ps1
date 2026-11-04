<#
.SYNOPSIS
Signs one exe (Authenticode, SHA-256, with a time stamp) and checks the signature. build_exe.py runs it.

.DESCRIPTION
The certificate comes from the Windows certificate store (-Thumbprint; a certificate on a USB token shows there
too, and the token asks for its PIN), or from a .pfx file (-PfxPath; its password in the environment variable
VGCS_SIGN_PFX_PASSWORD, never on a command line). A .pfx is only read into memory: the certificate store is not
changed.

Windows PowerShell's Set-AuthenticodeSignature writes the older Authenticode time stamp, not an RFC 3161 one
(found 2026-11-05). Windows accepts both, and either keeps the signature valid after the certificate has ended.

-CheckOnly: only check that the certificate can sign code, before the long build.
-AllowUntrusted: for a test certificate only: a signature that Windows does not trust does not stop the build.
-TimestampUrl none: no time stamp (the tests, which never reach the internet).
#>
[CmdletBinding()]
param(
    [string]$Path = '',
    [string]$Thumbprint = '',
    [string]$PfxPath = '',
    [string]$TimestampUrl = 'http://timestamp.digicert.com',
    [switch]$AllowUntrusted,
    [switch]$CheckOnly
)
$ErrorActionPreference = 'Stop'
$codeSigningUse = '1.3.6.1.5.5.7.3.3'

if ($Thumbprint) {
    # A thumbprint copied from Windows' certificate window can hold spaces and an invisible mark.
    $clean = ($Thumbprint -replace '[^0-9A-Fa-f]', '').ToUpperInvariant()
    $cert = Get-ChildItem -Path Cert:\CurrentUser\My, Cert:\LocalMachine\My |
        Where-Object { $_.Thumbprint -eq $clean } | Select-Object -First 1
    if (-not $cert) {
        throw "No certificate with the thumbprint $clean in Cert:\CurrentUser\My or Cert:\LocalMachine\My. " +
              'List the code signing ones: Get-ChildItem Cert:\CurrentUser\My -CodeSigningCert'
    }
} elseif ($PfxPath) {
    $password = $env:VGCS_SIGN_PFX_PASSWORD
    if ($null -eq $password) { throw 'Put the password of the .pfx file in the environment variable VGCS_SIGN_PFX_PASSWORD.' }
    $cert = New-Object System.Security.Cryptography.X509Certificates.X509Certificate2($PfxPath, $password)
} else {
    throw 'Give -Thumbprint or -PfxPath.'
}

if (-not $cert.HasPrivateKey) {
    throw "The certificate '$($cert.Subject)' has no private key here (a USB token: plug it in, with its driver installed)."
}
$uses = @($cert.Extensions |
    Where-Object { $_ -is [System.Security.Cryptography.X509Certificates.X509EnhancedKeyUsageExtension] } |
    ForEach-Object { $_.EnhancedKeyUsages } | ForEach-Object { $_.Value })
if ($uses -notcontains $codeSigningUse) {
    throw "The certificate '$($cert.Subject)' is not for code signing (its uses: $($uses -join ', '))."
}
if ($cert.NotAfter -lt (Get-Date)) {
    throw "The certificate '$($cert.Subject)' ended on $($cert.NotAfter.ToString('yyyy-MM-dd'))."
}
if ($CheckOnly) {
    Write-Output "OK      certificate: $($cert.Subject), thumbprint $($cert.Thumbprint), until $($cert.NotAfter.ToString('yyyy-MM-dd'))"
    exit 0
}
if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) { throw "There is no file $Path to sign." }

$sign = @{ FilePath = $Path; Certificate = $cert; HashAlgorithm = 'SHA256' }
$stamped = $TimestampUrl -and $TimestampUrl -ne 'none'
if ($stamped) { $sign.TimestampServer = $TimestampUrl }
$null = Set-AuthenticodeSignature @sign

$check = Get-AuthenticodeSignature -FilePath $Path
if (-not $check.SignerCertificate -or $check.SignerCertificate.Thumbprint -ne $cert.Thumbprint) {
    throw "$Path is not signed with $($cert.Thumbprint) after signing: $($check.StatusMessage)"
}
if ($stamped -and -not $check.TimeStamperCertificate) {
    throw "$Path has no time stamp: did the server $TimestampUrl answer? Try again, or another server with --timestamp-url."
}
if ($check.Status -ne 'Valid' -and -not $AllowUntrusted) {
    throw "Windows does not trust the signature of ${Path}: $($check.StatusMessage)"
}
$stamper = if ($check.TimeStamperCertificate) { $check.TimeStamperCertificate.Subject } else { 'none' }
Write-Output "OK      signed $(Split-Path -Leaf $Path): $($cert.Subject), Windows says $($check.Status), time stamp: $stamper"
