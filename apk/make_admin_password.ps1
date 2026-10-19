<#
.SYNOPSIS
Makes the admin password for Advanced Mode in customer APKs (task Z). Run it once.

.DESCRIPTION
Advanced Mode, with the drone's parameters, asks for the admin password. A customer build
(apk\build_android.ps1 -Customer) puts this password into the app. Only a salted hash of it
is kept (PBKDF2-HMAC-SHA256 with 100000 rounds, as the app checks it), in %USERPROFILE%\.vama
beside the release key, never in the repo. The password itself is kept nowhere: keep it in a
safe place, as this file cannot give it back.

The password is asked twice. It has 10 signs or more: letters, digits and keyboard signs
such as - _ . ! @ #, without spaces, so it can be typed on the RC's keyboard.
(A script can give it in the environment, in VAMA_ADMIN_PASSWORD_NEW, and is not asked.)

A new password (-Replace) is in the next customer build. APKs made before keep the old one.
Test builds always have the test password of apk\custom\src\AdminPassword.cc.

.EXAMPLE
powershell -ExecutionPolicy Bypass -File apk\make_admin_password.ps1
#>
[CmdletBinding()]
param(
    [string]$OutDir = (Join-Path $env:USERPROFILE '.vama'),
    # A new password in place of the one made before.
    [switch]$Replace
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Fail([string]$Message) {
    Write-Host "ERROR: $Message" -ForegroundColor Red
    exit 1
}

function Read-Password([string]$Prompt) {
    $secure = Read-Host -Prompt $Prompt -AsSecureString
    $bstr = [System.Runtime.InteropServices.Marshal]::SecureStringToBSTR($secure)
    try {
        return [System.Runtime.InteropServices.Marshal]::PtrToStringBSTR($bstr)
    } finally {
        [System.Runtime.InteropServices.Marshal]::ZeroFreeBSTR($bstr)
    }
}

function ConvertTo-Hex([byte[]]$Bytes) {
    return ([System.BitConverter]::ToString($Bytes) -replace '-', '').ToLower()
}

$file = Join-Path $OutDir 'vama-admin-password.txt'
if ((Test-Path -LiteralPath $file) -and -not $Replace) {
    Fail "$file is already there. To make a new password, run again with -Replace."
}

if ($env:VAMA_ADMIN_PASSWORD_NEW) {
    $password = $env:VAMA_ADMIN_PASSWORD_NEW
} else {
    $password = Read-Password 'New admin password'
    $again = Read-Password 'The same password again'
    if ($password -cne $again) { Fail 'The two passwords are not the same. Nothing was kept.' }
}
if ($password.Length -lt 10) { Fail 'The password is too short: 10 signs or more. Nothing was kept.' }
if ($password -cnotmatch '^[\x21-\x7E]+$') {
    Fail 'Use only letters, digits and keyboard signs such as - _ . ! @ #, without spaces. Nothing was kept.'
}

# As the app checks it (AdminPassword.cc): PBKDF2-HMAC-SHA256 of the UTF-8 bytes, 32 bytes.
$iterations = 100000
$salt = New-Object byte[] 16
$random = [System.Security.Cryptography.RandomNumberGenerator]::Create()
$random.GetBytes($salt)
$bytes = [System.Text.Encoding]::UTF8.GetBytes($password)
$kdf = [System.Security.Cryptography.Rfc2898DeriveBytes]::new($bytes, $salt, $iterations, [System.Security.Cryptography.HashAlgorithmName]::SHA256)
try {
    $hash = $kdf.GetBytes(32)
} finally {
    $kdf.Dispose()
    [Array]::Clear($bytes, 0, $bytes.Length)
}

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$lines = @(
    '# The admin password for Advanced Mode in customer APKs, from apk\make_admin_password.ps1.',
    '# Its salted hash only, not the password. Never put this file in the repo.',
    "salt=$(ConvertTo-Hex $salt)",
    "hash=$(ConvertTo-Hex $hash)",
    "iterations=$iterations",
    "made=$((Get-Date).ToString('yyyy-MM-dd HH:mm'))"
)
[System.IO.File]::WriteAllLines($file, [string[]]$lines, (New-Object System.Text.ASCIIEncoding))

Write-Host "Admin password: $file (its salted hash only)"
Write-Host 'The next customer build puts it into the app (apk\build_android.ps1 -Customer).'
Write-Host 'Keep the password itself in a safe place. This file cannot give it back.' -ForegroundColor Yellow
