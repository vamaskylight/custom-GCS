<#
.SYNOPSIS
Makes the release signing key for the VAMA APK. Run it once.

.DESCRIPTION
Customer APKs are signed with this key (apk\build_android.ps1 -Customer).
Android installs an update only over an app that was signed with the same key.
So the key must never be lost, never be made twice and never be given away:

  - it is made outside the repo, in %USERPROFILE%\.vama (beside the VGCS license key),
  - its password is random and sits in a second file beside it,
  - the script stops if one of the two files is already there.

Back up both files as soon as they are made. Without them no later version can be
installed over a released app: every customer would have to uninstall first.

.EXAMPLE
powershell -ExecutionPolicy Bypass -File apk\make_release_key.ps1
#>
[CmdletBinding()]
param(
    [string]$OutDir = (Join-Path $env:USERPROFILE '.vama'),
    [string]$JavaHome = ''
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Fail([string]$Message) {
    Write-Host "ERROR: $Message" -ForegroundColor Red
    exit 1
}

if (-not $JavaHome) {
    $jdk = Get-ChildItem -Directory -Path (Join-Path $env:LOCALAPPDATA 'Programs\Eclipse Adoptium') -Filter 'jdk-21*' -ErrorAction SilentlyContinue |
        Sort-Object Name -Descending | Select-Object -First 1
    if ($jdk) { $JavaHome = $jdk.FullName }
}
$keytool = if ($JavaHome) { Join-Path $JavaHome 'bin\keytool.exe' } else { '' }
if (-not $keytool -or -not (Test-Path -LiteralPath $keytool)) { Fail 'JDK 21 not found. Pass -JavaHome.' }

$keystore = Join-Path $OutDir 'vama-release.keystore'
$passFile = "$keystore.pass"
foreach ($f in $keystore, $passFile) {
    if (Test-Path -LiteralPath $f) {
        Fail "$f is already there. The release key is made once and never again. (To look at it: keytool -list -v -keystore `"$keystore`")"
    }
}
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

# 32 letters and digits from the system's random source.
$alphabet = 'ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnopqrstuvwxyz23456789'
$random = [System.Security.Cryptography.RandomNumberGenerator]::Create()
$bytes = New-Object byte[] 64
$password = ''
while ($password.Length -lt 32) {
    $random.GetBytes($bytes)
    foreach ($b in $bytes) {
        # Bytes past the last full round of the alphabet are dropped, so every sign is as likely as the others.
        if ($b -lt 256 - (256 % $alphabet.Length) -and $password.Length -lt 32) { $password += $alphabet[$b % $alphabet.Length] }
    }
}
[System.IO.File]::WriteAllText($passFile, $password, (New-Object System.Text.ASCIIEncoding))

# The password goes to keytool through the environment, not on its command line.
$env:VAMA_RELEASE_KEY_NEW = $password
$saved = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
try {
    & $keytool -genkeypair -keystore $keystore -storetype PKCS12 -alias vama `
        -storepass:env VAMA_RELEASE_KEY_NEW -keyalg RSA -keysize 4096 -validity 10000 `
        -dname 'CN=VAMA GCS,O=VAMA,C=IN' 2>&1 | ForEach-Object { "$_" } | Out-Host
    $code = $LASTEXITCODE
    $listing = @()
    if ($code -eq 0) {
        $listing = @(& $keytool -list -v -keystore $keystore -alias vama -storepass:env VAMA_RELEASE_KEY_NEW 2>&1 | ForEach-Object { "$_" })
        if ($LASTEXITCODE -ne 0) { $code = $LASTEXITCODE }
    }
} finally {
    $ErrorActionPreference = $saved
    Remove-Item Env:\VAMA_RELEASE_KEY_NEW -ErrorAction SilentlyContinue
}
if ($code -ne 0 -or -not (Test-Path -LiteralPath $keystore)) {
    Remove-Item -LiteralPath $passFile -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $keystore -ErrorAction SilentlyContinue
    Fail 'keytool failed. Nothing was kept.'
}

Write-Host ''
Write-Host "Release key:  $keystore"
Write-Host "Its password: $passFile"
$listing | Where-Object { $_ -match 'SHA256:|Valid from|Owner:' } | ForEach-Object { Write-Host "  $($_.Trim())" }
Write-Host ''
Write-Host 'BACK UP BOTH FILES NOW, in a place that is not this PC.' -ForegroundColor Yellow
Write-Host 'Without them no later version can be installed over a released app.'
Write-Host 'Never put them in the repo and never send them to anyone.'
