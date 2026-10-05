<#
.SYNOPSIS
Builds the VAMA APK (our QGC custom build) for Android.

.DESCRIPTION
Uses the same CMake settings as QGC's own Android CI on a Windows host
(apk/qgc-src/.github/workflows/android.yml): Qt for Android plus the
msvc2022_64 desktop kit as the host, Ninja, and a signed APK.

The repo path has a space ("My project") and Android builds create very deep
paths, so the build runs through a short folder without spaces:
  <WorkRoot>\src      directory link to apk\qgc-src
  <WorkRoot>\build-*  build output

Run apk\fetch_qgc.ps1 first. Tool versions are listed in apk\README.md.

.EXAMPLE
powershell -ExecutionPolicy Bypass -File apk\build_android.ps1
powershell -ExecutionPolicy Bypass -File apk\build_android.ps1 -Abis arm64-v8a
#>
[CmdletBinding()]
param(
    [string]$QtRoot = 'C:\Qt\6.11.1',
    [string]$AndroidSdk = $(if ($env:ANDROID_HOME) { $env:ANDROID_HOME } else { Join-Path $env:LOCALAPPDATA 'Android\Sdk' }),
    [string]$NdkVersion = '27.2.12479018',
    [string]$JavaHome = '',
    [string]$Abis = 'arm64-v8a;armeabi-v7a',
    [ValidateSet('Release', 'Debug')]
    [string]$BuildType = 'Release',
    [string]$WorkRoot = 'C:\vama-apk',
    [string]$Keystore = (Join-Path $env:USERPROFILE '.android\vama-dev.keystore'),
    [switch]$ConfigureOnly
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Fail([string]$Message) {
    Write-Host "ERROR: $Message" -ForegroundColor Red
    exit 1
}

# Native tools write progress to stderr, which Windows PowerShell 5.1 turns
# into errors under 'Stop'. Run them with 'Continue' and judge the exit code.
function Invoke-Native([string]$Exe, [string[]]$Arguments) {
    $saved = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        & $Exe @Arguments | Out-Host
        return $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $saved
    }
}

$ApkDir = $PSScriptRoot
$SrcReal = Join-Path $ApkDir 'qgc-src'
if (-not (Test-Path -LiteralPath (Join-Path $SrcReal 'CMakeLists.txt'))) {
    Fail "QGC source missing. Run apk\fetch_qgc.ps1 first."
}
if (-not (Test-Path -LiteralPath (Join-Path $SrcReal 'custom\CMakeLists.txt'))) {
    Fail "apk\qgc-src\custom is not linked. Run apk\fetch_qgc.ps1 first."
}

# --- Tools ---------------------------------------------------------------
$abiList = @($Abis.Split(';') | ForEach-Object { $_.Trim() } | Where-Object { $_ })
$qtDirFor = @{ 'arm64-v8a' = 'android_arm64_v8a'; 'armeabi-v7a' = 'android_armv7'; 'x86_64' = 'android_x86_64'; 'x86' = 'android_x86' }
foreach ($abi in $abiList) {
    if (-not $qtDirFor.ContainsKey($abi)) { Fail "Unknown ABI '$abi'." }
    $kit = Join-Path $QtRoot $qtDirFor[$abi]
    if (-not (Test-Path -LiteralPath $kit)) { Fail "Qt kit for $abi not found at $kit" }
}
# Qt finds the other ABI kits next to the first one, so the first ABI is the main kit.
$QtTarget = Join-Path $QtRoot $qtDirFor[$abiList[0]]
$QtHost = Join-Path $QtRoot 'msvc2022_64'
if (-not (Test-Path -LiteralPath (Join-Path $QtHost 'bin'))) { Fail "Qt desktop host kit not found at $QtHost" }

$Ndk = Join-Path $AndroidSdk "ndk\$NdkVersion"
if (-not (Test-Path -LiteralPath $Ndk)) { Fail "Android NDK $NdkVersion not found at $Ndk" }

if (-not $JavaHome) {
    $jdk = Get-ChildItem -Directory -Path (Join-Path $env:LOCALAPPDATA 'Programs\Eclipse Adoptium') -Filter 'jdk-21*' -ErrorAction SilentlyContinue |
        Sort-Object Name -Descending | Select-Object -First 1
    if ($jdk) { $JavaHome = $jdk.FullName }
}
if (-not $JavaHome -or -not (Test-Path -LiteralPath (Join-Path $JavaHome 'bin\java.exe'))) {
    Fail 'JDK 21 not found. Pass -JavaHome.'
}

# QGC's code generators (MAVLink, settings) run in apk\qgc-src\.venv.
# QGC finds that venv with "if(WIN32)", which is false when the target is
# Android, so it looks for a Linux-style bin/python and fails. Passing the
# interpreter ourselves skips that check. QGC also notes that Python 3.14
# crashes the MAVLink generator on Windows, so the venv must be 3.10 to 3.13.
$VenvPython = Join-Path $SrcReal '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $VenvPython)) {
    $bootPython = $null
    foreach ($v in '3.12', '3.13', '3.11', '3.10') {
        $p = & py "-$v" -c 'import sys; print(sys.executable)' 2>$null
        if ($LASTEXITCODE -eq 0 -and $p) { $bootPython = "$p".Trim(); break }
    }
    if (-not $bootPython) { Fail 'Python 3.10 to 3.13 is needed to create apk\qgc-src\.venv.' }
    Write-Host "Creating apk\qgc-src\.venv with $bootPython"
    if ((Invoke-Native $bootPython @((Join-Path $SrcReal 'tools\setup\install_python.py'), 'scripts')) -ne 0) {
        Fail 'Creating the Python venv failed.'
    }
}
# No quotes inside the Python code: Windows PowerShell 5.1 strips them when
# passing arguments to a program.
$venvVersion = "$(& $VenvPython -c 'import sys; print(*sys.version_info[:2], sep=chr(46))')".Trim()
if ([version]$venvVersion -ge [version]'3.14') {
    Fail "apk\qgc-src\.venv uses Python $venvVersion. Delete apk\qgc-src\.venv and run again (it is rebuilt with 3.12)."
}

# CMake and Ninja from the Android SDK (newest version).
$cmakeDir = Get-ChildItem -Directory -Path (Join-Path $AndroidSdk 'cmake') -ErrorAction SilentlyContinue |
    Sort-Object { [version]$_.Name } -Descending | Select-Object -First 1
if (-not $cmakeDir) { Fail 'CMake not found in the Android SDK.' }
$CmakeBin = Join-Path $cmakeDir.FullName 'bin'
$Cmake = Join-Path $CmakeBin 'cmake.exe'

# --- Short work folder ---------------------------------------------------
New-Item -ItemType Directory -Force -Path $WorkRoot | Out-Null
$SrcLink = Join-Path $WorkRoot 'src'
if (Test-Path -LiteralPath $SrcLink) {
    $item = Get-Item -LiteralPath $SrcLink -Force
    $ok = $false
    if ($item.LinkType -eq 'Junction' -or $item.LinkType -eq 'SymbolicLink') {
        foreach ($t in @($item.Target)) {
            if ([System.IO.Path]::GetFullPath($t).TrimEnd('\') -eq [System.IO.Path]::GetFullPath($SrcReal).TrimEnd('\')) { $ok = $true }
        }
    }
    if (-not $ok) { Fail "$SrcLink exists and is not a link to $SrcReal." }
} else {
    New-Item -ItemType Junction -Path $SrcLink -Target $SrcReal | Out-Null
    Write-Host "Linked $SrcLink -> $SrcReal"
}
$BuildDir = Join-Path $WorkRoot "build-$BuildType"

# --- Signing (development key) -------------------------------------------
# Test builds are signed with a development key kept outside the repo.
# The release key for customers is made separately and never committed.
$keystoreAlias = 'vamadev'
$keystorePass = if ($env:VAMA_KEYSTORE_PASS) { $env:VAMA_KEYSTORE_PASS } else { 'vamadev' }
if (-not (Test-Path -LiteralPath $Keystore)) {
    New-Item -ItemType Directory -Force -Path (Split-Path $Keystore) | Out-Null
    Write-Host "Creating development keystore $Keystore"
    $code = Invoke-Native (Join-Path $JavaHome 'bin\keytool.exe') @(
        '-genkeypair', '-v', '-keystore', $Keystore, '-alias', $keystoreAlias,
        '-storepass', $keystorePass, '-keypass', $keystorePass,
        '-keyalg', 'RSA', '-keysize', '2048', '-validity', '10000',
        '-dname', 'CN=VAMA GCS Development,O=VAMA,C=IN')
    if ($code -ne 0) { Fail 'keytool failed.' }
}

# --- Environment for Qt, Gradle and the NDK --------------------------------
$env:JAVA_HOME = $JavaHome
$env:ANDROID_SDK_ROOT = $AndroidSdk
$env:ANDROID_HOME = $AndroidSdk
$env:ANDROID_NDK_ROOT = $Ndk
$env:QT_ANDROID_KEYSTORE_PATH = $Keystore
$env:QT_ANDROID_KEYSTORE_ALIAS = $keystoreAlias
$env:QT_ANDROID_KEYSTORE_STORE_PASS = $keystorePass
$env:QT_ANDROID_KEYSTORE_KEY_PASS = $keystorePass
# Longer timeouts and more retries: Gradle downloads its Android libraries on
# the first build, and the network here drops long or slow connections.
$env:GRADLE_OPTS = '-Dorg.gradle.daemon=false ' +
    '-Dorg.gradle.internal.http.connectionTimeout=120000 ' +
    '-Dorg.gradle.internal.http.socketTimeout=120000 ' +
    '-Dorg.gradle.internal.repository.max.retries=10 ' +
    '-Dorg.gradle.internal.repository.initial.backoff=2000'
$env:PATH = "$CmakeBin;$(Join-Path $JavaHome 'bin');$env:PATH"

Write-Host "Qt target:  $QtTarget"
Write-Host "Qt host:    $QtHost"
Write-Host "ABIs:       $Abis"
Write-Host "NDK:        $Ndk"
Write-Host "JDK:        $JavaHome"
Write-Host "CMake:      $Cmake"
Write-Host "Build dir:  $BuildDir"

# --- Configure -----------------------------------------------------------
$configureArgs = @(
    '-S', $SrcLink, '-B', $BuildDir, '-G', 'Ninja',
    "-DCMAKE_BUILD_TYPE=$BuildType",
    '-DCMAKE_WARN_DEPRECATED=FALSE',
    "-DCMAKE_TOOLCHAIN_FILE=$QtTarget\lib\cmake\Qt6\qt.toolchain.cmake",
    "-DCMAKE_PREFIX_PATH=$QtTarget",
    "-DQT_ANDROID_ABIS=$Abis",
    "-DQT_HOST_PATH=$QtHost",
    "-DANDROID_SDK_ROOT=$AndroidSdk",
    "-DANDROID_NDK_ROOT=$Ndk",
    '-DQT_ANDROID_SIGN_APK=ON',
    '-DQGC_BUILD_TESTING=OFF',
    "-DPython3_EXECUTABLE=$VenvPython",
    "-DPython_EXECUTABLE=$VenvPython"
)
if ((Invoke-Native $Cmake $configureArgs) -ne 0) { Fail 'CMake configure failed.' }
if ($ConfigureOnly) { Write-Host 'Configure done.'; exit 0 }

# --- Build ---------------------------------------------------------------
if ((Invoke-Native $Cmake @('--build', $BuildDir, '--parallel')) -ne 0) { Fail 'Build failed.' }

$apks = @(Get-ChildItem -Path $BuildDir -Recurse -Filter '*.apk' -ErrorAction SilentlyContinue |
    Sort-Object LastWriteTime -Descending)
if ($apks.Count -eq 0) { Fail 'Build finished but no APK was found.' }
Write-Host ''
Write-Host 'APK files:'
$apks | ForEach-Object { Write-Host ("  {0}  ({1:N1} MB)" -f $_.FullName, ($_.Length / 1MB)) }
