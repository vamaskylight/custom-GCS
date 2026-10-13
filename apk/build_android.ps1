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

The app marks people and vehicles with a small neural network. The library
that runs it (ncnn) is built first, by apk\build_ncnn.ps1, into <WorkRoot>\ai:
once for each ABI, from a source that is downloaded once and checked against
its hash. The finished APK is then checked for the finder in each ABI.
-NoFinder builds an app without it.

A build for customers (-Customer) is signed with the release key from
apk\make_release_key.ps1, is QGC's stable build (the app's own name has no
"Daily"), and has its own build folder. It is made only from committed files,
and only when the app menu names the version given with -Version.

.EXAMPLE
powershell -ExecutionPolicy Bypass -File apk\build_android.ps1
powershell -ExecutionPolicy Bypass -File apk\build_android.ps1 -Abis arm64-v8a
powershell -ExecutionPolicy Bypass -File apk\build_android.ps1 -Customer -Version 1.0.0 -CheckOnly
powershell -ExecutionPolicy Bypass -File apk\build_android.ps1 -Customer -Version 1.0.0
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
    # A build for customers. -Version is the VAMA version: the app menu must say
    # "Version <Version>", and the APK is saved as apk\build\VAMA-GCS-<Version>.apk.
    [switch]$Customer,
    [string]$Version = '',
    [string]$ReleaseKeystore = (Join-Path $env:USERPROFILE '.vama\vama-release.keystore'),
    # A customer build from files that are not committed (for trying the build itself only).
    [switch]$AllowUncommitted,
    # Check the tools, and for a customer build the key, the menu text and git. Build nothing.
    [switch]$CheckOnly,
    [switch]$ConfigureOnly,
    # An app without the object finder: it does not mark people and vehicles, and says so.
    # (The finder needs the ncnn library, which apk\build_ncnn.ps1 builds before the app.)
    [switch]$NoFinder
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

# The SHA-256 of a key's certificate, as 64 hex digits in lower case. Empty when
# the key does not open. The password goes through the environment, so it is
# never on a command line.
function Get-KeystoreDigest([string]$Keytool, [string]$Store, [string]$Alias, [string]$Pass) {
    $env:VAMA_KEYSTORE_CHECK = $Pass
    $saved = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $out = @(& $Keytool -list -v -keystore $Store -alias $Alias -storepass:env VAMA_KEYSTORE_CHECK 2>&1 | ForEach-Object { "$_" })
        if ($LASTEXITCODE -ne 0) { return '' }
    } finally {
        $ErrorActionPreference = $saved
        Remove-Item Env:\VAMA_KEYSTORE_CHECK -ErrorAction SilentlyContinue
    }
    foreach ($line in $out) {
        if ($line -match 'SHA-?256:\s*([0-9A-Fa-f:]{95})') { return ($Matches[1] -replace ':', '').ToLower() }
    }
    return ''
}

# The SHA-256 of the certificate an APK is signed with (apksigner). Empty when
# the APK does not verify.
function Get-ApkDigest([string]$Apksigner, [string]$Apk) {
    $saved = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $out = @(& $Apksigner verify --print-certs $Apk 2>&1 | ForEach-Object { "$_" })
        if ($LASTEXITCODE -ne 0) { return '' }
    } finally {
        $ErrorActionPreference = $saved
    }
    foreach ($line in $out) {
        if ($line -match 'certificate SHA-256 digest:\s*([0-9a-fA-F]{64})') { return $Matches[1].ToLower() }
    }
    return ''
}

# The ABIs of an APK whose code holds the object finder: the word "VAMA-FINDER ncnn"
# is in the app's own library only when the finder was built in.
function Get-ApkFinderAbis([string]$Apk) {
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    $latin1 = [System.Text.Encoding]::GetEncoding(28591)
    $found = New-Object System.Collections.Generic.List[string]
    $zip = [System.IO.Compression.ZipFile]::OpenRead($Apk)
    try {
        foreach ($entry in $zip.Entries) {
            # The app's own library is the large one; Qt's libraries are skipped.
            $name = $entry.FullName -replace '\\', '/'
            if ($name -notmatch '^lib/([^/]+)/lib[^/]*\.so$' -or $entry.Length -lt 20MB) { continue }
            $abi = $Matches[1]
            $stream = $entry.Open()
            try {
                $memory = New-Object System.IO.MemoryStream
                $stream.CopyTo($memory)
                $text = $latin1.GetString($memory.ToArray())
                if ($text.IndexOf('VAMA-FINDER ncnn ', [System.StringComparison]::Ordinal) -ge 0 -and -not $found.Contains($abi)) {
                    $found.Add($abi)
                }
            } finally {
                $stream.Dispose()
            }
        }
    } finally {
        $zip.Dispose()
    }
    return $found.ToArray()
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

# --- Signing ---------------------------------------------------------------
# Test builds are signed with a development key kept outside the repo.
# Customer builds are signed with the release key (apk\make_release_key.ps1).
$keystoreAlias = 'vamadev'
$keystorePass = if ($env:VAMA_KEYSTORE_PASS) { $env:VAMA_KEYSTORE_PASS } else { 'vamadev' }
$releaseDigest = ''
if ($Customer) {
    # Its own folder: a customer build never mixes with the files of a test build.
    $BuildDir = Join-Path $WorkRoot 'build-Customer'
    $Keystore = $ReleaseKeystore
    $keystoreAlias = 'vama'
    $passFile = "$ReleaseKeystore.pass"
    $keystorePass = ''
    if ($env:VAMA_RELEASE_KEYSTORE_PASS) { $keystorePass = $env:VAMA_RELEASE_KEYSTORE_PASS }
    elseif (Test-Path -LiteralPath $passFile) { $keystorePass = "$(Get-Content -LiteralPath $passFile -Raw)".Trim() }
    $problems = New-Object System.Collections.Generic.List[string]
    if ($BuildType -ne 'Release') { $problems.Add('A customer build is a Release build (leave -BuildType out).') }
    if ($Version -notmatch '^\d+\.\d+\.\d+$') { $problems.Add('Give the VAMA version, for example -Version 1.0.0') }
    if (-not (Test-Path -LiteralPath $ReleaseKeystore)) {
        $problems.Add("The release key is missing: $ReleaseKeystore. Make it once with apk\make_release_key.ps1, and back it up.")
    } elseif (-not $keystorePass) {
        $problems.Add("The release key's password is missing: $passFile")
    } else {
        $releaseDigest = Get-KeystoreDigest (Join-Path $JavaHome 'bin\keytool.exe') $ReleaseKeystore $keystoreAlias $keystorePass
        if (-not $releaseDigest) { $problems.Add("The release key $ReleaseKeystore does not open with its password, or has no key named '$keystoreAlias'.") }
    }
    # The app menu names the build. A customer must never get an app that says "Test build".
    $menuFile = Join-Path $ApkDir 'custom\src\SelectViewDropdown.qml'
    $menuText = ''
    foreach ($line in (Get-Content -LiteralPath $menuFile)) {
        if ($line -match 'qsTr\("((?:Test build|Version)[^"]*)"\)') { $menuText = $Matches[1] }
    }
    if ($Version -match '^\d+\.\d+\.\d+$' -and $menuText -notmatch "^Version $([regex]::Escape($Version))(\D|$)") {
        $problems.Add("The app menu says '$menuText'. Set it to 'Version $Version' in apk\custom\src\SelectViewDropdown.qml (and commit it).")
    }
    if (-not $AllowUncommitted) {
        $repo = Split-Path $ApkDir -Parent
        $dirty = @(& git -C $repo status --porcelain --untracked-files=all -- apk 2>$null)
        if ($LASTEXITCODE -ne 0) { $problems.Add('git status failed: a customer build is made from committed files only.') }
        elseif ($dirty.Count -gt 0) { $problems.Add("apk\ has $($dirty.Count) file(s) that are not committed. A customer build is made from committed files only.") }
    }
    if ($Version -and (Test-Path -LiteralPath (Join-Path $ApkDir "build\VAMA-GCS-$Version.apk"))) {
        $problems.Add("apk\build\VAMA-GCS-$Version.apk is already there. A version is built once: move the file away, or use the next version.")
    }
    if ($problems.Count -gt 0) {
        Write-Host 'This customer build cannot be made yet:' -ForegroundColor Red
        foreach ($p in $problems) { Write-Host "  - $p" -ForegroundColor Red }
        exit 1
    }
    Write-Host "Customer build: version $Version, release key certificate SHA-256 $releaseDigest"
} elseif (-not (Test-Path -LiteralPath $Keystore)) {
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
Write-Host "Signed by:  $Keystore"
Write-Host "Finder:     $(if ($NoFinder) { 'NOT in this build (-NoFinder): the app will not mark people and vehicles' } else { 'people and vehicles are marked (ncnn)' })"
if ($CheckOnly) { Write-Host 'Check done. Nothing was built.'; exit 0 }

# --- The object finder's library ---------------------------------------------
# The app marks people and vehicles with a small neural network, which the ncnn
# library runs. apk\build_ncnn.ps1 builds it once for each ABI into $WorkRoot\ai,
# where apk\custom\CMakeLists.txt looks for it. A library that is there already
# is left alone.
if ($NoFinder) {
    $env:VAMA_NO_FINDER = '1'
} else {
    Remove-Item Env:\VAMA_NO_FINDER -ErrorAction SilentlyContinue
    $code = Invoke-Native 'powershell.exe' @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
        (Join-Path $ApkDir 'build_ncnn.ps1'), '-AndroidSdk', $AndroidSdk, '-NdkVersion', $NdkVersion,
        '-Abis', $Abis, '-WorkRoot', $WorkRoot)
    if ($code -ne 0) {
        Fail "The object finder's library (ncnn) was not built: see the lines above. (-NoFinder builds an app without the finder.)"
    }
}

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
# QGC's stable build: the app's own name (its settings and its folder on the
# device) is "VAMA-GCS", without "Daily".
if ($Customer) { $configureArgs += '-DQGC_STABLE_BUILD=ON' }
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

# The object finder is in the app: its code carries a word that only a build
# with the finder and its library has (ObjectFinderNcnn.cc), for every ABI.
if (-not $NoFinder) {
    $withFinder = Get-ApkFinderAbis $apks[0].FullName
    foreach ($abi in $abiList) {
        if ($withFinder -notcontains $abi) { Fail "The APK's $abi code has no object finder in it ($($apks[0].FullName))." }
    }
    Write-Host "Object finder: in the APK for $($abiList -join ', ')"
}

if ($Customer) {
    # The file for customers: the signed APK, proven to be signed with the release key.
    $signed = $apks | Where-Object { $_.Name -like '*-signed.apk' } | Select-Object -First 1
    if (-not $signed) { Fail 'No signed APK was found.' }
    $buildTools = Get-ChildItem -Directory -Path (Join-Path $AndroidSdk 'build-tools') |
        Sort-Object { [version]($_.Name -replace '[^0-9.]', '') } -Descending | Select-Object -First 1
    $apkDigest = Get-ApkDigest (Join-Path $buildTools.FullName 'apksigner.bat') $signed.FullName
    if (-not $apkDigest) { Fail "The signature of $($signed.FullName) does not verify." }
    if ($apkDigest -ne $releaseDigest) { Fail "The APK is signed with certificate $apkDigest, not with the release key ($releaseDigest)." }
    $outDir = Join-Path $ApkDir 'build'
    New-Item -ItemType Directory -Force -Path $outDir | Out-Null
    $out = Join-Path $outDir "VAMA-GCS-$Version.apk"
    Copy-Item -LiteralPath $signed.FullName -Destination $out
    Write-Host ''
    Write-Host "Customer APK: $out"
    Write-Host "  SHA-256:    $((Get-FileHash -LiteralPath $out -Algorithm SHA256).Hash.ToLower())"
    Write-Host "  signed by:  the release key, certificate SHA-256 $apkDigest"
    Write-Host '  It does not install over a test build (another key): the test build is uninstalled first.'
}
