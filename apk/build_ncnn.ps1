<#
.SYNOPSIS
Builds the ncnn library, which runs the object finder's neural network.

.DESCRIPTION
The app marks people and vehicles in the video (apk\custom\src\Skydroid\ObjectFinder.h).
The network is run by ncnn (https://github.com/Tencent/ncnn, BSD 3-Clause). This script
gets one fixed release of its source, checks the file against the hash its makers
published, and builds a static library from it:
 - for each Android ABI of the APK, with the same NDK, API level and C++ library as the app,
 - with -ForHost also for this PC (MinGW), for the host tests.
The same options go to every build, so what the host tests prove holds for the phone's
library too, as far as a PC can prove it.

Everything is kept outside the repo, in <WorkRoot>\ai:
    downloads\ncnn-src\    the source zip
    src\ncnn-<release>\    the source
    build\<target>\        build folders
    ncnn-<release>\<target>\   the library and its headers; vama-ncnn.txt says how it was built
A target that is already built the same way is left alone. apk\build_android.ps1 runs this
script by itself, and apk\custom\CMakeLists.txt finds the library beside the build's source link.

.EXAMPLE
powershell -ExecutionPolicy Bypass -File apk\build_ncnn.ps1
powershell -ExecutionPolicy Bypass -File apk\build_ncnn.ps1 -ForHost -Abis ''
#>
[CmdletBinding()]
param(
    [string]$AndroidSdk = $(if ($env:ANDROID_HOME) { $env:ANDROID_HOME } else { Join-Path $env:LOCALAPPDATA 'Android\Sdk' }),
    [string]$NdkVersion = '27.2.12479018',
    [string]$Abis = 'arm64-v8a;armeabi-v7a',
    # The lowest Android the app runs on (QGC_QT_ANDROID_MIN_SDK_VERSION).
    [int]$AndroidApi = 28,
    [string]$WorkRoot = 'C:\vama-apk',
    # Also build for this PC, for the host tests.
    [switch]$ForHost,
    [string]$MinGW = 'C:\Qt\Tools\mingw1310_64',
    # Build again even when the library is there.
    [switch]$Force
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

# The release, and the SHA-256 of its source zip as published on its release page.
$NcnnRelease = '20260526'
$NcnnZipName = "ncnn-$NcnnRelease-full-source.zip"
$NcnnZipUrl = "https://github.com/Tencent/ncnn/releases/download/$NcnnRelease/$NcnnZipName"
$NcnnZipSha256 = '754659d6fe65545cf2ef4483ffb84526fea631f8764c44b150f1601d0fb4004b'
# Change this when the options below change: every target is then built again.
$Recipe = '1'

# One thread and no graphics card: the finder takes one core, the video has the rest.
# RTTI and exceptions as in the app, which is linked with this library.
$NcnnOptions = @(
    "-DNCNN_VERSION=$NcnnRelease",
    '-DNCNN_SHARED_LIB=OFF',
    '-DNCNN_VULKAN=OFF',
    '-DNCNN_OPENMP=OFF',
    '-DNCNN_SIMPLEOMP=OFF',
    '-DNCNN_BUILD_TOOLS=OFF',
    '-DNCNN_BUILD_EXAMPLES=OFF',
    '-DNCNN_BUILD_BENCHMARK=OFF',
    '-DNCNN_BUILD_TESTS=OFF',
    '-DNCNN_PYTHON=OFF',
    '-DNCNN_C_API=OFF',
    '-DNCNN_PLATFORM_API=OFF',
    '-DNCNN_SIMPLEOCV=OFF',
    '-DNCNN_PIXEL_ROTATE=OFF',
    '-DNCNN_PIXEL_AFFINE=OFF',
    '-DNCNN_PIXEL_DRAWING=OFF',
    '-DNCNN_DISABLE_RTTI=OFF',
    '-DNCNN_DISABLE_EXCEPTION=OFF'
)

function Fail([string]$Message) {
    Write-Host "ERROR: $Message" -ForegroundColor Red
    exit 1
}

# Native tools write progress to stderr, which Windows PowerShell 5.1 turns
# into errors under 'Stop'. Run them with 'Continue' and judge the exit code.
function Invoke-Quiet([string]$Exe, [string[]]$Arguments, [string]$Log) {
    $saved = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        & $Exe @Arguments 2>&1 | ForEach-Object { "$_" } | Set-Content -LiteralPath $Log -Encoding utf8
        return $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $saved
    }
}

$abiList = @($Abis.Split(';') | ForEach-Object { $_.Trim() } | Where-Object { $_ })
foreach ($abi in $abiList) {
    if ($abi -notin 'arm64-v8a', 'armeabi-v7a', 'x86_64', 'x86') { Fail "Unknown ABI '$abi'." }
}
if ($abiList.Count -eq 0 -and -not $ForHost) { Fail 'Nothing to build: give -Abis or -ForHost.' }

$cmakeDir = Get-ChildItem -Directory -Path (Join-Path $AndroidSdk 'cmake') -ErrorAction SilentlyContinue |
    Sort-Object { [version]$_.Name } -Descending | Select-Object -First 1
if (-not $cmakeDir) { Fail 'CMake not found in the Android SDK.' }
$CmakeBin = Join-Path $cmakeDir.FullName 'bin'
$Cmake = Join-Path $CmakeBin 'cmake.exe'
$Ninja = Join-Path $CmakeBin 'ninja.exe'
if (-not (Test-Path -LiteralPath $Ninja)) { Fail "Ninja not found at $Ninja" }

$Ndk = Join-Path $AndroidSdk "ndk\$NdkVersion"
if ($abiList.Count -gt 0 -and -not (Test-Path -LiteralPath $Ndk)) { Fail "Android NDK $NdkVersion not found at $Ndk" }
if ($ForHost -and -not (Test-Path -LiteralPath (Join-Path $MinGW 'bin\g++.exe'))) { Fail "MinGW not found at $MinGW" }

$AiRoot = Join-Path $WorkRoot 'ai'
$ZipDir = Join-Path $AiRoot 'downloads\ncnn-src'
$Zip = Join-Path $ZipDir $NcnnZipName
$Src = Join-Path $AiRoot "src\ncnn-$NcnnRelease"
$OutRoot = Join-Path $AiRoot "ncnn-$NcnnRelease"

# What a target is built from. A library with another text in its vama-ncnn.txt is built again.
function Get-Stamp([string]$Target, [string]$Toolchain) {
    return (@("ncnn $NcnnRelease", "source zip sha256 $NcnnZipSha256", "recipe $Recipe", "target $Target",
            "toolchain $Toolchain", "options $($NcnnOptions -join ' ')") -join "`n")
}

$targets = New-Object System.Collections.Generic.List[object]
foreach ($abi in $abiList) {
    $targets.Add(@{ Name = "android-$abi"; Stamp = (Get-Stamp "android-$abi" "NDK $NdkVersion, android-$AndroidApi, c++_shared"); Abi = $abi })
}
if ($ForHost) {
    $gccVersion = "$(& (Join-Path $MinGW 'bin\g++.exe') -dumpfullversion)".Trim()
    $targets.Add(@{ Name = 'host-mingw'; Stamp = (Get-Stamp 'host-mingw' "MinGW g++ $gccVersion"); Abi = '' })
}

$todo = New-Object System.Collections.Generic.List[object]
foreach ($t in $targets) {
    $out = Join-Path $OutRoot $t.Name
    $stampFile = Join-Path $out 'vama-ncnn.txt'
    $have = ''
    if ((Test-Path -LiteralPath $stampFile) -and (Test-Path -LiteralPath (Join-Path $out 'lib\libncnn.a'))) {
        $have = "$(Get-Content -LiteralPath $stampFile -Raw)".Trim() -replace "`r`n", "`n"
    }
    if ($Force -or $have -ne $t.Stamp) { $todo.Add($t) } else { Write-Host "ncnn $($t.Name): already built ($out)" }
}
if ($todo.Count -eq 0) { exit 0 }

# --- The source ------------------------------------------------------------
if (-not (Test-Path -LiteralPath $Zip)) {
    New-Item -ItemType Directory -Force -Path $ZipDir | Out-Null
    Write-Host "Downloading $NcnnZipUrl"
    $part = "$Zip.part"
    $saved = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    & curl.exe -sS -L --fail --retry 5 --retry-delay 5 -o $part $NcnnZipUrl
    $code = $LASTEXITCODE
    $ErrorActionPreference = $saved
    if ($code -ne 0) { Fail "The download failed (curl exit code $code). Try again, or put the file at $Zip yourself." }
    Move-Item -LiteralPath $part -Destination $Zip
}
$hash = (Get-FileHash -LiteralPath $Zip -Algorithm SHA256).Hash.ToLower()
if ($hash -ne $NcnnZipSha256) {
    Fail "$Zip has SHA-256 $hash. The release has $NcnnZipSha256. Delete the file and run again."
}
$srcMark = Join-Path $Src 'vama-unpacked.txt'
if (-not (Test-Path -LiteralPath $srcMark) -or "$(Get-Content -LiteralPath $srcMark -Raw)".Trim() -ne $NcnnZipSha256) {
    if (Test-Path -LiteralPath $Src) { Remove-Item -LiteralPath $Src -Recurse -Force }
    New-Item -ItemType Directory -Force -Path $Src | Out-Null
    Write-Host "Unpacking the source to $Src"
    # Windows' own tar reads a zip, and much faster than Expand-Archive does.
    $tar = Join-Path $env:SystemRoot 'System32\tar.exe'
    if (Test-Path -LiteralPath $tar) {
        $saved = $ErrorActionPreference
        $ErrorActionPreference = 'Continue'
        & $tar -xf $Zip -C $Src
        $code = $LASTEXITCODE
        $ErrorActionPreference = $saved
        if ($code -ne 0) { Fail "Unpacking $Zip failed (tar exit code $code)." }
    } else {
        Expand-Archive -LiteralPath $Zip -DestinationPath $Src
    }
    if (-not (Test-Path -LiteralPath (Join-Path $Src 'CMakeLists.txt'))) { Fail "The zip did not unpack to a source folder: $Src" }
    Set-Content -LiteralPath $srcMark -Value $NcnnZipSha256 -Encoding ascii
}

# --- Build -----------------------------------------------------------------
$pathBefore = $env:PATH
foreach ($t in $todo) {
    $build = Join-Path $AiRoot "build\$($t.Name)"
    $out = Join-Path $OutRoot $t.Name
    foreach ($dir in $build, $out) {
        if (Test-Path -LiteralPath $dir) { Remove-Item -LiteralPath $dir -Recurse -Force }
    }
    New-Item -ItemType Directory -Force -Path $build | Out-Null
    $arguments = @('-S', $Src, '-B', $build, '-G', 'Ninja', "-DCMAKE_MAKE_PROGRAM=$Ninja", '-DCMAKE_BUILD_TYPE=Release',
        "-DCMAKE_INSTALL_PREFIX=$out") + $NcnnOptions
    if ($t.Abi) {
        $env:PATH = "$CmakeBin;$pathBefore"
        $arguments += @("-DCMAKE_TOOLCHAIN_FILE=$Ndk\build\cmake\android.toolchain.cmake", "-DANDROID_ABI=$($t.Abi)",
            "-DANDROID_PLATFORM=android-$AndroidApi", '-DANDROID_STL=c++_shared')
    } else {
        $env:PATH = "$CmakeBin;$(Join-Path $MinGW 'bin');$pathBefore"
        $arguments += @("-DCMAKE_C_COMPILER=$(Join-Path $MinGW 'bin\gcc.exe')", "-DCMAKE_CXX_COMPILER=$(Join-Path $MinGW 'bin\g++.exe')")
    }
    Write-Host "ncnn $($t.Name): building (a few minutes) ..."
    $started = Get-Date
    $log = Join-Path $AiRoot "build\$($t.Name)"
    if ((Invoke-Quiet $Cmake $arguments "$log-configure.log") -ne 0) { Fail "ncnn $($t.Name): CMake configure failed. See $log-configure.log" }
    if ((Invoke-Quiet $Cmake @('--build', $build, '--parallel') "$log-build.log") -ne 0) { Fail "ncnn $($t.Name): the build failed. See $log-build.log" }
    if ((Invoke-Quiet $Cmake @('--install', $build) "$log-install.log") -ne 0) { Fail "ncnn $($t.Name): the install failed. See $log-install.log" }
    if (-not (Test-Path -LiteralPath (Join-Path $out 'lib\libncnn.a'))) { Fail "ncnn $($t.Name): no library came out. See $log-build.log" }
    Copy-Item -LiteralPath (Join-Path $Src 'LICENSE.txt') -Destination (Join-Path $out 'LICENSE.txt')
    Set-Content -LiteralPath (Join-Path $out 'vama-ncnn.txt') -Value $t.Stamp -Encoding utf8
    $size = (Get-Item -LiteralPath (Join-Path $out 'lib\libncnn.a')).Length / 1MB
    Write-Host ("ncnn {0}: built in {1:N0} s, library {2:N1} MB ({3})" -f $t.Name, ((Get-Date) - $started).TotalSeconds, $size, $out)
}
$env:PATH = $pathBefore
exit 0
