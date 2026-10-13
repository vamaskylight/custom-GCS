<#
.SYNOPSIS
Proves, without building an APK, that the object finder and the ncnn library compile
and link for each Android ABI of the app.

.DESCRIPTION
Builds apk\custom\test\android_link_check.cc with the finder's sources and the ncnn
library of apk\build_ncnn.ps1 into a shared library, with the NDK's compiler and the
flags the APK build uses for our code (C++20, -O3, thin LTO, -Wall -Wextra -Wshadow
-Werror, API 28, the shared C++ library). The link allows nothing undefined.
What this cannot prove: that the code runs right on the phone's processor. The host
tests run the same sources on this PC.

.EXAMPLE
powershell -ExecutionPolicy Bypass -File apk\custom\test\run_android_link_check.ps1
#>
[CmdletBinding()]
param(
    [string]$AndroidSdk = $(if ($env:ANDROID_HOME) { $env:ANDROID_HOME } else { Join-Path $env:LOCALAPPDATA 'Android\Sdk' }),
    [string]$NdkVersion = '27.2.12479018',
    [string]$Abis = 'arm64-v8a;armeabi-v7a',
    [int]$AndroidApi = 28,
    [string]$WorkRoot = 'C:\vama-apk',
    [string]$NcnnRelease = '20260526'
)

$ErrorActionPreference = 'Stop'
$TestDir = $PSScriptRoot
$SrcDir = Join-Path (Split-Path $TestDir -Parent) 'src\Skydroid'
$Bin = Join-Path $AndroidSdk "ndk\$NdkVersion\toolchains\llvm\prebuilt\windows-x86_64\bin"
$Clang = Join-Path $Bin 'clang++.exe'
$Strip = Join-Path $Bin 'llvm-strip.exe'
$Readelf = Join-Path $Bin 'llvm-readelf.exe'
if (-not (Test-Path -LiteralPath $Clang)) {
    Write-Host "ERROR: the NDK compiler is not at $Clang" -ForegroundColor Red
    exit 1
}
$OutDir = Join-Path $env:TEMP 'vama-apk-tests\android-link'
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

$targetFor = @{ 'arm64-v8a' = 'aarch64-none-linux-android'; 'armeabi-v7a' = 'armv7-none-linux-androideabi' }
$failed = 0
foreach ($abi in @($Abis.Split(';') | ForEach-Object { $_.Trim() } | Where-Object { $_ })) {
    if (-not $targetFor.ContainsKey($abi)) {
        Write-Host "ERROR: unknown ABI '$abi'" -ForegroundColor Red
        $failed++
        continue
    }
    $ncnn = Join-Path $WorkRoot "ai\ncnn-$NcnnRelease\android-$abi"
    $lib = Join-Path $ncnn 'lib\libncnn.a'
    if (-not (Test-Path -LiteralPath $lib)) {
        Write-Host "ERROR: $lib is missing. Run apk\build_ncnn.ps1 first." -ForegroundColor Red
        $failed++
        continue
    }
    $out = Join-Path $OutDir "libvama_finder_check_$abi.so"
    $arguments = @("--target=$($targetFor[$abi])$AndroidApi", '-std=c++20', '-O3', '-fPIC', '-flto=thin',
        '-fdata-sections', '-ffunction-sections', '-fvisibility=default', '-fstack-protector-strong',
        '-Wall', '-Wextra', '-Wshadow', '-Werror', '-pthread',
        '-I', $SrcDir, '-isystem', (Join-Path $ncnn 'include'),
        (Join-Path $TestDir 'android_link_check.cc'), (Join-Path $SrcDir 'ObjectFinder.cc'),
        (Join-Path $SrcDir 'ObjectFinderNcnn.cc'), $lib,
        '-shared', '-Wl,--no-undefined', '-Wl,--gc-sections', '-Wl,-z,max-page-size=16384', '-o', $out)
    if ($abi -eq 'armeabi-v7a') { $arguments = @('-march=armv7-a', '-mthumb') + $arguments }
    $saved = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    & $Clang @arguments 2>&1 | ForEach-Object { "$_" } | Out-Host
    $code = $LASTEXITCODE
    $ErrorActionPreference = $saved
    if ($code -ne 0) {
        Write-Host "android link check ${abi}: FAILED (the compiler's words are above)" -ForegroundColor Red
        $failed++
        continue
    }
    # What the finder adds to the app, about: this library without its symbols.
    $stripped = "$out.stripped"
    Copy-Item -LiteralPath $out -Destination $stripped -Force
    & $Strip --strip-all $stripped
    $needed = @(& $Readelf -d $out | Where-Object { $_ -match 'NEEDED' } | ForEach-Object { if ($_ -match '\[(.+)\]') { $Matches[1] } })
    Write-Host ("android link check {0}: OK, {1:N1} MB of code (stripped), needs {2}" -f $abi,
        ((Get-Item -LiteralPath $stripped).Length / 1MB), ($needed -join ', '))
}
exit $failed
