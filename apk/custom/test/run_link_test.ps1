<#
.SYNOPSIS
Builds and runs the host tests of the app's Qt code: the camera link with Follow Me
and the marks on people and vehicles (real Qt code, fake camera, stand-in drone,
stand-in network), the drone side of Follow Me (real MAVLink headers), the object
finder with its real network on pictures from the air, the map packs from VGCS
with QGC's own tile store code, the camera profiles, and the admin password of
Advanced Mode (as a test build and as a customer build).

.DESCRIPTION
Needs the Qt 6.11.1 MinGW desktop kit (C:\Qt\6.11.1\mingw_64), MinGW 13.1
(C:\Qt\Tools\mingw1310_64), and CMake and Ninja from the Android SDK.
The test with the real network also needs the ncnn library for this PC:
    powershell -ExecutionPolicy Bypass -File apk\build_ncnn.ps1 -ForHost -Abis ''

.EXAMPLE
powershell -ExecutionPolicy Bypass -File apk\custom\test\run_link_test.ps1
#>
[CmdletBinding()]
param(
    [string]$QtKit = 'C:\Qt\6.11.1\mingw_64',
    [string]$MinGW = 'C:\Qt\Tools\mingw1310_64',
    [string]$AndroidSdk = (Join-Path $env:LOCALAPPDATA 'Android\Sdk'),
    # Build against another copy of src\Skydroid (used for mutation checks).
    [string]$SourceDir = '',
    [string]$BuildDir = (Join-Path $env:TEMP 'vama-link-test-build')
)

$ErrorActionPreference = 'Stop'

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

foreach ($p in $QtKit, $MinGW) {
    if (-not (Test-Path -LiteralPath $p)) { Write-Host "ERROR: $p not found." -ForegroundColor Red; exit 1 }
}
$cmakeDir = Get-ChildItem -Directory -Path (Join-Path $AndroidSdk 'cmake') | Sort-Object { [version]$_.Name } -Descending | Select-Object -First 1
$cmakeBin = Join-Path $cmakeDir.FullName 'bin'
$env:PATH = "$cmakeBin;$(Join-Path $MinGW 'bin');$(Join-Path $QtKit 'bin');$env:PATH"

$src = Join-Path $PSScriptRoot 'link'
$build = $BuildDir
$srcArg = if ($SourceDir) { "-DSKYDROID_SRC=$SourceDir" } else { "-DSKYDROID_SRC=$(Join-Path (Split-Path $PSScriptRoot -Parent) 'src\Skydroid')" }

$code = Invoke-Native (Join-Path $cmakeBin 'cmake.exe') @(
    '-S', $src, '-B', $build, '-G', 'Ninja', '-DCMAKE_BUILD_TYPE=Debug', $srcArg,
    "-DCMAKE_PREFIX_PATH=$QtKit",
    "-DCMAKE_CXX_COMPILER=$(Join-Path $MinGW 'bin\g++.exe')")
if ($code -ne 0) { Write-Host 'ERROR: configure failed.' -ForegroundColor Red; exit 1 }

$code = Invoke-Native (Join-Path $cmakeBin 'cmake.exe') @('--build', $build)
if ($code -ne 0) { Write-Host 'ERROR: build failed.' -ForegroundColor Red; exit 1 }

# QtTest output does not always reach a redirected console, so write it to a file.
# follow_drone_test is there only when the MAVLink headers of an APK build were found,
# marks_net_test only when the ncnn library was built for this PC, the two map pack tests
# only when the QGC source is in apk/qgc-src.
$failed = 0
foreach ($test in 'skydroid_link_test', 'follow_drone_test', 'marks_net_test', 'map_pack_test', 'map_pack_loader_test',
                  'camera_profile_test', 'admin_password_test', 'admin_password_team_test') {
    $exe = Join-Path $build "$test.exe"
    if (-not (Test-Path -LiteralPath $exe)) {
        Write-Host "$test was not built (see the configure output)." -ForegroundColor Yellow
        continue
    }
    $result = Join-Path $build "result_$test.txt"
    Remove-Item -LiteralPath $result -ErrorAction SilentlyContinue
    $p = Start-Process -FilePath $exe -ArgumentList @('-o', "$result,txt") -Wait -PassThru -NoNewWindow
    Write-Host "${test}:"
    Get-Content -LiteralPath $result | Where-Object { $_ -match '^(FAIL|Totals)|failure location' } | ForEach-Object { Write-Host $_ }
    $failed += $p.ExitCode
}
exit $failed
