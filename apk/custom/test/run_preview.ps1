<#
.SYNOPSIS
Builds and runs the camera screen preview, and saves screenshots.

.DESCRIPTION
Loads the real FlyViewCustomLayer.qml and SkydroidLink with stand-ins for QGC
and a fake camera on 127.0.0.1. Then opens Application Settings by QGC's own
address and checks the VAMA copy of AppSettings.qml (VAMA mark on General, no
Help page). Prints every QML warning and check, and saves screenshots
(1_main.png to 17_app_settings.png) in -OutDir.
Needs the Qt 6.11.1 MinGW kit and MinGW 13.1, like run_link_test.ps1.

.EXAMPLE
powershell -ExecutionPolicy Bypass -File apk\custom\test\run_preview.ps1
#>
[CmdletBinding()]
param(
    [string]$QtKit = 'C:\Qt\6.11.1\mingw_64',
    [string]$MinGW = 'C:\Qt\Tools\mingw1310_64',
    [string]$AndroidSdk = (Join-Path $env:LOCALAPPDATA 'Android\Sdk'),
    [string]$BuildDir = (Join-Path $env:TEMP 'vama-preview-build'),
    [string]$OutDir = (Join-Path $env:TEMP 'vama-preview')
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

$code = Invoke-Native (Join-Path $cmakeBin 'cmake.exe') @(
    '-S', (Join-Path $PSScriptRoot 'preview'), '-B', $BuildDir, '-G', 'Ninja', '-DCMAKE_BUILD_TYPE=Debug',
    "-DCMAKE_PREFIX_PATH=$QtKit",
    "-DCMAKE_CXX_COMPILER=$(Join-Path $MinGW 'bin\g++.exe')")
if ($code -ne 0) { Write-Host 'ERROR: configure failed.' -ForegroundColor Red; exit 1 }
$code = Invoke-Native (Join-Path $cmakeBin 'cmake.exe') @('--build', $BuildDir)
if ($code -ne 0) { Write-Host 'ERROR: build failed.' -ForegroundColor Red; exit 1 }

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$log = Join-Path $OutDir 'preview.log'
# The output folder may contain spaces, so it is passed in quotes.
$p = Start-Process -FilePath (Join-Path $BuildDir 'vama_preview.exe') -ArgumentList @('"' + $OutDir + '"') `
    -Wait -PassThru -NoNewWindow -RedirectStandardError $log
Get-Content -LiteralPath $log | ForEach-Object { Write-Host $_ }
exit $p.ExitCode
