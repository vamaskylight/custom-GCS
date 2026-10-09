<#
.SYNOPSIS
Builds and runs the host unit tests for our C++ code that has no Qt in it.

.DESCRIPTION
Uses the MinGW compiler from Qt's tools (C:\Qt\Tools\mingw1310_64), or any
g++ on PATH. Regenerate the expected values first if the VGCS protocol changed:
    python apk/custom/test/gen_skydroid_vectors.py

.EXAMPLE
powershell -ExecutionPolicy Bypass -File apk\custom\test\run_tests.ps1
#>
[CmdletBinding()]
param(
    [string]$Compiler = ''
)

$ErrorActionPreference = 'Stop'
$TestDir = $PSScriptRoot
$SrcDir = Join-Path (Split-Path $TestDir -Parent) 'src\Skydroid'

if (-not $Compiler) {
    $mingw = 'C:\Qt\Tools\mingw1310_64\bin\g++.exe'
    if (Test-Path -LiteralPath $mingw) {
        $Compiler = $mingw
        $env:PATH = "$(Split-Path $mingw);$env:PATH"
    } elseif (Get-Command g++ -ErrorAction SilentlyContinue) {
        $Compiler = 'g++'
    } else {
        Write-Host 'ERROR: no g++ found. Install Qt tools_mingw1310 or pass -Compiler.' -ForegroundColor Red
        exit 1
    }
}

$OutDir = Join-Path $env:TEMP 'vama-apk-tests'
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

# Each test: its own .cc file plus the source file it checks.
$tests = @(
    @{ Name = 'skydroid_top_test'; Source = 'SkydroidTop.cc' },
    @{ Name = 'laser_geo_test'; Source = 'LaserGeo.cc' },
    @{ Name = 'object_tracker_test'; Source = 'ObjectTracker.cc' },
    @{ Name = 'object_tracker_walk_test'; Source = 'ObjectTracker.cc' }
)

$failed = 0
foreach ($t in $tests) {
    $exe = Join-Path $OutDir "$($t.Name).exe"
    $saved = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    & $Compiler -std=c++17 -O2 -Wall -Wextra -Werror -I $SrcDir `
        (Join-Path $TestDir "$($t.Name).cc") (Join-Path $SrcDir $t.Source) -o $exe | Out-Host
    $compileCode = $LASTEXITCODE
    $ErrorActionPreference = $saved
    if ($compileCode -ne 0) {
        Write-Host "ERROR: $($t.Name) did not compile." -ForegroundColor Red
        $failed++
        continue
    }
    Write-Host -NoNewline "$($t.Name): "
    & $exe
    if ($LASTEXITCODE -ne 0) { $failed++ }
}
exit $failed
