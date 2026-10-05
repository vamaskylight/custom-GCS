<#
.SYNOPSIS
Downloads the pinned QGroundControl source into apk\qgc-src and links our custom build into it.

.DESCRIPTION
1. Reads the QGC release tag from apk\qgc-version.txt.
2. Clones that release into apk\qgc-src. This folder is gitignored and never committed.
3. The first time only, creates apk\custom from QGC's own custom-example of the same release.
4. Links apk\qgc-src\custom to apk\custom with a directory junction.
   QGC builds a custom build from its "custom" folder, so it builds our code,
   and every edit is saved in apk\custom, which is in git.

Safe to run again. It never deletes anything.
If apk\qgc-src is on a different release, it stops and says what to do.

.EXAMPLE
powershell -ExecutionPolicy Bypass -File apk\fetch_qgc.ps1
#>
[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$ApkDir = $PSScriptRoot
$VersionFile = Join-Path $ApkDir 'qgc-version.txt'
$SrcDir = Join-Path $ApkDir 'qgc-src'
$CustomDir = Join-Path $ApkDir 'custom'
$LinkPath = Join-Path $SrcDir 'custom'
$QgcRepo = 'https://github.com/mavlink/qgroundcontrol.git'

function Fail([string]$Message) {
    Write-Host "ERROR: $Message" -ForegroundColor Red
    exit 1
}

# Runs git and returns its exit code. Windows PowerShell 5.1 turns git's
# normal stderr output (progress, hints) into errors when $ErrorActionPreference
# is Stop, so git runs with Continue and only the exit code decides.
function Invoke-Git([string[]]$GitArgs, [switch]$Quiet) {
    $saved = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        if ($Quiet) {
            $out = & git @GitArgs 2>$null
        } else {
            # Out-Host so git's text is shown, not returned with the result.
            & git @GitArgs | Out-Host
            $out = $null
        }
        return [pscustomobject]@{ ExitCode = $LASTEXITCODE; Output = $out }
    } finally {
        $ErrorActionPreference = $saved
    }
}

function Get-FullPath([string]$Path) {
    return [System.IO.Path]::GetFullPath($Path).TrimEnd('\')
}

# 1. The pinned release.
if (-not (Test-Path -LiteralPath $VersionFile)) { Fail "Missing $VersionFile" }
$tag = Get-Content -LiteralPath $VersionFile |
    ForEach-Object { $_.Trim() } |
    Where-Object { $_ -and -not $_.StartsWith('#') } |
    Select-Object -First 1
if (-not $tag) { Fail "No QGC release tag found in $VersionFile" }
Write-Host "QGC release: $tag"

if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    Fail 'git is not installed or not on PATH.'
}

# 2. The QGC source.
if (-not (Test-Path -LiteralPath $SrcDir)) {
    Write-Host "Cloning QGroundControl $tag into $SrcDir"
    $r = Invoke-Git @('clone', '--branch', $tag, '--depth', '1', $QgcRepo, $SrcDir)
    if ($r.ExitCode -ne 0) { Fail 'git clone failed. Check the network and the tag in qgc-version.txt.' }
} else {
    $r = Invoke-Git @('-C', $SrcDir, 'tag', '--points-at', 'HEAD') -Quiet
    $tagsHere = @($r.Output | ForEach-Object { "$_".Trim() })
    if ($r.ExitCode -ne 0 -or -not ($tagsHere -contains $tag)) {
        Fail ("$SrcDir is not QGC $tag. " +
              "Remove the link first with: cmd /c rmdir `"$LinkPath`" " +
              "then delete $SrcDir and run this script again.")
    }
    Write-Host "QGC source already at $tag"
}

# 3. Our custom build. Created once from QGC's example, then it is ours.
$customCMake = Join-Path $CustomDir 'CMakeLists.txt'
if (-not (Test-Path -LiteralPath $customCMake)) {
    if ((Test-Path -LiteralPath $CustomDir) -and
        @(Get-ChildItem -LiteralPath $CustomDir -Force).Count -gt 0) {
        Fail "$CustomDir has files but no CMakeLists.txt. Fix or empty it, then run again."
    }
    $example = Join-Path $SrcDir 'custom-example'
    if (-not (Test-Path -LiteralPath $example)) { Fail "QGC $tag has no custom-example folder." }
    Write-Host "Creating $CustomDir from QGC $tag custom-example"
    New-Item -ItemType Directory -Force -Path $CustomDir | Out-Null
    Copy-Item -Path (Join-Path $example '*') -Destination $CustomDir -Recurse -Force
}

# 4. The link QGC builds through.
if (Test-Path -LiteralPath $LinkPath) {
    $item = Get-Item -LiteralPath $LinkPath -Force
    $isOurLink = $false
    if ($item.LinkType -eq 'Junction' -or $item.LinkType -eq 'SymbolicLink') {
        foreach ($t in @($item.Target)) {
            if ((Get-FullPath $t) -eq (Get-FullPath $CustomDir)) { $isOurLink = $true }
        }
    }
    if (-not $isOurLink) {
        Fail "$LinkPath exists and is not a link to $CustomDir. Move it away, then run again."
    }
    Write-Host "Link already in place: $LinkPath -> $CustomDir"
} else {
    New-Item -ItemType Junction -Path $LinkPath -Target $CustomDir | Out-Null
    Write-Host "Linked $LinkPath -> $CustomDir"
}

Write-Host ''
Write-Host 'Done. Edit only apk\custom. Build steps are in apk\README.md.'
