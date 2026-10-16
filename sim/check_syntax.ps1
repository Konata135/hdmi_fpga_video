# ============================================================================
# check_syntax.ps1 -- compile rtl/ with ModelSim vlog as a fast typo check
# before running a full Anlogic TD build.
#
#   pwsh -File sim/check_syntax.ps1 [-Vlog <path to vlog.exe>]
#
# The vendor cells instantiated by the design are replaced by the empty stubs
# in sim/mm_blackbox_stubs.v.  Nothing here is synthesised.
# ============================================================================
param(
    [string] $Vlog = 'F:\quartus\modelsim_ase\win32aloem\vlog.exe'
)

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$lib  = Join-Path ([System.IO.Path]::GetTempPath()) ('mm_syntax_' + [System.Guid]::NewGuid().ToString('N').Substring(0, 8))

if (-not (Test-Path -LiteralPath $Vlog)) {
    throw "vlog.exe not found at '$Vlog'. Pass -Vlog <path>."
}

try {
    New-Item -ItemType Directory -Force -Path $lib | Out-Null

    & (Join-Path (Split-Path -Parent $Vlog) 'vlib.exe') (Join-Path $lib 'work') | Out-Null

    $sources = @()
    $sources += Get-ChildItem -LiteralPath (Join-Path $root 'rtl') -Filter '*.v' -Recurse |
                Sort-Object FullName | Select-Object -ExpandProperty FullName
    $sources += (Join-Path $root 'sim\mm_blackbox_stubs.v')

    Push-Location $lib
    try {
        & $Vlog -work work @sources
        if ($LASTEXITCODE -ne 0) { throw "vlog failed with exit code $LASTEXITCODE" }
    } finally {
        Pop-Location
    }

    Write-Host "syntax check passed ($($sources.Count) files)"
} finally {
    Remove-Item -LiteralPath $lib -Recurse -Force -ErrorAction SilentlyContinue
}

