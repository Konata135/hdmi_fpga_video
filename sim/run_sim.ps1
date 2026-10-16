# ============================================================================
# run_sim.ps1 -- compile and run the self-checking testbenches
#
#   pwsh -File sim/run_sim.ps1
#   pwsh -File sim/run_sim.ps1 -Tb tb_hdmi_tx
#   pwsh -File sim/run_sim.ps1 -Vlog <path to vlog.exe>
#
# Every testbench prints "RESULT: PASS" or "RESULT: FAIL" on its last lines.
# ============================================================================
param(
    [string[]] $Tb   = @('tb_m1_video', 'tb_tmds_encoder', 'tb_hdmi_tx', 'tb_seg_display',
                         'tb_spi_master', 'tb_sd_cmd', 'tb_sd_init',
                         'tb_sd_sec', 'tb_sd_scan', 'tb_sd_probe',
                         'tb_img_load',
                         'tb_sdram', 'tb_sdram_selftest', 'tb_afifo',
                         'tb_frame_wr', 'tb_frame_rd',
                         'tb_frame_chain'),
    [string]   $Vlog = 'F:\quartus\modelsim_ase\win32aloem\vlog.exe'
)

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$bin  = Split-Path -Parent $Vlog
$lib  = Join-Path ([System.IO.Path]::GetTempPath()) ('mm_sim_' + [System.Guid]::NewGuid().ToString('N').Substring(0, 8))

if (-not (Test-Path -LiteralPath $Vlog)) {
    throw "vlog.exe not found at '$Vlog'. Pass -Vlog <path>."
}

$failed = @()

try {
    New-Item -ItemType Directory -Force -Path $lib | Out-Null
    & (Join-Path $bin 'vlib.exe') (Join-Path $lib 'work') | Out-Null

    $sources  = Get-ChildItem -LiteralPath (Join-Path $root 'rtl') -Filter '*.v' -Recurse |
                Sort-Object FullName | Select-Object -ExpandProperty FullName
    $sources += (Join-Path $root 'sim\mm_blackbox_stubs.v')
    # behavioural models that stand in for the device macros (an SDRAM is a
    # macro, not a stub: the controller needs something that answers it)
    $sources += (Join-Path $root 'sim\mm_sdram_device.v')
    foreach ($t in $Tb) { $sources += (Join-Path $root "sim\$t.v") }

    Push-Location $lib
    try {
        & $Vlog -work work @sources | Out-Host
        if ($LASTEXITCODE -ne 0) { throw "vlog failed with exit code $LASTEXITCODE" }

        foreach ($t in $Tb) {
            Write-Host ""
            Write-Host "=== $t ==="
            $out = & (Join-Path $bin 'vsim.exe') -c -do "run -all; quit -f" "work.$t" 2>&1
            $out | Where-Object { $_ -match 'RESULT|FAIL|^\s+[A-C]\.|frame ' } | ForEach-Object { Write-Host $_ }
            if ($out -match 'RESULT: PASS') {
                Write-Host "PASS  $t"
            } else {
                Write-Host "FAIL  $t"
                $failed += $t
            }
        }
    } finally {
        Pop-Location
    }
} finally {
    Remove-Item -LiteralPath $lib -Recurse -Force -ErrorAction SilentlyContinue
}

if ($failed.Count -gt 0) { exit 1 }

