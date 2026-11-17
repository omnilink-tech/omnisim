# Deterministic open-loop PHYSICS PROBE -- DEPLOY launcher.
#
# Runs the g1_parity_probe world (welded-base, NO RL) in omnisim-bin and dumps a
# per-tick deploy trace, then (optionally) runs the matching TRAINER trace and
# diffs them with g1_parity_compare.py. This is the FIRST trainer<->deploy parity
# check that steps the REAL binary, not the omnisim_newton_runtime.py module driven in-process.
#
# The Newton env is sourced from SPEC.newton_env() (single source of truth), with
# the position-PD gains overridden to the probe's stand gains and the link-COM
# parity fix (OMNISIM_NEWTON_USE_LINK_COM=1) enabled -- the central model mend.
#
# Usage:
#   powershell -File scripts/dev/run_g1_parity_probe.ps1 [-Duration 90] [-Gui]
#                  [-Ticks 120] [-Sequence sinusoid] [-Compare]
param(
    [int]$Duration = 90,
    [switch]$Gui,
    [int]$Ticks = 0,                 # 0 -> SPEC.PROBE_DURATION_TICKS
    [int]$Settle = -1,               # <0 -> SPEC.PROBE_SETTLE_TICKS
    [string]$Sequence = "",          # "" -> SPEC.PROBE_SEQUENCE
    [switch]$Compare,                # also run the trainer trace + diff
    [switch]$RecordSettle,           # diagnostic: record the settle transient too
    [switch]$Free,                   # FREE-base lane (realistic stand on the ground)
    [switch]$NoLinkCom               # diagnostic: run WITHOUT the COM mend
)
$ErrorActionPreference = "Stop"
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
# Lane: welded base (staticBase, chaos-free articulation) vs free base (realistic
# stand on the ground). NOTE: the welded lane currently hits a binary staticBase
# defect -- the joint from the static pelvis to the thigh loses its position
# actuator, so the hips free-swing (see docs/developer/binary-parity-probe.md).
# Until that is fixed, -Free is the working parity lane (compare the pre-fall window).
if ($Free)                  { $world = "$root\projects\policies\research\worlds\g1_parity_probe_free.omniworld" }
elseif ($env:G1_PROBE_WORLD) { $world = $env:G1_PROBE_WORLD }
else                        { $world = "$root\projects\policies\research\worlds\g1_parity_probe.omniworld" }
$outdir = "$root\_scratch\parity"
New-Item -ItemType Directory -Force -Path $outdir | Out-Null
$deployTrace = "$outdir\deploy_trace.json"
$trainerTrace = "$outdir\trainer_trace.json"
Remove-Item $deployTrace -ErrorAction SilentlyContinue

$env:OMNISIM_HOME = $root
$env:PYTHONUTF8   = "1"
$env:PYTHONPATH   = $root

# --- Newton env from the single source of truth (SPEC.newton_env) -------------
$emit = @"
import json, sys
sys.path.insert(0, r'$root')
from projects.policies.research.backends import g1_physics_spec as S
e = dict(S.newton_env())
# probe uses the deterministic STAND gains (stiff position-PD), not the walk gains
e['OMNISIM_NEWTON_TARGET_KE'] = S._num(S.PROBE_KE)
e['OMNISIM_NEWTON_TARGET_KD'] = S._num(S.PROBE_KD)
print(json.dumps(e))
"@
$envJson = $emit | python -
if ($LASTEXITCODE -ne 0) { Write-Error "failed to compute SPEC.newton_env()"; exit 1 }
$envObj = $envJson | ConvertFrom-Json
foreach ($p in $envObj.PSObject.Properties) { Set-Item -Path "Env:$($p.Name)" -Value $p.Value }

# --- parity mend + deploy-faithful extras not in newton_env() ------------------
if ($NoLinkCom) { Remove-Item Env:\OMNISIM_NEWTON_USE_LINK_COM -ErrorAction SilentlyContinue }
else            { $env:OMNISIM_NEWTON_USE_LINK_COM = "1" }   # link-COM parity (the central mend)
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
# Probe spawns STRAIGHT and ramps to the pose during settle (the controller drives
# both sides from the identical straight-leg IC), so disable the deploy pose seed
# to keep the IC deterministic and matched to the trainer harness.
$env:OMNISIM_NEWTON_SEED_POSE    = "0"
Remove-Item Env:\OMNISIM_NEWTON_SEED_REBUILD -ErrorAction SilentlyContinue
$env:OMNISIM_REQUIRE_NEWTON      = "1"   # fail loudly instead of silently running ODE
$env:OMNISIM_LOG_PATH            = "$outdir\omnisim_log_parity.txt"

# --- probe controller knobs ---------------------------------------------------
$env:G1_PROBE_TRACE = $deployTrace
if ($Ticks -gt 0)      { $env:G1_PROBE_TICKS = "$Ticks" } else { Remove-Item Env:\G1_PROBE_TICKS -ErrorAction SilentlyContinue }
if ($Settle -ge 0)     { $env:G1_PROBE_SETTLE = "$Settle" } else { Remove-Item Env:\G1_PROBE_SETTLE -ErrorAction SilentlyContinue }
if ($Sequence -ne "")  { $env:G1_PROBE_SEQ   = $Sequence } else { Remove-Item Env:\G1_PROBE_SEQ -ErrorAction SilentlyContinue }
if ($RecordSettle)     { $env:G1_PROBE_RECORD_SETTLE = "1"; $env:OMNISIM_DEBUG_JOINTS = "$outdir\deploy_joint_diag.txt"; $env:OMNISIM_NEWTON_SAVE_MJCF = "$outdir\deploy_model.xml" } else { Remove-Item Env:\G1_PROBE_RECORD_SETTLE -ErrorAction SilentlyContinue; Remove-Item Env:\OMNISIM_DEBUG_JOINTS -ErrorAction SilentlyContinue; Remove-Item Env:\OMNISIM_NEWTON_SAVE_MJCF -ErrorAction SilentlyContinue }

Write-Host "=== G1 parity probe (deploy) link_com=$($env:OMNISIM_NEWTON_USE_LINK_COM) ke=$($env:OMNISIM_NEWTON_TARGET_KE) kd=$($env:OMNISIM_NEWTON_TARGET_KD) ===" -ForegroundColor Cyan
if ($Gui) {
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 12
}

if (-not (Test-Path $deployTrace)) {
    Write-Host "  NO deploy trace produced -- check $($env:OMNISIM_LOG_PATH)" -ForegroundColor Yellow
    exit 2
}
Write-Host "  deploy trace: $deployTrace" -ForegroundColor Green

if ($Compare) {
    $tk = if ($Ticks -gt 0) { $Ticks } else { "" }
    Write-Host "`n=== trainer trace (matching lane) ===" -ForegroundColor Cyan
    if ($Free) {
        # free base on the ground (matches g1_parity_probe_free.wbt: spawn 0.78, ground on)
        $targs = @("$root\projects\policies\research\training\g1_parity_probe.py", "--free-base", "--out", $trainerTrace, "--spawn-z", "0.78")
    } else {
        $targs = @("$root\projects\policies\research\training\g1_parity_probe.py", "--static-base", "--no-ground", "--out", $trainerTrace, "--spawn-z", "1.20")
    }
    if ($tk -ne "") { $targs += @("--ticks", "$tk") }
    if ($Settle -ge 0) { $targs += @("--settle", "$Settle") }
    if ($Sequence -ne "") { $targs += @("--sequence", $Sequence) }
    if ($RecordSettle) { $targs += @("--record-settle") }
    python -u @targs
    Write-Host "`n=== parity diff ===" -ForegroundColor Cyan
    python -u "$root\projects\policies\research\training\g1_parity_compare.py" $trainerTrace $deployTrace
}
