# Robot-agnostic deterministic physics-parity probe -- DEPLOY launcher.
#
# Runs <robot>_parity_probe(.wbt|_free.wbt) in omnisim-bin (generic parity_probe
# controller), dumps a per-tick deploy trace, then (with -Compare) runs the
# matching trainer trace (robot_parity_probe.py) and diffs them (g1_parity_compare.py).
# Newton env (gains/substeps/mu) is sourced from the robot's parity-probe spec;
# OMNISIM_NEWTON_USE_LINK_COM=1 is the link-COM parity mend.
#
#   powershell -File scripts/dev/run_parity_probe.ps1 -Robot h1 -Compare
#   powershell -File scripts/dev/run_parity_probe.ps1 -Robot valkyrie -Free -Sequence hold -Compare
param(
    [Parameter(Mandatory=$true)][string]$Robot,
    [int]$Duration = 150,
    [switch]$Gui,
    [int]$Ticks = 0,
    [int]$Settle = -1,
    [string]$Sequence = "sinusoid",
    [switch]$Compare,
    [switch]$RecordSettle,
    [switch]$Free,
    [switch]$NoLinkCom
)
$ErrorActionPreference = "Stop"
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = if ($Free) { "$root\projects\policies\worlds\${Robot}_parity_probe_free.wbt" }
         else       { "$root\projects\policies\worlds\${Robot}_parity_probe.wbt" }
if (-not (Test-Path $world)) { Write-Error "no world $world (run gen_parity_probe_world.py --robot $Robot)"; exit 1 }
$outdir = "$root\_scratch\parity"
New-Item -ItemType Directory -Force -Path $outdir | Out-Null
$deployTrace  = "$outdir\${Robot}_deploy.json"
$trainerTrace = "$outdir\${Robot}_trainer.json"
Remove-Item $deployTrace -ErrorAction SilentlyContinue

$env:OMNISIM_HOME = $root
$env:PYTHONUTF8   = "1"
$env:PYTHONPATH   = $root

# --- gains / substeps / mu / lifted-z from the parity-probe spec --------------
$emit = @"
import json, sys
sys.path.insert(0, r'$root')
from projects.policies.research.backends import parity_probe_spec as PP
s = PP.load('$Robot')
print(json.dumps({'ke': s.ke, 'kd': s.kd, 'substeps': s.substeps,
                  'mu': s.ground_mu, 'spawn_z': s.spawn_z, 'lifted': s.lifted_z()}))
"@
$cfg = ($emit | python -) | ConvertFrom-Json
if ($LASTEXITCODE -ne 0) { Write-Error "failed to load spec for $Robot"; exit 1 }

$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "$($cfg.substeps)"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_NEWTON_MJWARP       = "1"
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "0"   # spawn straight; the probe ramps+settles to the pose
$env:OMNISIM_NEWTON_TARGET_KE    = "$($cfg.ke)"
$env:OMNISIM_NEWTON_TARGET_KD    = "$($cfg.kd)"
$env:OMNISIM_NEWTON_GROUND_MU    = "$($cfg.mu)"
if ($NoLinkCom) { Remove-Item Env:\OMNISIM_NEWTON_USE_LINK_COM -ErrorAction SilentlyContinue }
else            { $env:OMNISIM_NEWTON_USE_LINK_COM = "1" }   # link-COM parity mend
$env:OMNISIM_REQUIRE_NEWTON      = "1"
$env:OMNISIM_LOG_PATH            = "$outdir\omnisim_log_${Robot}.txt"

$env:PROBE_ROBOT = $Robot
$env:PROBE_TRACE = $deployTrace
if ($Ticks -gt 0)   { $env:PROBE_TICKS = "$Ticks" } else { Remove-Item Env:\PROBE_TICKS -ErrorAction SilentlyContinue }
if ($Settle -ge 0)  { $env:PROBE_SETTLE = "$Settle" } else { Remove-Item Env:\PROBE_SETTLE -ErrorAction SilentlyContinue }
$env:PROBE_SEQ = $Sequence
if ($RecordSettle) { $env:PROBE_RECORD_SETTLE = "1"; $env:OMNISIM_DEBUG_JOINTS = "$outdir\${Robot}_joint_diag.txt"; $env:OMNISIM_NEWTON_SAVE_MJCF = "$outdir\${Robot}_deploy_model.xml" }
else { Remove-Item Env:\PROBE_RECORD_SETTLE -ErrorAction SilentlyContinue; Remove-Item Env:\OMNISIM_DEBUG_JOINTS -ErrorAction SilentlyContinue; Remove-Item Env:\OMNISIM_NEWTON_SAVE_MJCF -ErrorAction SilentlyContinue }

Write-Host "=== $Robot parity probe (deploy) lane=$(if($Free){'free'}else{'welded'}) ke=$($cfg.ke) kd=$($cfg.kd) link_com=$($env:OMNISIM_NEWTON_USE_LINK_COM) ===" -ForegroundColor Cyan
if ($Gui) {
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 12
}
if (-not (Test-Path $deployTrace)) { Write-Host "  NO deploy trace -- check $($env:OMNISIM_LOG_PATH)" -ForegroundColor Yellow; exit 2 }
Write-Host "  deploy trace: $deployTrace" -ForegroundColor Green

if ($Compare) {
    Write-Host "`n=== $Robot trainer trace (matching lane) ===" -ForegroundColor Cyan
    $targs = @("$root\projects\policies\research\training\robot_parity_probe.py", "--robot", $Robot, "--out", $trainerTrace, "--sequence", $Sequence)
    if ($Free) { $targs += @("--free-base", "--spawn-z", "$($cfg.spawn_z)") }
    else       { $targs += @("--static-base", "--no-ground", "--spawn-z", "$($cfg.lifted)") }
    if ($Ticks -gt 0)  { $targs += @("--ticks", "$Ticks") }
    if ($Settle -ge 0) { $targs += @("--settle", "$Settle") }
    if ($RecordSettle) { $targs += @("--record-settle") }
    python -u @targs
    Write-Host "`n=== $Robot parity diff ===" -ForegroundColor Cyan
    python -u "$root\projects\policies\research\training\g1_parity_compare.py" $trainerTrace $deployTrace
}
