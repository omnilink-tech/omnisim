# LIVE OmniSim GUI demo: G1 STANDS robustly and WAVES, in the real product GUI.
#
# This is the payoff of the composite-inertia engine fix (WbSolid.cpp): the deploy's fixed-joint
# rollup used to keep only the LEADER link's inertia/CoM when merging fixed-jointed children,
# building a laterally-asymmetric, wrong-inertia model that tipped under SolverMuJoCo. The gated
# OMNISIM_NEWTON_COMPOSITE_INERTIA=1 path composes the TRUE mass+CoM+inertia (parallel-axis), so
# the deploy model == the proven native add_urdf model -> the robot stands with a pure stiff
# pose-hold (NO RL, NO balance feedback): verified 1364 s ZERO falls headless.
#
# The wave is a pure FRONTAL-PLANE right-arm bob (straight arm, shoulder-roll oscillation). The
# stand's forward CoM margin is razor-thin, so the wave deliberately avoids any forward arm
# component (a bent elbow or forward shoulder-pitch tips it). Stand+wave verified 385 s clean.
#
# Usage: powershell -File scripts/dev/run_g1_standwave_composite_demo.ps1 [-Duration 90] [-Port 1240] [-NoWave]
param(
    [int]$Duration = 90,
    [int]$Port = 1240,
    [switch]$NoWave
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\worlds\g1_standwave_deploy_solo.wbt"
$csv   = "$root\_scratch\g1_gentle_wave.csv"
$log   = "$root\_scratch\standwave_composite_demo.log"

$env:OMNISIM_HOME = $root
$env:PYTHONUTF8   = "1"
# --- deploy-faithful Newton/MuJoCo solver config ---
$env:OMNISIM_NEWTON_STATICS         = "1"
$env:OMNISIM_NEWTON_SUBSTEPS        = "8"
$env:OMNISIM_NEWTON_FORCE_MUJOCO    = "1"
$env:OMNISIM_NEWTON_MJWARP          = "1"
$env:OMNISIM_URDF_USE_INERTIA       = "1"
$env:OMNISIM_NEWTON_BASE_GUARD      = "1"
$env:OMNISIM_NEWTON_SEED_POSE       = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD    = "1"
$env:OMNISIM_NEWTON_GROUND_MU       = "2.0"
$env:OMNISIM_NEWTON_COMPOSITE_INERTIA = "1"   # <-- THE ENGINE FIX
$env:OMNISIM_NEWTON_TARGET_KE       = "800"   # stiff position PD -> deterministic pose-hold stand
$env:OMNISIM_NEWTON_TARGET_KD       = "40"
# Arms stay at the global leg stiffness (ke=800): the stiff arm tracks tightly so it does NOT
# swing and perturb the razor-thin-margin stand (any softer ke lets the arm lag/swing during the
# raise and tips it). The arm looks smooth NOT by softening it but by making the wave SLOW
# (~0.3 Hz, quasi-static vs the arm's ~0.05 s settling time -> near-zero tracking lag -> no torque
# saturation -> no whip). So we deliberately do NOT set OMNISIM_NEWTON_ARM_KE here.
$env:OMNISIM_NEWTON_ARM_KE          = $null
$env:OMNISIM_NEWTON_ARM_KD          = $null
# --- pure deterministic stand: NO RL policy, NO balance feedback ---
$env:G1_BALANCE_FALLBACK = "1"
$env:G1_ARMS_ANKLE_KP    = "0"
$env:G1_ARMS_ANKLE_KD    = "0"
$env:G1_ARM_POSE         = "down"
$env:G1_POLICY_ONNX      = $null
$env:G1_DEPLOY_LOG       = $log
$env:OMNISIM_LOG_PATH    = "$root\_scratch\omnisim_log_standwave_composite.txt"

if ($NoWave) {
    $env:G1_STANDWAVE_REF = $null
    Write-Host "Mode: STAND only (no wave)."
} else {
    if (-not (Test-Path $csv)) { python -u "$root\_scratch\make_gentle_wave.py" }
    $env:G1_STANDWAVE_REF = $csv
    Write-Host "Mode: STAND + frontal-plane WAVE ($csv)."
}

# Launch the binary DIRECTLY (NOT headless_runner --gui): the GUI helper omits --batch, and
# WITHOUT --batch OmniSim runs an interactive stepping path that tips the robot at ~1.8s. With
# --batch the SAME visible/rendered window stands rock-solid (verified 115s). We keep rendering
# ON (no --no-rendering, no --minimize) so the window is visible, add --batch for the stable
# stepping path, and --mode=realtime to pace 1x for watching. OMNISIM_NO_WARMUP=1 = cold: the
# warm-reload rebuilds the Newton model and the composite-inertia fix does NOT survive it.
$env:OMNISIM_NO_WARMUP = "1"
$env:PATH = "$root\msys64\mingw64\bin;" + $env:PATH
$bin = "$root\msys64\mingw64\bin\omnisim-bin.exe"
$argList = @("--mode=realtime", "--batch", $world, "--stdout", "--stderr", "--port=$Port")
Write-Host "Launching LIVE OmniSim GUI (visible, --batch, real-time, ${Duration}s, port $Port). Close the window to stop early."
$proc = Start-Process -FilePath $bin -ArgumentList $argList -PassThru
Start-Sleep -Seconds $Duration
if (-not $proc.HasExited) { $proc.Kill(); $proc.WaitForExit() }

Write-Host "`n===== STAND+WAVE (composite) RESULT ====="
if (Test-Path $log) {
    $fall = (Select-String -Path $log -Pattern "FALL" | Measure-Object).Count
    $last = Get-Content $log | Select-Object -Last 1
    if ($fall -eq 0) { Write-Host "  PASS - no fall. last: $last" }
    else             { Write-Host "  fell (FALL lines=$fall). last: $last" }
}
