# Run the DETERMINISTIC BRAIN walk (projects/policies/control/brain/g1_brain.py).
#
# Same world / physics as run_g1_walk_vc_deploy.ps1, but instead of the ONNX
# policy the G1 is driven by the hand-written deterministic controller:
#   targets = ghost feedforward + capture-point foot placement + ankle/hip
#             balance + height/posture/phase feedback.
#
# Baseline to beat: the OPEN-LOOP ghost (run_g1_openloop_baseline.ps1) falls at
# t=1.30s. Any working brain layer must survive longer and travel further.
#
# Tune gains WITHOUT editing code by exporting G1_BRAIN_* before calling, e.g.
#   $env:G1_BRAIN_KP_AR='-4.0'; $env:G1_BRAIN_KFP_Y='0.15'
#   & scripts/dev/run_g1_brain.ps1 -Duration 40
# (see BrainConfig in g1_brain.py for the full list).
#
# Usage:  run_g1_brain.ps1 [-Duration 40] [-Gui] [-CpuEngine] [-Debug]
param(
    [int]$Duration = 40,
    [double]$Ke = 400,     # STIFF position PD (stand-proven). The deterministic
    [double]$Kd = 60,      # brain tracks the ghost faithfully at high gain; the
    #                        soft KE=100 the RL walk used lets the pose sag + tip.
    [double]$RampS = 1.5,  # gentle ramp into the deep squat (avoid the stiff fold)
    [switch]$Gui,
    [switch]$CpuEngine,
    [switch]$Debug
)
$root  = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\policies\worlds\g1_walk_arms_deploy.wbt"
$log   = "$root\_scratch\g1_brain.log"
$trace = "$root\_scratch\g1_brain_trace.csv"
Remove-Item $log, $trace -ErrorAction SilentlyContinue

# --- turn the brain ON ---
$env:G1_BRAIN = "1"
if ($Debug) { $env:G1_DEBUG = "1" } else { Remove-Item Env:G1_DEBUG -ErrorAction SilentlyContinue }

# --- gait recipe (the controller's GP, kept consistent with the brain) ---
$env:G1_GAIT_MODEL    = "human"
$env:G1_GAIT_STYLE    = "winter"
$env:G1_GAIT_VX       = "0.4"
$env:G1_GAIT_RAMP_S   = "2.0"
$env:G1_GAIT_A_LAT    = "0.05"
$env:G1_GAIT_A_ARM    = "0.25"
$env:G1_GAIT_HIP_SCALE = "0.9"
Remove-Item Env:G1_START_PHASE -ErrorAction SilentlyContinue
# no policy needed
Remove-Item Env:G1_POLICY_ONNX -ErrorAction SilentlyContinue
$env:G1_BALANCE_FALLBACK = "0"

# --- engine: match the trainer/deploy exactly (mujoco_warp, kp100) ---
$env:OMNISIM_HOME                = $root
$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
if ($CpuEngine) { Remove-Item Env:OMNISIM_NEWTON_MJWARP -ErrorAction SilentlyContinue }
else            { $env:OMNISIM_NEWTON_MJWARP = "1" }
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"
$env:OMNISIM_NEWTON_TARGET_KE    = "$Ke"
$env:OMNISIM_NEWTON_TARGET_KD    = "$Kd"
$env:OMNISIM_NEWTON_GROUND_MU    = "2.0"
$env:G1_RAMP_S                   = "$RampS"   # gentle settle into the deep squat
$env:OMNISIM_LOG_PATH = "$root\_scratch\omnisim_log_brain.txt"
$env:G1_DEPLOY_LOG    = $log
$env:G1_TRACE_REF     = $trace

Write-Host "BRAIN walk: ${Duration}s wall (note: --duration is WALL clock)..."
if ($Gui) {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration --gui --realtime 2>&1 | Select-Object -Last 4
} else {
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration 2>&1 | Select-Object -Last 4
}

Write-Host "`n===== BRAIN WALK RESULT (baseline to beat: fall@1.30s) ====="
if (Test-Path $log) {
    $fallLine = Select-String -Path $log -Pattern "FALL" | Select-Object -First 1
    $last = Get-Content $log | Select-Object -Last 1
    if ($fallLine) { Write-Host "  first FALL: $($fallLine.Line.Trim())" }
    else           { Write-Host "  NO FALL (survived upright the whole run)" }
    Write-Host "  last: $last"
} else {
    Write-Host "  (no deploy log produced)"
}
