# WATCH the G1 rise from a chair, stand 5 s, and sit back down -- mimicking its
# translucent ghost, dynamically balancing the whole way (RL).
#
# Two G1s side by side: a kinematic translucent GHOST that DISPLAYS the shared
# sit->stand->sit reference (or the robot's recorded ACHIEVED motion), and the
# real, Newton-simulated robot that MIMICS it -- FREE base, never pinned, the RL
# policy keeps it balanced through the rise/stand/descend.
#
# Modes:
#   (default)  robot tracks the reference under the RL policy; ghost shows the
#              ACHIEVABLE motion (runs/<Policy>/achieved.csv) if it exists.
#   -Record    record the robot's achieved motion to runs/<Policy>/achieved.csv
#              (ghost shows the idealized reference). Run this ONCE, then re-run
#              without -Record to see the robot match its achievable ghost.
#
# Usage:
#   powershell -File scripts/dev/run_g1_sitstand_mimic.ps1 [-Duration 60] [-Fast] [-Headless] [-Record] [-Policy <name>]
param(
    [int]$Duration = 60,
    [switch]$Fast,
    [switch]$Headless,
    [switch]$Record,
    [string]$Policy = "gpu_g1_sitstand_trackfeas2"
)
$root = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$world = "$root\projects\policies\research\worlds\g1_sitstand_mimic.omniworld"
$runDir = "$root\projects\policies\research\training\runs\$Policy"
$onnx = "$runDir\policy.onnx"
$achieved = "$runDir\achieved.csv"

# The PLANNER reference (ghost-tracking Stage B) the policy was trained to track.
# The controller builds the SAME step-indexed REF_* tables the trainer used.
$refNpz = "$runDir\reference.npz"
if (Test-Path $refNpz) {
    # G1_SITSTAND_ prefix: OmniSim forwards these to the controller process.
    $env:G1_SITSTAND_REF_NPZ = $refNpz
    Write-Host "Planner reference: $refNpz"
} else {
    Remove-Item Env:\G1_SITSTAND_REF_NPZ -ErrorAction SilentlyContinue
    Write-Host "No reference.npz -- controller falls back to the legacy IK reference"
}

$env:OMNISIM_HOME                = $root
$env:OMNISIM_NEWTON_STATICS      = "1"
$env:OMNISIM_NEWTON_SUBSTEPS     = "4"      # 16ms step / 4 = 4ms substep = trainer PHYS_DT
$env:OMNISIM_NEWTON_FORCE_MUJOCO = "1"
$env:OMNISIM_URDF_USE_INERTIA    = "1"
$env:OMNISIM_NEWTON_BASE_GUARD   = "1"
$env:OMNISIM_NEWTON_SEED_POSE    = "1"      # spawn already seated (legs don't fall straight + jam)
$env:OMNISIM_NEWTON_SEED_REBUILD = "1"
$env:OMNISIM_NEWTON_TARGET_KE    = "100"    # MATCH trainer kp100; higher (250) over-amplifies the policy's corrections -> oscillates/falls
$env:G1_SITSTAND_RES_SCALE       = "0.3"    # MUST equal the trainer --res-scale (trackfeas2 = 0.3)

if (Test-Path $onnx) {
    $env:G1_SITSTAND_POLICY = $onnx
    Write-Host "RL policy: $onnx"
} else {
    Remove-Item Env:\G1_SITSTAND_POLICY -ErrorAction SilentlyContinue
    Write-Host "NO policy at $onnx -- FEEDFORWARD-only (will likely fall; train first)"
}

if ($Record) {
    $env:G1_SITSTAND_RECORD = $achieved
    Remove-Item Env:\G1_GHOST_REPLAY -ErrorAction SilentlyContinue
    Write-Host "RECORD mode -> $achieved (ghost shows idealized reference)"
} else {
    Remove-Item Env:\G1_SITSTAND_RECORD -ErrorAction SilentlyContinue
    if (Test-Path $achieved) {
        $env:G1_GHOST_REPLAY = $achieved
        Write-Host "Achievable ghost: replaying $achieved"
    } else {
        Remove-Item Env:\G1_GHOST_REPLAY -ErrorAction SilentlyContinue
        Write-Host "No achieved.csv yet -- ghost shows idealized reference (run -Record first)"
    }
}

$env:G1_SITSTAND_LOG  = "$root\_scratch\g1_sitstand_mimic.log"
$env:OMNISIM_LOG_PATH = "$root\_scratch\omnisim_log_sitstand_mimic.txt"

if ($Headless) {
    Write-Host "Sit-stand mimic (HEADLESS verify) -- $Duration s. Telemetry: $env:G1_SITSTAND_LOG"
    python -u "$root\scripts\dev\headless_runner.py" $world --duration $Duration
} elseif ($Fast) {
    Write-Host "Sit-stand mimic (GUI, unpaced) -- close the window to stop."
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --duration $Duration
} else {
    Write-Host "Sit-stand mimic (GUI, real-time) -- close the window to stop."
    python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
}
