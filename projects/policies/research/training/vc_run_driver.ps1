# VELOCITY-CONDITIONED run: warm from shape-c8 (the deployable walker), add
# --vx-cmd-max so the policy learns a commandable forward speed INCLUDING 0
# (= stand). Commanding 0 -> decelerate + stand; any speed -> walk: the
# stop-in-the-middle milestone in ONE policy, no fragile hand-off. Kept close
# to c8's recipe (shape reward, no COM/adaptive-phase) to stay deployable.
# Writes long_c*.train.log for the live dashboard.
# Usage: vc_run_driver.ps1 [-Chunks 10] [-ItersPerChunk 1000]
param([int]$Chunks = 10, [int]$ItersPerChunk = 1000)
$ErrorActionPreference = "Continue"
$root   = (Resolve-Path "$PSScriptRoot\..\..\..").Path
$trainer= "$root\projects\policies\research\training\gpu_mjwarp_g1_walk_trainer.py"
$mjcf   = "$root\projects\robots\unitree\g1\urdf\g1_full_kp100.mjcf.xml"
$runtag = "gpu_g1_walk29_vc"
$warm   = "$root\projects\policies\research\training\runs\gpu_g1_walk26_shape_c8\policy.pt"

$arch = "--obs-stack 4 --obs-lookahead 0.1,0.4 --hidden-dims 512,512,512 --vx-cmd-max 0.45 --vx-phase-freeze 0.0"
$gait = "--hold-arms --gait-model human --gait-style winter --gait-a-arm 0.25 --gait-a-lat 0.05 --vx-target 0.4 --gait-freq 1.3 --gait-ramp-s 2.0 --seed-gait-pose --rest-start-frac 0.4 --gait-hip-scale 0.9"
$dr   = "--dr-mass-scale 0.10 --dr-friction-scale 0.20 --dr-damping-scale 0.20 --dr-actuator-kp-scale 0.15 --dr-actuator-kv-scale 0.15 --dr-push-prob 0.02 --dr-push-vmax 0.6 --dr-obs-noise 0.01 --dr-action-latency-max 1 --dr-init-q-band 0.08 --dr-init-vx-bias 0.4 --dr-init-tilt-band 0.05 --dr-init-vel-band 0.1"
# c8's style recipe (shape reward) + velocity conditioning. No COM/adaptive-
# phase (those broke Newton deploy); stay close to the deployable c8.
# Loose velocity sigma everywhere (the robust, proven recipe). Tried tightening
# it at the stand (--vel-sigma-stand 0.015) to kill the ~0.13 m/s creep, but the
# sharp reward made the policy over-actuate the launch-hold stand -> launch fall.
# The gait-clock freeze and the deploy pure-pose stand both also broke (freeze =
# OOD launch; pure pose = laterally unstable). So: robust walk + a policy-based
# stand with a mild residual creep. vx-phase-freeze OFF.
$rw   = "--rw-sched -5 --rw-slip -0.5 --vel-l1 -0.3 --upright 1.5 --vel 2.0 --rw-shape 2.0 --shape-sigma 0.01"

for ($c = 1; $c -le $Chunks; $c++) {
    if ($c -le 6) { $lr = "1e-4"; $ent = "0.0005"; $cl = "-1.6" }
    else          { $lr = "5e-5"; $ent = "0.0003"; $cl = "-1.8" }
    $save = "$root\projects\policies\research\training\runs\$runtag`_c$c\policy.pt"
    $init = "--init-from `"$warm`""
    if ($c -gt 1) { $init = "--init-from `"$root\projects\policies\research\training\runs\$runtag`_c$($c-1)\policy.pt`"" }
    $cmd = "python -u `"$trainer`" --envs 4096 --iters $ItersPerChunk --rollout 12 --max-ep 1875 --lr $lr --ent-coef $ent --log-std-clamp $cl --dr-seed $(150 + $c) $arch $gait $dr $rw $init --mjcf `"$mjcf`" --save `"$save`""
    Invoke-Expression "$cmd *> `"$root\_scratch\long_c$c.train.log`""
    if ($LASTEXITCODE -ne 0) { "TRAIN FAILED chunk $c" | Add-Content "$root\_scratch\long_run_curve.csv"; break }
}
"DONE vc chunks=$Chunks" | Add-Content "$root\_scratch\long_run_curve.csv"
