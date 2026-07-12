# WEIGHT-TRANSFER run: warm from shape-c8, add the COM-over-stance-foot
# reward + torso-stillness + the ADAPTIVE PHASE + a bigger sway, to get
# DELIBERATE, posture-stable stepping (the drunk-gait fix, S14.4). The COM
# reward should narrow the stance NATURALLY (supported COM needs no wide
# base) instead of forcing it. rest-starts train launch recovery (the
# calibration walked 12.5m in-trainer but had a sim2deploy launch gap).
# Writes long_c*.train.log for the live dashboard.
# Usage: com_run_driver.ps1 [-Chunks 10] [-ItersPerChunk 1000]
param([int]$Chunks = 10, [int]$ItersPerChunk = 1000)
$ErrorActionPreference = "Continue"
$root   = (Resolve-Path "$PSScriptRoot\..\..\..").Path
$trainer= "$root\projects\policies\research\training\gpu_mjwarp_g1_walk_trainer.py"
$mjcf   = "$root\projects\robots\unitree\g1\urdf\g1_full_kp100.mjcf.xml"
$runtag = "gpu_g1_walk28_com"
$warm   = "$root\projects\policies\research\training\runs\gpu_g1_walk26_shape_c8\policy.pt"

$arch = "--obs-stack 4 --obs-lookahead 0.1,0.4 --hidden-dims 512,512,512"
$gait = "--hold-arms --gait-model human --gait-style winter --gait-a-arm 0.25 --gait-a-lat 0.08 --vx-target 0.4 --gait-freq 1.3 --gait-ramp-s 2.0 --seed-gait-pose --rest-start-frac 0.4 --gait-hip-scale 0.9"
$dr   = "--dr-mass-scale 0.10 --dr-friction-scale 0.20 --dr-damping-scale 0.20 --dr-actuator-kp-scale 0.15 --dr-actuator-kv-scale 0.15 --dr-push-prob 0.03 --dr-push-vmax 0.8 --dr-obs-noise 0.01 --dr-action-latency-max 1 --dr-init-q-band 0.08 --dr-init-vx-bias 0.4 --dr-init-tilt-band 0.06 --dr-init-vel-band 0.12"
# COM-over-stance (weight transfer) + torso stillness + adaptive phase, on top
# of the sagittal shape reward. NO forced lateral narrowing (COM does it).
$rw   = "--rw-sched -5 --rw-slip -0.5 --vel-l1 -0.3 --upright 1.5 --vel 2.0 --rw-shape 2.0 --shape-sigma 0.01 --rw-com 2.0 --com-sigma 0.004 --rw-torso -0.5 --phase-gate-tilt 4.0 --phase-gate-rate 0.03 --phase-gate-floor 0.2"

for ($c = 1; $c -le $Chunks; $c++) {
    if ($c -le 6) { $lr = "1e-4"; $ent = "0.0005"; $cl = "-1.6" }
    else          { $lr = "5e-5"; $ent = "0.0003"; $cl = "-1.8" }
    $save = "$root\projects\policies\research\training\runs\$runtag`_c$c\policy.pt"
    $init = "--init-from `"$warm`""
    if ($c -gt 1) { $init = "--init-from `"$root\projects\policies\research\training\runs\$runtag`_c$($c-1)\policy.pt`"" }
    $cmd = "python -u `"$trainer`" --envs 4096 --iters $ItersPerChunk --rollout 12 --max-ep 1875 --lr $lr --ent-coef $ent --log-std-clamp $cl --dr-seed $(140 + $c) $arch $gait $dr $rw $init --mjcf `"$mjcf`" --save `"$save`""
    Invoke-Expression "$cmd *> `"$root\_scratch\long_c$c.train.log`""
    if ($LASTEXITCODE -ne 0) { "TRAIN FAILED chunk $c" | Add-Content "$root\_scratch\long_run_curve.csv"; break }
}
"DONE com chunks=$Chunks" | Add-Content "$root\_scratch\long_run_curve.csv"
