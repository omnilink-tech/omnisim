# SHAPE-REWARD run: warm-start from c16, replace the per-joint swing-track
# with the HOLISTIC shape reward (--rw-shape: match the ghost silhouette via
# FK knee+toe Cartesian keypoints). Calibration (400 iters) already pulled
# the ankle 1.23->0.83 while holding knee 0.87/hip 0.67 and still walking;
# this run consolidates the balanced shape-matched gait. Chunked + warm-
# chained + self-checkpointing; writes train logs to long_c*.train.log so
# the existing live dashboard + plot work unchanged.
#
# Usage: shape_run_driver.ps1 [-Chunks 8] [-ItersPerChunk 1000]
param([int]$Chunks = 8, [int]$ItersPerChunk = 1000)
$ErrorActionPreference = "Continue"
$root   = (Resolve-Path "$PSScriptRoot\..\..\..").Path
$trainer= "$root\projects\policies\research\training\gpu_mjwarp_g1_walk_trainer.py"
$mjcf   = "$root\projects\robots\unitree\g1\urdf\g1_full_kp100.mjcf.xml"
$runtag = "gpu_g1_walk26_shape"
$warm   = "$root\projects\policies\research\training\runs\gpu_g1_walk25_long_c16\policy.pt"

$arch = "--obs-stack 4 --obs-lookahead 0.1,0.4 --hidden-dims 512,512,512"
$gait = "--hold-arms --gait-model human --gait-style winter --gait-a-arm 0.25 --gait-a-lat 0.05 --vx-target 0.4 --gait-freq 1.3 --gait-ramp-s 2.0 --seed-gait-pose --rest-start-frac 0.4 --gait-hip-scale 0.9"
$dr   = "--dr-mass-scale 0.10 --dr-friction-scale 0.20 --dr-damping-scale 0.20 --dr-actuator-kp-scale 0.15 --dr-actuator-kv-scale 0.15 --dr-push-prob 0.01 --dr-push-vmax 0.5 --dr-obs-noise 0.01 --dr-action-latency-max 1 --dr-init-q-band 0.08 --dr-init-vx-bias 0.4 --dr-init-tilt-band 0.05 --dr-init-vel-band 0.1"
$rw   = "--rw-sched -5 --rw-slip -0.5 --vel-l1 -0.3 --upright 1.5 --vel 2.0 --rw-shape 2.5 --shape-sigma 0.01"

for ($c = 1; $c -le $Chunks; $c++) {
    if ($c -le 5) { $lr = "1e-4"; $ent = "0.0005"; $cl = "-1.65" }
    else          { $lr = "5e-5"; $ent = "0.0003"; $cl = "-1.8" }
    $save = "$root\projects\policies\research\training\runs\$runtag`_c$c\policy.pt"
    $init = "--init-from `"$warm`""
    if ($c -gt 1) { $init = "--init-from `"$root\projects\policies\research\training\runs\$runtag`_c$($c-1)\policy.pt`"" }
    $cmd = "python -u `"$trainer`" --envs 4096 --iters $ItersPerChunk --rollout 12 --max-ep 1875 --lr $lr --ent-coef $ent --log-std-clamp $cl --dr-seed $(120 + $c) $arch $gait $dr $rw $init --mjcf `"$mjcf`" --save `"$save`""
    Invoke-Expression "$cmd *> `"$root\_scratch\long_c$c.train.log`""
    if ($LASTEXITCODE -ne 0) { "TRAIN FAILED chunk $c" | Add-Content "$root\_scratch\long_run_curve.csv"; break }
}
"DONE shape chunks=$Chunks" | Add-Content "$root\_scratch\long_run_curve.csv"
