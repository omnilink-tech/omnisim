# VELOCITY-CONDITIONED OmniQuad run: warm from gpu_omniquad_walk_main (the deployable
# straight walker), add --vx-cmd-max so ONE policy learns a commandable forward
# speed INCLUDING 0 (= stand). Command 0 -> decelerate + stand on four feet
# (statically stable for a quadruped); any speed -> walk. The G1 walk29_vc
# "stop-in-the-middle" milestone, ported to OmniQuad (see vc_run_driver.ps1).
#
# Kept close to the c6 walker recipe (trot reference + residual, gentle DR,
# foot sched/slip rewards, vel-L1). The vel-L1 term penalises |vx - vx_cmd|
# linearly, which is what suppresses the forward creep the non-VC policy showed
# during a commanded stand. Lowish lr from chunk 1 to preserve the warm walk.
# Writes omniquad_vc_c*.train.log for inspection.
# Usage: omniquad_vc_run_driver.ps1 [-Chunks 6] [-ItersPerChunk 500]
param([int]$Chunks = 6, [int]$ItersPerChunk = 500)
$ErrorActionPreference = "Continue"
$root    = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path
$env:OMNISIM_HOME = $root
$trainer = "$root\projects\policies\research\training\gpu_mjwarp_omniquad_walk_trainer.py"
$mjcf    = "C:\tmp\omniquad_newton_fixed2.xml"
$runtag  = "gpu_omniquad_vc"
$warm    = "$root\projects\policies\research\inference\policies\gpu_omniquad_walk_main\policy.pt"

$arch = "--vx-cmd-max 0.45"
$gait = "--vx-target 0.4 --gait-freq 1.4 --gait-duty 0.6 --gait-step-height 0.06 --gait-body-h 0.55 --gait-ramp-s 1.0 --seed-gait-pose --rest-start-frac 0.3 --dr-init-vx-bias 0.3"
# c6 reward recipe; vel-L1 supplies the stand-creep gradient (gaussian alone is
# flat far from target). Default gentle DR applies (mass .10/fric .20/kp .15).
$rw   = "--rw-sched -5 --rw-slip -0.5 --vel-l1 -0.3 --upright 1.0 --vel 2.0 --max-ep 1250"

for ($c = 1; $c -le $Chunks; $c++) {
    if ($c -le 4) { $lr = "1e-4"; $ent = "0.0005"; $cl = "-1.6" }
    else          { $lr = "5e-5"; $ent = "0.0003"; $cl = "-1.8" }
    $save = "$root\projects\policies\research\training\runs\$runtag`_c$c\policy.pt"
    if ($c -eq 1) { $init = "--init-from `"$warm`"" }
    else          { $init = "--init-from `"$root\projects\policies\research\training\runs\$runtag`_c$($c-1)\policy.pt`"" }
    $cmd = "python -u `"$trainer`" --envs 4096 --iters $ItersPerChunk --rollout 12 --lr $lr --ent-coef $ent --log-std-clamp $cl --dr-seed $(150 + $c) $arch $gait $rw $init --mjcf `"$mjcf`" --save `"$save`""
    Write-Host "[omniquad-vc] chunk $c -> $save (lr=$lr)"
    Invoke-Expression "$cmd *> `"$root\_scratch\omniquad_vc_c$c.train.log`""
    if ($LASTEXITCODE -ne 0) { Write-Host "TRAIN FAILED chunk $c"; break }
}
Write-Host "DONE omniquad vc chunks=$Chunks"
