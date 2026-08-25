# Run N parallel instances of the OMNIARM6 bin-picking + sort demo headless and report
# each one's result. Proves the demo is consistent + repeatable under load: every
# instance should empty + sort all 36 cubes within the time budget.
#
# Each instance is an independent omnisim-bin process (the supported multi-instance
# pattern, AGENTS.md 3e): it auto-allocates its TCP port and gets a UNIQUE log path,
# result file, and warm-up token so they never collide.
#
# Usage:  powershell -ExecutionPolicy Bypass -File run_bin_picking_parallel.ps1 [-N 5] [-Duration 1600]
param(
    [int]$N = 5,
    [int]$Duration = 1600
)
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$bin = "$root\msys64\mingw64\bin\omnisim-bin.exe"
$world = "$root\projects\samples\demos\worlds\flagship\omniarm6_bin_picking.omniworld"
$stamp = [int][double]::Parse((Get-Date -UFormat %s))
# clear any stale warm-up sentinels so every instance warms up fresh
Remove-Item "$env:TEMP\_omnisim_warmup_*.flag" -ErrorAction SilentlyContinue

Write-Host "Launching $N parallel bin-picking instances (duration ${Duration}s each)..."
$procs = @()
for ($i = 1; $i -le $N; $i++) {
    $out = "$root\_par_out_$i.txt"
    $log = "$root\_par_log_$i.txt"
    Remove-Item $out, $log -ErrorAction SilentlyContinue
    $envPairs = @{
        OMNISIM_HOME                      = $root
        OMNISIM_NEWTON_STATICS            = "1"
        OMNISIM_NEWTON_COMPOUND_COLLIDERS = "1"
        OMNISIM_NEWTON_GROUND_MU          = "1.5"
        BINPICK_MOVE_DUR                  = "1.2"
        OMNISIM_LOG_PATH                  = $log
        BINPICK_OUT                       = $out
        OMNISIM_WARMUP_TOKEN              = "par_${stamp}_$i"
        BINPICK_AUTOQUIT                  = "1"
    }
    foreach ($k in $envPairs.Keys) { Set-Item -Path "Env:$k" -Value $envPairs[$k] }
    $p = Start-Process -FilePath $bin `
        -ArgumentList @($world, "--batch", "--mode=fast", "--no-rendering", "--minimize", "--stdout", "--stderr") `
        -PassThru -WindowStyle Hidden
    $procs += [pscustomobject]@{ i = $i; proc = $p; out = $out }
    Write-Host "  instance $i -> pid $($p.Id), out=$out"
    Start-Sleep -Milliseconds 800        # stagger launches so the warm-up reloads don't collide
}

Write-Host "Waiting for all $N instances (timeout ${Duration}s)..."
$deadline = (Get-Date).AddSeconds($Duration + 120)
foreach ($e in $procs) {
    $remaining = ($deadline - (Get-Date)).TotalSeconds
    if ($remaining -gt 0) { $e.proc.WaitForExit([int]($remaining * 1000)) | Out-Null }
    if (-not $e.proc.HasExited) { try { $e.proc.Kill() } catch {} }
}

Write-Host "`n===== PARALLEL RESULTS ($N instances) ====="
$pass = 0
foreach ($e in $procs) {
    $line = "(no result)"
    if (Test-Path $e.out) {
        $r = Select-String -Path $e.out -Pattern "RESULT" | Select-Object -Last 1
        if ($r) { $line = $r.Line }
    }
    # Repeatability check: the suction demo (real vacuum cup + flush, velocity-pinned
    # hold) sorts the whole bin. All instances must match the same deterministic result.
    $ok = if ($line -match "sorted=36/36") { $pass++; "PASS" } else { "----" }
    Write-Host ("  [{0}] {1}  {2}" -f $e.i, $ok, $line)
}
Write-Host "===== $pass / $N matched the expected sorted=36/36 ====="
