# OmniTug 500 WAREHOUSE COURIER — OmniLink natural-language pick-and-deliver demo.
#
# An operator (or the OmniLink agent) tells the rover, in plain language, which
# package to pick from which bay and which dock to deliver it to. The rover
# A*-routes the aisle grid (known facility map), loads the package onto its deck,
# drives to the dock, and sets it down. Multi-stop routes supported.
#
# Usage:
#   powershell -File scripts/dev/run_omnitug500_courier.ps1            # windowed, interactive chat
#   powershell -File scripts/dev/run_omnitug500_courier.ps1 -SelfTest  # headless smoke + numeric verify
#   powershell -File scripts/dev/run_omnitug500_courier.ps1 -Regen     # regenerate the world first
#
# Interactive: right-click the rover -> Show Robot Window, then type e.g.
#   "take the package from bay B to dock 2"
#   "collect from bay A and bay C, deliver both to dock 3"
#   "go to bay E"   /   "return to the charging dock"   /   "status"   /   "stop"
# Offline it uses the controller's regex router; set OMNI_KEY for the OmniLink agent.
param(
    [switch]$SelfTest,
    [switch]$Regen,
    [int]$Duration = 600,
    [int]$Port = 8765
)
$root  = (Resolve-Path "$PSScriptRoot\..\..").Path
$world = "$root\projects\robots\omnisim\omnitug500\worlds\omnitug500_courier.omniworld"

if ($Regen) {
    Write-Host "Regenerating world + layout..." -ForegroundColor Cyan
    python "$root\scripts\dev\gen_courier_world.py"
}
if (-not (Test-Path $world)) {
    Write-Host "World missing; generating it..." -ForegroundColor Yellow
    python "$root\scripts\dev\gen_courier_world.py"
}

$env:OMNISIM_HOME = $root
$env:PYTHONUTF8   = "1"

if ($SelfTest) {
    # Newton physics must drive (the rover carry + contacts are tuned for it).
    $env:OMNISIM_REQUIRE_NEWTON = "1"
    $env:OMNISIM_LOG_PATH = "$root\_scratch\omnitug500_courier_selftest.log"
    Write-Host "=== OMNITUG500 courier self-test (headless) ===" -ForegroundColor Cyan
    $sim = Start-Process -FilePath "python" `
        -ArgumentList "-u","$root\scripts\dev\headless_runner.py",$world,"--duration","$Duration" `
        -PassThru -WindowStyle Hidden
    try {
        python -u "$root\scripts\dev\omnitug500_courier_selftest.py" --port $Port
        $code = $LASTEXITCODE
    } finally {
        # Kill the headless_runner python AND the omnisim-bin it spawned (a bare
        # Stop-Process on the parent orphans the grandchild simulator).
        if ($sim) {
            Get-CimInstance Win32_Process -Filter "ParentProcessId=$($sim.Id)" -ErrorAction SilentlyContinue |
                Where-Object { $_.Name -like "omnisim-bin*" -or $_.Name -like "webots-bin*" } |
                ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
            if (-not $sim.HasExited) { Stop-Process -Id $sim.Id -Force -ErrorAction SilentlyContinue }
        }
    }
    if ($code -eq 0) { Write-Host "SELF-TEST PASS" -ForegroundColor Green }
    else             { Write-Host "SELF-TEST FAIL (exit $code)" -ForegroundColor Yellow }
    exit $code
}

Write-Host "=== OMNITUG500 Warehouse Courier (windowed) ===" -ForegroundColor Cyan
Write-Host "Right-click the rover -> Show Robot Window, then type:" -ForegroundColor Gray
Write-Host '  "take the package from bay B to dock 2"' -ForegroundColor Gray
Write-Host '  "collect from bay A and bay C, deliver to dock 3"' -ForegroundColor Gray
if (-not $env:OMNI_KEY) {
    Write-Host "(OMNI_KEY not set -> offline regex router. Set OMNI_KEY for the OmniLink agent.)" -ForegroundColor DarkGray
}
python -u "$root\scripts\dev\headless_runner.py" $world --gui --realtime --duration $Duration
