# show_wgpu_demo.ps1 — launch a world in the wgpu MAIN VIEW (full-material + multi-cascade shadows +
# world-general ambient), so you can see the new renderer live. Run it in YOUR OWN terminal (the window
# opens on your desktop and stays until you close it). Optional: pass a world path.
#
#   powershell -File scripts\dev\show_wgpu_demo.ps1
#   powershell -File scripts\dev\show_wgpu_demo.ps1 projects\samples\demos\worlds\showcase\warehouse_husky.wbt
#
param([string]$World = "projects\samples\demos\worlds\showcase\warehouse_husky.wbt")

$repo = Split-Path -Parent (Split-Path -Parent $PSCommandPath)   # scripts\dev -> repo root
$bin  = Join-Path $repo "msys64\mingw64\bin\omnisim-bin.exe"
if (-not (Test-Path $bin)) { Write-Host "omnisim-bin not found at $bin — build it first." -ForegroundColor Red; exit 1 }
if (-not [System.IO.Path]::IsPathRooted($World)) { $World = Join-Path $repo $World }
if (-not (Test-Path $World)) { Write-Host "world not found: $World" -ForegroundColor Red; exit 1 }

$env:WEBOTS_HOME = $repo
$env:OMNISIM_HOME = $repo
$env:OMNISIM_WGPU_MAINVIEW_FORCE = "1"   # route the main view through wgpu (not WREN)
$env:OMNISIM_WGPU_MAINVIEW_CSM   = "1"   # full-material multi-cascade shadow path

Write-Host "Launching the wgpu main view: $World"
Write-Host "(Drag in the viewport to orbit/zoom. Close the window when done.)"
& $bin $World --mode=run --port=1242
