# OmniSim shadow optimization — before and after

The images are unchanged. The shared renderer now reuses stationary local-light
shadows and skips objects outside each light's view when a redraw is needed.

| Scene | Before capture | After capture | Local-shadow draws per warm frame |
|---|---|---|---:|
| House | [Before](O:/omnisim/.local-runs/shadow-pipeline/house-capped/house-0-before.png) | [After](O:/omnisim/.local-runs/shadow-pipeline/house-capped/house-0-after.png) | 4,860 → 0 |
| Warehouse stress variant | [Before](O:/omnisim/.local-runs/shadow-pipeline/warehouse-static/warehouse-0-before.png) | [After](O:/omnisim/.local-runs/shadow-pipeline/warehouse-static/warehouse-0-after.png) | 76,650 → 0 |
| City control | [Before](O:/omnisim/.local-runs/shadow-pipeline/city-capped/city-0-before.png) | [After](O:/omnisim/.local-runs/shadow-pipeline/city-capped/city-0-after.png) | 0 → 0 |

All three scenes were compared twice, reversing the run order. Every image pair
matches exactly in decoded RGB, including repeated runs of the same settings.

| Scene | CPU render phase before | CPU render phase after |
|---|---:|---:|
| House | 3.88–4.00 ms | 2.89–2.96 ms |
| Warehouse stress variant | 14.45–17.49 ms | 4.67–5.18 ms |

Ranges show the two run medians on this laptop (RTX 3060 Laptop GPU, Windows 11,
machine 9722d23d12a3). These are CPU measurements, not GPU timings or FPS gains.
The city had no shadow work to remove; its timing variation did not support a
reliable performance conclusion.

Both sides used the same test binary, 1896×1113 captures, a 5 FPS drawing cap,
and disabled GI baking, temporal filtering, camera effects, and autoexposure.
The warehouse temporarily enabled ceiling-light shadows and stopped the conveyor
animation. City traffic was temporarily stopped. Original scene files were preserved.

Moving objects still require shadow updates. The engine regression separately
verified thirteen edit states in the main view and camera sensor, including moving
lights and objects, resizing, visibility changes, and insertion/removal.

[Detailed measurements and limitations](O:/omnisim/docs/developer/local-shadow-performance.md)

![House after optimization — identical to the before capture](O:/omnisim/.local-runs/shadow-pipeline/house-capped/house-0-after.png)
