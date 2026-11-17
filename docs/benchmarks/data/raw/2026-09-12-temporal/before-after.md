# Rendering comparison

The new live rendering path checks whether old pixels still belong to the visible surface, aligns anti-aliasing camera offsets, and reduces trails when color changes. Optional GPU measurements now identify the cost of each rendering stage.

The captures below compare the former and new temporal resolves at the same fixed view. They are stationary image controls; motion correctness is checked separately with controlled GPU tests. Indirect-light baking was disabled equally for these checks.

Across the house, warehouse, and city, disabling anti-aliasing produced an exact pixel match with the previous installed renderer. Turning GPU measurement on also preserved those pixels exactly. Other simulator tasks were running, so these measurements do not establish an FPS improvement.

## House

Before — previous temporal resolve:

![house before](O:/omnisim/.local-runs/temporal-pipeline/house/house-0-legacy.png)

After — depth-checked temporal resolve:

![house after](O:/omnisim/.local-runs/temporal-pipeline/house/house-0-validated.png)

## Warehouse

Before — previous temporal resolve:

![warehouse before](O:/omnisim/.local-runs/temporal-pipeline/other-scenes/warehouse-0-legacy.png)

After — depth-checked temporal resolve:

![warehouse after](O:/omnisim/.local-runs/temporal-pipeline/other-scenes/warehouse-0-validated.png)

## City

Before — previous temporal resolve:

![city before](O:/omnisim/.local-runs/temporal-pipeline/other-scenes/city-0-legacy.png)

After — depth-checked temporal resolve:

![city after](O:/omnisim/.local-runs/temporal-pipeline/other-scenes/city-0-validated.png)

## Remaining work

Fast objects and deforming surfaces still need their own motion vectors for accurate history tracking. Dynamic indirect light/reflections and acceleration of optional Photo mode are separate follow-on work.
