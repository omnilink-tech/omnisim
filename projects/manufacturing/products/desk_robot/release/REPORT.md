# Desk robot: as made on the twins

Every number here is the **twin's**: the parts were made on OmniSim's machine twins and measured from the engine's own record of the material (the laser and CNC sheets' height maps, the printer's beads). No part has been made on a real machine yet.

## The design's fits

| Check | Value |
|---|---|
| tyre_stretch_pct | 4.12 |
| wheel_d_mm | 39.32 |
| axle_z_mm | 19.66 |
| plate_underside_mm | 11.66 |
| caster_mm | 11.658 |
| level_error_mm | 0.003 |
| track_mm | 100.8 |
| shaft_overlap_with_milled_bore_mm2 | 0.0 |
| shaft_clear_of_rim_mm | 0.2 |
| shaft_end_inside_rim_mm | 3.4 |
| clamp_holes_from_plate_edge_mm | 3.05 |
| speed_no_load_m_s | 0.679 |

## Laser: the plates (3 mm plywood, Sienci LaserBeam 7W twin)

| Plate | Cut free | Size (mm) | Drawn (mm) | Holes cut | Hole ⌀ error (mm) | Job | Wall (s) |
|---|---|---|---|---|---|---|---|
| bottom_plate | True | [90.1, 80.1] | [90.0, 80.0] | 13/13 | -0.144 to -0.118 | 5451 lines, done | 137.5 |
| top_plate | True | [90.1, 80.1] | [90.0, 80.0] | 7/7 | -0.245 to -0.119 | 3500 lines, done | 231.6 |

The engraving: 0.125 mm deep in the letters, letters 35.33 mm² (drawn 35.64), char 0.876 inside them and 0.023 around them.

## CNC: the hubs (6061 aluminium, LongMill twin)

| Hub at (mm) | OD (mm) | Drawn | Bore area (mm²) | Drawn | The D-shaft fits |
|---|---|---|---|---|---|
| [-13.0, 0.004] | 15.994 | 16.0 | 6.7525 | 6.7796 | True (0 of 2481 cells blocked) |
| [13.0, 0.004] | 15.994 | 16.0 | 6.7525 | 6.7796 | True (0 of 2481 cells blocked) |

Cut free: 2 of 2. Job: 30933 lines, done, 2 tool changes.

## Printing: the clamps, rims and tray (PLA, Sovol SV08 twin)

- Planned print time 1599.264 s; plastic out of the nozzle 10317.5 mm³, laid 10311.61 mm³; nozzle crashes 0.
- 57 layers; height 11.45 mm (drawn 11.5).
- Outer walls against the STL: -0.0476 to 0.0712 mm (5th to 95th percentile), mean 0.0219 mm. The first two layers read millimetres off: that is the slicer's skirt line around the parts, not the parts.

## The assembled robot on Newton

| | Predicted | Measured |
|---|---|---|
| Mass (g) | 219.0 | (from the BOM) |
| Top speed, no load (m/s) | 0.6794 | 0.6715 |
| Spin rate, no load (rad/s) | 13.4802 | 13.2634 |
| Wheel radius (mm) | 19.66 | 19.595 (speed / wheel rate) |
| Track (mm) | 100.8 | 100.31 (from the spin) |

Heading drift over the straight run: -0.004 degrees.
