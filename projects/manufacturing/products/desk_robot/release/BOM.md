# Desk robot: bill of materials

Total mass, as designed: **219.0 g** (printed masses are solid-body volumes x PLA density: an upper bound (the slicer's infill makes them lighter)).

## Made

| Part | Process | Material | Qty | Mass each (g) | Drawing | Machine file |
|---|---|---|---|---|---|---|
| bottom_plate | laser | 3 mm birch plywood | 1 | 14.37 | [drawings/bottom_plate.svg](drawings/bottom_plate.svg) | [cam/bottom_plate.gcode](cam/bottom_plate.gcode) |
| top_plate | laser | 3 mm birch plywood | 1 | 14.21 | [drawings/top_plate.svg](drawings/top_plate.svg) | [cam/top_plate.gcode](cam/top_plate.gcode) |
| motor_clamp | printing | PLA | 2 | 0.84 | [drawings/motor_clamp.svg](drawings/motor_clamp.svg) | [cam/print_plate.gcode](cam/print_plate.gcode) |
| rim | printing | PLA | 2 | 5.74 | [drawings/rim.svg](drawings/rim.svg) | [cam/print_plate.gcode](cam/print_plate.gcode) |
| tray | printing | PLA | 1 | 3.83 | [drawings/tray.svg](drawings/tray.svg) | [cam/print_plate.gcode](cam/print_plate.gcode) |
| hub | cnc | 6061 aluminium | 2 | 1.95 | [drawings/hub.svg](drawings/hub.svg) | [cam/hubs.nc](cam/hubs.nc) |

## Bought

| Item | Spec | Qty | Mass each (g) | Source |
|---|---|---|---|---|
| gearmotor | Pololu #3065, 100:1 Micro Metal Gearmotor HPCB 6V (the shaft is taken as centred on the 10 mm height (the drawing shows it so; not dimensioned)) | 2 | 9.5 | https://www.pololu.com/product/3065/specs; drawing https://www.pololu.com/file/0J949/micro-metal-gearmotors-dimensions.pdf |
| caster | Pololu #951 ball caster, 3/8 in metal ball, with its 1/16 in spacer | 1 | 4.0 | https://www.pololu.com/product/951; drawing https://www.pololu.com/file/0J1636/pololu-ball-caster-with-0-375in-ball.pdf |
| tyre | O-ring 34 x 2 mm NBR 70 (DIN 3771 / ISO 3601-1), stretched onto the rim | 2 | 0.4 | https://tameson.com/products/s24cr-x100-nbr-o-ring-34-x-2mm-od-38mm-70-shore-a-100-pieces; stretch <= 5 %: Parker O-Ring Handbook ORD 5700 s3.3 |
| controller | Seeed Studio XIAO ESP32-S3 (no mounting holes: it sits in the printed tray; mass estimated) | 1 | 3.0 | https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/ |
| driver | Pololu #2130 DRV8833 dual motor driver carrier | 1 | 1.0 | https://www.pololu.com/product/2130/specs |
| battery_holder | Keystone 2478, 4 x AA side by side, wire leads (held by a hook-and-loop strap through two slots (its hole positions are not dimensioned); mass estimated) | 1 | 15.0 | https://www.keyelco.com/userAssets/file/M65p28.pdf |
| cells | AA alkaline cell | 4 | 23.0 | Energizer E91 datasheet, typical weight 23 g |
| standoff | Wurth 970250321, M3 female/female hex standoff, 25 mm (mass estimated (5.5 mm hex, 25 mm, steel)) | 4 | 2.6 | https://www.we-online.com/components/products/datasheet/970250321.pdf |
| screw_m3 | M3 x 8 socket head cap screw, ISO 4762 (into the standoffs, 5 mm of thread) | 8 | 0.9 | https://www.fasteners.eu/standards/ISO/4762/ |
| screw_m2_clamp | M2 x 8 socket head cap screw + M2 nut ISO 4032 (motor clamps) | 4 | 0.5 | https://www.fasteners.eu/standards/ISO/4762/ |
| screw_m16_caster | M1.6 x 8 socket head cap screw + M1.6 nut (caster, heads in its housing's notches) (an M2 cap head (3.8 mm) does not fit the 3.0 mm notches in Pololu's housing (its STEP)) | 2 | 0.28 | https://www.fasteners.eu/standards/ISO/4762/ |
| screw_m2_hub | M2 x 8 socket head cap screw (hub into rim, self-tapping in PLA) | 6 | 0.35 | https://www.fasteners.eu/standards/ISO/4762/ |
| screw_m2_tray | M2 x 8 socket head cap screw + M2 nut ISO 4032 (tray) | 2 | 0.5 | https://www.fasteners.eu/standards/ISO/4762/ |
| strap | hook-and-loop strap, 20 mm wide, 200 mm | 1 | 3.0 | any |
| electrical | switch, regulator, fuse, diode, capacitors, resistors, JST SM pairs (5.65 g) and the routed wires (2.84 g) | 1 | 8.49 | electrical.py; release/electrical/ |

## Stock

- 3 mm birch plywood, two sheets of at least 98 x 88 mm (one per plate).
- 6061 aluminium flat bar, 4 mm, one blank 48 x 26 mm (both hubs), with blue tape and CA glue to hold it to the spoilboard.
- PLA filament, about 13 g (the slicer's figure for the print plate).

## Electrical

From `electrical.py`; the schematic, wiring diagram, wire list and power budget are in [electrical/](electrical/).

| Ref | Part | Pins | Source |
|---|---|---|---|
| BT1 | 4 x AA holder with wire leads (BOUGHT['battery_holder']) | +, - | https://www.keyelco.com/userAssets/file/M65p28.pdf |
| F1 | Bourns MF-R300 PTC resettable fuse, 3.0 A hold, 6.0 A trip, 30 V | 1, 2 | https://www.bourns.com/docs/product-datasheets/mf-r.pdf |
| J1 | JST SM 2-pin wire-to-wire pair (battery): SMP-02V-BC + 2x SHF-001T-0.8BS socket crimp / SMR-02V-B + 2x SYM-001T-P0.6 pin crimp | 1, 2 | https://www.jst-mfg.com/product/pdf/eng/eSM.pdf |
| SW1 | Pololu #2810 Mini MOSFET Slide Switch with Reverse Voltage Protection, LV | VIN, GND, VOUT, ON | https://www.pololu.com/product/2810 |
| U1 | Pololu #2119 5V Step-Up/Step-Down Voltage Regulator S7V7F5 | VIN, GND, VOUT | https://www.pololu.com/product/2119/specs |
| D1 | 1N5817 Schottky, 20 V 1 A, DO-41 | A, K | https://www.vishay.com/docs/88525/1n5817.pdf |
| U2 | Pololu #2130 DRV8833 dual motor driver carrier | VIN, VMM, GND, AOUT1, AOUT2, BOUT1, BOUT2, AIN1, AIN2, BIN1, BIN2, SLP, FLT, AISEN, BISEN | https://www.pololu.com/product/2130 |
| U3 | Seeed Studio XIAO ESP32-S3 | D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, 5V, GND, 3V3, BAT+, BAT- | https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/ |
| C1 | 100 uF 16 V low-ESR aluminium electrolytic, radial 6.3 mm (bulk, at the driver) | +, - | https://www.pololu.com/docs/0J16 (LC spikes); TI SLVSAR1E pin table (VM bypass) |
| C2 | 100 nF X7R ceramic, 5.08 mm leads (ADC filter) | 1, 2 | - |
| C3 | 100 nF ceramic across motor M1's terminals | 1, 2 | https://www.pololu.com/docs/0J15/9 |
| C4 | 100 nF ceramic across motor M2's terminals | 1, 2 | https://www.pololu.com/docs/0J15/9 |
| R1 | 100 kohm 1 % 0.25 W axial (battery sense, top) | 1, 2 | - |
| R2 | 47 kohm 1 % 0.25 W axial (battery sense, bottom) | 1, 2 | - |
| R3 | 3.3 kohm 5 % 0.25 W axial (nSLEEP pull-down: the driver sleeps until the firmware wakes it) | 1, 2 | - |
| J2 | JST SM 2-pin wire-to-wire pair (left motor) | 1, 2 | https://www.jst-mfg.com/product/pdf/eng/eSM.pdf |
| J3 | JST SM 2-pin wire-to-wire pair (right motor) | 1, 2 | https://www.jst-mfg.com/product/pdf/eng/eSM.pdf |
| M1 | Pololu #3065 gearmotor, LEFT (+y) | 1, 2 | https://www.pololu.com/product/3065/specs |
| M2 | Pololu #3065 gearmotor, RIGHT (-y), mounted mirrored | 1, 2 | https://www.pololu.com/product/3065/specs |

Mounted on the bottom plate (foam tape, no holes):

- **SW1** at x -20, y 27 mm: foam tape, or two M2 if the vendor drawing shows holes.
- **U1** at x -36, y 22 mm: foam tape (no mounting holes).

## Fasteners, joint by joint

Each length is the shortest ISO 4762 length that clamps the stack with a full pitch proud of the nut, or 1.5 d into a standoff (`bought_parts.joints`).

| Joint | Screw | Clamps (mm) | Nut | Proud of the nut / into the thread (mm) |
|---|---|---|---|---|
| standoff_bottom x 4 | M3 x 8 | 3.0 | into the standoff | 5.0 |
| standoff_top x 4 | M3 x 8 | 3.0 | into the standoff | 5.0 |
| motor_clamp x 4 | M2 x 8 | 5.0 | M2 | 1.4 |
| caster x 2 | M1.6 x 8 | 6.107 | M1.6 | 0.593 |
| tray x 2 | M2 x 8 | 5.0 | M2 | 1.4 |

## Wires: the cut list

Routed in 3D through the assembled chassis (`wiring.py`): each wire keeps its insulation 0.3 mm off every part, lies along the surfaces, and is cut to its routed length plus 10 mm at each end.

| Wire | From | To | Net | AWG | Colour | Cut (mm) | Clear of parts (mm) |
|---|---|---|---|---|---|---|---|
| W1 | BT1.+ | F1.1 | VBAT_RAW | 26 | red | 40 | 0.38 |
| W2 | BT1.- | J1.2 | GND | 26 | black | 56 | 0.38 |
| W3 | F1.2 | J1.1 | VBAT | 22 | red | 35 | 6.61 |
| W4 | J1.1 | SW1.VIN | VBAT | 22 | red | 45 | 1.32 |
| W5 | J1.2 | SW1.GND | GND | 22 | black | 45 | 0.32 |
| W6 | SW1.VOUT | U2.VIN | VSW | 22 | red | 61 | 0.41 |
| W7 | SW1.GND | U2.GND | GND | 22 | black | 76 | 0.54 |
| W8 | SW1.VOUT | U1.VIN | VSW | 26 | red | 52 | 0.49 |
| W9 | U1.VOUT | D1.A | 5V_REG | 26 | orange | 40 | 0.3 |
| W10 | U2.GND | U3.GND | GND | 26 | black | 65 | 0.19 |
| W11 | U3.GND | U1.GND | GND | 26 | black | 44 | 0.1 |
| W12 | U2.VMM | R1.1 | VMM | 28 | red | 57 | 0.38 |
| W13 | U3.D0 | U2.AIN1 | AIN1 | 28 | yellow | 55 | 0.45 |
| W14 | U3.D1 | U2.AIN2 | AIN2 | 28 | green | 56 | 0.27 |
| W15 | U3.D8 | U2.BIN1 | BIN1 | 28 | blue | 46 | 0.35 |
| W16 | U3.D9 | U2.BIN2 | BIN2 | 28 | violet | 49 | 0.45 |
| W17 | U3.D3 | U2.SLP | nSLEEP | 28 | brown | 47 | 0.33 |
| W18 | U3.D2 | U2.FLT | nFAULT | 28 | grey | 49 | 0.45 |
| W19 | U2.AOUT1 | J2.1 | MA1 | 24 | white | 42 | 0.46 |
| W20 | U2.AOUT2 | J2.2 | MA2 | 24 | blue | 42 | 0.46 |
| W21 | J2.1 | M1.1 | MA1 | 24 | white | 35 | 1.0 |
| W22 | J2.2 | M1.2 | MA2 | 24 | blue | 27 | 1.0 |
| W23 | U2.BOUT1 | J3.1 | MB1 | 24 | white | 31 | 0.52 |
| W24 | U2.BOUT2 | J3.2 | MB2 | 24 | blue | 35 | 0.77 |
| W25 | J3.1 | M2.2 | MB1 | 24 | white | 37 | 0.3 |
| W26 | J3.2 | M2.1 | MB2 | 24 | blue | 33 | 0.46 |

Total wire: 1200 mm.

## The vendors' CAD

The renders use the manufacturers' own 3D models where they have been fetched (`python bought_parts.py fetch`). They are not redistributed here: Pololu states no licence for its models, and Seeed's licence names its documents, not its models. Our own models, drawn from the vendors' dimension drawings, stand in for them everywhere else.

| Part | File | SHA-256 | Licence |
|---|---|---|---|
| gearmotor | https://www.pololu.com/file/0J2042/Pololu%20Micro%20Metal%20Gearmotor%203D%20(STEP)%20Models.zip | `eeafd5682d7a2c7d...` | none stated (all rights reserved) |
| caster | https://www.pololu.com/file/0J1637/pololu-ball-caster-with-0-375in-ball.step | `859916355cb72f3a...` | none stated (all rights reserved) |
| drv8833 | https://www.pololu.com/file/0J1616/drv8833-dual-motor-driver-carrier-step.step | `90960797d0f972d8...` | none stated (all rights reserved) |
| xiao | https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/seeed-studio-xiao-esp32s3-3d_model.zip | `0d552ed2cc7359ea...` | Seeed wiki: CC BY-SA 4.0 for documents; models not named |
