# A clothes rack, made with the CNC twin

A new product taken from a concept sketch to a finished, load-tested rack:
designed for the machine we have, its machined part milled and measured on the
LongMill twin, assembled step by step, and pushed over on Newton.

| | |
|---|---|
| Size | 1200 (bar) × 600 (base) × 1500 mm (floor to top of bar) |
| Frame | 30 × 2 mm aluminium round tube, 6063-T6 |
| Joints | four clamp connectors, each two identical halves milled from 6061-T6 |
| Mass | 6.1 kg |
| Rated load | 20 kg of clothes on the bar |

## Designed for the machine

The concept is a steel or aluminium tube frame with welded or fitted joints. A
LongMill is a router, so it cannot make tube. So:

- **The tubes are bought** (30 × 2 mm aluminium round tube) and sawn to length.
- **The joints are the machined part.** The rack has four, and they are all the
  same: a through bore for one tube (the top bar, or a base tube) and, at right
  angles, a bore for the upright. Each is a **two-piece clamp**, and both halves
  are the same part, so **one program makes all eight halves**.
- **The bores are 30.2 mm across, and the split faces stand 0.5 mm back from the
  bore centre.** A closed pair leaves 1 mm between the halves, so four M6 bolts
  clamp any tube from 29.2 to 30.2 mm. There is no welding, no thread in the
  aluminium, and nothing to flip: everything is cut from one side in one setup.

Every number comes from [`design.py`](design.py): dimensions, the connector's
exact surface, the bill of materials, the mass properties and the hand
calculations.

## Bill of materials

| Item | Qty | Spec | Made by |
|---|---|---|---|
| Top bar | 1 | 30 × 2 mm aluminium tube, 1200 mm | saw |
| Upright | 2 | 30 × 2 mm aluminium tube, 1419 mm | saw |
| Base tube | 2 | 30 × 2 mm aluminium tube, 600 mm | saw |
| Connector half | 8 | 6061-T6, 100 × 100 × 20 mm blank (100 × 20 flat bar sawn to 100 mm) | **CNC** |
| Bolt | 16 | M6 × 55 socket head cap screw, A2 stainless | buy |
| Nut | 16 | M6 nyloc, A2 | buy |
| Washer | 32 | M6 flat (DIN 125), A2 | buy |
| Levelling foot | 4 | M8 glide, 30 mm base, 20 mm set height | buy |
| Rivet nut | 4 | M8, for a 2 mm wall (11 mm hole) | buy |
| End cap | 6 | 30 mm round tube insert cap | buy |

Tube to buy: 5.24 m in all; two 3 m lengths cover the cut list (upright + bar from
one, upright + both base tubes from the other).

## Making the connectors

**Machine:** LongMill MK2 30x30, Makita RT0701C router. **Twin world:**
[`cnc_longmill_mk2_aluminium.omniworld`](../../../samples/demos/worlds/manufacturing/cnc_longmill_mk2_aluminium.omniworld)
(a 100 × 100 × 20 mm blank, the aluminium tool table, aluminium's cutting
energy in the load model).

1. **Blank:** saw 100 × 20 mm 6061 flat bar into 100 mm squares, one per half.
2. **Hold it** in a low vise or with side clamps on the two faces the bar bore
   does not leave through (X±50). The top face stays clear.
3. **Zero** X0 Y0 at the blank's centre, Z0 on its top.
4. **Run the program** (`design.py` writes it): T1 roughs and bores the four
   holes, M6 changes to T2, T2 finishes both bores.

| Tool | | Settings (starting point) |
|---|---|---|
| T1 | 1/4" single-flute upcut end mill | 18,000 rpm, 900 mm/min (0.05 mm/tooth), 0.5 mm deep, 2.5 mm over, plunge 150 mm/min |
| T1 | holes, Ø7 (M6 clearance) | helical G2, 0.3 mm a turn, 300 mm/min, out and back every 5 mm to clear chips |
| T2 | 1/4" two-flute ball nose | 18,000 rpm, 800 mm/min (0.022 mm/tooth), 0.4 mm stepover |

> These feeds are **chip-load arithmetic, not a published LongMill table**.
> Sienci cuts aluminium with a single-flute 1/4" end mill, but their aluminium
> pages refused automated reading. Cut the first half as a test, with an air
> blast or mist, and listen.

Preflight: **pass**, no envelope or line errors. **1 h 59 min of machine time
per half**, so about 16 h for all eight.

### One half, milled on the twin

The whole program ran on the twin, headless in fast mode, through the GRBL
sender with character counting: 146,392 lines, two tool changes, 21.6 min of
wall time for the 2 h of machine time. The milled half was then measured
against its drawing (`design.inspect_half`, height map at 0.1 mm):

| Measured | Milled | Drawn |
|---|---|---|
| Bore depth (bar / upright) | 14.599 / 14.597 mm | 14.600 mm |
| Bore width 1 mm under the split face | 29.975 mm | 30.051 mm |
| Bore width 7 mm down | 26.046 mm | 26.211 mm |
| Holes | Ø6.987 mm, centred within 0.001 mm | Ø7.0 mm |
| Split face | untouched (0.000 mm) | flat |
| Bore surface vs drawing, along the normal | +0.002 to +0.082 mm (p01 to p99), deepest −0.029 mm | 0 |
| Twin vs a perfect machine running the same program | −0.000 to +0.014 mm (p01 to p99) | |
| Material removed | 55,061 mm³ (the program's ideal cut: 55,122) | |

**The bores come out 0.04–0.08 mm narrow on their steep sides.** That's the
CAM, not the machine. The drop-cutter keeps a safety margin of half its grid's
diagonal, 0.07 mm at 0.1 mm, and on a steep wall that margin stays as stock. A
0.05 mm grid halves it: 0.04 mm at p99, at 1.8× the program length (checked
offline). For this clamp it does not matter, because the halves just close on
any tube from about 29.1 to 30.06 mm. A larger tube still clamps, only on the
bores' edges.

The assembly world draws the halves exactly as the twin milled them, tool marks
included (`assembly.py --zmap`).

## Preparing the tubes

1. Saw the cut list square and deburr the ends inside and out.
2. On each base tube, drill an 11 mm hole on the underside 270 mm each side of
   the centre, and set an M8 rivet nut in each.

## Assembly

[`clothes_rack_assembly.omniworld`](../../../samples/demos/worlds/manufacturing/clothes_rack_assembly.omniworld)
plays these steps, with captions, and loops. Tools: a 5 mm hex key, a 10 mm
spanner, a torque wrench, a rubber mallet.

1. Screw a levelling foot into each base tube's rivet nuts.
2. Seat each base tube in a connector half, on the outside.
3. Stand an upright in each connector's vertical bore, down onto the base tube
   (the upright stops 0.5 mm short of it).
4. Close each foot connector with a second half: both halves are the same part.
5. Four M6 × 55 bolts per connector, a washer under the head and under the nut,
   nyloc nuts on the inside. Finger-tight for now.
6. Hold the back half of a top connector on each upright's top.
7. Lay the top bar in both connectors, its ends 10 mm out of each.
8. Close the top connectors and bolt them like the feet.
9. Check it is square (the two diagonals are equal), then torque all 16 bolts
   to **5 N·m**, crosswise.
10. Tap in the six end caps, and set the feet so the rack stands level.

## Will it hold?

Hand calculations ([`design.checks`](design.py)), with 20 kg of clothes:

| Check | Result |
|---|---|
| Bar sag, simply supported over the 1080 mm span (the clamps only reduce it) | 2.7 mm |
| Bar bending stress | 22.9 MPa: safety factor 7.4 on 6063-T6 |
| Bolt preload at 5 N·m | 4.2 kN each |
| Grip on each tube before it slips (2 bolts, μ 0.3) | 5,000 N, against 128 N of load per upright |

Then the finished rack goes on Newton:
[`clothes_rack_load_test.omniworld`](../../../samples/demos/worlds/manufacturing/clothes_rack_load_test.omniworld).
The rack is one rigid body with its real mass, centre of mass and inertia from
`design.mass_properties` (clothes included), on four rubber feet (μ 0.6). It
settles, and then the bar is pushed with a force that rises at 2 N/s (back) or
4 N/s (sideways) until the far feet lift 1 mm.

| Push at the bar | Newton | Hand calculation |
|---|---|---|
| Back, 20 kg of clothes | **47.8 N** | 46.5 N |
| Sideways, 20 kg of clothes | **104.5 N** | 103.0 N |
| Back, empty | **11.5 N** | 10.9 N |
| Sideways, empty | **22.6 N** | 21.7 N |

Newton reads 1–6 % high, because it calls a tip only once the far feet are 1 mm
up while the force is still rising. Sliding never comes first: friction holds
154 N with the clothes on, and the rack crept 0.27 mm (back) and 0.48 mm
(sideways) before tipping.

> **Found by this test:** the engine applied `addForceWithOffset` torque about
> the body's origin, while Newton wants it about the centre of mass. On this rack,
> whose centre of mass is 1.02 m up, the first run (on the concept's 500 mm base)
> tipped at 23.5 N, 62 % of the right answer, 37.8 N. The engine is fixed (`OMNISIM_NEWTON_EXT_WRENCH_AT_COM`,
> `tests/test_newton_ext_wrench_at_com.py`).

**The base is 600 mm, not the concept's 500.** The first load test, on the
concept's 500 mm base, found the rack's weak point: empty, a 9.4 N push at the
bar (about the weight of 1 kg) tipped it backward or forward, because the feet
were only 440 mm apart under a 1.5 m bar. Lengthening the two base tubes to
600 mm (feet 540 mm apart) raises that to 11.5 N empty and 47.8 N loaded, from
39.2 N, at 200 mm more tube and no change to the CNC part. A lower bar, or
weight in the base, would raise it further.

## Regenerate

```bash
python projects/manufacturing/products/clothes_rack/design.py --out .tmp/clothes_rack
python projects/manufacturing/products/clothes_rack/assembly.py
python projects/manufacturing/products/clothes_rack/load_test.py
```

- `design.py` writes the half's STL, its G-code and `rack.json`: the bill of
  materials, the cut list and the checks.
- `assembly.py --zmap <dump>` draws the halves as the twin milled them.

## Not modelled

- **The joints are rigid.** Clamp slip and tube ovalising are hand
  calculations, not simulated.
- **Bar sag is calculated, not simulated:** the rack is one rigid body.
- **Clothes do not swing.** They are rigid with the rack. A hanging garment's
  weight acts at the bar either way, so the tipping forces do not change.
- **The feeds have not been cut on a real LongMill yet.**
- **Hand work is not simulated:** sawing, drilling, rivet nuts and torque.
