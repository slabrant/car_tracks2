# car_tracks2

3D-printable toy car track. A generator, not a model file: parts are described
by a path or an arm layout, and the geometry falls out.

---

## Print this

| file | what |
|---|---|
| `out/comb/comb_all.stl` | **calibration plate** — 18 coupons, 204 × 108 mm. Print this **first**. |
| `out/set/*.stl` | the 15 parts — straights, curves, ramp, junctions, support |

All parts lie flat, deck down, and bridge nothing. No supports needed.

The calibration plate settles two numbers by measurement rather than argument.
See `phase0/README.md` for how to read its tally marks and what to judge.

---

## Naming things

Enough vocabulary to talk about any part precisely. Full version, with the
junction and sweep terms too, is `docs/SPEC.md` §1.1.

**Across a piece** — the *section*:

```
              ####                        ####    ◄─ rail (guides the car)
              ####    ← guide height →    ####
              ####                        ####
              ##############################     ◄─ driving surface
              ##############################     ◄─ bed face
                              ▲
                            deck (the web between the rails)
              |◄────── channel width ──────►|
```

**Along a piece:**

```
   port ─►|◄─ lap zone ─►|◄──────── body ────────►|◄─ lap zone ─►|◄─ port
   plane  |              |                        |              |  plane
      tab ├──────►                                        ◄──────┤ tab
```

**At a port** — the joint. The section is cut at the centreline and at
mid-height, and each piece keeps two diagonally opposite quadrants:

```
        PORT FACE                          SIDE VIEW of one rail

     ┌────────┬────────┐              ────────────────┐
     │ notch  │  TAB   │   upper      piece A, upper  │  ◄── tab
     ├────────┼────────┤   ─────      ────────────────┼──────── lap plane
     │  TAB   │ notch  │   lower                      │ piece B, lower
     └────────┴────────┘              ────────────────┘
       -x        +x                   |◄─ notch ─►|◄─ tab ─►|
```

- **tab** — runs past the port plane, into the mate
- **notch** — cut back, to receive the mate's tab
- **lap** — where they overlap; **lap plane** is where they slide on each other
- **centreline slot** — the gap where the two halves pass
- **detent rib / groove** — the click
- **port** — one end of a piece; **port plane** — where two pieces meet

Junctions: **hub** (the middle), **arm** (one stub), **armpit** (where two arms
meet), **fillet** (a rounded armpit — its radius is a car's turn radius).
Supports: **stub** (the column), **leg** (a straight stood on end), **foot** (a
support turned over).

## The parts

| name | what | size |
|---|---|---|
| `straight_full` | one module | 96 mm |
| `straight_half` | half | 48 mm |
| `straight_quarter` | quarter | 24 mm |
| `curve_90` | right-angle turn | 96 mm radius |
| `curve_45` | half a turn; **use in pairs** | 96 mm radius |
| `curve_90_tight` | tighter right angle | 48 mm radius |
| `curve_90_banked` | leans into the turn | 10° |
| `ramp` | up to bridge height | 192 mm run, 48 mm rise |
| `x_junction` / `x_rounded` | four ways, square / filleted corners | |
| `t_junction` / `t_rounded` | three ways, one straight through | |
| `y_junction` / `y_rounded` | three ways at 120°, all alike | |
| `support` | a straight with a stub; turned over it is the foot | |

---

## The only file you need to edit

**`trackcore/config.py`**, class `Connector`, lines 107–112:

```python
lap_length: float = 6.0     # how far a tab reaches past the port plane
fit_clearance: float = 0.15 # gap on every mating face
detent_offset: float = 3.0  # where the click sits along the lap
detent_height: float = 0.35 # how deep the click is
detent_lead_angle_deg: float = 30.0   # shallow: pushing together
detent_return_angle_deg: float = 60.0 # steep: pulling apart
```

Change a number, regenerate, print. Nothing else needs touching — every part in
the catalogue derives its joint from these.

The class above it, `Body`, is the cross-section (24 × 4.7 mm U-channel). It is
measured against real track and should not be changed casually.

---

## Three commands

```bash
# regenerate every part
blender --background --python blender/run.py -- --all --outdir out/set

# regenerate the calibration plate
blender --background --python phase0/comb.py -- --outdir out/comb

# check what is on disk really is a valid solid
./.venv/bin/python check_stl.py 'out/set/*.stl'
```

Two more, when you want to look at something:

```bash
blender --background --python blender/gallery.py -- --outdir out/parts  # one image per part
blender --background --python blender/bridge.py                          # stand a bridge up
```

---

## What is where

```
trackcore/     the geometry. Pure Python + numpy, never imports bpy.
  config.py      >>> THE DIALS <<<  Body (section) and Connector (joint)
  edge_unit.py   the cross-section
  path.py        Line, Arc, Ramp, and how they chain
  frames.py      rotation-minimising frames, so sweeps do not twist
  sweep.py       Construction A: two-port pieces
  hub.py         Construction B: junctions
  graft.py       Construction C: supports
  connector.py   the joint, defined once and transformed to every port
  validate.py    the six mesh rules nothing may be exported without passing
  mesh.py        MeshData and primitives

parts/         the catalogue. Data only — a path or an arm layout, plus the grid.
blender/       the only place bpy appears: build, boolean, cleanup, export, renders
tests/         343 tests. Run with ./.venv/bin/python -m pytest tests/ -q
phase0/        calibration: the comb, and how to read it
docs/SPEC.md   why everything is the way it is
out/           generated. Not tracked.
```

`trackcore` never imports `bpy`, and a test enforces it. That is not purity — it
is why a geometry bug fails in a second on the command line instead of needing
somebody to open a viewport.

---

## State

Working and green: the section, all three constructions, the connector, the
15-part catalogue on a 96 mm grid, and bridges that stand on legs made of
ordinary straights.

Two things open, both in `docs/SPEC.md` §11:

- **Calibration.** `fit_clearance` was measured against the *previous*
  cross-section. The comb re-measures it, and measures `lap_length` for the
  first time.
- **How short the joint should be.** It is 6.0 mm, down from 8.0. Shorter holds
  *harder*, not worse: a shorter tab stiffens faster than the deflection it can
  take shrinks. The catalogue builds at every lap down to 4 mm, so what limits
  it is print resolution of the detent, not mechanics — which is what the comb
  measures.

`v1.0-i-profile` tags the previous design, which had rails above *and* below the
deck so a piece could be turned over. That symmetry is why nothing could lie flat
on a bed. §5.6 records what dropping it cost.
