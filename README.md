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

## The only file you need to edit

**`trackcore/config.py`**, class `Connector`, lines 107–112:

```python
lap_length: float = 8.0     # how far a tab reaches past the port plane
fit_clearance: float = 0.15 # gap on every mating face
detent_offset: float = 4.0  # where the click sits along the lap
detent_height: float = 0.50 # how deep the click is
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
tests/         341 tests. Run with ./.venv/bin/python -m pytest tests/ -q
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
- **`lap_length` is stuck at 8.0.** Shorter should be better — the mechanics say
  a shorter joint holds *harder* — but below about 7 mm a 45° arc comes out
  non-manifold. That is a boolean failure with a known cause and a known fix
  (§11.3), not a geometry conflict. Straights are unaffected, so the comb can
  still measure it.

`v1.0-i-profile` tags the previous design, which had rails above *and* below the
deck so a piece could be turned over. That symmetry is why nothing could lie flat
on a bed. §5.6 records what dropping it cost.
