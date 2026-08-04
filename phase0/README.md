# Phase 0 — calibrate the joint  ✅ COMPLETE

Phase 0 existed to answer three questions with a physical part in hand, before
any of the real geometry code got written. See `docs/SPEC.md` §10.

## Results

| Question | Answer |
|---|---|
| `fit_clearance` | **0.15 mm** — best of the comb; 0.10 bound, looser was sloppy |
| print orientation | **on its side**, rail face on the bed, with a brim — printed clean |
| assembly | **horizontal push**; a child managed it unaided |
| detents | two distinct clicks, firm |
| hang test | holds a dangling piece |
| lift test | stays connected — the diagonal split blocks vertical, as designed |
| pull-out | firm but separable by hand — `detent_return_angle` 60° confirmed |

Frozen into `Config` below and into SPEC.md §6.4. The rest of this file is the
procedure that produced them, kept so the calibration can be repeated on a
different printer or filament.

## What a coupon is

20 mm of straight track with one port on it, plus the 8 mm tab, so 28 mm long.
The port is genderless, so **two identical coupons mate with each other** —
rotate one 180° and push them together. There is no male part and no female
part to keep track of.

The port is the full diagonal split of SPEC.md §6.1: the whole cross-section is
cut at `x = 0` and `z = 0`, and each piece keeps two diagonally opposite
quadrants. So the joint is a **horizontal push only** — you cannot lower one
piece onto another, the geometry forbids it (§6.7).

Each coupon has 1–5 slots cut through the deck at its blank end. That is the
tally: slot count identifies which clearance it was printed at, and it stays
readable after the parts come off the bed and get mixed up.

| slots | fit_clearance |
|---|---|
| 1 | 0.10 mm |
| 2 | 0.15 mm |
| 3 | 0.20 mm |
| 4 | 0.25 mm |
| 5 | 0.30 mm |

## Generate

```bash
blender --background --python phase0/build_coupons.py -- --orientation side
```

Writes to `phase0/out/`:

- `comb_all.stl` — all five clearances, two coupons each, on one plate.
  **Print this one.** 167 mm across, so check it fits your bed.
- `coupon_c0.20_pair.stl` and friends — one clearance at a time, if you would
  rather iterate.

Useful flags: `--orientation side|flat|model`, `--clearances 0.18 0.22`,
`--pairs 4`, `--gap 10`, `--outdir somewhere`.

Every coupon is checked against all six of SPEC.md §7 before it is written, plus
a flip test (§9.18) and a mesh-level mate test (§9.19) that boolean-intersects a
coupon with a rotated copy of itself and requires the shared volume to be
exactly zero. If any of that fails the script exits non-zero and says why.

To re-check what actually landed on disk, including the float32 round trip:

```bash
python3 phase0/check.py 'phase0/out/*.stl'
```

## Look at it first

```bash
blender --background --python phase0/preview.py -- --out phase0/out/joint.png
```

Renders four views of two mated coupons. `joint_rail.png` is the one worth
looking at — it is the SPEC.md §6.2 side view, showing the half-lap and both
detent engagements.

## Print

Default orientation is `side`: one rail's outer face flat on the bed.

- Nothing bridges. The 21.6 mm channel never spans open air, so neither driving
  surface is a bridged underside. On a flip-symmetric track *both* faces are
  driving surfaces, which is why `flat` is the wrong default.
- Footprint per coupon is only 4.7 × 24 mm and the part stands 24 mm tall, so
  it is tippy. **Use a brim.**

Try `--orientation flat` as the comparison if you want to see the bridging for
yourself before ruling it out.

Suggested settings: 0.4 mm nozzle, 0.2 mm layers, 3 perimeters, no supports.
The thinnest wall is the 1.2 mm rail; the smallest feature is the 0.4 mm detent
rib. If your printer cannot resolve the rib, that is a finding — write it down.

## Test, and what to record

For each clearance, take the two coupons with matching tally slots and join
them.

1. **Fit.** Does it go together at all, and does it hold? You want the loosest
   clearance that still holds, not the tightest that goes together.
2. **Insertion force.** Can a child push them together? If the rail cracks
   instead of flexing, say so — that means `detent_height` is too tall or the
   lap is too short.
3. **Detents.** Can you feel the ribs click into the grooves, or is it just
   friction? Two clicks per joint is what should happen.
4. **Pull-out.** Hold one coupon, pull the other straight along the track. This
   is the *only* axis the joint cannot lock rigidly (§6.7), so it is the one
   that matters. If it pulls apart too easily, raise `detent_return_angle`; if
   it will not come apart at all, lower it.
5. **The hang test.** Hold one coupon horizontally in the air and let the other
   dangle from the joint. It should stay on: gravity is pure vertical load, and
   vertical is rigidly blocked by the diagonal split. Note how far it droops —
   that is the angular play, and it should be under about a degree.
6. **Lift test.** Try to lift one coupon straight off the other. It should be
   impossible, not merely stiff. If it comes off, the diagonal split is not
   doing its job and something is wrong.
7. **Flip.** Turn one coupon over and mate it again. It must behave identically.

Then write the answers into `Config` in `phase0/coupon.py` and into
`docs/SPEC.md` §6.4, and Phase 0 is done.

If it is the *retention* rather than the clearance you want to tune, regenerate
with a fixed clearance and vary the return face:

```bash
blender --background --python phase0/build_coupons.py -- \
    --clearances 0.20 --outdir phase0/out/detent
```

then edit `detent_return_angle` in `Config` between runs. 60° is the starting
point; steeper holds harder.

## Layout

| file | imports bpy | what it is |
|---|---|---|
| `geom.py` | no | MeshData, boxes, prisms, STL read/write |
| `validate.py` | no | the six SPEC.md §7 checks |
| `coupon.py` | no | `Config` and the coupon's boolean programme |
| `test_coupon.py` | no | headless tests, `python3 -m pytest phase0/ -q` |
| `check.py` | no | validate STLs already on disk |
| `build_coupons.py` | **yes** | build, validate, export |
| `preview.py` | **yes** | renders |

`geom.py`, `validate.py` and `coupon.py` are the seeds of `trackcore/`. The
Blender files are throwaway. The dependency rule from SPEC.md §8 is already in
force here and is enforced by a test.
