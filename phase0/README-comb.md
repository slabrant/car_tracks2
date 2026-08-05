# Phase 0 recalibration — U-channel section

`fit_clearance = 0.15` was measured against the **old I-section** joint. The
section changed, so it has to be measured again. `lap_length` has never been
measured at all.

Both are on one plate.

```bash
blender --background --python phase0/comb.py -- --outdir out/comb
```

`out/comb/comb_all.stl` — 18 coupons, 3 rows, **204 × 108 mm**. Print flat,
deck down; the U bridges nothing, so no supports.

## What is being varied

| axis | values |
|---|---|
| `fit_clearance` | 0.10, 0.15, 0.20 mm |
| `lap_length` | 5, 6, 8 mm |

`detent_height` is **not** a free axis. It is scaled with the lap so that strain
at the tab root stays at the 0.8 % the current joint runs at. Without that the
comparison is meaningless — a short lap would be tested carrying a detent it
would never be built with.

| lap | clearance 0.10 | 0.15 | 0.20 |
|---|---|---|---|
| 5 | 0.237 | 0.289 | 0.341 |
| 6 | 0.296 | 0.348 | 0.401 |
| 8 | 0.447 | 0.500 | 0.553 |

## Reading the marks

Pockets on the **+X rail's outer face**, two groups along the piece:

- **first group** = clearance: 1 mark = 0.10, 2 = 0.15, 3 = 0.20
- **second group** = lap: 1 mark = 5 mm, 2 = 6 mm, 3 = 8 mm

Two coupons per combination, because the port is genderless and a coupon mates
with its twin.

## What to judge

1. **Fit.** The loosest clearance that still holds, not the tightest that goes
   together.
2. **Insertion.** A child should manage it. A rail that cracks rather than
   flexes means the detent is too tall for that lap.
3. **Pull-out.** Prediction to check: shorter laps hold *better*, because tab
   stiffness grows as `1/a³` while the deflection it may take only falls as
   `a²`, so retention scales as `1/a`. If that is wrong, the model is wrong and
   the numbers above are worth nothing.
4. **Detent repeatability.** The real floor on shortening is print resolution,
   not strain. At 0.2 mm layers a 0.24 mm detent is barely more than one layer
   and should vary noticeably between coupons. Look for that.
5. **Lift.** No coupon pair should come apart upward at any setting. That is the
   Z-lock, and it is now a choice rather than a consequence (§5.6) — if a
   printed pair lifts apart, something is wrong that no test caught.

Write the answers into `Connector` in `trackcore/config.py`, then regenerate:

```bash
blender --background --python blender/run.py -- --all --outdir out/set
```

**A lap under about 7 mm will not build the whole catalogue yet** — a 45° arc
goes non-manifold. See SPEC.md §11.3. Straights are unaffected, which is why
the comb can measure it regardless.
