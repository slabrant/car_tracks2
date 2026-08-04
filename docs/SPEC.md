# Car Tracks 2 — Specification

Status: **Phases 0–4 complete.** The joint is calibrated against printed parts;
swept pieces and junctions build, validate and export; every port of every part
mates with every other; and the part set is laid out on a grid so loops close.
Phase 5 (supports) not started. See §10.

This document is the contract. If an implementation disagrees with it, the
implementation is wrong. If this document is wrong, fix this document first,
then the code.

---

## 0. What this system is

A track piece is built from one primitive: the **edge unit**, a sideways-T
cross-section (`⊢`) consisting of one rail plus its share of the deck.

```
   ##
   ##═════   rail (##) + deck stem (═), stem points inward
   ##
```

Every piece is **one edge unit per boundary chain**, with the deck filling the
interior. A piece with two ports has two chains, and the two edge units come out
as mirror images of each other — that pairing is what makes the familiar I-beam
cross-section. A junction with N ports has N chains and N edge units, related by
**rotation** rather than mirroring.

```
  N = 2  (straight, curve, ramp)        N = 4  (X junction)

    ##              ##                    ═══╗   ╔═══
    ##══════════════##                       ║ D ║
    ##              ##                     C ╚═╦═╝ B
    ^ chain A       ^ chain B                ╔═╝ ╚═╗
    mirror pair  ->  "I" profile           ══╝  A  ╚══
                                          4 chains, C4
```

The N=2 case is not special. It is just the smallest case.

### 0.1 Scope

In scope:

- The edge unit and the deck.
- Two-port pieces: straight, circular curve, vertical ramp, banked curve.
- Junctions: Y, T, X, and in general N straight arms at arbitrary angles, with
  square or filleted corners.
- Rotation-minimising frames so swept pieces do not twist.
- A genderless, flip-symmetric end connector, identical at every port of every
  piece.
- Bridge supports: a port grafted onto a short straight, with legs made of
  ordinary track pieces stood on end. Turned over, a support is its own foot.
- Watertight, manifold mesh output.
- STL / 3MF export via Blender.

Explicitly **out of scope**, and not to be built even if it looks easy:

- Moving switches or points. Junctions are static road; the child steers the car.
- Sloped or banked junctions. Junctions are level and prismatic — §5 depends on
  this.
- Free-form NURBS/Bezier paths. Paths are built from primitives (§4.1).
- Curved junction arms. Arms are straight stubs; curvature comes from attaching
  a curve piece.
- Scenery, a GUI, a Blender add-on.
- Backwards compatibility with `car_tracks` (v1).

If a task appears to require any of the above, stop and ask.

### 0.2 Why the previous four attempts failed

Recorded so the same holes are not fallen into again.

1. **Gendered connectors.** v1 used a male tab on one end and a female socket on
   the other. They mated, and the fit was sometimes loose, but the real damage
   was structural: gendering imposes an alternating parity constraint on the
   whole system. It makes turns directional and it makes junctions unbuildable —
   see §6.1. This is the root problem, and looseness was a separate, minor one.
2. **Non-manifold output.** Connectors were applied as boolean modifiers against
   a swept mesh with faces coincident at the deck plane, and nothing checked the
   result. §5, §7 and §9 fix this.
3. **Per-type generators.** `straight.py`, `curved.py`, `bridge.py`,
   `intersection.py` — four code paths, four places for the same bug, no shared
   notion of what a piece *is*. Collapsed to one primitive and two
   constructions.
4. **No headless tests.** Every geometry bug was invisible until a human opened
   a Blender viewport. §8's dependency rule exists to fix this and nothing else.

---

## 1. Conventions

Binding. Do not silently redefine.

- **Units: millimetres.** Every length here and in code is mm. The exported mesh
  is in mm.
- **Angles: radians in code, degrees in config and CLI.** Convert once, at the
  config boundary.
- **Local frame:** cross-sections are drawn in the **XZ plane**.
  - `+X` across the track (width), `x = 0` on the centreline
  - `+Z` up, `z = 0` on the deck mid-plane
  - `+Y` forward along the path
  - `(X, Y, Z)` right-handed
- **Plan view** means looking down `−Z` at the XY plane. Junction arm angles are
  measured in plan, CCW from `+X`.
- **Winding:** faces CCW seen from outside the solid; normals outward; signed
  volume positive.

| Term | Meaning |
|---|---|
| **edge unit** | the `⊢` primitive: one rail plus its deck stem |
| **rail** | the tall vertical member at a deck edge |
| **deck** | the thin horizontal web the cars drive on |
| **profile** | the closed 2D cross-section of an N=2 piece (two edge units) |
| **chain** | one connected run of boundary that carries a rail |
| **port** | a connector interface; one per arm; carries no rail |
| **arm** | one straight stub of a junction, centre to port |
| **armpit** | the corner where two adjacent arms' outer edges meet |
| **station** | one sampled point along a swept path |
| **ring** | a profile placed in 3D at one station |
| **lap** | the interdigitating half-height rail joint (§6.2) |

---

## 2. The edge unit and the derived profile

Parameters. Defaults carried from v1's `track_config.json` — these were measured
against real track and are trusted.

| Name | Default | Meaning |
|---|---|---|
| `width_outer` | 24.0 | outer rail face to outer rail face |
| `rail_thickness` | 1.2 | horizontal thickness of one rail |
| `rail_height_total` | 4.7 | full rail height, top face to bottom face |
| `deck_thickness` | 1.4 | thickness of the web |

Derived once, reused everywhere:

```
half_width    = width_outer / 2             = 12.0
rail_inner    = half_width - rail_thickness = 10.8
half_height   = rail_height_total / 2       =  2.35
half_deck     = deck_thickness / 2          =  0.7
channel_width = 2 * rail_inner              = 21.6
```

The edge unit occupies, in its own local frame with the rail's outer face at
`x = 0` and the stem running toward `+x`:

```
rail:  x ∈ [0, rail_thickness],   z ∈ [-half_height, +half_height]
stem:  x ∈ [rail_thickness, ...], z ∈ [-half_deck,   +half_deck  ]
```

### 2.1 The N=2 profile

Two edge units facing each other across `channel_width`, giving a closed
12-vertex polygon in CCW order:

```
        z
        ^
  +2.35 |  ####                          ####
        |  ####                          ####
  +0.70 |  ################################## <- deck top
   0.00 |--####--------------+-------------####----> x
  -0.70 |  ##################################
        |  ####                          ####
  -2.35 |  ####                          ####
           |  |                          |  |
        -12.0 -10.8                   +10.8 +12.0
```

```
( -half_width,  +half_height)   0     ( +half_width,  -half_height)   6
( -rail_inner,  +half_height)   1     ( +rail_inner,  -half_height)   7
( -rail_inner,  +half_deck  )   2     ( +rail_inner,  -half_deck  )   8
( +rail_inner,  +half_deck  )   3     ( -rail_inner,  -half_deck  )   9
( +rail_inner,  +half_height)   4     ( -rail_inner,  -half_height)  10
( +half_width,  +half_height)   5     ( -half_width,  -half_height)  11
```

**Vertex count is fixed at 12 for every station.** §4.3 depends on it.

Asserted at construction: all parameters `> 0`; `rail_thickness * 2 <
width_outer`; `deck_thickness < rail_height_total`; polygon simple and CCW.

### 2.2 The two mirrors — keep them straight

There are two distinct symmetries and only one of them survives at a junction.

- **Left–right mirror**, across the centreline. This is what turns `⊢` and `⊣`
  into an `I`. It exists **only when N = 2**. At a junction it is replaced by
  N-fold rotation.
- **Top–bottom mirror**, across `z = 0`, giving rails above *and* below the deck.
  This **holds for every piece including junctions**, because junctions are
  prismatic in z (§5). It is what makes a piece flippable, and the connector
  derivation in §6.1 depends on it.

---

## 3. Three constructions

| | A — swept | B — plan hub | C — graft |
|---|---|---|---|
| Used for | 2-port pieces | junctions, N ≥ 3 | supports |
| Can slope or bank | yes | no, level only | no |
| Path shape | line, arc, ramp | straight arms only | straight only |
| Built from | rings along a path (§4) | prismatic slabs (§5) | two A's unioned (§5.5) |
| Booleans | none | union of prisms, validated | one union, validated |
| Flippable | always | when the plan has a mirror axis | yes — flipped, it is the foot |

They share `config`, the edge unit, the connector (§6), validation (§7) and
export. They are three functions, not three subsystems, and none may grow a
special case for another.

---

## 4. Construction A — swept pieces

### 4.1 Paths

A path is a curve in 3D plus a roll angle, queried by arc length `s`.

```python
class Path(Protocol):
    length: float                        # mm
    def point(self, s: float) -> Vec3:   ...
    def tangent(self, s: float) -> Vec3: ...   # unit
    def roll(self, s: float) -> float:   ...   # radians about the tangent
```

Paths are **concatenated primitives**, not fitted splines. A physical track set
needs pieces whose ends sit at exact repeatable angles so they tile; a fitted
spline gives irrational end tangents and loops that never close.

| Primitive | Parameters | Notes |
|---|---|---|
| `Line` | `length` | straight |
| `Arc` | `radius`, `angle`, `bank=0` | horizontal arc; `angle > 0` turns left |
| `Ramp` | `run`, `rise` | vertical S-curve, horizontal tangent at both ends |

`Ramp` uses a smoothstep so it concatenates with `Line` without a kink:

```
z(u) = rise * (3u² − 2u³),   u = s_horizontal / run
```

`run` is the **horizontal** distance; true arc length is greater and is computed
numerically. `.length` reports true arc length.

`bank` rotates the profile about the tangent, ramped in and out over the first
and last 10% of the arc so banking never appears as a step.

`Path.chain(...)` asserts C0 and C1 continuity at every joint — position within
`1e-9`, tangent dot `> 1 − 1e-12`. A discontinuity is an error, not a warning.

**Curvature guard.** A sweep folds through itself, producing a self-intersecting
non-manifold mesh, when the radius of curvature drops below the profile's outer
half-width: the inner rail turns inside out. Require, for all `s`:

```
radius_of_curvature(s) > half_width * 1.5      (> 18.0 mm at defaults)
```

Violation raises `PathTooTightError` naming the primitive and its radius. Do not
clamp or fix up — fail.

### 4.2 Frames — the anti-twist requirement

**Do not use Frenet frames.** The Frenet normal is undefined where curvature is
zero (most of this track) and flips direction at inflections. It produces a 180°
twist at every straight-to-curve transition.

**Use a rotation-minimising frame by double reflection** (Wang, Jüttler, Zheng &
Liu, 2008). Stable, needs no curvature, about ten lines:

```
Given frame (T_i, R_i, S_i) at station i, and p_{i+1}, T_{i+1}:
  v1  = p_{i+1} − p_i
  c1  = v1 · v1
  R_L = R_i − (2/c1)(v1 · R_i) v1
  T_L = T_i − (2/c1)(v1 · T_i) v1
  v2  = T_{i+1} − T_L
  c2  = v2 · v2
  R_{i+1} = R_L − (2/c2)(v2 · R_L) v2
  S_{i+1} = T_{i+1} × R_{i+1}
```

Seed `R_0` as the component of world `+Z` orthogonal to `T_0`, normalised. If
`T_0` is within `1e-6` of vertical the seed is degenerate — raise, do not
silently substitute another axis.

Apply `roll(s)` about `T` **after** the RMF, never folded into it.

A profile point `(px, pz)` maps to world as:

```
world = p_i + px · R_i' + pz · S_i'
```

with `R_i'`, `S_i'` being `R_i`, `S_i` rotated by `roll(s_i)` about `T_i`.
Profile `+X → R` (across), profile `+Z → S` (up). Swapping or negating that pair
lays the track on its side; test 9.4 catches it.

**Station density.** Place stations so chord sag never exceeds `0.02 mm`:
adjacent-station angle `θ` satisfies `radius · (1 − cos(θ/2)) ≤ 0.02`. Straight
runs get stations only at their endpoints. Always place a station exactly at
every primitive boundary and at both path ends.

Do not expose a `resolution` parameter. Resolution follows from tolerance, which
is the thing that actually matters for a printed part.

### 4.3 Sweep → mesh

No booleans. Given `N` stations and the fixed 12-vertex profile:

- **Vertices:** `N × 12`, index `i*12 + j`.
- **Side faces:** for each `i ∈ [0, N−2]`, each `j ∈ [0, 11]`, the quad
  `(i,j), (i,j+1), (i+1,j+1), (i+1,j)` with `j+1` mod 12.
- **End caps:** the 12-gon at station `0` and at `N−1`, the first reversed.

Manifold by construction: every edge is used by exactly two faces, no vertex is
duplicated, no solver is involved. The only failure mode is self-intersection,
guarded in §4.1.

```python
@dataclass(frozen=True)
class MeshData:
    verts: np.ndarray        # (V, 3) float64, mm
    faces: list[list[int]]   # CCW, outward
```

`MeshData` is plain data and knows nothing about Blender.

---

## 5. Constructions B and C — junctions and grafts

A junction is `N` straight arms meeting at a hub. Because junctions are level
and unbanked, the whole piece is **prismatic in z**: three flat slabs, with the
top and bottom rail slabs identical. Flip symmetry is therefore automatic.

```
PLAN VIEW, X junction         SLAB STACK (side view)

    ##│      │##              +2.35 ┌──────────┐   rail slab
    ##│      │##                    │ rail_rgn │
  ════╡      ╞════      +0.70 ┌─────┴──────────┴─────┐
      │ hub  │                │      deck region     │
  ════╡      ╞════      -0.70 └─────┬──────────┬─────┘
    ##│      │##                    │ rail_rgn │
    ##│      │##              -2.35 └──────────┘
```

### 5.1 Arms

```python
@dataclass(frozen=True)
class Arm:
    angle: float          # radians, plan, CCW from +X
    port_distance: float  # mm, hub origin to port face
```

Arm `i` is a band of width `width_outer` centred on the ray at `angle`, with
direction `u_i = (cos θ_i, sin θ_i)` and left normal `n_i = (−sin θ_i, cos θ_i)`.
Its two side edges are the lines offset `±half_width` along `n_i`.

Standard layouts:

| Name | Arm angles |
|---|---|
| straight (as a degenerate hub) | 0°, 180° |
| T | 0°, 90°, 180° |
| Y | 90°, 210°, 330° |
| X | 0°, 90°, 180°, 270° |

Asserted: `N ≥ 3` for a junction; arms sorted by angle; every adjacent angular
gap `≥ 60°`; every `port_distance` large enough to clear the armpits and the
connector lap (§5.4).

### 5.2 The outline

Walk the arms in CCW angular order. Between adjacent arms `i` and `j`, take
arm `i`'s `+n` edge and arm `j`'s `−n` edge — the two facing edges — and:

- **They intersect** (angular gap < 180°) → that point is the **armpit**.
- **They are collinear** (gap exactly 180°, e.g. a straight-through pair, or the
  back of a T) → no vertex; the boundary runs straight through.
- **They diverge** (gap > 180°) → rejected by the `≥ 60°` and layout asserts;
  such a layout has no closed outline.

Optionally replace each armpit with an arc of radius `corner_radius` tangent to
both edges. **That arc is the curved road a car turns along**, so it is not
cosmetic. `corner_radius = 0` gives the square-cornered variant.

The outline is then, walking CCW: port face of arm `i`, arm `i`'s `+n` edge out
to the armpit, the armpit (or its fillet arc), arm `j`'s `−n` edge back in to
arm `j`'s port face, and so on.

A **port face** is the segment perpendicular to `u_i` at distance
`port_distance`, spanning `±half_width`.

### 5.3 Chains and slabs

Breaking the outline at the N port faces leaves exactly **N chains**. Each chain
carries one edge unit's rail; port faces carry none, because that is where the
next piece continues.

```
deck region   = the outline polygon
rail regions  = for each chain: the strip between that chain and its
                inward offset by rail_thickness, capped at both port faces
```

The rails therefore follow the filleted armpits, so a car turning through the
junction is **guided by the fillet**. On an X, the through-path's side rails are
interrupted where the crossing arms open — that is inherent to a crossing, not a
defect.

Build as prisms and union:

```
deck prism  : the outline,      z ∈ [−half_deck,   +half_deck  ]
rail prisms : each rail region, z ∈ [−half_height, +half_height]   (full height)
```

Rail prisms span the **full** height, not just above and below the deck, so
their overlap with the deck prism is volumetric rather than face-to-face.

**The deck slab uses the outline itself — do not inset it.** An earlier version
of this section called for insetting the deck by `rail_thickness / 2` along
rail-covered edges, reasoning that coplanar vertical faces are what generated
v1's non-manifold results. That trades one degeneracy for a worse one. An inset
boundary lands *inside* the rail's port-cap face rather than on its edge, so the
solver must split that face and recompute the point from a different expression.
On the X and T the two expressions agree bit for bit; on the Y, whose
coordinates are irrational, they differed by 4e-7 mm and the union came out
carrying sliver triangles — which passed every in-memory check and only surfaced
after a float32 STL round trip.

Using the outline directly means the deck and the rails consume the very same
`chain()` output, so their shared vertices are identical floats rather than
nearby ones. **Coplanar faces built from identical vertices are the easy case
for a boolean; near-coincident vertices are the hard one.** Prefer exact
coincidence to a near miss.

Booleans are permitted here under §7a's policy: every input a valid solid, the
result re-checked against every item in §7 before export. Use Blender's
`MANIFOLD` solver.

Polygon triangulation for the prism caps is ear clipping — roughly 60 lines, no
dependency. Do not add a 2D geometry library; the outline is constructed
directly from line intersections and tangent arcs, so generic polygon booleans
are not needed.

### 5.4 Flip symmetry of a junction

Flipping a piece is a 180° rotation about a horizontal axis, which in plan view
acts as a **mirror** about that axis. So:

> A junction is flippable **iff** its plan layout is mirror-symmetric about some
> in-plan axis.

T, Y and X all satisfy this. An asymmetric layout does not, and must either be
rejected or explicitly marked non-flippable. Assert at construction.

---

### 5.5 Construction C — grafts, and how a bridge stands up

A **support** is a short straight track section with a third port on the
**graft axis**, square to the track. The leg that reaches the ground is an
**ordinary track piece stood on end**. There is no separate foot part: a support
turned over is one (§5.6).

No new interface is invented. Because §6.1's port is genderless and identical
everywhere, a straight is already a structural column — that reuse is the payoff
of the connector derivation, and it is why supports cost almost nothing now
while they would have been a separate design problem under a gendered scheme.

**Geometry.** The stub is the track profile swept vertically, placed with its
`across` along world X and its `up` along world Y. The profile's rail flanges
then land exactly on the horizontal piece's own lower rails — both occupy
`|x| ∈ [rail_inner, half_width]` — and the web rises into the open channel to
meet the deck underside. The section simply continues downward, and the stub is
an I-beam column, which is the right shape for the job by accident of the track
already being one.

```
SECTION ACROSS THE TRACK, at a support

     ####                            ####     upper rails
     ##################################       deck
     ####                            ####     lower rails
     ####                            ####
     ####            ||              ####     stub flanges continue the
     ####            ||              ####     rails; stub web in the middle
     ####            ||              ####
     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~     port face, looking down
```

**Construction.** The horizontal piece is Construction A; the stub is
Construction A rotated; the two are unioned. Their flanges overlap by volume, so
this is boolean-friendly. §7a's policy applies.

**Loads.** The bridge presses down on the leg, so every vertical joint is in
**compression**. Tab tips bear on notch bottoms after `fit_clearance` of settle,
and compression is the joint's strong direction (§6.7) — a support column is
loaded on the good axis. The leg only hangs in tension when the whole bridge is
lifted, and a leg weighs a few grams.

**Heights.** Legs stack in whatever straight lengths the set contains, plus the
two stubs. For heights between increments, `pier(height)` is a
single-piece leg cut to length: same port, no new geometry, just different path
data.

### 5.6 What "flippable" actually means

An earlier draft of this section carved supports out as a deliberate exception
to flip symmetry. That was wrong, and it is recorded here because the mistake
came from testing the wrong thing rather than from getting the geometry wrong.

The requirement lives at the **port** level: every port is genderless and
flip-symmetric (§6.1). That is what makes a piece usable either way up, and it
holds for a support exactly as it holds for a straight.

Piece-level flip congruence — rotate the whole solid 180° about its long axis
and get an identical vertex set, test 9.18 — is a *consequence* for pieces whose
body happens to be symmetric about `z = 0`. It is a cheap, sharp test and worth
keeping. It is not itself the requirement.

A support's body is not symmetric about `z = 0`, so it fails 9.18. It is still
flippable, because flipping it is still legal and still useful:

| orientation | role |
|---|---|
| stub down | **support** — grafted into the bridge, leg hanging below |
| stub up | **foot** — the track section lies on the ground, leg plugs in above |

**So there is no separate foot part.** A support turned over *is* the foot. Its
own track section becomes the base, resting on the two lower rails for a
`channel_width` × piece-length footprint, and its two through-ports stay usable
at ground level, so a leg can rise straight out of ordinary ground-level track.

A complete bridge leg:

```
   ground track ── support, stub up  (this is the foot)
                        │
                     leg: an ordinary straight, stood on end
                        │
   bridge deck ──── support, stub down
```

Two part types in that picture, and one of them is a plain straight.

No part carries a hand-written "not flippable" flag, and none may acquire one.
A flag would be a place for a special case to hide; the geometry already says
everything that needs saying, and test 9.32 checks that the parts failing 9.18
are exactly the grafted ones and no others.

---

## 6. The connector

Every port on every piece — swept or junction — carries identical geometry,
expressed in that port's own outward-pointing frame.

### 6.1 Derivation — read before touching connector code

Two requirements:

- **Genderless.** No male end and female end. Every port is identical, and a
  port mates with a copy of itself rotated 180° about the vertical axis.
- **Flippable.** A piece rotated 180° about a horizontal axis is the same part.
  The track drives on either face.

Genderless is not a preference. **Junctions require it.** Gendering imposes
alternating parity around any connected structure:

- A **3-way** has an odd number of ports. Odd cycles cannot alternate, so a T or
  Y is impossible to gender consistently without a dedicated reversing adapter
  piece.
- A **4-way** can be gendered M/F/M/F, but that is only C2 symmetric, so an X
  loses its C4 symmetry and acquires a wrong-way-round orientation.

This is also the "forced asymmetry" in the two-port pieces: with gendered ends,
a piece inserted reversed to close a loop meets like with like.

Now model a port face as `P(x, z)` over the cross-section: `+1` where material
protrudes past the nominal port plane (**tab**), `−1` where it is cut back
(**notch**), `0` flat.

Genderless mating maps `x → −x`, and a tab must land in a notch:

```
P(−x, z) = −P(x, z)                       (1)   odd in x
```

Flipping maps `(x, z) → (−x, −z)` and must leave the part unchanged:

```
P(−x, −z) = P(x, z)                       (2)
```

Substituting (1) into (2):

```
P(x, −z) = −P(x, z)                       (3)   odd in z
```

`P` must be odd in x **and** odd in z. Two consequences, both load-bearing:

- **Anything uniform in z is impossible.** Uniform in z means
  `P(x, −z) = P(x, z)`, which with (3) forces `P ≡ 0` — a flat butt joint with
  no interlock. So every interlock confined to the deck plane, or to any single
  horizontal slab, cannot work. That includes v1's dovetail tab and every
  tab-one-side/notch-other sketch. No choice of dimensions rescues them.
- **The minimal solution is `P = sign(x)·sign(z)`:** tabs in the `(+x,+z)` and
  `(−x,−z)` quadrants, notches in `(−x,+z)` and `(+x,−z)`. **Diagonally opposed
  tabs.**

The minimal solution is applied to the **whole cross-section**, not only to the
rails: the port face is simply cut in half at `x = 0` and at `z = 0`, and each
piece keeps two diagonally opposite quadrants.

### 6.2 Geometry — the diagonal lap

Nominal port plane at `y = 0`, body at `y < 0`, `lap_length = L`.

Through the lap zone a piece keeps only its `(+x, +z)` and `(−x, −z)` quadrants,
and those run on past the port plane to `y = +L` as the tabs. The other two
quadrants are cut back to `y = −(L + fit_clearance)`, and that empty volume is
exactly what the mating piece fills.

```
PORT FACE, looking down the track      SIDE VIEW of the +X rail at a joint

   -X rail        +X rail                    |<--- L --->|<--- L --->|
  ┌───────┐      ┌───────┐            ~~~~~~~~~~~~~~~~~~~~~~~~┐
  │ notch │▒▒▒▒▒▒│  TAB  │            piece A, upper half     │
  ├───────┤ deck ├───────┤            ~~~~~~~~~~~~~~~~~~~~~~~~┘~~~~~~~~~~~
  │  TAB  │▒▒▒▒▒▒│ notch │            ────────────────────────┼─────────── z=0
  └───────┘      └───────┘                       ┌~~~~~~~~~~~~~~~~~~~~~~~~
       tab and notch run                         │     piece B, lower half
       right across the deck        ~~~~~~~~~~~~~┘~~~~~~~~~~~~~~~~~~~~~~~~
       as well as the rails                    y=-L        y=0        y=+L
```

Three clearances, all `fit_clearance`:

- **Vertical**, at the lap plane. Our upper half starts at `+fit_clearance/2`,
  the mating lower half stops at `−fit_clearance/2`.
- **Longitudinal**, at the tab tips. Notches are cut `L + fit_clearance` deep so
  a tab never bottoms out.
- **Lateral**, at the centreline. The split runs through `x = 0`, so in the lap
  zone our `+x` half slides past the mating piece's `−x` half. They need
  clearance, and the result is a slot one clearance wide running down the
  centreline for the length of the lap. On a 24 mm track that slot is 0.2 mm —
  narrower than an extrusion width, and a car straddles it.

Note what the diagonal split buys structurally: the overlap is the **entire**
cross-section, not just the rails, so the joint carries far more bending and
shear than a rails-only lap would.

There is no butt face anywhere, so nothing is a hard datum along the joint axis.
The detents locate the joint; see §6.7.

### 6.3 Retention — detent rib and groove

A bare lap slides apart. Retention is a **rib** on one lap face dropping into a
**groove** in the other.

Putting a rib at the same `(x, z)` quadrant on both parts makes rib meet rib.
The fix is a **longitudinal offset**. On each lap face, at signed distance `±d`
from the port plane, `d < L`:

- a **rib** at local `y = +d`
- a **groove** at local `y = −d`

Check: A's `+X` upper lap carries a rib at world `+d`, groove at world `−d`.
Piece B is the same part rotated 180° about Z, so its local `y` is world `−y`
and its local `−X` rail is the world `+X` rail. By flip symmetry (2), B's
`−X`-lower lap carries a rib at local `+d` (world `−d`) and a groove at local
`−d` (world `+d`). So:

- A's rib at world `+d` → B's groove at world `+d` ✓
- B's rib at world `−d` → A's groove at world `−d` ✓

Two engagements per rail, four per joint, part still identical at every port and
under flip.

**The ramp is asymmetric**: shallow on the insertion side, steep on the pull-out
side. Easy to push together, hard to pull apart. The mating rotation flips `y`,
so a groove is the *mirror* of a rib, grown by clearance.

**Detents live on the rails, not the deck.** The rail gives 2.35 mm of depth to
sink a groove into; half the deck is 0.7 mm and has none.

**The compliance comes from the lap length.** A rib can only seat if something
deflects. At `L = 4` the rail tab is a stubby 8 mm cantilever, effectively rigid
in z, so the rib could only seat by consuming rigid-body clearance — the very
thing being minimised — and the detent was worth almost nothing. At `L = 8` the
tab is a 16 mm cantilever, 1.2 × 2.25 mm in section, and 0.3 mm of deflection is
about **0.5 % strain**, comfortably inside PLA's elastic range. The rib then
seats by bending the tab, and there is free air above the upper tab and below
the lower one for it to bend into. Lap length is therefore a structural
parameter, not a styling one; do not shorten it without redoing this sum.

Ribs run **across** the rail and are inset from the rail inner face by
`fit_clearance`, so they do not rub along the mating piece's deck edge.

### 6.4 Parameters

| Name | Default | Meaning |
|---|---|---|
| `lap_length` | 8.0 | tab protrusion `L`, mm — also sets detent compliance |
| `fit_clearance` | **0.15** | gap on every mating face, mm — *calibrated, frozen* |
| `detent_offset` | 4.0 | rib/groove distance `d` from port plane, mm |
| `detent_height` | 0.50 | rib height above the lap face, mm |
| `detent_lead_angle` | 30° | insertion ramp, from the lap plane |
| `detent_return_angle` | 60° | pull-out face, from the lap plane |

Asserted: the lead-in is shallower than the return face; rib and groove do not
overlap in `y`; the rib stays on the tab and the groove inside the notch; the
groove does not cut through the rail; the centreline slot is narrower than the
deck.

**`fit_clearance` is not to be guessed.** It was calibrated by printing the
Phase 0 comb: **0.15 mm fitted best**, tighter bound and looser was sloppy. It
is frozen at that. Looseness is a calibration problem, not a modelling problem,
and editing geometry will not fix it.

`detent_return_angle` is the second knob. Steeper holds harder; too steep and a
child cannot take the track apart. Phase 0 answers it.

### 6.5 Assembly direction

**The joint is a prismatic joint with exactly one free axis: sliding along the
track.** Pieces are pushed together horizontally. Lowering one piece onto
another is not merely awkward, it is **geometrically impossible** — see §6.7.

### 6.6 How the connector is applied

The connector is defined **once**, in a port's own frame, as a set of cut solids
and addition solids, and transformed to wherever a port is. That single
definition is why a curve's angled end, a ramp's sloped end and a junction's arm
all carry byte-identical joints (test 9.21), and why neither `sweep.py` nor
`hub.py` contains a line about what a joint looks like.

Applying it is a boolean, under §7a. An earlier version of this section forbade
that and required explicit topology instead. That is not achievable and the
reasoning was wrong: through the lap zone a port's cross-section is **two
disjoint quadrants**, so it is not a ring, and the constant-vertex-count sweep
of §4.3 cannot express it. Building the surface by hand instead would mean
bespoke stitching code at every port of every construction — precisely the kind
of thing that produced v1's non-manifold output. The boolean is the safer path,
and §7a is what makes it safe.

Order matters: **cuts before additions**. A tab is added flush against the slot
and notch boundaries the cuts define, so adding first would leave a cut tool
coincident with a face it must not touch.

Two details are load-bearing, and both were found by the Y junction, whose port
planes sit at irrational angles where the X and T's arithmetic is exact:

- **There is no separate centreline-slot tool.** The two notch tools each reach
  one slot half-width past `x = 0`, so across `|x| < fit_clearance/2` they
  overlap and between them remove every z — which is the slot. A third tool for
  it is wholly contained in their union and contributes nothing but coplanar
  faces.
- **A detent's base is buried well behind its lap face**, not by `EPS`. A detent
  is a triangle with a protruding apex and a buried base; sunk only a hair, its
  sloped flanks cross the lap plane a fraction from the base corners and the
  solver must resolve near-parallel surfaces meeting at a sliver. The flank runs
  are measured over the whole triangle, base included, so burying the base
  deeper changes nothing a mating part can feel.

### 6.7 What the joint constrains, and what it cannot

Worth stating plainly, because the obvious worry about a lap joint is that it
just falls apart.

**Rigidly blocked, by material rather than by friction:**

- **Vertical.** At the `+X` rail our upper half sits above the mating piece; at
  the `−X` rail it sits below. Lifting is stopped by the `−X` rail, lowering by
  the `+X` rail. This is the whole point of the diagonal rule, and it is why a
  piece cantilevered off a joint in mid-air holds: gravity is pure `Z`, and `Z`
  is blocked.
- **Lateral.** Our deck edge fouls the mating piece's rail.
- **Pitch and roll**, by bearing, within the clearance. Angular play is of order
  `fit_clearance / lap_length` — about 1.4° at the defaults, which is why `L`
  was doubled.

**Not blocked rigidly, and provably cannot be:**

- **Sliding along the track.** The argument: flip symmetry makes `P` odd in `z`,
  so the joint is not z-prismatic and cannot be assembled by a straight slide
  along `Z`. Genderlessness makes `P` odd in `x`, ruling out `X` the same way.
  So the only straight-line assembly path is along `Y`. But anything assembled
  by sliding along `Y` is y-prismatic through its engagement, and y-prismatic
  geometry slides *out* exactly as freely as it slid in.

  **Therefore no rigid geometry can lock this joint along its own axis.**
  Retention on that one axis must be elastic. That is what the detents are for,
  and it is why their compliance budget in §6.3 matters.

**Consequence for bridges.** A piece cantilevered from a joint is fine. A
multi-segment span between two supports is not: bottom-fibre tension across a
lap has no load path, so a chain of these joints is a chain of near-hinges and
will sag progressively. Span with a single `Ramp` piece, or support the joints.
Do not try to fix this by stiffening the connector; the limit is the joint
topology, which §6.1 fixes.

## 7a. Boolean policy

Three constructions use booleans: the junction slab union (§5.3), the support
graft (§5.5) and the connector (§6.6). The rule is not "avoid them" — it is:

1. **Every input is a valid solid in its own right**, checked before the solver
   sees it. Otherwise a failure afterwards wrongly blames the solver.
2. **Prefer exact coincidence to a near miss.** Coplanar faces built from
   identical vertices are the easy case; vertices that are merely close are the
   hard one. Share the same computed points rather than recomputing them.
3. **Prefer volumetric overlap to face contact** where it costs nothing.
4. **Do not add a tool whose effect another tool already covers.** A redundant
   tool contributes only coplanar faces.
5. **Repair, then validate strictly.** An exact solver may leave sub-tolerance
   artifacts — a vertex inserted on an edge it touches, a triangle collinear to
   within a nanometre. They carry no volume and no printed part can express
   them, but §7 cannot tell a harmless sliver from a real one, and it should not
   try. Weld and dissolve at a tolerance far below anything a printer resolves
   (5 µm against a 150 µm smallest real feature), then run §7 unrelaxed. The
   tolerance is capped by an assertion so it can never approach real geometry.

Chasing every artifact back to its cause is endless, and it tempts you to loosen
§7. Loosening §7 is how v1 shipped unprintable parts.

---

## 7. Mesh validation

Runs in the core on `MeshData`, no Blender. Failure raises; it does not log.

1. **Edge manifoldness.** Every undirected edge used by exactly 2 faces.
2. **Consistent orientation.** Every interior edge traversed in opposite
   directions by its two faces.
3. **Outward normals.** Signed volume by the divergence theorem is `> 0`.
4. **No degenerate faces.** Every face area `> 1e-9`.
5. **No duplicate vertices.** No two within `1e-9`.
6. **Watertight.** `V − E + F = 2` (genus 0 for every piece in scope).

A piece failing any check is never exported. Applies equally to Construction A
output and to the post-union result of Construction B.

---

## 8. Modules and the dependency rule

```
car_tracks2/
  trackcore/            # pure Python + numpy. NO bpy. Not one import.
    config.py           # dataclasses, defaults, unit handling, validation
    edge_unit.py        # §2, and the derived 12-vertex profile
    path.py             # §4.1
    frames.py           # §4.2
    sweep.py            # §4.3   Construction A
    hub.py              # §5      Construction B
    connector.py        # §6, defined once in a port frame
    validate.py         # §7
    mesh.py             # MeshData, Piece, primitives, ear clipping
  blender/              # the only place bpy appears
    build.py            # MeshData -> bmesh -> object
    boolean.py          # §7a, MANIFOLD solver
    cleanup.py          # §7a.5, weld and dissolve below print resolution
    export.py           # STL / 3MF
    run.py              # entry point for `blender --background --python`
    mate_check.py       # §9.19 against finished solids
    preview.py          # renders
    assemble.py         # places parts at each other's ports and renders
  parts/                # part definitions; a straight is ~3 lines
  tests/                # pytest, system python, Blender not required
  docs/SPEC.md          # this file
```

**`trackcore` never imports `bpy`.** Grepping `bpy` under `trackcore/` returns
nothing, and a test asserts it.

The reason is feedback speed, not purity. Every geometry bug the previous
attempts shipped was invisible until someone opened a viewport. With this split,
`pytest` finds them in under a second, headless.

Blender does what it is genuinely good at and tedious to reimplement: robust
mesh datastructures, boolean solving, format export, preview rendering. It runs
as `blender --background --python`; the GUI is never required.

Dependencies: `numpy` (present, 1.26.4) and Blender (present, 5.1.0). Nothing
else without asking. In particular **no shapely** — §5.2 constructs the outline
directly.

---

## 9. Acceptance tests

A phase is done when its tests pass. Each maps to a way a previous attempt
failed.

**Edge unit and profile**

- 9.1 Profile has 12 vertices, is CCW, is simple, bounding box `24.0 × 4.7`.
- 9.2 Profile equals its own reflection about `x = 0` and about `z = 0`, to
  `1e-12`.
- 9.3 The profile is reconstructible as exactly two edge units; the edge unit is
  not duplicated as literal constants anywhere.

**Paths and frames**

- 9.4 **No twist.** Over `chain(Line, Arc, Line)` with `bank = 0`, every frame's
  up-vector satisfies `S · ẑ > 0.999`. *A Frenet implementation fails this at
  the straight-to-arc transition. That is the point of the test.*
- 9.5 `Arc(radius=100, angle=π/2)` ends at the analytic point and tangent, `1e-9`.
- 9.6 `chain` raises on a C1 discontinuity.
- 9.7 `Arc(radius=10, angle=π/2)` raises `PathTooTightError`.
- 9.8 Station spacing satisfies the §4.2 sag bound everywhere.

**Construction A**

- 9.9 A straight 84 mm piece has bounding box `24.0 × 84.0 × 4.7` to `1e-9`.
- 9.10 Its volume equals `profile_area × 84.0` to `1e-6` relative.
- 9.11 A 90° arc's volume equals `profile_area × arc_length` to `1e-3` relative.

**Construction B**

- 9.12 X, T and Y outlines have exactly N port faces and N chains.
- 9.13 An X with `corner_radius = 0` has square armpits at the analytic
  intersection points, to `1e-9`.
- 9.14 With `corner_radius = r > 0`, each fillet arc is tangent to both adjacent
  edges to `1e-9`.
- 9.15 A straight built as a hub (`0°, 180°`) agrees with Construction A's
  straight on volume and bounding box to `1e-6`.
- 9.16 Junction volume equals `deck_area × deck_thickness + Σ rail strip areas ×
  (rail_height_total − deck_thickness)`, to `1e-3` relative.
- 9.17 An asymmetric arm layout raises rather than producing a non-flippable
  part (§5.4). So does an angular gap over 180°, which no longer surrounds the
  centre, and a port too close to its armpit.
- 9.17a The deck slab and every rail slab share **bit-identical** boundary
  vertices, and no two vertices of a slab lie within a weld tolerance.

**Connector — the tests that were missing**

- 9.18 **Flip symmetry.** Rotate any finished piece 180° about a horizontal
  symmetry axis; the rounded, sorted vertex set is identical. A gendered or
  z-asymmetric connector fails here.
- 9.19 **Genderless mating.** A piece and a copy rotated 180° about Z and
  translated to the joint do not intersect, and the gap on every nominally
  mating face equals `fit_clearance` to `1e-6`.
- 9.20 **Rib/groove engagement.** All four rib–groove pairs at a joint overlap
  as intended in x, y and z; no rib meets a rib.
- 9.21 **Port interchangeability.** Every port of every piece — straight, curve,
  ramp, T, Y, X — presents byte-identical geometry in its own port frame.
- 9.22 **Chain closure.** Eight 45° arc pieces of equal radius, mated
  end-to-end, return to the start position and heading to `1e-6`.
- 9.23 **Junction round trip.** A loop of straights and curves closed through a
  T, entering by one arm and leaving by another, closes to `1e-6`.

**Supports — Construction C**

- 9.27 A support's three ports present identical geometry in their own port
  frames, indistinguishable from a straight's ports.
- 9.28 The downward port's frame is vertical, centred on the piece, and square
  to the track above it.
- 9.29 A support unions to exactly one watertight solid and passes all six §7
  checks after the union.
- 9.30 An ordinary straight, stood on end, mates the downward port with exactly
  `fit_clearance` — a leg is a track piece, not a special part.
- 9.31 A support turned over rests on a single plane — it is its own foot —
  and its through-ports stay mate-able in that orientation.
- 9.32 The parts failing piece-level congruence (9.18) are exactly the grafted
  ones, derived from geometry rather than from a flag. For each of those, every
  port still mates after the flip. No part carries a "not flippable" flag.

**Whole-mesh and architecture**

- 9.24 Every piece in `parts/` passes all six checks of §7, including junctions
  after the §5.3 union.
- 9.25 `grep -r "bpy" trackcore/` returns nothing.
- 9.26 The whole `trackcore` suite runs on system `python3` with Blender absent
  from `PATH`.

---

## 10. Phases

Do not start a phase before the previous one's tests pass. Do not work ahead.

**Phase 0 — calibrate the joint. Before any other code.** ✅ **COMPLETE**

The connector is the highest-risk part, so it was prototyped first and alone.
See `phase0/README.md`. A throwaway script emits a 20 mm coupon with one port,
and because the port is genderless two identical coupons mate with each other.
The comb printed `fit_clearance ∈ {0.10, 0.15, 0.20, 0.25, 0.30}`, identified by
1–5 slots cut through the deck at the blank end.

Every coupon was checked against all six §7 rules, plus a flip test (§9.18) and
a mesh-level mate test (§9.19) that boolean-intersects a coupon with a rotated
copy of itself and requires exactly zero shared volume. The mate test earned its
place immediately by finding the missing lateral relief now recorded in §6.2.

**Results, from printed parts. These are frozen inputs to every later phase.**

| Question | Answer |
|---|---|
| `fit_clearance` | **0.15 mm** — best of the comb; 0.10 bound, looser was sloppy |
| print orientation | **on its side**, one rail's outer face on the bed, with a brim |
| assembly | **horizontal push**, and a child managed it unaided |
| detents | felt as two distinct clicks; firm |
| hang test (§6.7) | holds a dangling piece, no reported droop |
| lift test | pieces stay connected — **the diagonal split blocks vertical, as designed** |
| pull-out along the axis | firm, but separable by hand — `detent_return_angle` **60° confirmed** |

The lift and hang results are the important ones: they confirm empirically what
§6.7 argues geometrically, that vertical load is carried by material rather than
by friction. And pull-out landing on "firm but will come apart" is exactly the
target for a toy — the one axis that can only be held elastically is held about
right.

**Phase 1 — edge unit and Construction A.** ✅ **COMPLETE**

`trackcore/` holds `config`, `edge_unit`, `path`, `frames`, `sweep`, `validate`
and `mesh`; `blender/` holds the export layer; `parts/` holds straight, curve,
ramp and s_bend as path data only. Tests 9.1–9.11 and 9.25–9.26 pass, 77 of
them, headless, in about a second.

Measured against the analytic answer: a straight comes out at exactly
`profile_area × length` (0.0000 % error), and a 90° arc at 0.026 % under the
Pappus volume, which is the expected chord deficit at a 0.02 mm sag tolerance
and shrinks when the tolerance is tightened.

Two things worth recording:

- The `s_bend` part exists only to exercise a straight → left turn → right turn
  → straight sequence, because that is where a Frenet frame flips its normal.
  Test 9.4 holds `up · ẑ > 0.999` across it. A Frenet implementation fails
  there, which is the point.
- Ear clipping was needed earlier than §8 implies. The end caps are the I-beam
  profile, which is **non-convex**, so a fan triangulation from one corner
  emits triangles crossing the open channel between the rails plus two exactly
  collinear ones. The signed sums used for area and volume survive that
  silently; an exported STL does not. `mesh.triangulate` ear-clips, and
  `tests/test_mesh.py` pins the defect so the cheap version cannot come back.

**Phase 2 — Construction B.** ✅ **COMPLETE**

`trackcore/hub.py` builds the plan outline, the chains, the fillets and the
slabs; `blender/boolean.py` unions them. `parts/` gains X, T and Y in square and
filleted variants — six entries, no new geometry code. Tests 9.12–9.17 and 9.24
pass, 164 in total.

Two things worth recording:

- **The fillet adds material, it does not cut it.** A car turning from one arm
  to the next hugs that armpit from the inside, so the arc is the surface it
  slides along and `corner_radius` is its turn radius. Cutting instead of
  adding produces a hub that pinches the turn rather than guiding it, and the
  two look equally plausible on paper.
- **The Y found a defect the X and T could not.** Its coordinates are
  irrational, so two expressions for the same point disagree in the last bits
  where the axis-aligned layouts agree exactly. See §5.3 on why the deck slab
  is no longer inset. The lesson generalises: test the layout whose arithmetic
  is not exact.

**Phase 3 — connectors on everything.** ✅ **COMPLETE**

`trackcore/connector.py` defines the joint once in a port frame; `sweep.py` and
`hub.py` each expose their port frames; `parts.build` applies it. Neither
construction contains a line about what a joint looks like. Tests 9.18–9.23
pass, 283 in total.

The decisive test is not any of the symmetry proofs but
`test_two_finished_parts_actually_fit_together`: it builds two real parts,
booleans, cleanup and all, mates them through `MATE`, intersects them and
requires **zero** shared volume. Every pairing tried passes — curve into X,
ramp into rounded T, S-bend into Y, rounded T into rounded X. Any port mates
any port, either way round, which is the whole claim of §6.1 discharged against
actual solids rather than against a diagram.

Two corrections to earlier sections came out of this phase: §6.6 (the connector
is applied by boolean, and the previous prohibition was not achievable) and
§7a, which replaces three scattered "this is the only sanctioned boolean"
claims with one policy.

**Phase 5 — supports.** Construction C: the grafted stub and `pier(height)`.
No foot part — a support turned over is one. Tests 9.27–9.32. Output: a bridge
that stands up on legs made of ordinary track.

**Phase 4 — the part set.** ✅ **COMPLETE**

Fourteen parts on a **layout grid**, `module = 96 mm`: three straights (full,
half, quarter), four curves (90°, 45°, tight 90° at half radius, banked 90°),
a ramp, and six junctions (X, T, Y, each square and filleted).

The grid is the point. A 90° curve of radius `module` advances exactly one
module on each axis and turns a right angle, so it tiles with the straights. A
junction's arms reach half a module, so passing through one consumes exactly
the distance of the straight it replaces. Loops therefore close instead of
almost closing — tested on five different loops. Two caveats worth stating
rather than discovering: 45° curves only land on the grid **in pairs**, and a
120° Y cannot tile a square grid at all. It is in the set because a three-way
with interchangeable ports is worth having.

`Grid` is one dataclass; changing `module` moves the whole set.

**No geometry code was added**, but Phase 4 did find a rule Phase 1 missed, now
in `path.DEFAULT_PORT_CLEAR`:

> **The cross-section must not roll, or pitch, inside a lap zone.**

The connector's cut tools are flat boxes aligned to the port frame (§6.6). A
banked 90° curve had reached six degrees of bank where the notches bite, so
they sliced the tilted section at the wrong height on each rail and the piece
came out **genus 3**. Horizontal curvature is harmless — it moves the section
sideways, not in z, and a notch removes everything below the lap plane whatever
its lateral position. Roll and vertical curvature are not.

Two fixes, and only one of them touched code. `Arc` now holds its bank flat over
each lap zone before easing in. The ramp instead gets **flat lead-ins** and is
pure part data: a smoothstep's vertical curvature is greatest at its ends,
exactly where the connector reaches, so each port sits on a straight. `run`
still measures the whole piece, so the set still tiles.

Every part fits a 220 mm bed; the longest is the ramp at 192 mm.

---

## 11. Assumptions and open questions

Recorded assumptions, made so work can proceed; overrule any of them and the
affected section changes.

- **A1. Rail gaps at crossings are left fully open.** No threshold ramp across
  an arm opening. Justification: cars are hand-pushed, so there is no speed to
  fly off at, and commercial track does the same. A `threshold_height` parameter
  is reserved but defaults to 0 and is not implemented in Phases 0–3.
- ~~**A2. Layouts are freeform.**~~ **Overruled in Phase 4.** The set is laid
  out on a grid: see §10 Phase 4. `Hub.auto` still derives the minimum port
  distance and is what the tests use; the catalogue uses `Hub.uniform` to pin
  arms to half a module instead.
- **A3. Junction arms are straight.** Curvature comes from attaching a curve
  piece to a port.

Still open, deferrable to Phase 4:

1. The standard radii, angles and lengths of the part set.
2. Whether the deck wants a grip texture.
3. ~~Bridge piers.~~ **Decided.** In scope as Construction C (§5.5). The flip
   symmetry objection was simply wrong: flip symmetry is a property of ports,
   not of piece bodies (§5.6), and a support turned over is its own foot.
