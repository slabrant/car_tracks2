"""The connector. docs/SPEC.md §6.

Pure Python + numpy. Never imports bpy.

The port is the **six-column split** of §6.1: two rails and four deck columns,
cut by a single horizontal plane and each keeping one side of it.

    R1 rail | D1 deck | D2 deck | D3 deck | D4 deck | R2 rail
    below   | below   | above   | below   | above   | above

Genderless needs one thing only: that the pattern be **odd in x**,
`P(-x, z) = -P(x, z)`, because two ports meet under `MATE`, a 180° turn about
the shared up axis. The signs above are odd, so the same part mates with
itself. It leaves four tabs — a rail always shares its handedness with the deck
column it touches and joins onto it — two reaching over the mate and two under.

**The plane lies inside the deck**, at `deck_mid`, not at the section's
mid-height. That is the whole design. A U-channel's deck sits wholly below
mid-height, so a plane there never touches it: the deck halves end up side by
side sharing nothing but a vertical face, and the only material lapping
vertically is the two rail laps. That version was built and shipped. It mated
cleanly, passed every rule in §7, and would have hinged apart under a car. In
the deck, every column laps half the deck's thickness over its mate, across
nearly the whole channel, in both directions.

Handedness changes three times across the section, and each change costs a
clearance-wide seam through the deck. All three fall in flat deck — at the
centreline and at `±deck_column` — and **none at a rail root**, which is the one
place a seam cannot go; see `deck_column`. They run *along* the direction of
travel, so a wheel rolls parallel to all three and never crosses one.

The two rails come out unlike: the plane is near the bottom of the section, so
one rail keeps a thin sliver below it and the other the tall part above. That
is why ribs go on one rail and grooves in the other, and why there are two of
each; see `Connector.detent_spacing`.

Everything here is expressed **once**, in a port's own frame, and transformed to
wherever the port is. That is what makes a curve's angled end and a junction's
arm carry the identical joint (test 9.21), and it is why there is no per-piece
connector code.

Port frame convention: origin on the port plane, `+Y` pointing **out** of the
piece, `+Z` up, `+X = Y × Z`. The body is at `y < 0`.
"""

from __future__ import annotations

import math

import numpy as np

from .config import DEFAULT, TrackConfig
from .mesh import MeshData, Pt2, box, prism_yz

EPS = 0.01
"""Overlap used to keep boolean inputs from meeting face-to-face."""

def port_extension(config: TrackConfig = DEFAULT) -> float:
    """How far a body is swept past each nominal port, mm.

    Every construction builds long and lets `cuts` trim: the tab is part of the
    body, not a solid glued to it. See `additions` for why."""
    return config.connector.lap_length

DETENT_SINK = 0.25
"""How far a detent's base is buried behind the lap face it sits on.

Not cosmetic. A detent is a triangle whose apex protrudes and whose base is
buried in the material behind. If the base sits only EPS behind the lap plane,
the triangle's sloped flanks cross that plane a hair from its base corners, and
the boolean has to resolve two near-parallel surfaces meeting at a sliver. On
the X and T the arithmetic is exact and it survives; on the Y, whose port planes
sit at irrational angles, it produced six degenerate triangles. Burying the base
properly moves the crossing well clear of the corners. The protruding shape is
unchanged — see `_detent_polygon`.
"""


def _detent_polygon(config: TrackConfig, y_centre: float, z_face: float,
                    out_dir: int, height: float, grow: float,
                    mirror: bool) -> list[Pt2]:
    """Asymmetric detent profile as a (y, z) polygon, §6.3.

    Shallow on the insertion side so the joint pushes together, steep on the
    pull-out side so it resists coming apart. ``mirror`` flips it in y, which is
    what the mating piece's rotation does to it, so a groove is the mirror of a
    rib grown by clearance.

    The flank runs are measured from the apex over the full triangle, base
    included, so that at `z_face` itself the half-widths come out at exactly
    `height / tan(angle)` regardless of how deep the base is buried. Burying the
    base therefore changes nothing a mating part can feel.
    """
    connector = config.connector
    span = height + DETENT_SINK
    lead = span / math.tan(math.radians(connector.detent_lead_angle_deg)) + grow
    back = span / math.tan(math.radians(connector.detent_return_angle_deg)) + grow
    base = z_face - out_dir * DETENT_SINK

    poly = [(y_centre + lead, base),
            (y_centre, z_face + out_dir * height),
            (y_centre - back, base)]
    if mirror:
        poly = [(2.0 * y_centre - y, z) for (y, z) in poly]

    area = sum(poly[i][0] * poly[(i + 1) % 3][1] - poly[(i + 1) % 3][0] * poly[i][1]
               for i in range(3))
    return poly if area > 0 else list(reversed(poly))


def additions(config: TrackConfig = DEFAULT) -> list[tuple[str, MeshData]]:
    """Material added beyond the port plane: only the detent ribs.

    The tabs are not here because they are not added — see below.
    """
    body, connector = config.body, config.connector
    hw, ri = body.half_width, body.rail_inner
    clear = connector.fit_clearance
    face = body.deck_mid - clear / 2.0     # top of the thin rail's lap
    dh = connector.detent_height

    # The **tabs are swept, not added** — every construction builds its body
    # `lap_length` past the nominal port along the end tangent, and the notch
    # cuts below trim that extension down to the six columns.
    #
    # They used to be boxes unioned on. At the port plane a box's side faces are
    # exactly coplanar with the body's rail faces, and on a curve they are
    # tangent there and diverge going in — a tangential union, which is the one
    # thing an exact solver cannot resolve cleanly. Whether it survived was
    # luck: a 45° arc built at an 8 mm lap and went non-manifold at 7, while a
    # 90° arc of the same radius was fine at both. Sweeping the tab removes the
    # second solid, so there is nothing to be tangent to.
    #
    # Ribs go on the **-x rail only**, the one this piece keeps below the split.
    # Its mate presents its own tall +x rail there, with room for a groove. The
    # +x rail carries the grooves instead; see `cuts`. That asymmetry is forced
    # — see `Connector.detent_spacing` — and is why there are two of them.
    return [
        (f"rib_nx_{index}", prism_yz(
            _detent_polygon(config, offset, face, +1, dh, 0.0, mirror=False),
            -hw, -ri - clear))
        for index, offset in enumerate(connector.detent_offsets)
    ]


def tab_area(config: TrackConfig = DEFAULT) -> float:
    """What one port carries across the port plane, mm².

    Two tabs below the split — the -x rail with the deck column beside it, and
    the third deck column — and two above: the second deck column, and the
    fourth with the +x rail beside it. Every one of them is a deck lamina
    thick, except the +x rail, which reaches from the split to the top of the
    section.

    Ought to come to a little under half the section: half, less what every
    mating face gives up to clearance. `test_a_port_keeps_a_little_under_half`
    checks exactly that, which is what makes this derivation and the section
    check each other rather than merely agree.
    """
    body, connector = config.body, config.connector
    half = connector.fit_clearance / 2.0
    q, lamina = deck_column(config), body.deck_lamina - half
    rail, ri, hw = body.rail_thickness, body.rail_inner, body.half_width

    below = (hw - q - half) + (q - 2.0 * half)
    above_deck = (q - 2.0 * half) + (q - half)
    tall = body.half_height - body.deck_mid - half
    return (below + above_deck) * lamina + rail * tall


def outer_margin(config: TrackConfig = DEFAULT) -> float:
    """How far a cut tool must reach past the rail's outer face, mm.

    The tools are straight boxes in the port frame; the body is not. Over a lap
    zone of reach `d` on a path of radius `R`, the section drifts sideways by
    about `d^2 / 2R`, so on one port of every curve the body leans *out* past a
    tool that only overshoots by EPS. What is left behind is not a chunk — it is
    a wedge tapering to nothing at the port plane, which is a tangential
    degeneracy and exactly what a solver cannot resolve.

    Sizing it for the tightest legal radius makes it curvature-proof. Reaching
    further out is free: past the rail's outer face there is nothing to cut.

    Found by a 45° arc that validated at an 8 mm lap and went non-manifold at
    6 mm — shorter reach, *less* drift, thinner wedge. The failure got worse as
    the geometry got tamer, which is the tell for a tolerance-edge artifact
    rather than a real conflict.
    """
    reach = config.connector.lap_length + config.connector.fit_clearance
    return EPS + reach * reach / (2.0 * config.min_radius)


def groove_depth(config: TrackConfig = DEFAULT) -> float:
    """How far a groove is sunk below the lap face it opens onto, mm.

    A groove has to swallow the mate's rib and no more. The rib's apex stands
    `detent_height` proud of *its* lap face, and the two lap faces are one
    clearance apart, so measured from this face the apex reaches
    `detent_height - fit_clearance`. Half a clearance of margin on top of that
    is `detent_height - fit_clearance/2`.

    It used to be `detent_height + fit_clearance/2` — a full clearance deeper
    than anything could ever reach into it. That was harmless while the groove
    sat in the middle of a tall rail. It stopped being harmless when the split
    moved into the deck: the groove now opens near the bottom of the rail, and
    sunk that much deeper its apex climbed above the *deck surface*, where the
    rail's inner face is no longer buried in deck but standing exposed a tenth
    of a millimetre away. On a curve the body wanders further than that, the
    two grazed, and every curve in the catalogue came out with a tunnel
    through it.
    """
    return config.connector.detent_height - config.connector.fit_clearance / 2.0


def deck_column(config: TrackConfig = DEFAULT) -> float:
    """Width of one deck column, mm. The deck is divided into four.

    The seams therefore fall at `0` and `±deck_column`, all of them in flat
    deck. None falls at a rail root, which is the one place a seam cannot go:
    the cut tools are straight boxes in the port frame while the body is not,
    so over the lap zone a curve's section wanders sideways further than the
    seam is wide, and a seam laid on the rail's concave inner corner *grazes*
    it instead of crossing it. That is a tangential degeneracy, and it broke
    `curve_45` into five non-manifold edges when the boundary was there.

    Keeping the rails clear of it is not a dodge, it is the point: a rail
    column is pure rail, and it shares its handedness with the deck column
    beside it, so no seam is needed at that join at all.
    """
    return config.body.rail_inner / 2.0


def cuts(config: TrackConfig = DEFAULT) -> list[tuple[str, MeshData]]:
    """Material removed behind the port plane: notches, slots and grooves.

    Six columns and one flat split plane at `deck_mid`. Reading across:

        R1 rail | D1 deck | D2 deck | D3 deck | D4 deck | R2 rail
        below   | below   | above   | below   | above   | above

    Odd in x, so the same part mates with itself (§6.1). Handedness changes at
    three places, all inside the deck — `±deck_column` and the centreline — and
    at neither rail root, because each rail runs the same way as the deck
    column it touches.
    """
    body, connector = config.body, config.connector
    hw, ri, hh = body.half_width, body.rail_inner, body.half_height
    lap, clear = connector.lap_length, connector.fit_clearance
    zf = xs = clear / 2.0
    back = -(lap + clear)     # notches are cut one clearance deeper than the tab
    depth = groove_depth(config)
    grow = clear / 2.0
    out = outer_margin(config)

    md = body.deck_mid
    db, dt = body.deck_bottom, body.deck_top
    q = deck_column(config)

    # Each tool overshoots its seam by `xs` rather than stopping on it, so that
    # consecutive tools overlap volumetrically instead of sharing a face. §7a
    # requires it, and the reason is concrete: stopped flush, two of these left
    # a zero-thickness sheet 6 mm long standing inside `curve_45`. Across each
    # seam the pair between them removes every z, which is what opens the
    # clearance slot there — so there are no separate slot tools.
    return [
        # R1 + D1 keep what is below the split, so take everything above it
        ("notch_lo_nx", box((-hw - out, back, md - zf),
                            (-q + xs, lap + EPS, hh + EPS))),
        # D2 keeps what is above
        ("notch_hi_nx", box((-q - xs, back, db - EPS),
                            (xs, lap + EPS, md + zf))),
        # D3 keeps what is below
        ("notch_lo_px", box((-xs, back, md - zf),
                            (q + xs, lap + EPS, hh + EPS))),
        # D4 + R2 keep what is above
        ("notch_hi_px", box((q - xs, back, db - EPS),
                            (hw + out, lap + EPS, md + zf))),

        # Detent grooves, the mirror of a rib grown by clearance. They go in the
        # +x rail, the tall one: this piece keeps it above the split, so there
        # are four millimetres of material to sink a groove into. The mate's
        # ribs arrive here. Our own ribs are on the -x rail — see `additions`.
        *[(f"groove_px_{index}", prism_yz(
            _detent_polygon(config, -offset, md + zf, +1, depth, grow,
                            mirror=True),
            ri + clear - grow, hw + out))
          for index, offset in enumerate(connector.detent_offsets)],
    ]


def validate(config: TrackConfig = DEFAULT) -> None:
    """§6.4's assertions, in terms of the geometry they protect."""
    body, connector = config.body, config.connector
    connector.validate()

    clear = connector.fit_clearance
    offsets = connector.detent_offsets
    lead = (connector.detent_height
            / math.tan(math.radians(connector.detent_lead_angle_deg)))
    depth = groove_depth(config)
    groove_lead = depth / math.tan(
        math.radians(connector.detent_lead_angle_deg)) + clear / 2.0
    groove_back = depth / math.tan(
        math.radians(connector.detent_return_angle_deg)) + clear / 2.0
    rib_back = (connector.detent_height
                / math.tan(math.radians(connector.detent_return_angle_deg)))

    if max(offsets) + lead >= connector.lap_length:
        raise ValueError("detent rib would overhang the end of the tab")
    if max(offsets) + groove_lead >= connector.lap_length + clear:
        raise ValueError("detent groove would run past the back of the notch")
    if groove_back >= min(offsets) - rib_back:
        raise ValueError("detent rib and groove would run into each other")
    if len(offsets) > 1 and min(
            b - a for a, b in zip(offsets, offsets[1:])) <= lead + rib_back:
        raise ValueError(
            "a rail's two detents would run into each other; "
            "detent_spacing is too small for this detent_height"
        )
    if depth <= 0.0:
        raise ValueError(
            "detent is shallower than half a clearance; there would be nothing "
            "for a groove to hold on to"
        )
    if body.deck_mid + clear / 2.0 + depth >= body.deck_top:
        raise ValueError(
            "the detent groove would break out above the deck surface, where "
            "the rail's inner face stands exposed; on a curve the body wanders "
            "further than the groove clears that face and the two graze"
        )
    if clear >= body.rail_inner:
        raise ValueError("centreline slot is wider than the deck")
    if clear / 2.0 >= body.deck_lamina:
        raise ValueError(
            "the lap plane's clearance is as thick as the deck lamina it "
            "splits; the deck would part instead of lapping"
        )
    if deck_column(config) <= 2.0 * clear:
        raise ValueError(
            f"a deck column is {deck_column(config):.3f} mm wide; the seams "
            f"either side of it would meet and it would carry no lap"
        )
    if connector.lap_length + clear >= body.width_outer:
        raise ValueError("lap is longer than the piece is wide; joints would "
                         "overlap on a short part")


# --------------------------------------------------------------------------
# port frames
# --------------------------------------------------------------------------


def port_matrix(origin, forward, up) -> np.ndarray:
    """A port's local-to-world transform.

    ``forward`` points out of the piece. The frame is right-handed like
    (X, Y, Z), so `across = forward × up`.
    """
    forward = np.asarray(forward, dtype=np.float64)
    forward = forward / np.linalg.norm(forward)
    up = np.asarray(up, dtype=np.float64)
    up = up - np.dot(up, forward) * forward
    up = up / np.linalg.norm(up)
    across = np.cross(forward, up)

    matrix = np.eye(4)
    matrix[:3, 0] = across
    matrix[:3, 1] = forward
    matrix[:3, 2] = up
    matrix[:3, 3] = np.asarray(origin, dtype=np.float64)
    return matrix


MATE = np.array([[-1.0, 0.0, 0.0, 0.0],
                 [0.0, -1.0, 0.0, 0.0],
                 [0.0, 0.0, 1.0, 0.0],
                 [0.0, 0.0, 0.0, 1.0]])
"""Two ports mate when one frame equals the other times this.

A 180° rotation about the shared `up`. That the *same* geometry mates with
itself under it is exactly what "genderless" means (§6.1).
"""


def applied(matrices, config: TrackConfig = DEFAULT
            ) -> tuple[list[MeshData], list[MeshData]]:
    """The connector's cut and addition solids, placed at every port."""
    validate(config)
    placed_cuts, placed_adds = [], []
    for matrix in matrices:
        placed_cuts.extend(mesh.transformed(matrix)
                           for _name, mesh in cuts(config))
        placed_adds.extend(mesh.transformed(matrix)
                           for _name, mesh in additions(config))
    return placed_cuts, placed_adds
