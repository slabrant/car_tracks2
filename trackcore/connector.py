"""The connector. docs/SPEC.md §6.

Pure Python + numpy. Never imports bpy.

The port is the **four-column split** of §6.1. Read across the section there
are four columns — outer rail, deck, deck, outer rail — and each is cut by a
horizontal plane and keeps one side of it, alternating:

    ┌────────┬────────┬────────┬────────┐
    │ notch  │  TAB   │ notch  │  TAB   │   above the split
    ├────────┼────────┼────────┼────────┤   <- split
    │  TAB   │ notch  │  TAB   │ notch  │   below the split
    └────────┴────────┴────────┴────────┘
      -x rail  -x deck  +x deck  +x rail

Four tabs, two reaching over their mate and two reaching under it. Genderless
needs one thing only: that the pattern be **odd in x**, `P(-x, z) = -P(x, z)`,
because two ports meet under `MATE`, a 180° turn about the shared up axis. The
column signs above are odd, so the same part mates with itself.

The split height is **not one plane**. It is mid-height through the rail
columns and mid-deck through the deck columns. A single flat plane at `z = 0`
is what the first U-channel version used, and on a U the deck lies wholly below
mid-height, so that plane never touched the deck at all: the only material
lapping vertically was the two rail laps, a rail thickness wide apiece. Stepping
the split into the deck laps half its thickness across nearly the whole channel
instead — an order of magnitude more area, in both directions. That is what
makes a bridge hold together rather than hinge apart.

The price is a vertical mating face wherever the handedness changes, and four
alternating columns means three of them across the section. Each costs a
clearance-wide slot through the deck, running along the direction of travel for
the length of the lap. There is no way round it, and it is why the split runs
*along* the road rather than across it: a wheel rolls parallel to all three and
never crosses one. The middle slot is the centreline, which was always there.
See `root_inset` for where the other two sit, which is not where you would
first put them.

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

    The four tabs are not here because they are not added — see below.
    """
    body, connector = config.body, config.connector
    hw, ri = body.half_width, body.rail_inner
    clear = connector.fit_clearance
    zf = clear / 2.0          # lap plane offset
    d, dh = connector.detent_offset, connector.detent_height

    return [
        # Only the detent ribs. The **tabs are swept, not added** — every
        # construction builds its body `lap_length` past the nominal port along
        # the end tangent, and the notch cuts below trim that extension down to
        # the four tab columns.
        #
        # The ribs sit on the rail laps, which the four-column split left where
        # they were. They are the joint's *click*; the deck columns are its
        # strength. Putting a detent on the deck laps too is open — it is a lot
        # of area going unused — but a rib on an internal horizontal face is a
        # harder thing to print than one on a rail, and the click already works.
        #
        # They used to be boxes unioned on. At the port plane a box's side faces
        # are exactly coplanar with the body's rail faces, and on a curve they
        # are tangent there and diverge going in — a tangential union, which is
        # the one thing an exact solver cannot resolve cleanly. Whether it
        # survived was luck: a 45° arc built at an 8 mm lap and went
        # non-manifold at 7, while a 90° arc of the same radius was fine at
        # both. Sweeping the tab removes the second solid, so there is nothing
        # to be tangent to.
        ("rib_px", prism_yz(
            _detent_polygon(config, +d, zf, -1, dh, 0.0, mirror=False),
            ri + clear, hw)),
        ("rib_nx", prism_yz(
            _detent_polygon(config, +d, -zf, +1, dh, 0.0, mirror=False),
            -hw, -ri - clear)),
    ]


def root_inset(config: TrackConfig = DEFAULT) -> float:
    """How far inside the rail's inner face the column boundary sits, mm.

    The obvious place to change handedness is exactly at the rail root, x =
    rail_inner, since that is where the deck column stops being deck. It is the
    one place it cannot go. The cut tools are straight boxes in the port frame
    and the body is not: over a lap zone of reach `d` on a radius `R` the
    section wanders sideways by about `d² / 2R`, which is more than the whole
    width of the clearance slot. The rail's inner corner — a concave edge
    running the length of the piece — then grazes the slot's face instead of
    crossing it, and `curve_45` came apart into five non-manifold edges.

    Set the boundary a clear drift inside the deck instead and the slot cuts
    the deck square, top and bottom, whatever the curvature does. Sized for the
    tightest legal radius, so it is curvature-proof rather than lucky.

    The rail column then owns a strip of deck as well as its rail. That costs
    nothing: the strip is below the rail's mid-height split, so it goes whole to
    one piece rather than being lapped, exactly as it did before.
    """
    reach = config.connector.lap_length + config.connector.fit_clearance
    drift = reach * reach / (2.0 * config.min_radius)
    return drift + config.connector.fit_clearance


def tab_area(config: TrackConfig = DEFAULT) -> float:
    """What one port carries across the port plane, mm².

    Five pieces, not four, because the rail columns reach inboard of their
    rails (see `root_inset`) and so one of them takes a strip of deck along
    with it — whole, since that strip lies below the rail column's split. The
    other rail column's strip goes to the mate.

    Ought to come to a little under half the section: half, less what every
    mating face gives up to clearance. `test_a_port_keeps_a_little_under_half`
    checks exactly that, which is what makes this derivation and the section
    check each other rather than merely agree.
    """
    body, connector = config.body, config.connector
    half = connector.fit_clearance / 2.0
    column = body.rail_inner - root_inset(config)
    lamina = body.deck_lamina - half

    rails = 2.0 * body.rail_thickness * (body.half_height - half)
    decks = 2.0 * (column - 2.0 * half) * lamina
    strip = (body.rail_inner - column - half) * body.deck_thickness
    return rails + decks + strip


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


def cuts(config: TrackConfig = DEFAULT) -> list[tuple[str, MeshData]]:
    """Material removed behind the port plane: notches, slot and grooves."""
    body, connector = config.body, config.connector
    hw, ri = body.half_width, body.rail_inner
    hh = body.half_height
    lap, clear = connector.lap_length, connector.fit_clearance
    zf = clear / 2.0
    xs = clear / 2.0
    back = -(lap + clear)     # notches are cut one clearance deeper than the tab
    d = connector.detent_offset
    depth = connector.detent_height + clear / 2.0
    grow = clear / 2.0
    out = outer_margin(config)

    md = body.deck_mid
    db, dt = body.deck_bottom, body.deck_top
    col = ri - root_inset(config)      # deck | rail column boundary

    return [
        # -- rail columns, split at mid-height ---------------------------------
        # +x keeps what is above the lap plane, -x what is below. The column
        # reaches inboard of the rail itself, so it takes the strip of deck
        # between `col` and the rail root with it — see `root_inset`.
        #
        # It starts at `col - xs`, reaching right across the boundary slot
        # below rather than stopping against it. Everything it takes in there
        # the slot takes too, so the solid is the same either way — but tools
        # that stop flush against each other share a face, and §7a is explicit
        # that coincident faces are to be replaced by overlap wherever there is
        # a choice. Stopped flush, this pair left a zero-thickness sheet 6 mm
        # long standing in the middle of `curve_45`.
        ("notch_rail_px", box((col - xs, back, -hh - EPS),
                              (hw + out, lap + EPS, zf))),
        ("notch_rail_nx", box((-hw - out, back, -zf),
                              (-col + xs, lap + EPS, hh + EPS))),

        # -- deck columns, split at mid-deck, handedness reversed --------------
        # Reversed so each *side* of the track carries one tab over and one
        # under. Run them the same way as the rails and both of the joint's
        # upward-facing laps end up on the same side, which resists lift on one
        # rail and lets the other hinge.
        #
        # Each reaches one slot half-width past the centreline, so over
        # |x| < xs the two overlap and between them take every z in the deck.
        # That *is* the centreline slot of §6.2 — there is no separate tool for
        # it. An earlier version had one, and being wholly contained in the
        # union of these two it contributed nothing but coplanar faces; on the
        # Y, whose port planes sit at irrational angles, the solver turned them
        # into six degenerate triangles.
        ("notch_deck_px", box((-xs, back, md - zf),
                              (col + xs, lap + EPS, dt + EPS))),
        ("notch_deck_nx", box((-col - xs, back, db - EPS),
                              (xs, lap + EPS, md + zf))),

        # -- the column boundaries ---------------------------------------------
        # Handedness flips here, so the material on either side belongs to
        # different pieces and needs clearance between them. Only through the
        # deck: above it, inboard of the rails, there is nothing to cut.
        ("boundary_px", box((col - xs, back, db - EPS),
                            (col + xs, lap + EPS, dt + EPS))),
        ("boundary_nx", box((-col - xs, back, db - EPS),
                            (-col + xs, lap + EPS, dt + EPS))),
        # detent grooves, the mirror of a rib grown by clearance
        ("groove_px", prism_yz(
            _detent_polygon(config, -d, zf, +1, depth, grow, mirror=True),
            ri + clear - grow, hw + out)),
        ("groove_nx", prism_yz(
            _detent_polygon(config, -d, -zf, -1, depth, grow, mirror=True),
            -hw - out, -ri - clear + grow)),
    ]


def validate(config: TrackConfig = DEFAULT) -> None:
    """§6.4's assertions, in terms of the geometry they protect."""
    body, connector = config.body, config.connector
    connector.validate()

    clear = connector.fit_clearance
    lead = (connector.detent_height
            / math.tan(math.radians(connector.detent_lead_angle_deg)))
    depth = connector.detent_height + clear / 2.0
    groove_lead = depth / math.tan(
        math.radians(connector.detent_lead_angle_deg)) + clear / 2.0
    groove_back = depth / math.tan(
        math.radians(connector.detent_return_angle_deg)) + clear / 2.0
    rib_back = (connector.detent_height
                / math.tan(math.radians(connector.detent_return_angle_deg)))

    if connector.detent_offset + lead >= connector.lap_length:
        raise ValueError("detent rib would overhang the end of the tab")
    if connector.detent_offset + groove_lead >= connector.lap_length + clear:
        raise ValueError("detent groove would run past the back of the notch")
    if groove_back >= connector.detent_offset - rib_back:
        raise ValueError("detent rib and groove would run into each other")
    if clear / 2.0 + depth >= body.half_height:
        raise ValueError("detent groove would cut through the rail")
    if clear >= body.rail_inner:
        raise ValueError("centreline slot is wider than the deck")
    if clear / 2.0 >= body.deck_lamina:
        raise ValueError(
            "the lap plane's clearance is as thick as the deck lamina it "
            "splits; the deck would part instead of lapping"
        )
    column = body.rail_inner - root_inset(config)
    if column <= 2.0 * clear:
        raise ValueError(
            f"the deck column is {column:.3f} mm wide; the centreline slot and "
            f"the column boundary would meet and the deck would carry no lap"
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
