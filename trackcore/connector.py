"""The connector. docs/SPEC.md §6.

Pure Python + numpy. Never imports bpy.

The port is the full diagonal split of §6.1: the whole cross-section is cut at
x = 0 and z = 0, and the piece keeps the (+x, +z) and (−x, −z) quadrants, which
run on past the port plane as tabs. That pattern is the minimal solution to
"genderless and flippable", and §6.1 proves nothing confined to one horizontal
slab can work at all.

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

TAB_ROOT_OVERLAP = 1.0
"""How far a tab reaches back **into** the body before the port plane, mm.

A tab's underside is coplanar with the face the notch cut has just created, so
the two interlock only through the volume where they overlap. At EPS that volume
is 0.01 mm deep — technically an overlap, practically at the solver's tolerance.
It survived on straights and 90° arcs and failed on a 45° arc the moment the lap
was shortened, which is the signature of a tolerance-edge degeneracy rather than
a real geometric conflict.

Reaching a whole millimetre back costs nothing: that region is body material the
notch never removes, so the tab only duplicates what is already there."""

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
    """Material added beyond the port plane: the two tabs and their ribs."""
    body, connector = config.body, config.connector
    hw, ri = body.half_width, body.rail_inner
    hh = body.half_height
    lap, clear = connector.lap_length, connector.fit_clearance
    zf = clear / 2.0          # lap plane offset
    xs = clear / 2.0          # centreline slot half-width
    d, dh = connector.detent_offset, connector.detent_height
    root = min(TAB_ROOT_OVERLAP, connector.lap_length / 2.0)

    return [
        # The (+x, +z) and (-x, -z) quadrants, running past the port plane. On
        # a U-channel the deck lies wholly below the split plane, so it belongs
        # entirely to the lower quadrant and is never cut in z — no thin
        # tongues. The price is an asymmetric pair: the upper tab is rail only,
        # the lower tab is rail plus half the deck. Both ports carry one of
        # each, so the two mating pieces are still balanced.
        ("tab_rail_px", box((ri, -root, zf), (hw, lap, hh))),
        ("tab_rail_nx", box((-hw, -root, -hh), (-ri, lap, -zf))),
        ("tab_deck_nx", box((-ri - EPS, -root, body.deck_bottom),
                            (-xs, lap, body.deck_top))),
        # detent ribs, inset from the rail inner face so they do not rub along
        # the mating piece's deck edge
        ("rib_px", prism_yz(
            _detent_polygon(config, +d, zf, -1, dh, 0.0, mirror=False),
            ri + clear, hw)),
        ("rib_nx", prism_yz(
            _detent_polygon(config, +d, -zf, +1, dh, 0.0, mirror=False),
            -hw, -ri - clear)),
    ]


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

    return [
        # The two notch quadrants, each reaching one slot half-width *past* the
        # centreline. Over |x| < xs they therefore overlap and between them
        # remove every z, which is exactly the centreline slot of §6.2 — so
        # there is no separate slot tool. An earlier version had one, and being
        # wholly contained in the union of these two it contributed nothing but
        # coplanar faces; on the Y, whose port planes sit at irrational angles,
        # the solver turned those into six degenerate triangles.
        ("notch_px", box((-xs, back, -hh - EPS), (hw + out, EPS, zf))),
        ("notch_nx", box((-hw - out, back, -zf), (xs, EPS, hh + EPS))),
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
