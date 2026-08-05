"""docs/SPEC.md §9.18–9.23: the connector.

These are the tests the earlier attempts did not have. Gendered connectors and
joints that did not mate would all have failed here in under a second.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from parts import CATALOGUE, HUBS, PATHS, build, port_frames, straight
from trackcore import DEFAULT, MATE, Arc, Path, connector, port_matrices
from trackcore.connector import additions, cuts
from trackcore.hub import direction

BODY = DEFAULT.body
CONN = DEFAULT.connector
TOL = 1e-9

CUTS_PER_PORT = len(cuts(DEFAULT))
ADDS_PER_PORT = len(additions(DEFAULT))


def _pool(meshes) -> set[tuple[float, float, float]]:
    return {tuple(np.round(v, 7)) for mesh in meshes for v in mesh.verts}


def _flip_about(angle: float) -> np.ndarray:
    """180° rotation about the horizontal axis at ``angle``. Flipping a piece."""
    n = np.array([math.cos(angle), math.sin(angle), 0.0])
    matrix = np.eye(4)
    matrix[:3, :3] = 2.0 * np.outer(n, n) - np.eye(3)
    return matrix


# -- the tools themselves ----------------------------------------------------


def test_every_connector_tool_is_a_valid_solid():
    from trackcore import check
    for label, mesh in cuts(DEFAULT) + additions(DEFAULT):
        check(mesh, name=label)


def _notches():
    return {label: mesh.bounds() for label, mesh in cuts(DEFAULT)
            if label.startswith("notch")}


def test_the_joint_cannot_be_lifted_apart():
    """The Z-lock, and the most important test in this file.

    On the old flippable section this was a *corollary*: flip symmetry forced P
    odd in z, and odd-in-z is what puts our material above the mate's on one
    rail and below it on the other. Flippability is gone, so nothing forces it.
    Genderless alone forces P odd in **x**, and a plain vertical split satisfies
    that while lifting straight apart with no resistance at all.

    Since the tabs are swept rather than added (§6.6), the tab shape lives in
    the *complement* of the notches, so that is what this checks: one notch must
    remove the material **below** the split plane and the other the material
    **above** it. A notch spanning the full height would mean a split uniform in
    z — still perfectly genderless, and it would come apart upward.
    """
    (px_lo, px_hi), (nx_lo, nx_hi) = _notches()["notch_px"], _notches()["notch_nx"]
    half = BODY.half_height

    assert px_lo[2] < -half, "notch_px must clear the bottom of the section"
    assert 0.0 < px_hi[2] < half, "notch_px must stop above the split plane"
    assert nx_hi[2] > half, "notch_nx must clear the top of the section"
    assert -half < nx_lo[2] < 0.0, "notch_nx must stop below the split plane"


def test_the_tabs_occupy_diagonally_opposite_quadrants():
    """What survives the notches is the (+x, +z) and (-x, -z) quadrants."""
    (px_lo, _px_hi), (_nx_lo, nx_hi) = (_notches()["notch_px"],
                                        _notches()["notch_nx"])
    half = BODY.half_width
    assert px_lo[0] < 0.0, "notch_px must reach past the centreline"
    assert nx_hi[0] > 0.0, "notch_nx must reach past the centreline"
    assert _notches()["notch_px"][1][0] > half, "notch_px must clear the +x rail"
    assert _notches()["notch_nx"][0][0] < -half, "notch_nx must clear the -x rail"


def test_nothing_is_glued_on_but_the_detents():
    """§6.6. Tabs are swept; only the ribs are added solids."""
    assert {label for label, _ in additions(DEFAULT)} == {"rib_px", "rib_nx"}


def test_the_notches_reach_past_the_port_plane_to_trim_the_extension():
    """The body is swept `lap_length` long; these cuts are what shape the tab."""
    for label in ("notch_px", "notch_nx"):
        assert _notches()[label][1][1] >= CONN.lap_length


def test_tabs_protrude_exactly_one_lap_length():
    for _name, mesh in additions(DEFAULT):
        assert mesh.bounds()[1][1] <= CONN.lap_length + TOL


def test_notches_are_cut_one_clearance_deeper_than_the_tab():
    """So a tab never bottoms out before the joint closes, §6.2."""
    by_name = dict(cuts(DEFAULT))
    for name in ("notch_px", "notch_nx"):
        assert by_name[name].bounds()[0][1] == pytest.approx(
            -(CONN.lap_length + CONN.fit_clearance), abs=TOL)


def test_the_two_notches_between_them_cut_the_centreline_slot():
    """§6.2. They overlap across |x| < clearance/2 and remove every z there, so
    there is no separate slot tool to contribute coplanar faces."""
    by_name = dict(cuts(DEFAULT))
    px_lo, px_hi = by_name["notch_px"].bounds()
    nx_lo, nx_hi = by_name["notch_nx"].bounds()

    half = CONN.fit_clearance / 2.0
    assert px_lo[0] == pytest.approx(-half, abs=TOL)
    assert nx_hi[0] == pytest.approx(+half, abs=TOL)
    # and together they span the full height across that strip
    assert px_lo[2] < -BODY.half_height and nx_hi[2] > BODY.half_height
    assert px_hi[2] >= nx_lo[2], "the notches must overlap in z, not merely touch"


# -- 9.19 --------------------------------------------------------------------


def _boxes(group, skip=("rib_",)):
    return [(label, *mesh.bounds()) for label, mesh in group
            if not any(label.startswith(s) for s in skip)]


def _overlap(a_lo, a_hi, b_lo, b_hi) -> float:
    return min(min(a_hi[i], b_hi[i]) - max(a_lo[i], b_lo[i]) for i in range(3))


def test_the_two_notches_are_point_reflections_of_each_other():
    """This *is* the diagonal split, stated as a symmetry.

    `P` odd in x and odd in z together mean `P(-x, -z) = P(x, z)`, so reflecting
    the port face through its own centre maps notch to notch. A vertical split
    would be odd in x alone and would fail here.
    """
    px_lo, px_hi = _notches()["notch_px"]
    nx_lo, nx_hi = _notches()["notch_nx"]
    for axis in (0, 2):                       # x and z
        assert -px_hi[axis] == pytest.approx(nx_lo[axis], abs=1e-9)
        assert -px_lo[axis] == pytest.approx(nx_hi[axis], abs=1e-9)
    # y is deliberately *not* symmetric: a notch is cut one clearance deeper
    # than the tab is long, so a tab never bottoms out before the joint closes
    assert px_lo[1] == pytest.approx(nx_lo[1], abs=1e-12)
    assert px_hi[1] - px_lo[1] > CONN.lap_length


def test_the_lap_faces_clear_each_other_by_exactly_one_clearance():
    """We keep the upper half above +clearance/2, the mate the lower half below
    -clearance/2, so the lap faces are exactly one clearance apart."""
    ours = _notches()["notch_px"][1][2]
    theirs = _notches()["notch_nx"][0][2]
    assert ours - theirs == pytest.approx(CONN.fit_clearance, abs=1e-12)


def test_the_centreline_halves_clear_each_other_by_exactly_one_clearance():
    """Each notch reaches half a clearance past x = 0, so the two surviving
    halves are one clearance apart — the slot down the deck."""
    ours = _notches()["notch_nx"][1][0]
    theirs = _notches()["notch_px"][0][0]
    assert ours - theirs == pytest.approx(CONN.fit_clearance, abs=1e-12)


def test_the_notch_is_cut_deeper_than_the_tab_is_long():
    """So a tab never bottoms out before the joint closes, §6.2."""
    for label in ("notch_px", "notch_nx"):
        depth = -_notches()[label][0][1]
        assert depth == pytest.approx(CONN.lap_length + CONN.fit_clearance,
                                      abs=1e-12)


# -- 9.20 --------------------------------------------------------------------


def _yz(mesh):
    n = len(mesh.verts) // 2
    return [(float(v[1]), float(v[2])) for v in mesh.verts[:n]]


def _span_at_z(poly, z: float):
    hits = []
    for i in range(len(poly)):
        y1, z1 = poly[i]
        y2, z2 = poly[(i + 1) % len(poly)]
        if abs(z1 - z2) < 1e-15:
            if abs(z1 - z) < 1e-12:
                hits.extend([y1, y2])
            continue
        if (z1 - z) * (z2 - z) <= 0:
            hits.append(y1 + (z - z1) / (z2 - z1) * (y2 - y1))
    return (min(hits), max(hits)) if hits else None


def test_the_rib_seats_inside_the_partner_groove():
    by_add = dict(additions(DEFAULT))
    by_cut = dict(cuts(DEFAULT))
    rib = _yz(by_add["rib_px"])
    groove = [(-y, z) for (y, z) in _yz(by_cut["groove_nx"])]

    face = -CONN.fit_clearance / 2.0
    apex = CONN.fit_clearance / 2.0 - CONN.detent_height
    assert apex < face, "the rib must reach past the partner's lap face"

    for z in np.linspace(face, apex, 12)[1:]:
        rib_span = _span_at_z(rib, float(z))
        groove_span = _span_at_z(groove, float(z))
        assert groove_span is not None
        if rib_span is None:
            continue
        assert groove_span[0] <= rib_span[0] + TOL
        assert groove_span[1] >= rib_span[1] - TOL


def test_no_rib_meets_a_rib():
    """The longitudinal offset in §6.3 exists precisely to prevent this."""
    by_name = dict(additions(DEFAULT))
    ours = by_name["rib_px"].bounds()
    theirs = by_name["rib_nx"].transformed(MATE).bounds()
    assert _overlap(*ours, *theirs) <= TOL


def test_the_return_face_is_steeper_than_the_lead_in():
    """Easy to push together, hard to pull apart, §6.3."""
    rib = _yz(dict(additions(DEFAULT))["rib_px"])
    apex = min(rib, key=lambda p: p[1])
    base = [p for p in rib if p is not apex]
    lead = max(base, key=lambda p: p[0])[0] - apex[0]
    back = apex[0] - min(base, key=lambda p: p[0])[0]
    assert lead > back


def test_the_detent_base_is_buried_well_behind_the_lap_face():
    """Sunk only a hair, the flanks cross the lap plane beside the base corners
    and the boolean turns that into slivers on irrational port angles."""
    rib = _yz(dict(additions(DEFAULT))["rib_px"])
    base_z = max(z for _y, z in rib)
    assert base_z - CONN.fit_clearance / 2.0 >= 0.1


# -- 9.21 --------------------------------------------------------------------


@pytest.mark.parametrize("name", CATALOGUE)
def test_every_port_of_every_part_carries_identical_geometry(name):
    """Transformed back into its own frame, every port is the same object."""
    piece = build(name, DEFAULT)
    canonical_cuts = _pool([m for _l, m in cuts(DEFAULT)])
    canonical_adds = _pool([m for _l, m in additions(DEFAULT)])

    matrices = port_frames(name)
    assert len(piece.cuts) == CUTS_PER_PORT * len(matrices)
    assert len(piece.additions) == ADDS_PER_PORT * len(matrices)

    for index, matrix in enumerate(matrices):
        inverse = np.linalg.inv(matrix)
        got_cuts = piece.cuts[index * CUTS_PER_PORT:(index + 1) * CUTS_PER_PORT]
        got_adds = piece.additions[index * ADDS_PER_PORT:(index + 1) * ADDS_PER_PORT]
        assert _pool([m.transformed(inverse) for m in got_cuts]) == canonical_cuts
        assert _pool([m.transformed(inverse) for m in got_adds]) == canonical_adds


def test_a_curve_and_a_junction_present_the_same_port():
    curve_piece = build("curve_90", DEFAULT)
    hub_piece = build("x_junction", DEFAULT)
    curve_frames = port_matrices(PATHS["curve_90"]())
    hub_frames = HUBS["x_junction"]().port_matrices()

    a = _pool([m.transformed(np.linalg.inv(curve_frames[0]))
               for m in curve_piece.additions[:ADDS_PER_PORT]])
    b = _pool([m.transformed(np.linalg.inv(hub_frames[0]))
               for m in hub_piece.additions[:ADDS_PER_PORT]])
    assert a == b


# -- 9.22 --------------------------------------------------------------------


def test_eight_forty_five_degree_arcs_close_a_loop():
    loop = Path.chain(*[Arc(radius=100.0, angle=math.radians(45.0))] * 8)
    assert loop.end_transform == pytest.approx(np.eye(4), abs=1e-9)
    assert loop.point(loop.length) == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)
    assert loop.tangent(loop.length) == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)


def test_a_rectangle_of_straights_and_curves_closes():
    quarter = Arc(radius=60.0, angle=math.radians(90.0))
    loop = Path.chain(*[x for _ in range(4)
                        for x in (__import__("trackcore").Line(70.0), quarter)])
    assert loop.end_transform == pytest.approx(np.eye(4), abs=1e-9)


def test_two_pieces_laid_end_to_end_present_mating_frames():
    """A's far port and B's near port must be related by MATE."""
    a = port_matrices(straight(84.0))
    placed = straight(84.0)
    b_world = Path.chain(*placed.primitives).end_transform
    b_near = b_world @ port_matrices(straight(42.0))[0]
    assert b_near == pytest.approx(a[1] @ MATE, abs=1e-9)


# -- 9.23 --------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(HUBS))
def test_a_straight_attaches_squarely_to_every_junction_port(name):
    hub = HUBS[name]()
    for index, matrix in enumerate(hub.port_matrices()):
        # a piece attached here sits at port_matrix @ MATE @ inverse(its near port)
        near = port_matrices(straight(84.0))[0]
        world = matrix @ MATE @ np.linalg.inv(near)

        origin = world[:3, 3]
        heading = world[:3, :3] @ np.array([0.0, 1.0, 0.0])
        arm = hub.arms[index]
        expected = arm.port_distance * direction(arm.angle)

        assert origin[:2] == pytest.approx(expected, abs=1e-9)
        assert origin[2] == pytest.approx(0.0, abs=1e-9)
        assert heading[:2] == pytest.approx(direction(arm.angle), abs=1e-9)


def test_opposite_arms_of_a_t_carry_collinear_track():
    hub = HUBS["t_junction"]()
    headings = []
    for matrix in hub.port_matrices():
        world = matrix @ MATE @ np.linalg.inv(port_matrices(straight(84.0))[0])
        headings.append(world[:3, :3] @ np.array([0.0, 1.0, 0.0]))
    dots = [abs(float(np.dot(a, b))) for i, a in enumerate(headings)
            for b in headings[i + 1:]]
    assert pytest.approx(1.0, abs=1e-9) in dots, "a T has one straight-through pair"


# -- guards ------------------------------------------------------------------


def test_a_piece_too_short_for_two_joints_is_rejected():
    with pytest.raises(ValueError, match="notches would meet"):
        build("straight_full", DEFAULT, length=12.0)


def test_connector_config_guards_fire():
    from trackcore.config import Connector, TrackConfig
    with pytest.raises(ValueError, match="overhang"):
        connector.validate(TrackConfig(connector=Connector(
            detent_offset=CONN.lap_length + 1.0)))
    # a detent taller than the rail can hold must be refused; which guard names
    # it depends on the lap, so this asserts only that one of them does
    with pytest.raises(ValueError):
        connector.validate(TrackConfig(connector=Connector(detent_height=2.25)))
