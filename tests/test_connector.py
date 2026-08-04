"""docs/SPEC.md §9.18–9.23: the connector.

These are the tests the earlier attempts did not have. Gendered connectors and
joints that did not mate would all have failed here in under a second.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from parts import CATALOGUE, HUBS, PATHS, build, straight, x_junction
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


def test_the_port_pattern_is_odd_in_both_x_and_z():
    """§6.1. P must be odd in x and odd in z; the minimal solution is
    diagonally opposed tabs. Anything uniform in z forces P identically zero."""
    flip = _flip_about(math.pi / 2.0)  # (x, y, z) -> (-x, y, -z)
    for group in (cuts(DEFAULT), additions(DEFAULT)):
        meshes = [mesh for _label, mesh in group]
        assert _pool(meshes) == _pool([m.transformed(flip) for m in meshes]), (
            "the port geometry is not invariant under (x, z) -> (-x, -z)"
        )


def test_the_tabs_occupy_diagonally_opposite_quadrants():
    by_name = dict(additions(DEFAULT))
    for name in ("tab_rail_px", "tab_deck_px"):
        lo, hi = by_name[name].bounds()
        assert lo[0] > 0 and lo[2] > 0, f"{name} should be the (+x, +z) quadrant"
    for name in ("tab_rail_nx", "tab_deck_nx"):
        lo, hi = by_name[name].bounds()
        assert hi[0] < 0 and hi[2] < 0, f"{name} should be the (-x, -z) quadrant"


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


# -- 9.18 --------------------------------------------------------------------


def test_a_straight_is_congruent_to_itself_flipped():
    """Rotate the whole piece 180° about its long axis. A gendered connector,
    or one asymmetric in z, fails here."""
    piece = build("straight_full", DEFAULT, length=84.0)
    flip = _flip_about(math.pi / 2.0)
    for group in (piece.solids, piece.cuts, piece.additions):
        assert _pool(group) == _pool([m.transformed(flip) for m in group])


@pytest.mark.parametrize("name", sorted(HUBS))
def test_every_junction_is_congruent_to_itself_flipped(name):
    """§5.4: flipping acts as a mirror in plan, so the layout's mirror axis is
    the flip axis."""
    hub = HUBS[name]()
    axis = hub.mirror_axis()
    assert axis is not None
    piece = build(name, DEFAULT)
    flip = _flip_about(axis)
    for group in (piece.solids, piece.cuts, piece.additions):
        assert _pool(group) == _pool([m.transformed(flip) for m in group]), (
            f"{name} is not congruent under a flip about its mirror axis"
        )


# -- 9.19 --------------------------------------------------------------------


def _boxes(group, skip=("rib_",)):
    return [(label, *mesh.bounds()) for label, mesh in group
            if not any(label.startswith(s) for s in skip)]


def _overlap(a_lo, a_hi, b_lo, b_hi) -> float:
    return min(min(a_hi[i], b_hi[i]) - max(a_lo[i], b_lo[i]) for i in range(3))


def test_two_ports_mate_without_sharing_any_volume():
    """A port mates with a copy of itself rotated 180° about the shared up.
    That the *same* geometry mates with itself is what genderless means."""
    ours = _boxes(additions(DEFAULT))
    theirs = [(f"mate:{label}", *mesh.transformed(MATE).bounds())
              for label, mesh in additions(DEFAULT)
              if not label.startswith("rib_")]

    for a_label, a_lo, a_hi in ours:
        for b_label, b_lo, b_hi in theirs:
            gap = _overlap(a_lo, a_hi, b_lo, b_hi)
            assert gap <= TOL, f"{a_label} interpenetrates {b_label} by {gap:.4f} mm"


def test_the_lap_faces_clear_each_other_by_exactly_one_clearance():
    by_name = dict(additions(DEFAULT))
    ours = by_name["tab_rail_px"].bounds()[0][2]                 # we keep upper
    theirs = by_name["tab_rail_nx"].transformed(MATE).bounds()[1][2]
    assert ours - theirs == pytest.approx(CONN.fit_clearance, abs=1e-12)


def test_the_centreline_halves_clear_each_other_by_exactly_one_clearance():
    by_name = dict(additions(DEFAULT))
    ours = by_name["tab_deck_px"].bounds()[0][0]
    theirs = by_name["tab_deck_px"].transformed(MATE).bounds()[1][0]
    assert ours - theirs == pytest.approx(CONN.fit_clearance, abs=1e-12)


def test_every_tab_lands_inside_the_mating_notch():
    notches = [(label, *mesh.transformed(MATE).bounds())
               for label, mesh in cuts(DEFAULT) if label.startswith("notch")]
    for label, mesh in additions(DEFAULT):
        if not label.startswith("tab"):
            continue
        lo, hi = mesh.bounds()
        assert any(all(n_lo[i] <= lo[i] + TOL and hi[i] <= n_hi[i] + TOL
                       for i in range(3))
                   for _n, n_lo, n_hi in notches), f"{label} is not contained"


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

    matrices = (port_matrices(PATHS[name]()) if name in PATHS
                else HUBS[name]().port_matrices())
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
        connector.validate(TrackConfig(connector=Connector(detent_offset=9.0)))
    # tall enough to break through the rail, but not so tall that the lead-in
    # runs off the tab first — otherwise the earlier guard fires instead
    with pytest.raises(ValueError, match="through the rail"):
        connector.validate(TrackConfig(connector=Connector(detent_height=2.25)))
