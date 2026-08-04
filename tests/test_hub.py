"""docs/SPEC.md §9.12–9.17: Construction B, junctions.

Everything here is headless. The one thing that cannot be is the prism union
itself, which lives in `tests/test_blender.py`.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from parts import straight, t_junction, x_junction, y_junction
from trackcore import DEFAULT, Arm, Hub, HubInvalid, check, profile_area, sweep
from trackcore.hub import direction, left_normal, offset_polyline, polygon_area
from trackcore.mesh import shoelace

BODY = DEFAULT.body
TOL = 1e-9

LAYOUTS = {"x": x_junction, "t": t_junction, "y": y_junction}


def _pairs(points):
    return [(float(p[0]), float(p[1])) for p in points]


# -- 9.12 --------------------------------------------------------------------


@pytest.mark.parametrize("name,builder", sorted(LAYOUTS.items()))
@pytest.mark.parametrize("radius", [0.0, 12.0])
def test_a_hub_has_one_chain_and_one_port_face_per_arm(name, builder, radius):
    hub = builder(radius)
    n = len(hub.arms)
    assert len([hub.chain(i) for i in range(n)]) == n
    assert len(hub.rail_regions()) == n
    assert len([hub.port_edges(i) for i in range(n)]) == n


@pytest.mark.parametrize("name,builder", sorted(LAYOUTS.items()))
@pytest.mark.parametrize("radius", [0.0, 12.0])
def test_the_outline_is_a_simple_ccw_polygon(name, builder, radius):
    outline = _pairs(builder(radius).outline())
    assert shoelace(outline) > 0, "outline must be CCW"
    assert len(outline) >= 3


@pytest.mark.parametrize("name,builder", sorted(LAYOUTS.items()))
def test_port_faces_are_one_track_width_across(name, builder):
    hub = builder(0.0)
    for i in range(len(hub.arms)):
        right, left = hub.port_edges(i)
        assert float(np.linalg.norm(left - right)) == pytest.approx(
            BODY.width_outer, abs=TOL)


# -- 9.13 --------------------------------------------------------------------


def test_a_square_x_has_armpits_at_the_analytic_intersections():
    hub = x_junction(0.0)
    expected = {(+BODY.half_width, +BODY.half_width),
                (-BODY.half_width, +BODY.half_width),
                (-BODY.half_width, -BODY.half_width),
                (+BODY.half_width, -BODY.half_width)}
    found = set()
    for i in range(4):
        point = hub.armpit(i, (i + 1) % 4)
        assert point is not None
        found.add((round(float(point[0]), 9), round(float(point[1]), 9)))
    assert found == {(round(x, 9), round(y, 9)) for x, y in expected}


def test_opposite_arms_have_no_armpit_because_their_edges_are_collinear():
    """The 180° case: a straight-through pair, or the back of a T."""
    hub = t_junction(0.0)
    gaps = [round(math.degrees(g)) for g in hub.gaps()]
    assert 180 in gaps
    straight_through = gaps.index(180)
    j = (straight_through + 1) % len(hub.arms)
    assert hub.armpit(straight_through, j) is None
    assert len(hub.chain(straight_through)) == 2


# -- 9.14 --------------------------------------------------------------------


@pytest.mark.parametrize("name,builder", sorted(LAYOUTS.items()))
@pytest.mark.parametrize("radius", [6.0, 12.0, 25.0])
def test_every_fillet_arc_is_tangent_to_both_adjacent_edges(name, builder, radius):
    hub = builder(radius)
    n = len(hub.arms)
    for i in range(n):
        j = (i + 1) % n
        if hub.armpit(i, j) is None:
            continue
        chain = hub.chain(i)
        arc = chain[1:-1]
        assert len(arc) >= 2, "a filleted corner should sample an arc"

        # the arc's first and last points lie on the two arm edges
        on_i = float(np.dot(arc[0], left_normal(hub.arms[i].angle)))
        on_j = float(np.dot(arc[-1], left_normal(hub.arms[j].angle)))
        assert on_i == pytest.approx(BODY.half_width, abs=1e-9)
        assert on_j == pytest.approx(-BODY.half_width, abs=1e-9)

        # and every arc point is exactly `radius` from a common centre
        centre = _fit_centre(arc)
        radii = [float(np.linalg.norm(p - centre)) for p in arc]
        assert max(radii) - min(radii) < 1e-9
        assert radii[0] == pytest.approx(radius, abs=1e-9)

        # tangency, exactly: the radius to each end point is perpendicular to
        # that arm's direction. Comparing a sampled chord to the tangent would
        # only ever be right to half a step angle.
        assert float(np.dot(arc[0] - centre, direction(hub.arms[i].angle))
                     ) == pytest.approx(0.0, abs=1e-9)
        assert float(np.dot(arc[-1] - centre, direction(hub.arms[j].angle))
                     ) == pytest.approx(0.0, abs=1e-9)


def _fit_centre(arc) -> np.ndarray:
    """Circumcentre of the first, middle and last arc samples."""
    a, b, c = arc[0], arc[len(arc) // 2], arc[-1]
    matrix = np.array([[2 * (b[0] - a[0]), 2 * (b[1] - a[1])],
                       [2 * (c[0] - a[0]), 2 * (c[1] - a[1])]])
    rhs = np.array([b @ b - a @ a, c @ c - a @ a])
    return np.linalg.solve(matrix, rhs)


def test_a_bigger_fillet_pushes_the_ports_further_out():
    """True of Hub.auto, which derives the reach. The catalogue instead pins
    port distance to the layout grid (Hub.uniform), and `validate` rejects a
    fillet too big for it rather than silently growing the piece."""
    small = Hub.auto([0.0, 90.0, 180.0, 270.0], 6.0).arms[0].port_distance
    large = Hub.auto([0.0, 90.0, 180.0, 270.0], 25.0).arms[0].port_distance
    assert large > small


# -- 9.15 --------------------------------------------------------------------


def test_a_straight_built_as_a_hub_agrees_with_the_swept_one():
    """Two arms at 0° and 180° is a degenerate hub, and it had better be the
    same solid as Construction A's straight."""
    hub = Hub.auto([0.0, 180.0])
    length = hub.arms[0].port_distance + hub.arms[1].port_distance

    assert hub.expected_volume() == pytest.approx(
        profile_area(BODY) * length, rel=1e-9)

    swept = sweep(straight(length))
    lo, hi = swept.bounds()
    outline = np.array(hub.outline())
    assert (hi - lo)[0] == pytest.approx(BODY.width_outer, abs=1e-9)
    assert outline[:, 0].max() - outline[:, 0].min() == pytest.approx(
        length, abs=1e-9)
    assert outline[:, 1].max() - outline[:, 1].min() == pytest.approx(
        BODY.width_outer, abs=1e-9)


# -- 9.16 --------------------------------------------------------------------


@pytest.mark.parametrize("name,builder", sorted(LAYOUTS.items()))
@pytest.mark.parametrize("radius", [0.0, 12.0])
def test_analytic_volume_matches_the_slab_decomposition(name, builder, radius):
    hub = builder(radius)
    outline = abs(polygon_area(hub.outline()))
    rails = sum(abs(polygon_area(r)) for r in hub.rail_regions())
    expected = (outline * BODY.deck_thickness
                + rails * (BODY.rail_height_total - BODY.deck_thickness))
    assert hub.expected_volume() == pytest.approx(expected, rel=1e-12)
    assert hub.expected_volume() > 0


@pytest.mark.parametrize("name,builder", sorted(LAYOUTS.items()))
def test_rail_strips_are_one_rail_thickness_wide(name, builder):
    hub = builder(0.0)
    for i, region in enumerate(hub.rail_regions()):
        chain = hub.chain(i)
        length = sum(float(np.linalg.norm(b - a))
                     for a, b in zip(chain, chain[1:]))
        area = abs(polygon_area(region))
        # a strip of constant width; mitred corners make it slightly more
        assert area >= length * BODY.rail_thickness - 1e-9
        assert area < length * BODY.rail_thickness * 1.5


# -- 9.17 --------------------------------------------------------------------


def test_an_asymmetric_layout_is_rejected_rather_than_built_unflippable():
    with pytest.raises(HubInvalid, match="mirror axis"):
        Hub.auto([0.0, 90.0, 200.0]).validate()


@pytest.mark.parametrize("name,builder", sorted(LAYOUTS.items()))
@pytest.mark.parametrize("radius", [0.0, 12.0])
def test_the_standard_layouts_all_have_a_mirror_axis(name, builder, radius):
    assert builder(radius).mirror_axis() is not None


def test_a_gap_wider_than_180_degrees_is_rejected():
    with pytest.raises(HubInvalid, match="180"):
        Hub.auto([0.0, 60.0, 120.0]).validate()


def test_an_elbow_is_not_a_hub():
    """Two arms at 90° leaves a 270° gap. That is a curve, not a junction."""
    with pytest.raises(HubInvalid, match="180"):
        Hub.auto([0.0, 90.0]).validate()


def test_arms_must_be_in_ccw_order():
    with pytest.raises(HubInvalid, match="CCW"):
        Hub((Arm(math.pi, 40.0), Arm(0.0, 40.0))).validate()


def test_a_port_too_close_to_the_armpit_is_rejected():
    tight = Hub(tuple(Arm(math.radians(a), 13.0)
                      for a in (0.0, 90.0, 180.0, 270.0)))
    with pytest.raises(HubInvalid, match="port_distance"):
        tight.validate()


# -- slabs -------------------------------------------------------------------


@pytest.mark.parametrize("name,builder", sorted(LAYOUTS.items()))
@pytest.mark.parametrize("radius", [0.0, 12.0])
def test_every_slab_is_a_valid_solid_before_any_boolean(name, builder, radius):
    for index, solid in enumerate(builder(radius).solids()):
        check(solid, name=f"{name} slab {index}")


@pytest.mark.parametrize("name,builder", sorted(LAYOUTS.items()))
def test_a_hub_produces_one_deck_slab_and_one_rail_slab_per_arm(name, builder):
    hub = builder(0.0)
    assert len(hub.solids()) == 1 + len(hub.arms)


def test_slabs_span_the_right_heights():
    hub = x_junction(0.0)
    deck, *rails = hub.solids()
    assert deck.bounds()[0][2] == pytest.approx(-BODY.half_deck, abs=TOL)
    assert deck.bounds()[1][2] == pytest.approx(+BODY.half_deck, abs=TOL)
    for rail in rails:
        assert rail.bounds()[0][2] == pytest.approx(-BODY.half_height, abs=TOL)
        assert rail.bounds()[1][2] == pytest.approx(+BODY.half_height, abs=TOL)


def test_the_deck_and_the_rails_share_identical_boundary_vertices():
    """§5.3. The deck slab uses the outline directly, so every vertex it shares
    with a rail slab is the *same float*, not merely a nearby one.

    An earlier version inset the deck by half a rail thickness to avoid
    coplanar vertical faces. That put the deck's boundary inside the rail's
    port-cap face instead of on its edge, forcing the solver to recompute the
    point from a different expression. On the Y, whose coordinates are
    irrational, the two answers differed by 4e-7 mm and the union came out with
    sliver triangles — invisible until the float32 STL round trip. Coplanar
    faces built from identical vertices are the easy case for a boolean;
    near-coincident vertices are the hard one.
    """
    for builder in (x_junction, t_junction, y_junction):
        for radius in (0.0, 12.0):
            hub = builder(radius)
            deck = {p.tobytes() for p in hub.deck_region()}
            for region in hub.rail_regions():
                shared = [p for p in region if p.tobytes() in deck]
                assert shared, "a rail must sit on the deck's boundary"

            # and nothing is merely *close*: no two distinct deck vertices are
            # within a weld tolerance of each other
            points = np.array(hub.deck_region())
            gaps = np.linalg.norm(points[:, None] - points[None, :], axis=-1)
            np.fill_diagonal(gaps, np.inf)
            assert gaps.min() > 1e-4, f"{builder.__name__} r={radius} has a sliver"


def test_the_deck_reaches_every_port_plane():
    hub = x_junction(0.0)
    deck = np.array(hub.deck_region())
    reach = hub.arms[0].port_distance
    assert deck[:, 0].max() == pytest.approx(reach, abs=TOL)
    assert deck[:, 1].max() == pytest.approx(reach, abs=TOL)


# -- offsetting --------------------------------------------------------------


def test_offsetting_a_straight_polyline_just_translates_it():
    line = [np.array([0.0, 0.0]), np.array([10.0, 0.0])]
    out = offset_polyline(line, 2.0)
    assert out[0] == pytest.approx([0.0, 2.0], abs=TOL)
    assert out[1] == pytest.approx([10.0, 2.0], abs=TOL)


def test_offsetting_mitres_a_corner():
    """Offset is to the left of travel. Walking -X then +Y, left is (-1, -1)."""
    corner = [np.array([10.0, 0.0]), np.array([0.0, 0.0]),
              np.array([0.0, 10.0])]
    out = offset_polyline(corner, 1.0)
    assert out[1] == pytest.approx([-1.0, -1.0], abs=TOL)


def test_offsetting_rejects_a_repeated_point():
    with pytest.raises(ValueError, match="repeated"):
        offset_polyline([np.array([0.0, 0.0]), np.array([0.0, 0.0])], 1.0)
