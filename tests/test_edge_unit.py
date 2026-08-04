"""docs/SPEC.md §9.1–9.3: the edge unit and the profile it builds."""

from __future__ import annotations

import pathlib

import pytest

from trackcore import Body, EdgeUnit, profile, profile_area
from trackcore.edge_unit import PROFILE_VERTS
from trackcore.mesh import is_simple, shoelace

BODY = Body()
REPO = pathlib.Path(__file__).resolve().parent.parent


# -- 9.1 ---------------------------------------------------------------------


def test_profile_has_twelve_vertices_and_is_a_simple_polygon():
    pts = profile(BODY)
    assert len(pts) == PROFILE_VERTS
    assert is_simple(pts)


def test_profile_is_ccw_seen_from_plus_y():
    """Which in raw (x, z) parameter order is clockwise, i.e. negative area."""
    assert shoelace(profile(BODY)) < 0


def test_profile_bounding_box_is_the_track_section():
    pts = profile(BODY)
    xs = [p[0] for p in pts]
    zs = [p[1] for p in pts]
    assert max(xs) - min(xs) == pytest.approx(BODY.width_outer, abs=1e-12)
    assert max(zs) - min(zs) == pytest.approx(BODY.rail_height_total, abs=1e-12)


def test_profile_area_is_two_rails_plus_the_deck():
    rails = 2 * BODY.rail_thickness * BODY.rail_height_total
    deck = BODY.channel_width * BODY.deck_thickness
    assert profile_area(BODY) == pytest.approx(rails + deck, abs=1e-12)


# -- 9.2 ---------------------------------------------------------------------


def test_profile_is_symmetric_about_both_axes():
    pts = {(round(x, 12), round(z, 12)) for x, z in profile(BODY)}
    assert {(-x, z) for x, z in pts} == pts
    assert {(x, -z) for x, z in pts} == pts


# -- 9.3 ---------------------------------------------------------------------


def test_profile_is_exactly_two_edge_units():
    plus = {(round(x, 12), round(z, 12)) for x, z in EdgeUnit(BODY, +1).points()}
    minus = {(round(x, 12), round(z, 12)) for x, z in EdgeUnit(BODY, -1).points()}
    pts = {(round(x, 12), round(z, 12)) for x, z in profile(BODY)}

    assert len(plus) == 6 and len(minus) == 6
    assert plus | minus == pts
    assert not (plus & minus)


def test_the_two_edge_units_are_mirror_images():
    plus = EdgeUnit(BODY, +1).points()
    minus = EdgeUnit(BODY, -1).points()
    for (px, pz), (mx, mz) in zip(plus, minus):
        assert px == pytest.approx(-mx, abs=1e-12)
        assert pz == pytest.approx(mz, abs=1e-12)


def test_edge_unit_rejects_a_bad_side():
    with pytest.raises(ValueError):
        EdgeUnit(BODY, 0)


def test_derived_dimensions_are_never_written_out_as_literals():
    """The cross-section exists in one place. §9.3.

    Only config.py may hold measured numbers; nothing may hard-code a value
    derived from them, because that is how four generators drifted apart in v1.
    """
    # half_width (12.0) is deliberately not in this list: it collides with
    # ordinary polynomial coefficients. The rest are distinctive enough.
    derived = ["10.8", "2.35", "21.6"]
    for source in sorted((REPO / "trackcore").glob("*.py")):
        text = source.read_text()
        for literal in derived:
            assert literal not in text, (
                f"{source.name} hard-codes the derived value {literal}; "
                f"take it from Body instead"
            )


def test_profile_rejects_an_impossible_body():
    with pytest.raises(ValueError):
        profile(Body(rail_thickness=20.0))
    with pytest.raises(ValueError):
        profile(Body(deck_thickness=9.0))
