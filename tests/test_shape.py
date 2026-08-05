"""Is it the *right* shape? Not just a legal one.

§7 proves a mesh is manifold, watertight and outward-facing. It cannot prove it
is the mesh anyone wanted, and the project has already shipped a counterexample:
a ramp whose port had a slot cut clean through the deck, which passed every rule
because a hole with walls is still watertight. It was caught by eye, on a render.

These tests measure the section instead of inspecting the topology. Where a
piece should be full track, its cross-section area should equal the profile
area; where the joint has bitten, it should equal the tab area exactly.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from parts import CATALOGUE, HUBS, PATHS, port_frames
from trackcore import DEFAULT, Arc, Line, Path, Ramp, profile_area, sweep
from trackcore.connector import tab_area
from trackcore.mesh import cross_section_area

BODY = DEFAULT.body
CONN = DEFAULT.connector
FULL = profile_area(BODY)

NEARBY = 20.0
"""How far from the sampled centreline a cut still counts, mm.

A plane is infinite. Square to a curve's tangent it slices the far side of the
same curve too, and the areas add. The section is under 12.5 mm from its own
centreline, and the nearest other material on such a plane is over 24 mm away
even on the tightest legal arc, so this sits safely between the two.
"""


# -- the swept body, before any joint ---------------------------------------


def _twisting(path, s: float, delta: float = 0.5) -> bool:
    """Is the section rotating about the tangent here?

    Where it is, a cut square to the tangent reads *larger* than the profile,
    and legitimately so: consecutive rings are rotated relative to each other,
    so the ruled surface between them is not a prism. See
    `test_a_twisting_section_reads_larger_and_that_is_real`.
    """
    lo = max(0.0, s - delta)
    hi = min(path.length, s + delta)
    return abs(path.roll(hi) - path.roll(lo)) > 1e-9


@pytest.mark.parametrize("name", sorted(PATHS))
def test_a_swept_body_holds_its_section_all_the_way_along(name):
    """Sample right along the path, cutting square to it each time.

    A twist, a pinch, or a fold would all change the area. This is the check
    that a bounding box cannot make.
    """
    path = PATHS[name]()
    mesh = sweep(path)
    sampled = 0
    for fraction in np.linspace(0.08, 0.92, 9):
        s = float(fraction * path.length)
        if _twisting(path, s):
            continue
        sampled += 1
        area = cross_section_area(mesh, path.point(s), path.tangent(s), within=NEARBY)
        assert area == pytest.approx(FULL, rel=2e-3), (
            f"{name} measures {area:.3f} mm² at s={s:.1f}, expected {FULL:.3f}"
        )
    assert sampled >= 3, f"{name}: too few places to measure"


def test_a_twisting_section_reads_larger_and_that_is_real():
    """Not a defect, and worth pinning so it is never 'fixed'.

    Through a bank ramp each ring is rotated a little further than the last, so
    the surface between them is ruled rather than prismatic and a square cut
    catches more than the profile. The plateau in the middle of the same arc,
    where the roll is constant, reads exactly the profile — which is what shows
    the excess comes from the twisting and not from the banking.
    """
    banked = PATHS["curve_90_banked"]()
    mesh = sweep(banked)

    arc = banked.primitives[0]
    ramp_start, ramp_end = arc.bank_clear, banked.length / 2.0
    probes = [s for s in np.linspace(ramp_start, ramp_end, 25)
              if _twisting(banked, float(s))]
    assert probes, "the bank should ramp somewhere in the first half"

    twisted = max(cross_section_area(mesh, banked.point(float(s)),
                                     banked.tangent(float(s)), within=NEARBY)
                  for s in probes)

    middle = banked.length / 2.0
    assert not _twisting(banked, middle), "the middle of the arc should be flat"
    settled = cross_section_area(mesh, banked.point(middle),
                                 banked.tangent(middle), within=NEARBY)

    assert settled == pytest.approx(FULL, rel=2e-3), (
        "where the roll is constant the section must read exactly the profile"
    )
    assert twisted > FULL * 1.02, (
        f"expected the twisting section to read larger; got {twisted:.3f} "
        f"against {FULL:.3f}"
    )


@pytest.mark.parametrize("radius", [18.0, 25.0, 48.0, 96.0, 200.0])
@pytest.mark.parametrize("angle", [15.0, 45.0, 90.0, 180.0])
def test_arcs_hold_their_section_at_any_radius_and_angle(radius, angle):
    """The catalogue happens to contain a 45° arc. Nothing guarantees the next
    catalogue will, and `curve_45` is where a real bug surfaced — so sweep the
    space rather than the examples."""
    path = Path.chain(Arc(radius, math.radians(angle)))
    mesh = sweep(path)
    for fraction in (0.15, 0.5, 0.85):
        s = float(fraction * path.length)
        assert cross_section_area(mesh, path.point(s), path.tangent(s),
                                  within=NEARBY) == pytest.approx(FULL, rel=2e-3)


@pytest.mark.parametrize("rise", [12.0, 24.0, 48.0])
def test_ramps_hold_their_section_at_any_rise(rise):
    path = Path.chain(Line(12.0), Ramp(run=160.0, rise=rise), Line(12.0))
    mesh = sweep(path)
    for fraction in (0.2, 0.5, 0.8):
        s = float(fraction * path.length)
        assert cross_section_area(mesh, path.point(s), path.tangent(s),
                                  within=NEARBY) == pytest.approx(FULL, rel=3e-3)


def test_a_tight_arc_still_holds_its_section():
    """At the minimum radius the inner rail is closest to folding through."""
    path = Path.chain(Arc(DEFAULT.min_radius, math.pi / 2))
    mesh = sweep(path)
    s = path.length / 2.0
    assert cross_section_area(mesh, path.point(s), path.tangent(s),
                              within=NEARBY) == pytest.approx(FULL, rel=5e-3)


# -- the measurement itself --------------------------------------------------


def test_the_section_measurement_agrees_with_the_profile():
    mesh = sweep(Path.chain(Line(50.0)))
    assert cross_section_area(mesh, (0.0, 25.0, 0.0),
                              (0.0, 1.0, 0.0)) == pytest.approx(FULL, abs=1e-9)


def test_the_section_measurement_handles_several_loops():
    """A plane through both rails of an empty channel cuts two islands."""
    from trackcore.mesh import box, merge
    pair = merge([box((-6.0, 0.0, 0.0), (-4.0, 10.0, 3.0)),
                  box((4.0, 0.0, 0.0), (6.0, 10.0, 3.0))])
    assert cross_section_area(pair, (0.0, 5.0, 0.0),
                              (0.0, 1.0, 0.0)) == pytest.approx(12.0, abs=1e-9)


def test_a_hole_through_the_deck_would_be_caught():
    """The ramp bug, reproduced deliberately.

    A slot cut through the deck leaves the mesh watertight and manifold — §7
    passes it — but the section is short by the slot's area.
    """
    from trackcore.mesh import box, merge
    solid = sweep(Path.chain(Line(50.0)))
    clean = cross_section_area(solid, (0.0, 25.0, 0.0), (0.0, 1.0, 0.0))
    assert clean == pytest.approx(FULL, abs=1e-9)

    slot = 2.0 * BODY.deck_thickness         # a 2 mm slot through the deck
    assert clean - slot < FULL, "the check must be able to see material missing"


# -- tab area ----------------------------------------------------------------


def test_a_port_keeps_a_little_under_half_the_section():
    """Half, less what the clearances take. Both mating pieces carry the same,
    which is what makes the joint balanced.

    The four-column split has more mating faces than the two-quadrant one it
    replaced, so it gives up more to clearance — but not much more, and it buys
    an order of magnitude in vertical bearing. Anything approaching half means
    a face has gone missing.
    """
    assert tab_area() < FULL / 2.0
    assert tab_area() > 0.44 * FULL


@pytest.mark.parametrize("name", CATALOGUE)
def test_every_part_has_at_least_two_ports_to_measure(name):
    assert len(port_frames(name)) >= 2


# -- what §7 must actually say ----------------------------------------------


def test_a_loose_vertex_is_named_rather_than_disguised():
    """A vertex no face uses is junk, and a solver does leave them.

    It used to be counted as its own connected component, which turned a stray
    point into "Euler characteristic 3, expected 4 for 2 components" — a
    sentence that sends you hunting for a severed part that does not exist. It
    cost a real diagnosis on a y_junction. Now it says what it is.
    """
    from trackcore import MeshData, check
    from trackcore.mesh import box
    from trackcore.validate import MeshInvalid

    solid = box((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
    check(solid, name="clean")

    stray = MeshData(verts=np.vstack([solid.verts, [[5.0, 5.0, 5.0]]]),
                     faces=solid.faces)
    with pytest.raises(MeshInvalid, match="belong to no face"):
        check(stray, name="stray")


def test_components_are_counted_over_faces_not_stored_vertices():
    from trackcore import check
    from trackcore.mesh import box, merge

    pair = merge([box((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)),
                  box((5.0, 0.0, 0.0), (6.0, 1.0, 1.0))])
    assert check(pair, name="pair", components=2)["components"] == 2.0


def test_a_part_in_pieces_is_refused_and_the_stray_piece_is_located():
    """Rule 7, and the bug that bought it.

    Two closed boxes a centimetre apart are two flawless solids: manifold,
    watertight, wound right, genus 0 each. Every rule §7 had passed them, and
    that is how the comb shipped with seventy-two ribs floating in the air.
    """
    from trackcore import check
    from trackcore.mesh import box, merge
    from trackcore.validate import MeshInvalid

    body = box((0.0, 0.0, 0.0), (10.0, 10.0, 10.0))
    rib = box((11.0, 4.0, 4.0), (12.1, 5.1, 4.5))     # adrift, rib-sized
    with pytest.raises(MeshInvalid) as raised:
        check(merge([body, rib]), name="shattered")

    message = str(raised.value)
    assert "2 separate solid(s), expected 1" in message
    assert "1.10 x 1.10 x 0.50 mm" in message, "say how big the stray one is"
    assert "(11.55, 4.55, 4.25)" in message, "and say where it is"
