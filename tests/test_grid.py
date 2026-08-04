"""docs/SPEC.md §10 Phase 4: the part set and the layout grid.

Phase 4 adds no geometry code. These tests are about the *set* — that its
pieces are commensurate, that loops close, and that every port stays flat where
the connector reaches in.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from parts import (CATALOGUE, GRID, HUBS, NOT_PARTS, PATHS, build, curve,
                   port_frames)
from trackcore import DEFAULT, Arc, Line, Path, Ramp
from trackcore.path import DEFAULT_PORT_CLEAR

TOL = 1e-9
LAP = DEFAULT.connector.lap_length + DEFAULT.connector.fit_clearance


# -- the catalogue -----------------------------------------------------------


def test_the_catalogue_is_the_printable_set_only():
    assert len(CATALOGUE) == len(set(CATALOGUE))
    assert not (set(CATALOGUE) & NOT_PARTS)
    assert set(CATALOGUE) == (set(PATHS) - NOT_PARTS) | set(HUBS)


@pytest.mark.parametrize("name", CATALOGUE)
def test_every_part_is_pure_data(name):
    """A part is a path or an arm layout. Nothing else is allowed to be one."""
    assert name in PATHS or name in HUBS


# -- commensurability --------------------------------------------------------


def test_a_ninety_degree_curve_advances_one_module_on_each_axis():
    end = PATHS["curve_90"]().end_transform
    assert abs(end[0, 3]) == pytest.approx(GRID.module, abs=TOL)
    assert abs(end[1, 3]) == pytest.approx(GRID.module, abs=TOL)
    assert end[2, 3] == pytest.approx(0.0, abs=TOL)


def test_two_forty_five_degree_curves_land_where_one_ninety_does():
    pair = Path.chain(*[Arc(GRID.module, math.radians(45.0))] * 2)
    assert pair.end_transform == pytest.approx(
        PATHS["curve_90"]().end_transform, abs=1e-9)


def test_the_straights_are_module_fractions():
    lengths = {name: PATHS[name]().length for name in
               ("straight_full", "straight_half", "straight_quarter")}
    assert lengths["straight_full"] == pytest.approx(GRID.module, abs=TOL)
    assert lengths["straight_half"] == pytest.approx(GRID.half, abs=TOL)
    assert lengths["straight_quarter"] == pytest.approx(GRID.quarter, abs=TOL)


@pytest.mark.parametrize("name", sorted(HUBS))
def test_a_junction_substitutes_for_one_full_straight(name):
    """Arms reach half a module, so passing through a junction consumes exactly
    the same distance as the straight it replaces."""
    hub = HUBS[name]()
    for arm in hub.arms:
        assert arm.port_distance == pytest.approx(GRID.half, abs=TOL)
    assert 2.0 * GRID.half == pytest.approx(
        PATHS["straight_full"]().length, abs=TOL)


def test_a_ramp_spans_a_whole_number_of_modules():
    path = PATHS["ramp"]()
    end = path.end_transform
    assert end[1, 3] == pytest.approx(GRID.module * GRID.ramp_modules, abs=1e-6)
    assert end[2, 3] == pytest.approx(GRID.deck_height, abs=1e-6)


def test_the_bridge_deck_clears_a_car_underneath():
    clear = GRID.deck_height - DEFAULT.body.rail_height_total
    assert clear > 35.0, f"only {clear:.1f} mm under the deck"


# -- loops close -------------------------------------------------------------


def test_four_quarter_turns_close():
    loop = Path.chain(*[Arc(GRID.module, math.radians(90.0))] * 4)
    assert loop.end_transform == pytest.approx(np.eye(4), abs=1e-9)


def test_a_rectangle_of_straights_and_curves_closes():
    quarter = Arc(GRID.module, math.radians(90.0))
    loop = Path.chain(*[p for _ in range(4)
                        for p in (Line(GRID.module), quarter)])
    assert loop.end_transform == pytest.approx(np.eye(4), abs=1e-9)


def test_an_oval_of_mixed_straights_closes():
    quarter = Arc(GRID.module, math.radians(90.0))
    loop = Path.chain(Line(GRID.module), quarter, quarter,
                      Line(GRID.half), Line(GRID.half),
                      quarter, quarter)
    assert loop.end_transform == pytest.approx(np.eye(4), abs=1e-9)


def test_eight_forty_fives_close():
    loop = Path.chain(*[Arc(GRID.module, math.radians(45.0))] * 8)
    assert loop.end_transform == pytest.approx(np.eye(4), abs=1e-9)


def test_a_tight_curve_closes_its_own_loop():
    loop = Path.chain(*[Arc(GRID.half, math.radians(90.0))] * 4)
    assert loop.end_transform == pytest.approx(np.eye(4), abs=1e-9)


# -- every port must be flat where the connector reaches in ------------------


@pytest.mark.parametrize("name", [n for n in CATALOGUE if n in PATHS])
def test_the_lap_zone_at_every_port_is_free_of_roll(name):
    """§6.6's cut tools are flat boxes in the port frame. If the section has
    rolled by the time they reach in, they slice it at the wrong height on each
    rail. A banked curve without this came out genus 3."""
    path = PATHS[name]()
    for s in np.linspace(0.0, LAP, 12):
        assert path.roll(float(s)) == pytest.approx(0.0, abs=1e-12)
        assert path.roll(float(path.length - s)) == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("name", [n for n in CATALOGUE if n in PATHS])
def test_the_lap_zone_at_every_port_is_free_of_vertical_curvature(name):
    """Horizontal curvature is harmless — it moves the section sideways, not in
    z. Vertical curvature is not, which is why a ramp gets flat lead-ins."""
    path = PATHS[name]()
    for s in np.linspace(0.0, LAP, 12):
        for probe in (float(s), float(path.length - s)):
            rise = abs(float(path.tangent(probe)[2]))
            assert rise < 1e-9, f"{name} pitches at s={probe:.2f}"


def test_a_bare_ramp_pitches_inside_its_lap_zone():
    """The bug this rule exists for, pinned.

    A smoothstep's vertical curvature is greatest at its **ends** — exactly
    where the connector reaches in. Without flat leads the section has pitched
    twelve degrees by the time the flat cut boxes bite, and they slice a slot
    clean through the deck and interrupt both rails.

    The damning part: that broken piece is still one component, still watertight,
    still passes every §7 rule. Validation cannot tell "manifold" from
    "correct", so this rule is enforced on the *path*, before any mesh exists.
    """
    bare = Path.chain(Ramp(run=84.0, rise=34.0))
    pitch = max(abs(float(bare.tangent(float(s))[2]))
                for s in np.linspace(0.0, LAP, 12))
    assert pitch > 0.1, "expected a bare ramp to pitch where the connector bites"

    fitted = PATHS["ramp"]()
    assert max(abs(float(fitted.tangent(float(s))[2]))
               for s in np.linspace(0.0, LAP, 12)) < 1e-9


def test_a_banked_arc_reaches_its_full_bank_in_the_middle():
    path = PATHS["curve_90_banked"]()
    assert path.roll(path.length / 2.0) == pytest.approx(
        math.radians(GRID.bank_deg), abs=1e-12)


def test_a_banked_arc_too_short_to_stay_flat_is_rejected():
    with pytest.raises(ValueError, match="lap zones stay flat"):
        Arc(radius=20.0, angle=math.radians(30.0), bank=math.radians(10.0))


def test_an_unbanked_arc_of_any_length_is_still_allowed():
    Arc(radius=20.0, angle=math.radians(30.0))


# -- build volume ------------------------------------------------------------


BED = (220.0, 220.0, 250.0)


@pytest.mark.parametrize("name", CATALOGUE)
def test_every_part_fits_a_common_print_bed(name):
    piece = build(name, DEFAULT, connectors=False)
    lo = np.min([s.bounds()[0] for s in piece.solids], axis=0)
    hi = np.max([s.bounds()[1] for s in piece.solids], axis=0)
    size = sorted(hi - lo)
    assert size[2] <= max(BED[0], BED[1]), (
        f"{name} is {size[2]:.0f} mm on its longest axis; the bed is "
        f"{max(BED[0], BED[1]):.0f} mm"
    )


# -- ports -------------------------------------------------------------------


@pytest.mark.parametrize("name", CATALOGUE)
def test_every_part_has_at_least_two_ports(name):
    assert len(port_frames(name)) >= 2


def test_the_grid_can_be_retuned_from_one_number():
    """Changing the module must move the whole set, not half of it."""
    from parts import Grid
    bigger = Grid(module=120.0)
    assert bigger.half == 60.0 and bigger.quarter == 30.0
    assert bigger.fillet == bigger.quarter


def test_a_curve_helper_still_accepts_overrides():
    assert curve(radius=50.0, angle_deg=30.0).length == pytest.approx(
        50.0 * math.radians(30.0), abs=TOL)
