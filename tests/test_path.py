"""docs/SPEC.md §9.5–9.8: path primitives, continuity, guards, stations."""

from __future__ import annotations

import math

import numpy as np
import pytest

from trackcore import Arc, Line, Path, PathDiscontinuous, PathTooTightError, Ramp
from trackcore.mesh import translation

SAG = 0.02


# -- 9.5 ---------------------------------------------------------------------


def test_arc_ends_where_the_analytic_solution_says():
    """A left turn curves toward -X: right is forward x up = +X."""
    path = Path.chain(Arc(radius=100.0, angle=math.pi / 2))
    assert path.length == pytest.approx(100.0 * math.pi / 2, abs=1e-12)
    assert path.point(path.length) == pytest.approx([-100.0, 100.0, 0.0], abs=1e-9)
    assert path.tangent(path.length) == pytest.approx([-1.0, 0.0, 0.0], abs=1e-9)


def test_a_right_turn_mirrors_a_left_one():
    left = Path.chain(Arc(radius=100.0, angle=math.pi / 3))
    right = Path.chain(Arc(radius=100.0, angle=-math.pi / 3))
    lp, rp = left.point(left.length), right.point(right.length)
    assert lp[0] == pytest.approx(-rp[0], abs=1e-12)
    assert lp[1] == pytest.approx(rp[1], abs=1e-12)


def test_line_is_a_straight_run_along_plus_y():
    path = Path.chain(Line(60.0))
    assert path.point(30.0) == pytest.approx([0.0, 30.0, 0.0], abs=1e-12)
    assert path.tangent(0.0) == pytest.approx([0.0, 1.0, 0.0], abs=1e-12)


def test_ramp_is_horizontal_at_both_ends_so_it_chains_without_a_kink():
    ramp = Ramp(run=84.0, rise=20.0)
    assert ramp.tangent(0.0) == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)
    assert ramp.tangent(ramp.length) == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)
    # true arc length exceeds the horizontal run
    assert ramp.length > ramp.run
    Path.chain(Line(20.0), ramp, Line(20.0))  # must not raise


def test_ramp_reports_true_arc_length_not_the_run():
    ramp = Ramp(run=100.0, rise=30.0)
    walked = 0.0
    previous = ramp.point(0.0)
    for i in range(1, 20001):
        current = ramp.point(ramp.length * i / 20000)
        walked += float(np.linalg.norm(current - previous))
        previous = current
    assert walked == pytest.approx(ramp.length, rel=1e-6)


# -- 9.6 ---------------------------------------------------------------------


class _Broken:
    """A primitive that lies about where it ends. Only the check catches it."""

    length = 10.0

    def __init__(self, position_error=0.0, tangent_error=0.0):
        self.position_error = position_error
        self.tangent_error = tangent_error

    def point(self, s):
        return np.array([0.0, s, 0.0])

    def tangent(self, s):
        return np.array([0.0, 1.0, 0.0])

    def roll(self, s):
        return 0.0

    def curvature(self, s):
        return 0.0

    def min_radius_of_curvature(self):
        return math.inf

    def stations(self, sag):
        return [0.0, self.length]

    def end_transform(self):
        from trackcore.mesh import rotation_z
        return (translation(0.0, self.length + self.position_error, 0.0)
                @ rotation_z(self.tangent_error))


def test_chain_raises_on_a_position_gap():
    with pytest.raises(PathDiscontinuous, match="apart"):
        Path.chain(_Broken(position_error=0.5), Line(10.0))


def test_chain_raises_on_a_kink():
    with pytest.raises(PathDiscontinuous, match="kink"):
        Path.chain(_Broken(tangent_error=0.05), Line(10.0))


def test_chain_accepts_a_clean_join():
    path = Path.chain(Line(60.0), Arc(100.0, math.radians(45.0)), Line(60.0))
    assert path.length == pytest.approx(
        60.0 + 100.0 * math.radians(45.0) + 60.0, abs=1e-12)


def test_a_path_needs_at_least_one_primitive():
    with pytest.raises(ValueError):
        Path([])


# -- 9.7 ---------------------------------------------------------------------


def test_a_tight_arc_is_rejected_at_construction():
    with pytest.raises(PathTooTightError, match="inside out"):
        Arc(radius=10.0, angle=math.pi / 2)


def test_the_guard_is_the_profile_half_width_times_the_factor():
    from trackcore import DEFAULT
    assert DEFAULT.min_radius == pytest.approx(18.0, abs=1e-12)
    Arc(radius=18.0, angle=math.pi / 2)          # exactly on the limit is fine
    with pytest.raises(PathTooTightError):
        Arc(radius=17.9, angle=math.pi / 2)


def test_a_short_steep_ramp_is_rejected():
    with pytest.raises(PathTooTightError, match="radius"):
        Ramp(run=20.0, rise=30.0)


def test_check_curvature_re_checks_against_the_real_profile():
    path = Path.chain(Arc(radius=20.0, angle=math.pi / 2))
    path.check_curvature(18.0)
    with pytest.raises(PathTooTightError, match="below the minimum"):
        path.check_curvature(30.0)


# -- 9.8 ---------------------------------------------------------------------


def _max_sag(path: Path, stations: list[float]) -> float:
    """Largest deviation between a chord and the true curve it spans."""
    worst = 0.0
    for a, b in zip(stations, stations[1:]):
        pa, pb = path.point(a), path.point(b)
        mid = path.point(0.5 * (a + b))
        chord = pb - pa
        span = float(np.linalg.norm(chord))
        if span == 0.0:
            continue
        offset = mid - pa
        along = float(np.dot(offset, chord)) / span
        worst = max(worst, float(np.linalg.norm(offset - along * chord / span)))
    return worst


@pytest.mark.parametrize("radius,angle_deg", [(100.0, 90.0), (25.0, 45.0),
                                              (18.0, 180.0)])
def test_arc_stations_respect_the_sag_bound(radius, angle_deg):
    path = Path.chain(Arc(radius, math.radians(angle_deg)))
    stations = path.stations(SAG)
    assert _max_sag(path, stations) <= SAG + 1e-9


def test_ramp_stations_respect_the_sag_bound():
    path = Path.chain(Ramp(run=120.0, rise=34.0))
    assert _max_sag(path, path.stations(SAG)) <= SAG + 1e-9


def test_straight_runs_get_only_their_endpoints():
    assert Path.chain(Line(200.0)).stations(SAG) == [0.0, 200.0]


def test_every_primitive_boundary_gets_a_station():
    path = Path.chain(Line(60.0), Arc(100.0, math.radians(45.0)), Line(60.0))
    stations = path.stations(SAG)
    for boundary in path.starts + [path.length]:
        assert min(abs(s - boundary) for s in stations) < 1e-9


def test_stations_are_sorted_and_unique():
    path = Path.chain(Line(60.0), Arc(100.0, math.radians(90.0)), Line(60.0))
    stations = path.stations(SAG)
    assert stations == sorted(stations)
    assert all(b - a > 1e-9 for a, b in zip(stations, stations[1:]))


def test_sag_tolerance_must_be_positive():
    with pytest.raises(ValueError):
        Path.chain(Line(10.0)).stations(0.0)


# -- banking -----------------------------------------------------------------


def test_bank_eases_in_and_out_so_it_never_appears_as_a_step():
    bank = math.radians(12.0)
    arc = Arc(radius=100.0, angle=math.pi / 2, bank=bank)
    assert arc.roll(0.0) == pytest.approx(0.0, abs=1e-12)
    assert arc.roll(arc.length) == pytest.approx(0.0, abs=1e-12)
    assert arc.roll(arc.length / 2) == pytest.approx(bank, abs=1e-12)


def test_a_banked_arc_chains_onto_a_straight_without_a_roll_step():
    Path.chain(Line(40.0),
               Arc(100.0, math.pi / 2, bank=math.radians(15.0)),
               Line(40.0))
