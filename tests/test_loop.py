"""The vertical loop. docs/SPEC.md §4.1.

A loop is the first part in the set whose path leaves the ground, and the
first that has to be told which way up its section goes. Three things have to
hold, and none of them holds by accident:

- it must not pass through itself where it crosses at the bottom;
- the channel must face the loop's centre the whole way round, so the car is
  held on the inside and is upside down over the top;
- both ports must come out level, or the piece mates with nothing.

The third is the one that takes work. See `Loop._accumulate_twist`.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from parts import PATHS
from trackcore import (DEFAULT, Line, Loop, Path, PathDiscontinuous,
                       build_frames)
from trackcore.path import DEFAULT_LOOP_DRIFT

BODY = DEFAULT.body
SAG = DEFAULT.tolerances.chord_sag


def _frames(name: str = "loop"):
    return build_frames(PATHS[name](), SAG)


# -- it closes on itself without touching -----------------------------------


def test_a_loop_that_does_not_leave_its_ports_room_is_refused():
    """The reason the drift exists, and the reason it is more than a width.

    A vertical circle ends where it began, so with no drift the piece crosses
    its own entry — not a joint, just two solids in the same place. The guard
    is on the primitive because no later stage can see it: a self-intersecting
    sweep still comes out manifold, since `sweep.py` builds its topology from
    the station grid rather than from the geometry, and §7 would pass a mesh
    describing an impossible solid.

    One track width is not enough either. With the forward offset cancelled
    the two ports sit side by side, and each one's cut tools overshoot its rail
    by `outer_margin`; at a drift of exactly a width they would cut into each
    other. See `DEFAULT_LOOP_DRIFT`.
    """
    for bad in (0.1, BODY.width_outer, BODY.width_outer + 1.0):
        with pytest.raises(ValueError, match="needs at least"):
            Loop(radius=48.0, drift=bad)

    Loop(radius=48.0, drift=DEFAULT_LOOP_DRIFT)


def test_the_drift_is_exactly_what_two_ports_need_to_sit_side_by_side():
    """Derived, not chosen. It was `width + 2.0` and the 2 mm was arbitrary."""
    from trackcore.connector import outer_margin

    assert DEFAULT_LOOP_DRIFT == pytest.approx(
        BODY.width_outer + 2.0 * outer_margin(DEFAULT))

    reach = BODY.half_width + outer_margin(DEFAULT)
    other = DEFAULT_LOOP_DRIFT - BODY.half_width
    assert other >= reach, (
        f"a port's tools reach x={reach:.2f} and the other run's rail starts "
        f"at x={other:.2f}; they would cut into each other"
    )


def test_the_two_ends_of_a_loop_pass_beside_each_other():
    """Measured on the path, not argued from the drift.

    Sampled points that are far apart *along* the track must stay at least a
    track's width apart *in space*. The bottom crossing is where that is
    tight, and one drift is what it comes to.
    """
    path = PATHS["loop"]()
    s = np.linspace(0.0, path.length, 400)
    points = np.array([path.point(float(v)) for v in s])

    gap = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=-1)
    far_apart = np.abs(s[:, None] - s[None, :]) > 30.0
    closest = float(gap[far_apart].min())

    assert closest >= BODY.width_outer, (
        f"two runs of the loop come within {closest:.2f} mm, closer than the "
        f"{BODY.width_outer:.1f} mm the track is wide"
    )
    assert closest == pytest.approx(DEFAULT_LOOP_DRIFT, abs=0.5), (
        "the closest approach should be the drift itself, at the crossing"
    )


# -- which way up ------------------------------------------------------------


def test_the_section_is_always_square_to_the_path():
    """Up is perpendicular to the tangent everywhere, which is what makes it a
    section rather than a smear. Independent of any formula for where up
    should point."""
    frames = _frames()
    for point, up, tangent in zip(frames.points, frames.up, frames.tangent):
        assert abs(float(np.dot(up, tangent))) < 1e-9, (
            f"at z={point[2]:.1f} the section is not square to the path"
        )


def test_the_channel_faces_the_loop_centre_all_the_way_round():
    """Up is the inward radial direction, so the car is held on the inside.

    Measured against the plain circle's radial, `(0, -sin u, cos u)`, which is
    where the channel would face if the loop only went round. It also pulls
    back — `close` cancels the leads, see `Loop.close` — and a path that leans
    carries its section with it, so the two part company by a few degrees in
    the middle of the turn. That lean is correct: it is the bank a car
    following this path would want. What matters is that it stays small and
    that it is gone by the time the ports come round, which
    `test_both_ports_come_out_level` measures exactly.
    """
    frames = _frames()
    path = PATHS["loop"]()
    lead, turn = path.primitives[0].length, path.primitives[1]

    worst = 0.0
    for s_along, point, up in zip(frames.s, frames.points, frames.up):
        if point[2] < 1.0:            # the flat lead-ins, not the loop itself
            continue
        u = 2.0 * math.pi * (float(s_along) - lead) / turn.length
        radial = np.array([0.0, -math.sin(u), math.cos(u)])
        lean = math.degrees(math.acos(min(1.0, abs(float(np.dot(up, radial))))))
        worst = max(worst, lean)
        assert float(np.dot(up, radial)) > 0.0, (
            f"at z={point[2]:.1f} the channel faces {np.round(up, 3)}, which is "
            f"away from the loop's centre, not toward it"
        )

    assert worst < 12.0, f"the section leans {worst:.1f} deg out of the loop's plane"
    assert worst > 1.0, (
        "no lean at all would mean the pull-back is not in the geometry; "
        "see Loop.close"
    )


def test_the_car_is_upside_down_over_the_top():
    """The point of the whole part, and it should be measurable, not assumed."""
    frames = _frames()
    top = int(np.argmax(frames.points[:, 2]))
    assert frames.points[top][2] == pytest.approx(2.0 * 48.0, abs=0.5)
    assert frames.up[top][2] == pytest.approx(-1.0, abs=1e-3), (
        "over the top the deck must be above the car, not below it"
    )


# -- and level again at both ends -------------------------------------------


def test_both_ports_come_out_level():
    """Without the roll correction the exit sits 31 degrees over.

    The rotation-minimising frame does not follow the helix's torsion, so it
    arrives at the exit rolled by very nearly `drift / radius`. A port at that
    angle mates with nothing in the set, which makes this the test that says
    the correction is load-bearing rather than decorative.
    """
    frames = _frames()
    for label, up in (("entry", frames.up[0]), ("exit", frames.up[-1])):
        assert up == pytest.approx(np.array([0.0, 0.0, 1.0]), abs=1e-6), (
            f"the {label} port is not level: up is {np.round(up, 4)}"
        )


def test_the_twist_a_loop_hands_on_is_about_drift_over_radius():
    """The rule of thumb, pinned — and pinned as *not* zero.

    A loop stepping `drift` sideways over `radius` twists the section by about
    `drift / radius`, whatever the drift is eased by. It is what the exit
    straight has to carry, and at the defaults it is half a radian.
    """
    for radius in (40.0, 48.0, 72.0):
        turn = Loop(radius=radius)
        assert turn.twist == pytest.approx(turn.drift / radius, rel=0.02)

    assert Loop(radius=48.0).twist > 0.5, "the defaults should twist half a radian"


def test_an_exit_straight_at_the_wrong_roll_is_refused_outright():
    """The failure the correction prevents, and it is refused, not tolerated.

    Chaining a plain `Line` after the loop is the version that came first, and
    the interesting part is that it never gets as far as a mesh: a straight at
    zero roll steps in roll against a loop that ends at 0.54 rad, and `Path`
    calls that discontinuous — which it is, physically. The section would jump.
    """
    turn = Loop(radius=48.0)
    for wrong_roll in (0.0, turn.twist / 2.0, -turn.twist):
        with pytest.raises(PathDiscontinuous, match="step in roll"):
            Path.chain(Line(5.15), turn, Line(5.15, roll_offset=wrong_roll))

    # and the right one chains, which is the whole difference
    Path.chain(Line(5.15), turn, Line(5.15, roll_offset=turn.twist))


# -- the ends still behave like ends ----------------------------------------


def test_a_loop_leaves_and_rejoins_heading_the_same_way():
    """`end_transform` is a pure sideways step: no advance, no turn.

    Which is what lets a loop sit in a run of straights — the track comes out
    pointing the way it went in, one drift across.
    """
    turn = Loop(radius=48.0)
    end = turn.end_transform()

    assert end[:3, :3] == pytest.approx(np.eye(3), abs=1e-12), "a loop must not turn"
    assert end[:3, 3] == pytest.approx(
        np.array([DEFAULT_LOOP_DRIFT, 0.0, 0.0]), abs=1e-9)

    assert turn.tangent(0.0) == pytest.approx(np.array([0.0, 1.0, 0.0]), abs=1e-9)
    assert turn.tangent(turn.length) == pytest.approx(
        np.array([0.0, 1.0, 0.0]), abs=1e-9)


def test_a_linearly_drifting_loop_would_enter_at_an_angle():
    """Why the drift is eased rather than spread evenly.

    Grown linearly the lateral rate is `drift / 2*pi*radius` at the ends
    instead of zero, so the piece enters crabbing several degrees off and
    `Path` refuses to chain it to a straight. The smoothstep is what buys the
    exact `+Y` at both ends that the test above measures.
    """
    turn = Loop(radius=48.0)
    crab = math.degrees(math.atan(turn.drift / (2.0 * math.pi * turn.radius)))
    assert crab > 4.0, "the angle being avoided should be a real one"

    assert build_frames(PATHS["loop"](), SAG).tangent[0] == pytest.approx(
        np.array([0.0, 1.0, 0.0]), abs=1e-9)
