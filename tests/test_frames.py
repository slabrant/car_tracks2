"""docs/SPEC.md §9.4: the frames must not twist.

The headline test here is the one that fails for a Frenet implementation. The
Frenet normal is undefined where curvature is zero, which is most of this
track, and flips at inflections, so it twists at every straight-to-arc
transition. That is why the spec mandates a rotation-minimising frame.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from parts import s_bend
from trackcore import Arc, DegenerateFrame, Line, Path, Ramp, build_frames
from trackcore.frames import seed_across

SAG = 0.02
UP = np.array([0.0, 0.0, 1.0])


# -- 9.4 ---------------------------------------------------------------------


def test_no_twist_across_straight_to_arc_transitions():
    path = Path.chain(Line(60.0), Arc(100.0, math.radians(90.0)), Line(60.0))
    frames = build_frames(path, SAG)
    dots = frames.up @ UP
    assert dots.min() > 0.999, f"frame rolled over: min up.z = {dots.min():.6f}"


def test_no_twist_through_an_inflection():
    """Left turn straight into a right turn. Frenet flips its normal here."""
    frames = build_frames(s_bend(radius=100.0, angle_deg=45.0), SAG)
    dots = frames.up @ UP
    assert dots.min() > 0.999


def test_no_twist_over_a_ramp():
    """On a ramp the frame tilts with the slope, so up.z drops by design. The
    no-twist condition is that `across` stays horizontal: the track climbs, it
    does not corkscrew."""
    path = Path.chain(Line(30.0), Ramp(run=120.0, rise=34.0), Line(30.0))
    frames = build_frames(path, SAG)
    assert np.allclose(frames.across[:, 2], 0.0, atol=1e-12)
    assert (frames.up @ UP).min() > 0.9   # tilt, never inversion


# -- frame sanity ------------------------------------------------------------


def test_frames_are_orthonormal_and_right_handed():
    path = Path.chain(Line(40.0), Arc(80.0, math.radians(120.0)), Line(40.0))
    frames = build_frames(path, SAG)

    for name, vectors in (("tangent", frames.tangent),
                          ("across", frames.across),
                          ("up", frames.up)):
        norms = np.linalg.norm(vectors, axis=1)
        assert np.allclose(norms, 1.0, atol=1e-12), f"{name} is not unit"

    assert np.allclose(np.einsum("ij,ij->i", frames.across, frames.tangent),
                       0.0, atol=1e-12)
    assert np.allclose(np.einsum("ij,ij->i", frames.up, frames.tangent),
                       0.0, atol=1e-12)
    # across x tangent = up, the same handedness as (X, Y, Z)
    assert np.allclose(np.cross(frames.across, frames.tangent), frames.up,
                       atol=1e-12)


def test_the_seed_frame_puts_across_on_plus_x_for_a_plus_y_tangent():
    across = seed_across(np.array([0.0, 1.0, 0.0]))
    assert across == pytest.approx([1.0, 0.0, 0.0], abs=1e-12)


def test_a_vertical_start_tangent_raises_rather_than_guessing():
    with pytest.raises(DegenerateFrame):
        seed_across(np.array([0.0, 0.0, 1.0]))


# -- roll --------------------------------------------------------------------


def test_bank_rolls_the_frame_and_only_in_the_middle():
    bank = math.radians(20.0)
    path = Path.chain(Arc(100.0, math.pi / 2, bank=bank))
    frames = build_frames(path, SAG)

    assert frames.up[0] @ UP == pytest.approx(1.0, abs=1e-9)
    assert frames.up[-1] @ UP == pytest.approx(1.0, abs=1e-9)

    middle = len(frames.up) // 2
    assert frames.up[middle] @ UP == pytest.approx(math.cos(bank), abs=1e-6)


def test_an_unbanked_path_keeps_across_horizontal():
    path = Path.chain(Line(50.0), Arc(60.0, math.radians(90.0)))
    frames = build_frames(path, SAG)
    assert np.allclose(frames.across[:, 2], 0.0, atol=1e-12)


def test_stations_and_frames_line_up():
    path = Path.chain(Line(20.0), Arc(50.0, math.radians(60.0)))
    frames = build_frames(path, SAG)
    assert len(frames.s) == len(frames.points) == len(frames.up)
    assert frames.s[0] == 0.0
    assert frames.s[-1] == pytest.approx(path.length, abs=1e-9)
    for i in (0, len(frames.s) // 2, -1):
        assert frames.points[i] == pytest.approx(path.point(float(frames.s[i])),
                                                 abs=1e-12)
