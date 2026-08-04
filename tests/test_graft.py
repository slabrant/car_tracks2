"""docs/SPEC.md §9.27–9.32: Construction C, supports."""

from __future__ import annotations

import math

import numpy as np
import pytest

from parts import (CATALOGUE, DECLARES_UP, GRAFTED, GRAFTS, GRID, HUBS, PATHS,
                   build, declares_up, pier, port_frames, support)
from trackcore import DEFAULT, MATE, Graft, GraftInvalid, check, leg_length
from trackcore.connector import additions, cuts

BODY = DEFAULT.body
CONN = DEFAULT.connector
TOL = 1e-9


def _pool(meshes):
    return {tuple(np.round(v, 7)) for mesh in meshes for v in mesh.verts}


def _flip(angle: float = math.pi / 2.0) -> np.ndarray:
    n = np.array([math.cos(angle), math.sin(angle), 0.0])
    m = np.eye(4)
    m[:3, :3] = 2.0 * np.outer(n, n) - np.eye(3)
    return m


# -- 9.27 --------------------------------------------------------------------


def test_a_support_has_three_ports():
    assert len(port_frames("support")) == 3


def test_a_supports_ports_are_the_same_object_as_every_other_port():
    """Covered generally by §9.21; asserted here because it is the whole reason
    a leg can be an ordinary straight."""
    piece = build("support", DEFAULT)
    canonical = _pool([m for _l, m in additions(DEFAULT)])
    per_port = len(additions(DEFAULT))
    for index, matrix in enumerate(port_frames("support")):
        placed = piece.additions[index * per_port:(index + 1) * per_port]
        back = [m.transformed(np.linalg.inv(matrix)) for m in placed]
        assert _pool(back) == canonical


# -- 9.28 --------------------------------------------------------------------


def test_the_stub_port_points_straight_down_and_sits_on_the_centreline():
    frame = port_frames("support")[2]
    assert frame[:3, 1] == pytest.approx([0.0, 0.0, -1.0], abs=TOL)
    assert frame[0, 3] == pytest.approx(0.0, abs=TOL)
    assert frame[1, 3] == pytest.approx(support().length / 2.0, abs=TOL)
    assert frame[2, 3] == pytest.approx(-support().depth, abs=TOL)


def test_the_stub_frame_is_right_handed_like_every_other_port():
    frame = port_frames("support")[2]
    across, forward, up = frame[:3, 0], frame[:3, 1], frame[:3, 2]
    assert np.cross(forward, up) == pytest.approx(across, abs=TOL)
    for a, b in ((across, forward), (forward, up), (up, across)):
        assert float(a @ b) == pytest.approx(0.0, abs=TOL)


def test_the_stub_never_rises_into_the_driving_channel():
    """A stub poking above the deck would stop a car dead."""
    _body, stub = support().solids(DEFAULT)
    assert stub.bounds()[1][2] <= BODY.half_deck + TOL


def test_the_stub_flanges_land_on_the_body_rails():
    """§5.5: both occupy |x| in [rail_inner, half_width], so the section simply
    continues downward and the union has volume to work with."""
    body, stub = support().solids(DEFAULT)
    assert stub.bounds()[0][0] == pytest.approx(-BODY.half_width, abs=TOL)
    assert stub.bounds()[1][0] == pytest.approx(+BODY.half_width, abs=TOL)
    assert stub.bounds()[0][2] < body.bounds()[0][2], "the stub must reach below"
    overlap = body.bounds()[1][2] - stub.bounds()[0][2]
    assert overlap > 0, "stub and body must overlap by volume, not merely touch"


# -- 9.31, and the arithmetic that makes a bridge stand ----------------------


def test_a_support_turned_over_rests_on_a_single_plane():
    """It is its own foot. No separate part, and no flag saying so."""
    piece = build("support", DEFAULT, connectors=False)
    flipped = [m.transformed(_flip()) for m in piece.solids]
    lows = [m.bounds()[0][2] for m in flipped]
    assert min(lows) == pytest.approx(-BODY.half_height, abs=TOL)
    # and the stub now points up
    assert max(m.bounds()[1][2] for m in flipped) == pytest.approx(
        support().depth, abs=TOL)


def test_a_foot_presents_its_through_ports_at_ordinary_track_height():
    """So a leg can rise straight out of ground-level track."""
    assert support().foot_base_height(DEFAULT) == pytest.approx(
        BODY.half_height, abs=TOL)


def test_a_foot_a_leg_and_a_support_reach_exactly_the_ramp_deck_height():
    """The claim Phase 5 exists to make. Ground to bridge deck is the foot's own
    half height, its stub depth, the leg, then the support's stub depth — and
    that has to land where the ramp puts you, or bridges do not close."""
    depth = support().depth
    leg = leg_length(GRID.deck_height, depth)
    stack = BODY.half_height + depth + leg + depth
    ramp_top = BODY.half_height + GRID.deck_height
    assert stack == pytest.approx(ramp_top, abs=1e-9)


def test_the_leg_is_an_ordinary_catalogue_straight():
    """Not a special part. That reuse is the payoff of §6.1."""
    leg = leg_length(GRID.deck_height, support().depth)
    assert leg == pytest.approx(GRID.quarter, abs=TOL)
    assert PATHS["straight_quarter"]().length == pytest.approx(leg, abs=TOL)
    assert pier(leg).length == pytest.approx(leg, abs=TOL)


# -- 9.32 --------------------------------------------------------------------


def _flip_through(centre: np.ndarray, angle: float) -> np.ndarray:
    """180° rotation about the horizontal axis at ``angle`` through ``centre``."""
    n = np.array([math.cos(angle), math.sin(angle), 0.0])
    rotation = 2.0 * np.outer(n, n) - np.eye(3)
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = centre - rotation @ centre
    return matrix


def _has_a_flip_axis(piece) -> bool:
    """Is there any horizontal axis this piece can be turned over about?

    Searched rather than assumed, because the axis is not the long axis in
    general: a straight turns about its centreline, a ramp about the cross axis
    through its midpoint, and a **curve about the bisector through its arc
    centre**. Assuming the long axis marks every curve as unflippable, which is
    wrong and was the first version of this test.
    """
    verts = np.vstack([m.verts for m in piece.solids])
    # the axis must pass through the vertex centroid: a symmetry maps the point
    # set to itself, so it fixes the centroid, and a 180° rotation fixes only
    # its own axis. The bounding-box centre is *not* good enough — a 45° arc's
    # box is not centred on its symmetry axis.
    centre = verts.mean(axis=0)
    for degrees in np.arange(0.0, 180.0, 2.5):
        flip = _flip_through(centre, math.radians(float(degrees)))
        moved = (verts - centre) @ flip[:3, :3].T + centre
        # matched by distance, not by rounded equality: the residual here is
        # 1e-14 and a rounded comparison lets a single vertex straddle a
        # boundary and fail an exact symmetry
        gap = np.linalg.norm(moved[:, None, :] - verts[None, :, :],
                             axis=-1).min(axis=1).max()
        if gap < 1e-6:
            return True
    return False


def test_the_parts_with_no_flip_axis_are_exactly_those_that_declare_an_up():
    """Derived from the construction, not declared. A part that acquired a
    hand-written 'not flippable' flag would be a place for a special case to
    hide.

    Note what this does *not* claim: that a piece turns about its long axis. It
    claims only that some horizontal axis exists, which is what "you can turn it
    over" actually means.
    """
    unflippable = {name for name in CATALOGUE
                   if not _has_a_flip_axis(build(name, DEFAULT, connectors=False))}
    assert unflippable == DECLARES_UP


def test_two_kinds_of_part_declare_an_up_direction_and_both_earn_it():
    """A graft's stub would point at the ceiling; a banked curve turned over
    leans the wrong way through the turn. Gravity decided both."""
    assert DECLARES_UP == GRAFTED | {"curve_90_banked"}
    assert declares_up("support") and declares_up("curve_90_banked")
    assert not declares_up("curve_90") and not declares_up("x_rounded")


def test_no_part_carries_a_not_flippable_flag():
    import pathlib
    repo = pathlib.Path(__file__).resolve().parent.parent
    for source in sorted((repo / "trackcore").glob("*.py")) + \
            sorted((repo / "parts").glob("*.py")):
        text = source.read_text().lower()
        assert "flippable=" not in text and "not_flippable" not in text


# -- guards ------------------------------------------------------------------


def test_a_stub_too_shallow_for_a_joint_is_rejected():
    with pytest.raises(GraftInvalid, match="no room for a joint"):
        Graft(length=GRID.half, depth=4.0).validate(DEFAULT)


def test_a_support_too_short_for_its_own_end_joints_is_rejected():
    with pytest.raises(GraftInvalid, match="from its own end joints"):
        Graft(length=20.0, depth=GRID.support_depth).validate(DEFAULT)


def test_a_support_is_in_the_catalogue_and_is_grafted():
    assert "support" in CATALOGUE
    assert set(GRAFTS) == GRAFTED
    assert "support" not in PATHS and "support" not in HUBS
