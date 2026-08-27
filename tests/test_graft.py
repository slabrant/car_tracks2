"""docs/SPEC.md §9.27–9.32: Construction C, supports."""

from __future__ import annotations

import math

import numpy as np
import pytest

from parts import (CATALOGUE, GRAFTED, GRAFTS, GRID, HUBS, PATHS, build,
                   pier, port_frames, support)
from trackcore import DEFAULT, MATE, Graft, GraftInvalid, check, leg_length
from trackcore.connector import additions, cuts

BODY = DEFAULT.body
CONN = DEFAULT.connector
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
    assert stub.bounds()[1][2] <= BODY.deck_top + TOL


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


def test_a_support_turned_over_still_makes_a_foot():
    """A support inverted rests on its rail tops with the stub pointing up.

    On the I-section it rested on rails that were there anyway. On a U it rests
    on the rail *tops*, which is a narrower base — worth knowing when Phase 6
    decides how a foot actually sits.
    """
    piece = build("support", DEFAULT, connectors=False)
    flipped = [m.transformed(_flip()) for m in piece.solids]
    assert min(m.bounds()[0][2] for m in flipped) == pytest.approx(
        -BODY.half_height, abs=TOL)
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


def test_no_piece_can_be_turned_over_any_more():
    """A U-channel has a right way up by construction: inverted, the channel
    faces the floor. On the old I-section most pieces were congruent to
    themselves flipped and grafts were the exception; now the whole distinction
    is gone, which is why `declares_up` went with it."""
    for name in ("straight_full", "curve_90", "x_junction", "support"):
        piece = build(name, DEFAULT, connectors=False)
        flipped = [m.transformed(_flip()) for m in piece.solids]
        assert _pool(piece.solids) != _pool(flipped)


def test_no_part_carries_a_not_flippable_flag():
    import pathlib
    repo = pathlib.Path(__file__).resolve().parent.parent
    for source in sorted((repo / "trackcore").glob("*.py")) + \
            sorted((repo / "parts").glob("*.py")):
        text = source.read_text().lower()
        assert "flippable=" not in text and "not_flippable" not in text


# -- guards ------------------------------------------------------------------


def test_a_stub_too_shallow_for_a_joint_is_rejected():
    shallow = CONN.lap_length + CONN.fit_clearance - 0.1   # derived; see below
    with pytest.raises(GraftInvalid, match="no room for a joint"):
        Graft(length=GRID.half, depth=shallow).validate(DEFAULT)


def test_a_support_too_short_for_its_own_end_joints_is_rejected():
    too_short = 2.0 * (BODY.half_height + CONN.lap_length)
    with pytest.raises(GraftInvalid, match="from its own end joints"):
        Graft(length=too_short - 1.0, depth=GRID.support_depth).validate(DEFAULT)


def test_a_support_is_in_the_catalogue_and_is_grafted():
    assert "support" in CATALOGUE
    assert set(GRAFTS) == GRAFTED
    assert "support" not in PATHS and "support" not in HUBS
