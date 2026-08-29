"""docs/SPEC.md §9.18–9.23: the connector.

These are the tests the earlier attempts did not have. Gendered connectors and
joints that did not mate would all have failed here in under a second.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from parts import CATALOGUE, HUBS, PATHS, build, port_frames, straight
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


def _notches():
    return {label: mesh.bounds() for label, mesh in cuts(DEFAULT)
            if label.startswith("notch")}


SPLIT = BODY.deck_mid
"""One flat plane, and it lies inside the deck. §6.1."""


def _seams():
    """Every x where the handedness changes, and so a clearance slot opens."""
    from trackcore.connector import columns
    return [hi for hi, *_ in [(c[1],) for c in columns(DEFAULT)]][:-1]


def _removed(x: float, z: float, y: float = 1.0) -> bool:
    """Is this point of the port face cut away? The tabs are the complement."""
    for _label, (lo, hi) in _notches().items():
        if (lo[0] <= x <= hi[0] and lo[1] <= y <= hi[1] and lo[2] <= z <= hi[2]):
            return True
    return False


def _in_section(x: float, z: float) -> bool:
    if abs(x) > BODY.half_width:
        return False
    if abs(x) >= BODY.rail_inner:
        return abs(z) <= BODY.half_height              # rail
    return BODY.deck_bottom <= z <= BODY.deck_top      # deck


def _sample_points(margin: float = 0.25):
    """Section points well clear of every clearance band, so a hit or a miss
    is about the design and not about which side of a 0.15 mm slot we landed."""
    for x in np.linspace(-BODY.half_width + 0.05, BODY.half_width - 0.05, 181):
        if any(abs(x - s) < margin for s in _seams()):
            continue
        for z in np.linspace(BODY.deck_bottom + 0.05, BODY.half_height - 0.05, 61):
            if abs(z - SPLIT) < margin or not _in_section(float(x), float(z)):
                continue
            yield float(x), float(z)


def test_the_port_is_genderless():
    """The one property the whole joint rests on, tested directly.

    Two ports meet under `MATE`, a 180° turn about the shared up axis, which
    sends x to -x and leaves z alone. So the mate's material at `(x, z)` is
    ours at `(-x, z)`, and for tab to meet notch everywhere the pattern must
    satisfy `kept(x, z) == removed(-x, z)`.

    Sampled over the section rather than argued from the tools' bounding
    boxes — a design can be odd in x on paper and lose it in the arithmetic,
    and this notices either way. It is also indifferent to how many columns
    there are, which the bounds arithmetic was not.
    """
    checked = 0
    for x, z in _sample_points():
        if not _in_section(-x, z):
            continue
        checked += 1
        assert _removed(x, z) != _removed(-x, z), (
            f"({x:.2f}, {z:.2f}) and its mirror are both "
            f"{'cut away' if _removed(x, z) else 'kept'}; the port would meet "
            f"{'a void' if _removed(x, z) else 'itself'} there"
        )
    assert checked > 2000, "not enough of the section was sampled"


def test_the_columns_alternate_all_the_way_across():
    """§6.1. Up, down, up, down, across the whole section.

    The count is a dial now, and what has to hold at any setting is the
    alternation, since that is what makes the pattern odd in x and the part
    mate with its own twin. The seam count follows: one fewer than the columns.
    """
    from trackcore.connector import columns
    expected = CONN.column_count
    sides = [keeps_above for _lo, _hi, keeps_above in columns(DEFAULT)]
    assert len(sides) == expected
    assert all(a != b for a, b in zip(sides, sides[1:])), sides

    # and the same read off the geometry rather than the model
    z = BODY.deck_bottom + 0.2
    xs = np.linspace(-BODY.half_width + 0.05, BODY.half_width - 0.05, 4001)
    states = [_removed(float(x), z) for x in xs]
    changes = sum(1 for a, b in zip(states, states[1:]) if a != b)
    assert changes == expected - 1, (
        f"expected {expected - 1} seams across the section, saw {changes}"
    )


def test_an_odd_column_count_is_refused():
    """An odd count cannot alternate and stay odd in x, so the part would stop
    mating with its own twin. Caught in config, before any geometry."""
    from trackcore.config import Connector
    for bad in (1, 3, 5):
        with pytest.raises(ValueError, match="odd in x"):
            Connector(column_count=bad).validate()
    Connector(column_count=4).validate()


def test_no_seam_lands_near_a_rail_root():
    """The rails are part of the teeth now, so nothing is cut out there — but
    a column count that put a seam near a rail root would bring the old
    `curve_45` failure back, and `validate` refuses it rather than trusting the
    default to stay lucky."""
    from trackcore.config import Connector, TrackConfig
    from trackcore.connector import columns, root_inset

    keep_out = root_inset(DEFAULT)
    for _lo, hi, _above in columns(DEFAULT)[:-1]:
        assert abs(abs(hi) - BODY.rail_inner) >= keep_out, hi

    # 22 columns puts a seam at 10.909, a fifth of a millimetre off the root
    with pytest.raises(ValueError, match="rail root"):
        connector.validate(TrackConfig(connector=Connector(column_count=22)))


def test_a_rail_column_carries_no_deck():
    """What the split's outermost seam is for.

    A rail column is pure rail: it runs the same way as the deck column beside
    it, so the join between them needs no clearance and there is no seam at the
    rail root. That matters twice over — the road has no slot where a wheel
    runs closest to the rail, and no cut plane grazes the concave corner that
    runs the length of the piece, which is what broke `curve_45`.
    """
    z = BODY.deck_bottom + 0.2                  # inside the deck
    for side in (+1.0, -1.0):
        root = side * BODY.rail_inner
        inboard = _removed(root - side * 0.3, z)
        outboard = _removed(root + side * 0.3, z)
        assert inboard == outboard, (
            f"handedness changes at the rail root x={root:.2f}; the rail would "
            f"take a strip of deck and the road would carry a slot there"
        )


def test_the_split_lies_inside_the_deck_so_the_deck_laps():
    """The bug this whole design exists to fix.

    The first U-channel joint split at mid-height. On a U the deck lies wholly
    below mid-height, so that plane never touched it: the deck halves sat side
    by side sharing only a vertical face, and every bit of resistance to a
    vertical load came from the two rail laps. It mated cleanly, passed every
    rule in §7, and would have hinged apart under a car.
    """
    assert BODY.deck_bottom < SPLIT < BODY.deck_top
    z_lo, z_hi = SPLIT - 0.2, SPLIT + 0.2
    for x in np.linspace(-BODY.rail_inner + 0.4, BODY.rail_inner - 0.4, 97):
        x = float(x)
        if any(abs(x - s) < 0.3 for s in _seams()):
            continue
        assert _removed(x, z_lo) != _removed(x, z_hi), (
            f"at x={x:.2f} the deck is not lapped: the same piece keeps both "
            f"sides of the split"
        )


def test_the_deck_carries_most_of_the_vertical_bearing():
    """Not merely lapped — lapped over most of the width."""
    z_lo, z_hi = SPLIT - 0.2, SPLIT + 0.2
    xs = np.linspace(-BODY.half_width + 0.05, BODY.half_width - 0.05, 4001)
    step = float(xs[1] - xs[0])
    deck = sum(step for x in xs if abs(x) < BODY.rail_inner
               and _removed(float(x), z_lo) != _removed(float(x), z_hi))
    rail = sum(step for x in xs if abs(x) >= BODY.rail_inner
               and _removed(float(x), z_lo) != _removed(float(x), z_hi))
    assert deck > 4.0 * rail, (
        f"deck laps {deck:.2f} mm of width against the rails' {rail:.2f}; the "
        f"deck should dominate"
    )
    assert deck + rail > 0.85 * 2.0 * BODY.rail_inner


def test_nothing_is_glued_on_but_the_detents():
    """§6.6. Tabs are swept; only the ribs are added solids."""
    assert all(label.startswith("rib_") for label, _ in additions(DEFAULT))
    assert additions(DEFAULT), "there should be at least one rib"


def test_the_notches_reach_past_the_port_plane_to_trim_the_extension():
    """The body is swept `lap_length` long; these cuts are what shape the tab."""
    for label, bounds in _notches().items():
        assert bounds[1][1] >= CONN.lap_length, label


def test_tabs_protrude_exactly_one_lap_length():
    for _name, mesh in additions(DEFAULT):
        assert mesh.bounds()[1][1] <= CONN.lap_length + TOL


def test_every_cut_is_taken_one_clearance_deeper_than_the_tab_is_long():
    """So a tab never bottoms out before the joint closes, §6.2."""
    for name, mesh in cuts(DEFAULT):
        if name.startswith("groove"):
            continue
        assert mesh.bounds()[0][1] == pytest.approx(
            -(CONN.lap_length + CONN.fit_clearance), abs=TOL), name


def test_every_seam_is_exactly_one_clearance_wide():
    """One wherever the handedness changes. Each is a slot down the deck, and
    each runs along the direction of travel so a wheel rolls parallel to it
    rather than across it."""
    from trackcore.connector import deck_column
    z = BODY.deck_bottom + 0.2
    xs = np.linspace(-BODY.rail_inner, BODY.rail_inner, 40001)
    step = float(xs[1] - xs[0])
    # a seam is where *both* halves are cut away, so nothing is left there
    gap = sum(step for x in xs
              if _removed(float(x), z) and _removed(float(x), z + 1.0))
    assert gap == pytest.approx(len(_seams()) * CONN.fit_clearance,
                                abs=6.0 * step)
    assert deck_column(DEFAULT) > 2.0 * CONN.fit_clearance


# -- 9.19 --------------------------------------------------------------------


def _boxes(group, skip=("rib_",)):
    return [(label, *mesh.bounds()) for label, mesh in group
            if not any(label.startswith(s) for s in skip)]


def _overlap(a_lo, a_hi, b_lo, b_hi) -> float:
    return min(min(a_hi[i], b_hi[i]) - max(a_lo[i], b_lo[i]) for i in range(3))


def test_the_lap_faces_clear_each_other_by_exactly_one_clearance():
    """We keep what is above the split, the mate what is below it, so the two
    lap faces are exactly one clearance apart."""
    above = min(lo[2] for label, (lo, _hi) in _notches().items()
                if lo[2] > SPLIT - 1.0 and lo[2] > BODY.deck_bottom)
    below = max(hi[2] for _label, (_lo, hi) in _notches().items()
                if hi[2] < SPLIT + 1.0)
    assert below - above == pytest.approx(CONN.fit_clearance, abs=1e-12)


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


def _ribs():
    return dict(additions(DEFAULT))


def _grooves():
    return {label: mesh for label, mesh in cuts(DEFAULT)
            if label.startswith("groove")}


def test_the_rib_seats_inside_the_partner_groove():
    """Every rib we present must land inside a groove the mate offers."""
    face = SPLIT - CONN.fit_clearance / 2.0
    apex = face + CONN.detent_height
    assert apex > SPLIT + CONN.fit_clearance / 2.0, (
        "the rib must reach past the partner's lap face")

    for label, rib_mesh in _ribs().items():
        index = label.rsplit("_", 1)[1]
        partner = _grooves()[f"groove_nx_{index}"].transformed(MATE)
        rib = _yz(rib_mesh)
        groove = _yz(partner)
        for z in np.linspace(face, apex, 12)[1:]:
            rib_span = _span_at_z(rib, float(z))
            if rib_span is None:
                continue
            groove_span = _span_at_z(groove, float(z))
            assert groove_span is not None, f"{label} has no groove at z={z:.3f}"
            assert groove_span[0] <= rib_span[0] + TOL
            assert groove_span[1] >= rib_span[1] - TOL


def test_no_rib_meets_a_rib():
    """Ribs are all on one rail and grooves all on the other, so under MATE a
    rib can only ever arrive opposite a groove. §6.3's longitudinal offset then
    keeps our own rib clear of the one coming the other way."""
    ours = [mesh.bounds() for mesh in _ribs().values()]
    theirs = [mesh.transformed(MATE).bounds() for mesh in _ribs().values()]
    for a in ours:
        for b in theirs:
            assert _overlap(*a, *b) <= TOL


def test_a_rail_gets_two_detents_because_the_split_made_them_unlike():
    """`Connector.detent_spacing`, and why it exists.

    The split plane sits in the deck, so one rail keeps the thin sliver below
    it and the other the tall part above. A groove can only be sunk into the
    tall one, so ribs go on one rail and grooves on the other — half the
    engagements of a joint that could put one of each on both. Two of each buys
    them back.
    """
    assert len(_ribs()) == 2
    assert len(_grooves()) == 2
    assert len(CONN.detent_offsets) == 2
    near, far = CONN.detent_offsets
    assert near < CONN.detent_offset < far
    assert far - near == pytest.approx(CONN.detent_spacing * CONN.lap_length)


def test_the_return_face_is_steeper_than_the_lead_in():
    """Easy to push together, hard to pull apart, §6.3."""
    rib = _yz(next(iter(_ribs().values())))
    apex = max(rib, key=lambda p: p[1])
    base = [p for p in rib if p is not apex]
    lead = max(base, key=lambda p: p[0])[0] - apex[0]
    back = apex[0] - min(base, key=lambda p: p[0])[0]
    assert lead > back


def test_the_detent_base_is_buried_well_behind_the_lap_face():
    """Sunk only a hair, the flanks cross the lap plane beside the base corners
    and the boolean turns that into slivers on irrational port angles."""
    rib = _yz(next(iter(_ribs().values())))
    base_z = min(z for _y, z in rib)
    assert (SPLIT - CONN.fit_clearance / 2.0) - base_z >= 0.1


# -- 9.21 --------------------------------------------------------------------


@pytest.mark.parametrize("name", CATALOGUE)
def test_every_port_of_every_part_carries_identical_geometry(name):
    """Transformed back into its own frame, every port is the same object."""
    piece = build(name, DEFAULT)
    canonical_cuts = _pool([m for _l, m in cuts(DEFAULT)])
    canonical_adds = _pool([m for _l, m in additions(DEFAULT)])

    matrices = port_frames(name)
    assert len(piece.cuts) == CUTS_PER_PORT * len(matrices)
    # the connector's own additions come first; a brace, if the part has one,
    # is unioned on after them and is not port geometry
    from parts import genus
    connector_adds = ADDS_PER_PORT * len(matrices)
    assert len(piece.additions) == connector_adds + genus(name)

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
    # derived, not a constant: what counts as too short is two laps plus two
    # clearances, so a literal here goes stale the moment the lap changes —
    # which it did, and this test passed a 12 mm piece as "too short" for a
    # joint that by then needed 6.3
    reach = 2.0 * (CONN.lap_length + CONN.fit_clearance)
    with pytest.raises(ValueError, match="notches would meet"):
        build("straight_full", DEFAULT, length=reach - 0.1)


def test_a_rib_whose_buried_base_runs_off_the_tab_is_refused():
    """The overhang guard has to measure the whole rib, base included.

    A detent is a triangle whose apex protrudes and whose base is buried
    `DETENT_SINK` behind the lap face, so material reaches
    `(height + DETENT_SINK) / tan(lead)` ahead of the apex — 0.43 mm further
    than the part a mate can feel. Measuring only the proud part passed a
    config whose ribs hung 0.14 mm past the tab tip, where they land on the
    floor of the mate's notch before the lap faces seat.

    This config is exactly that case: proud footprint inside the tab, whole
    rib not.
    """
    from trackcore.config import Connector, TrackConfig
    from trackcore.connector import DETENT_SINK

    lap, height = 3.0, 0.35
    offset = 2.1
    tan_lead = math.tan(math.radians(30.0))
    assert offset + height / tan_lead < lap, "the proud tip fits; that was the trap"
    assert offset + (height + DETENT_SINK) / tan_lead > lap

    with pytest.raises(ValueError, match="overhang"):
        connector.validate(TrackConfig(connector=Connector(
            lap_length=lap, detent_offset=offset, detent_height=height,
            detent_spacing=0.0)))


def test_connector_config_guards_fire():
    from trackcore.config import Connector, TrackConfig
    with pytest.raises(ValueError, match="overhang"):
        connector.validate(TrackConfig(connector=Connector(
            detent_offset=CONN.lap_length + 1.0)))
    # a detent taller than the rail can hold must be refused; which guard names
    # it depends on the lap, so this asserts only that one of them does
    with pytest.raises(ValueError):
        connector.validate(TrackConfig(connector=Connector(detent_height=2.25)))
