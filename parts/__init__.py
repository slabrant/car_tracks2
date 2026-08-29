"""The part set. docs/SPEC.md §8, §10 Phase 4.

A part is data, not code. Everything here is a path or an arm layout; if adding
one required editing `sweep.py`, `hub.py` or `connector.py`, an earlier phase
was wrong. Nothing in this file computes geometry.

The set is built on a **layout grid** so that loops close. See `Grid`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import partial
from typing import Callable

from trackcore import (DEFAULT, Arc, Graft, Hub, Line, Loop, Path, Piece,
                       Ramp, TrackConfig, applied, port_matrices, sweep,
                       swept_with_ports)
from trackcore.brace import brace
from trackcore.connector import port_extension
from trackcore.connector import validate as validate_connector
from trackcore.mesh import translation
from trackcore.path import DEFAULT_LOOP_DRIFT, DEFAULT_PORT_CLEAR

PathBuilder = Callable[..., Path]
HubBuilder = Callable[..., Hub]


@dataclass(frozen=True)
class Grid:
    """The layout module. Change `module` and the whole set follows.

    Everything is a multiple of it, which is what makes a loop close instead of
    almost closing. A 90° curve of radius `module` advances exactly one module
    along each axis and turns a right angle, so it tiles with the straights. A
    junction whose arms reach half a module substitutes for one full straight in
    every direction it serves.

    45° curves do **not** land on the grid on their own — two of them make a
    90° and land where one would. That is the same bargain every sectional
    track system makes, and it is worth stating rather than discovering.

    A 120° Y cannot tile a square grid at all. It is in the set because a
    three-way with interchangeable ports is worth having, not because it fits.
    """

    module: float = 96.0
    deck_height: float = 48.0
    """How high a bridge deck sits. Half a module, so legs stack in modules."""

    ramp_modules: float = 2.0
    """How many modules a ramp takes to reach `deck_height`."""

    bank_deg: float = 10.0

    @property
    def half(self) -> float:
        return self.module / 2.0

    @property
    def quarter(self) -> float:
        return self.module / 4.0

    @property
    def fillet(self) -> float:
        """Junction corner radius, and the turn radius a car follows through it."""
        return self.quarter

    @property
    def support_depth(self) -> float:
        """How far a support's stub reaches below its deck.

        Chosen so a stack lands exactly on `deck_height`: the foot's stub, a
        quarter-module leg, and the support's stub. That is why the leg is an
        ordinary `straight_quarter` and not a special part.
        """
        return (self.deck_height - self.quarter) / 2.0


GRID = Grid()


# -- Construction A ----------------------------------------------------------


def straight(length: float = GRID.module) -> Path:
    return Path.chain(Line(length))


def curve(radius: float = GRID.module, angle_deg: float = 90.0,
          bank_deg: float = 0.0) -> Path:
    return Path.chain(Arc(radius=radius,
                          angle=math.radians(angle_deg),
                          bank=math.radians(bank_deg)))


def ramp(run: float = GRID.module * GRID.ramp_modules,
         rise: float = GRID.deck_height,
         lead: float = DEFAULT_PORT_CLEAR) -> Path:
    """A rise with a flat run at each end.

    The leads are not decoration. A smoothstep's vertical curvature is greatest
    at its ends — exactly where the connector reaches in — and the cut tools are
    flat boxes aligned to the port frame. Putting each port on a straight keeps
    the lap zones flat. `run` is the whole piece, so the set still tiles: the
    rise simply takes up what the leads leave.
    """
    return Path.chain(Line(lead), Ramp(run=run - 2.0 * lead, rise=rise),
                      Line(lead))


def loop(radius: float = GRID.half, drift: float = DEFAULT_LOOP_DRIFT,
         lead: float = DEFAULT_PORT_CLEAR) -> Path:
    """A vertical loop, on a flat lead at each end.

    Radius is half a module, so the loop stands one module tall and its
    circle is one module across — the same 48 mm that `curve_90_tight` turns
    through horizontally, stood on its edge.

    The leads are the same bargain the ramp makes, for the same reason: the
    loop's vertical curvature is greatest nowhere in particular — it is a
    circle, so it is the same everywhere — and the connector's cut tools are
    flat boxes. Without a flat lead they would bite into a section that has
    already pitched six degrees. `ramp` documents what that does.

    The piece advances `2 * lead` along the track as well as stepping across,
    so a run of loops walks diagonally rather than sideways. `Loop.close`
    cancels that — it is written and tested — and is deliberately not set here.
    Cancelling it puts both ports at the same station, which reads tidily and
    buys nothing: the two runs stay abreast either way, so it neither allows a
    weld nor changes any clearance. Leaving the offset in keeps the ports well
    apart, which is the more forgiving of the two when something at one port
    needs room.

    Unlike every other part in the set, this one does not lie flat and does
    not tile. Both are noted in the README; neither is fixable by geometry.
    """
    turn = Loop(radius=radius, drift=drift)
    return Path.chain(Line(lead), turn,
                      Line(lead, roll_offset=turn.twist))


def s_bend(radius: float = GRID.module, angle_deg: float = 45.0,
           lead: float = 20.0) -> Path:
    """Not a part. It exists because straight → left → right → straight is
    where a Frenet frame flips its normal, which test 9.4 relies on."""
    a = math.radians(angle_deg)
    return Path.chain(Line(lead), Arc(radius, a), Arc(radius, -a), Line(lead))


# -- Construction B ----------------------------------------------------------


def _hub(angles: list[float], corner_radius: float) -> Hub:
    return Hub.uniform(angles, GRID.half, corner_radius)


X_ARMS = [0.0, 90.0, 180.0, 270.0]
T_ARMS = [0.0, 90.0, 180.0]
Y_ARMS = [90.0, 210.0, 330.0]


def x_junction(corner_radius: float = 0.0) -> Hub:
    return _hub(X_ARMS, corner_radius)


def t_junction(corner_radius: float = 0.0) -> Hub:
    return _hub(T_ARMS, corner_radius)


def y_junction(corner_radius: float = 0.0) -> Hub:
    """Three arms at 120°, so all three ports are interchangeable."""
    return _hub(Y_ARMS, corner_radius)


def x_rounded(corner_radius: float = GRID.fillet) -> Hub:
    return x_junction(corner_radius)


def t_rounded(corner_radius: float = GRID.fillet) -> Hub:
    return t_junction(corner_radius)


def y_rounded(corner_radius: float = GRID.fillet) -> Hub:
    return y_junction(corner_radius)


# -- Construction C ----------------------------------------------------------


def support(length: float = GRID.half,
            depth: float = GRID.support_depth) -> Graft:
    """A short straight with a stub square to it, §5.5.

    Turned over it is its own foot, so there is no separate foot part.
    """
    return Graft(length=length, depth=depth)


def pier(height: float = GRID.quarter) -> Path:
    """A leg, cut to length.

    Deliberately just a straight. The port is genderless and identical
    everywhere, so a track piece stood on end already *is* a structural column;
    this name exists only so a layout reads as intended.
    """
    return straight(height)


# -- registry ----------------------------------------------------------------


PATHS: dict[str, PathBuilder] = {
    "straight_full": partial(straight, length=GRID.module),
    "straight_half": partial(straight, length=GRID.half),
    "straight_quarter": partial(straight, length=GRID.quarter),
    "curve_90": partial(curve, radius=GRID.module, angle_deg=90.0),
    "curve_45": partial(curve, radius=GRID.module, angle_deg=45.0),
    "curve_90_tight": partial(curve, radius=GRID.half, angle_deg=90.0),
    "curve_90_banked": partial(curve, radius=GRID.module, angle_deg=90.0,
                               bank_deg=GRID.bank_deg),
    "ramp": ramp,
    "loop": loop,
    "s_bend": s_bend,
}

HUBS: dict[str, HubBuilder] = {
    "x_junction": x_junction,
    "t_junction": t_junction,
    "y_junction": y_junction,
    "x_rounded": x_rounded,
    "t_rounded": t_rounded,
    "y_rounded": y_rounded,
}

GRAFTS: dict[str, Callable[..., Graft]] = {
    "support": support,
}

NOT_PARTS = {"s_bend"}
"""In PATHS because tests need them; not in the printable set."""

GRAFTED = set(GRAFTS)
"""Parts with a stub.

There used to be a companion set for "declares an up direction", because on the
old flippable I-section most pieces could be turned over and grafts and banked
curves could not. On a U-channel **every** piece has a right way up — turned
over, the channel faces the floor — so the distinction sorts nothing and is
gone.
"""


def declares_up(name: str) -> bool:
    """Does this part's geometry have a right way up? §5.6.

    Two things do, and both because gravity already decided: a **graft**, whose
    stub would otherwise point at the ceiling, and a **banked** curve, which
    turned over leans the wrong way through the turn.

    Derived, never declared. A part carrying a hand-written "not flippable"
    flag would be a place for a special case to hide.
    """
    if name in GRAFTS:
        return True
    if name in PATHS:
        return any(getattr(primitive, "bank", 0.0) != 0.0
                   for primitive in PATHS[name]().primitives)
    return False

CATALOGUE: list[str] = ([n for n in sorted(PATHS) if n not in NOT_PARTS]
                        + sorted(HUBS) + sorted(GRAFTS))




def _braces(path: Path, config: TrackConfig) -> list:
    """Every brace a path asks for, placed in the path's own frame.

    The straight either side of a primitive bounds how far its brace may
    reach: past the lead there is nothing alongside to tie to. See
    `trackcore.brace`.
    """
    placed = []
    primitives, transforms = path.primitives, path.transforms
    for index, (prim, matrix) in enumerate(zip(primitives, transforms)):
        neighbours = [primitives[k].length for k in (index - 1, index + 1)
                      if 0 <= k < len(primitives)]
        reach = min(neighbours) if neighbours else 0.0
        placed += [mesh.transformed(matrix)
                   for _label, mesh in brace(prim, config, reach)]
    return placed


def genus(name: str, config: TrackConfig = DEFAULT, **kwargs) -> int:
    """How many holes a finished part has. §7 needs telling.

    One for anything braced: tying two runs of the same swept body together
    closes a ring, and a ring is a torus. Nothing else in the set has a hole,
    and nothing should — an unexpected one is a tunnel a boolean bored by
    accident, which is exactly what the rule is for.

    Derived from the same `brace` call that builds the geometry, so the two
    cannot disagree.
    """
    if name not in PATHS:
        return 0
    return len(_braces(PATHS[name](**kwargs), config))


def port_frames(name: str, config: TrackConfig = DEFAULT, **kwargs) -> list:
    """The port frames of a part, in the order its connectors were applied."""
    if name in PATHS:
        return port_matrices(PATHS[name](**kwargs), config)
    if name in HUBS:
        return HUBS[name](**kwargs).port_matrices(config)
    if name in GRAFTS:
        return GRAFTS[name](**kwargs).port_matrices(config)
    raise KeyError(f"unknown part {name!r}; have {CATALOGUE}")


def build(name: str, config: TrackConfig = DEFAULT, connectors: bool = True,
          **kwargs) -> Piece:
    """Build a part by name, whichever construction it uses.

    The connector is the same object at every port of every part (§6), so it is
    applied here rather than inside either construction. Neither `sweep.py` nor
    `hub.py` knows what a joint looks like.
    """
    extend = port_extension(config) if connectors else 0.0

    if name in PATHS:
        path = PATHS[name](**kwargs)
        matrices = port_matrices(path, config)
        solids: tuple = ((swept_with_ports(path, config),) if extend
                         else (sweep(path, config),))
        reach = 2.0 * (config.connector.lap_length + config.connector.fit_clearance)
        if connectors and path.length <= reach:
            raise ValueError(
                f"{name} is {path.length:.1f} mm long but two joints need "
                f"{reach:.1f} mm; the notches would meet in the middle"
            )
    elif name in HUBS:
        hub = HUBS[name](**kwargs)
        matrices = hub.port_matrices(config)
        solids = tuple(hub.solids(config, extend=extend))
    elif name in GRAFTS:
        graft = GRAFTS[name](**kwargs)
        matrices = graft.port_matrices(config)
        solids = tuple(graft.solids(config, extend=extend))
    else:
        raise KeyError(f"unknown part {name!r}; have {CATALOGUE}")

    # Anything that needs tying to itself says so, and says it as a solid to be
    # unioned on at the end — after the port cuts, which is the only reason a
    # `loop`'s brace survives the notches it runs straight past. It is
    # structure, not joint, so a part built without connectors still gets it.
    braces = _braces(path, config) if name in PATHS else []

    if not connectors:
        return Piece(name=name, solids=solids, additions=tuple(braces))

    validate_connector(config)
    cuts, additions = applied(matrices, config)
    return Piece(name=name, solids=solids, cuts=tuple(cuts),
                 additions=tuple(additions) + tuple(braces))
