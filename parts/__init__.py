"""Part definitions. docs/SPEC.md §8.

A part is data, not code. Construction A parts are a path; Construction B parts
are an arm layout. If adding a part requires editing `sweep.py` or `hub.py`,
the construction is wrong.
"""

from __future__ import annotations

import math
from typing import Callable

from trackcore import DEFAULT, Arc, Hub, Line, Path, Piece, Ramp, TrackConfig, sweep

PathBuilder = Callable[..., Path]
HubBuilder = Callable[..., Hub]

ROUNDED = 12.0
"""Default fillet radius, mm. A car turning through the corner follows it."""


# -- Construction A ----------------------------------------------------------


def straight(length: float = 84.0) -> Path:
    return Path.chain(Line(length))


def curve(radius: float = 100.0, angle_deg: float = 90.0,
          bank_deg: float = 0.0) -> Path:
    return Path.chain(Arc(radius=radius,
                          angle=math.radians(angle_deg),
                          bank=math.radians(bank_deg)))


def ramp(run: float = 84.0, rise: float = 34.0) -> Path:
    return Path.chain(Ramp(run=run, rise=rise))


def s_bend(radius: float = 100.0, angle_deg: float = 45.0,
           lead: float = 20.0) -> Path:
    """A straight, a left turn, a right turn and a straight.

    Not a standard part; it exists because it exercises every frame transition
    a track can contain, which is what test 9.4 needs.
    """
    a = math.radians(angle_deg)
    return Path.chain(Line(lead), Arc(radius, a), Arc(radius, -a), Line(lead))


# -- Construction B ----------------------------------------------------------


def x_junction(corner_radius: float = 0.0) -> Hub:
    return Hub.auto([0.0, 90.0, 180.0, 270.0], corner_radius)


def t_junction(corner_radius: float = 0.0) -> Hub:
    return Hub.auto([0.0, 90.0, 180.0], corner_radius)


def y_junction(corner_radius: float = 0.0) -> Hub:
    """Three arms at 120°, so all three ports are interchangeable."""
    return Hub.auto([90.0, 210.0, 330.0], corner_radius)


def x_rounded(corner_radius: float = ROUNDED) -> Hub:
    return x_junction(corner_radius)


def t_rounded(corner_radius: float = ROUNDED) -> Hub:
    return t_junction(corner_radius)


def y_rounded(corner_radius: float = ROUNDED) -> Hub:
    return y_junction(corner_radius)


# -- registry ----------------------------------------------------------------


PATHS: dict[str, PathBuilder] = {
    "straight": straight,
    "curve": curve,
    "ramp": ramp,
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

CATALOGUE: list[str] = sorted(PATHS) + sorted(HUBS)


def build(name: str, config: TrackConfig = DEFAULT, **kwargs) -> Piece:
    """Build a part by name, whichever construction it uses."""
    if name in PATHS:
        return Piece(name=name, solids=(sweep(PATHS[name](**kwargs), config),))
    if name in HUBS:
        return HUBS[name](**kwargs).piece(name, config)
    raise KeyError(f"unknown part {name!r}; have {CATALOGUE}")
