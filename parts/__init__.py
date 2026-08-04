"""Part definitions. docs/SPEC.md §8.

A part is path data, not code. If adding a part requires editing sweep.py, the
sweep is wrong.

Phase 1 ships flat-ended straights and curves. Ramps and banked curves are
Phase 4, and they add entries here and nothing else.
"""

from __future__ import annotations

import math
from typing import Callable

from trackcore import Arc, Line, Path, Ramp

Builder = Callable[..., Path]


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


CATALOGUE: dict[str, Builder] = {
    "straight": straight,
    "curve": curve,
    "ramp": ramp,
    "s_bend": s_bend,
}
