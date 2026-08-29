"""The loop's brace. docs/SPEC.md §4.1.

Pure Python + numpy. Never imports bpy.

A `loop` stands 100 mm tall on two runs of track 24 mm wide, passing each other
a millimetre apart at the bottom. Nothing ties them together, and a ring that
tall on a base that narrow folds sideways. This is the block that ties them.

**Why a brace and not a weld.** Bringing the two runs into contact — one track
width apart, rail face on rail face — would make a single 2.4 mm wall and need
no extra geometry at all. `path.DEFAULT_LOOP_DRIFT` sets out why it cannot be
had: the loop comes down parallel to its own entry, so welding the runs welds
them through both lap zones, and each port's cut tools reach past their own
rail far enough to gouge the run alongside. A brace is the same bracing without
that, because of *when* it happens.

**It is an addition, and that is the whole trick.** `Piece.stages` runs the
cuts before the additions, so a solid unioned on at the end cannot be cut by
the port tools — the same reason the detent ribs are additions rather than part
of the swept body. The brace is placed straight across the two runs without
regard for where the notches fall, and comes out whole. It also fills back the
few hundredths of a millimetre those tools skim off the far run's outer rail on
their way past.
"""

from __future__ import annotations

import numpy as np

from .config import DEFAULT, TrackConfig
from .connector import port_extension
from .mesh import MeshData, box
from .path import Loop

BITE = 0.2
"""How far the brace reaches into each run, mm.

A union wants overlap, not contact — two solids meeting exactly on a face are
the one thing an exact solver cannot be relied on to resolve, which this
project has now met four times. Two tenths is well inside the 1.2 mm rail it
merges with and far more than the drift wanders over the braced length.
"""


def brace(primitive, config: TrackConfig = DEFAULT,
          reach: float = 0.0) -> list[tuple[str, MeshData]]:
    """The brace for one primitive, in that primitive's own frame.

    Empty for anything that is not a `Loop`, so a caller can run it over every
    primitive of every path and let the geometry decide.

    ``reach`` is how far past each end of the loop there is straight lead to
    tie to — past that, the other run has stopped being track.

    The brace does not use all of it. It reaches **into** each run by `BITE` so
    the union has something to work with, and a mate's tab comes the other way
    through that same rail for `port_extension` past the port plane. Overlap
    them and the two pieces no longer mate: measured at 1.19 mm³ of shared
    solid before this was allowed for. So the lap comes off each end, which
    leaves `2 * (fit_clearance + 2.0)` of brace — the `+ 2.0` in
    `path.DEFAULT_PORT_CLEAR`, which is there to keep the port zone clear of
    whatever the middle of a piece is doing, being exactly what buys the room.
    """
    if not isinstance(primitive, Loop):
        return []

    clear = reach - port_extension(config)
    if clear <= 0.0:
        return []

    body = config.body
    inner = body.half_width - BITE            # into this run's rail
    outer = primitive.drift - inner           # into the other run's

    # The loop leaves at y = 0 and rejoins at y = -close, so the stretch where
    # both runs are down at the bottom runs between those, plus a lead either
    # side.
    near = -primitive.close - clear
    far = clear

    # Full section height. The two runs are a few tenths apart in z by the ends
    # of this span — the loop is already climbing — and a brace the whole height
    # of the section meets both wherever they are.
    lo = (inner, near, body.deck_bottom)
    hi = (outer, far, body.half_height)
    return [("loop_brace", box(lo, hi))]


def spans(config: TrackConfig = DEFAULT, drift: float = None) -> float:
    """The gap the brace closes, mm. For tests and for saying so out loud."""
    from .path import DEFAULT_LOOP_DRIFT

    drift = DEFAULT_LOOP_DRIFT if drift is None else drift
    return drift - config.body.width_outer


def volume(primitive, config: TrackConfig = DEFAULT, reach: float = 0.0) -> float:
    """What the brace adds, mm³, before it is merged into the runs."""
    made = brace(primitive, config, reach)
    if not made:
        return 0.0
    lo, hi = made[0][1].bounds()
    return float(np.prod(hi - lo))
