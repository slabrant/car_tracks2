"""The edge unit, and the profile built from two of them. docs/SPEC.md §2.

Pure Python + numpy. Never imports bpy.

The edge unit is the primitive: one rail plus its share of the deck, the `⊢`.
A two-port piece is two of them mirrored, which is what makes the familiar
I-beam profile. A junction is N of them rotated (§5). Nothing else in the
system is allowed to write the cross-section out as literal numbers.
"""

from __future__ import annotations

from dataclasses import dataclass

from .config import Body
from .mesh import Pt2, is_simple, shoelace

PROFILE_VERTS = 12
"""Fixed for every station. sweep.py depends on it (§4.3)."""


@dataclass(frozen=True)
class EdgeUnit:
    """One rail plus its deck stem, on one side of the centreline.

    ``side`` is +1 for the `+X` edge and -1 for the `-X` edge.
    """

    body: Body
    side: int

    def __post_init__(self) -> None:
        if self.side not in (1, -1):
            raise ValueError("side must be +1 or -1")

    def points(self) -> list[Pt2]:
        """The six outline points this edge unit contributes to the profile.

        Ordered from the deck-top end to the deck-bottom end, so that the two
        units concatenate into the closed profile without any seam handling.
        """
        b = self.body
        s = float(self.side)
        return [
            (s * b.rail_inner, +b.half_deck),    # inner face, deck top
            (s * b.rail_inner, +b.half_height),  # inner face, rail top
            (s * b.half_width, +b.half_height),  # outer face, rail top
            (s * b.half_width, -b.half_height),  # outer face, rail bottom
            (s * b.rail_inner, -b.half_height),  # inner face, rail bottom
            (s * b.rail_inner, -b.half_deck),    # inner face, deck bottom
        ]


def profile(body: Body) -> list[Pt2]:
    """The closed 12-vertex cross-section in the XZ plane, §2.1.

    Ordered CCW as seen from `+Y`, which in raw (x, z) parameter order is
    clockwise. sweep.py relies on that convention for its winding.
    """
    body.validate()
    plus = EdgeUnit(body, +1).points()
    minus = list(reversed(EdgeUnit(body, -1).points()))

    # `minus` now runs deck-bottom to deck-top on the -X side. Rotate so the
    # profile starts at the outer top-left corner, matching §2.1's listing.
    pts = minus[3:] + plus + minus[:3]

    if len(pts) != PROFILE_VERTS:
        raise AssertionError(f"profile must have {PROFILE_VERTS} vertices")
    if shoelace(pts) >= 0:
        raise AssertionError("profile must be CW in (x, z), i.e. CCW from +Y")
    if not is_simple(pts):
        raise AssertionError("profile self-intersects")
    return pts


def profile_area(body: Body) -> float:
    """Cross-sectional area, mm². Positive."""
    return abs(shoelace(profile(body)))
