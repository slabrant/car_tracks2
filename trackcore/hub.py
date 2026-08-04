"""Construction B: junctions. docs/SPEC.md §5.

Pure Python + numpy. Never imports bpy.

A junction is N straight arms meeting at a hub. Because junctions are level and
unbanked the whole piece is prismatic in z — three flat slabs, with the top and
bottom rail slabs identical, so flip symmetry is automatic.

One builder covers Y, T, X and any arm count. There is no per-junction-type
code, because that is how v1 ended up with four generators drifting apart.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config import DEFAULT, TrackConfig
from .mesh import MeshData, Piece, Pt2, prism, shoelace

TAU = 2.0 * math.pi
COLLINEAR_TOL = 1e-9


class HubInvalid(ValueError):
    """An arm layout that cannot produce a closed, flippable hub."""


def direction(angle: float) -> np.ndarray:
    return np.array([math.cos(angle), math.sin(angle)])


def left_normal(angle: float) -> np.ndarray:
    return np.array([-math.sin(angle), math.cos(angle)])


@dataclass(frozen=True)
class Arm:
    """One straight stub, centre to port face. §5.1."""

    angle: float          # radians, plan, CCW from +X
    port_distance: float  # mm


@dataclass(frozen=True)
class Hub:
    arms: tuple[Arm, ...]
    corner_radius: float = 0.0

    # -- validation ------------------------------------------------------

    def gaps(self) -> list[float]:
        """Angular gap from each arm to the next, CCW. Sums to 2π."""
        angles = [a.angle for a in self.arms]
        return [(angles[(i + 1) % len(angles)] - angles[i]) % TAU
                for i in range(len(angles))]

    def validate(self, config: TrackConfig = DEFAULT) -> None:
        if len(self.arms) < 2:
            raise HubInvalid("a hub needs at least two arms")
        angles = [a.angle % TAU for a in self.arms]
        if angles != sorted(angles):
            raise HubInvalid("arms must be given in CCW angular order")
        if self.corner_radius < 0:
            raise HubInvalid("corner_radius must be zero or positive")

        for i, gap in enumerate(self.gaps()):
            if gap <= COLLINEAR_TOL:
                raise HubInvalid(f"arms {i} and {i + 1} are coincident")
            if gap > math.pi + COLLINEAR_TOL:
                raise HubInvalid(
                    f"the gap after arm {i} is {math.degrees(gap):.1f}°, over "
                    f"180°. The arms no longer surround the centre, so the "
                    f"outline would need a back edge. Use a curve, not a hub."
                )

        if self.mirror_axis() is None:
            raise HubInvalid(
                "this arm layout has no in-plan mirror axis, so the piece "
                "would not be flippable (§5.4)"
            )

        # every armpit must fall inside its arms, with room left for the joint
        clearance = (config.connector.lap_length
                     + config.connector.fit_clearance)
        for i in range(len(self.arms)):
            j = (i + 1) % len(self.arms)
            point = self.armpit(i, j)
            if point is None:
                continue
            reach = self._fillet_offset(i, j)
            for k in (i, j):
                arm = self.arms[k]
                along = float(np.dot(point, direction(arm.angle))) + reach
                if along + clearance >= arm.port_distance:
                    raise HubInvalid(
                        f"arm {k} port_distance {arm.port_distance:.1f} mm is "
                        f"too short: the armpit reaches {along:.1f} mm and the "
                        f"joint needs {clearance:.1f} mm more. Use Hub.auto()."
                    )

    def mirror_axis(self) -> float | None:
        """An in-plan mirror axis, if the layout has one. §5.4.

        Flipping a piece is a 180° rotation about a horizontal axis, which in
        plan view acts as a mirror. So a junction is flippable exactly when its
        plan layout is mirror-symmetric.
        """
        arms = sorted(((a.angle % TAU, a.port_distance) for a in self.arms))
        for i in range(len(arms)):
            for j in range(i, len(arms)):
                axis = (arms[i][0] + arms[j][0]) / 2.0
                mirrored = sorted((((2.0 * axis - angle) % TAU), distance)
                                  for angle, distance in arms)
                if all(abs(((a[0] - b[0] + math.pi) % TAU) - math.pi) < 1e-9
                       and abs(a[1] - b[1]) < 1e-9
                       for a, b in zip(arms, mirrored)):
                    return axis % math.pi
        return None

    # -- geometry --------------------------------------------------------

    def armpit(self, i: int, j: int) -> np.ndarray | None:
        """Where arm i's left edge meets arm j's right edge. §5.2.

        None when the two edges are collinear, which is the 180° case: a
        straight-through pair, or the back of a T. The boundary simply runs
        through, with no vertex.
        """
        half_width = DEFAULT.body.half_width
        return self._armpit(i, j, half_width)

    def _armpit(self, i: int, j: int, half_width: float) -> np.ndarray | None:
        n_i = left_normal(self.arms[i].angle)
        n_j = left_normal(self.arms[j].angle)
        matrix = np.array([n_i, n_j])
        det = float(np.linalg.det(matrix))
        if abs(det) < COLLINEAR_TOL:
            return None
        return np.linalg.solve(matrix, np.array([half_width, -half_width]))

    def _fillet_offset(self, i: int, j: int) -> float:
        """How far the fillet's tangent point sits past the armpit."""
        if self.corner_radius <= 0:
            return 0.0
        gap = (self.arms[j].angle - self.arms[i].angle) % TAU
        return self.corner_radius / math.tan(gap / 2.0)

    def port_edges(self, i: int, config: TrackConfig = DEFAULT
                   ) -> tuple[np.ndarray, np.ndarray]:
        """The port face, right edge to left edge — CCW around the hub."""
        arm = self.arms[i]
        u = direction(arm.angle)
        n = left_normal(arm.angle)
        half_width = config.body.half_width
        return (arm.port_distance * u - half_width * n,
                arm.port_distance * u + half_width * n)

    def chain(self, i: int, config: TrackConfig = DEFAULT,
              sag: float | None = None) -> list[np.ndarray]:
        """One boundary chain: arm i's port face round to arm i+1's. §5.3.

        A fillet at the armpit **adds** material rather than cutting it away.
        A car turning from arm i to arm j hugs that corner from the inside, so
        the arc is what it slides along, and `corner_radius` is its turn radius.
        Getting this backwards produces a hub that pinches instead of guiding.
        """
        sag = config.tolerances.chord_sag if sag is None else sag
        j = (i + 1) % len(self.arms)
        start = self.port_edges(i, config)[1]
        end = self.port_edges(j, config)[0]

        point = self._armpit(i, j, config.body.half_width)
        if point is None:
            return [start, end]
        if self.corner_radius <= 0:
            return [start, point, end]

        u_i, u_j = direction(self.arms[i].angle), direction(self.arms[j].angle)
        gap = (self.arms[j].angle - self.arms[i].angle) % TAU
        radius = self.corner_radius
        offset = radius / math.tan(gap / 2.0)

        t1 = point + offset * u_i
        t2 = point + offset * u_j
        bisector = u_i + u_j
        bisector = bisector / np.linalg.norm(bisector)
        centre = point + (radius / math.sin(gap / 2.0)) * bisector

        return [start, *_arc(centre, t1, t2, sag), end]

    # -- regions ---------------------------------------------------------

    def outline(self, config: TrackConfig = DEFAULT) -> list[np.ndarray]:
        """The hub's plan outline, CCW."""
        points: list[np.ndarray] = []
        for i in range(len(self.arms)):
            right, _left = self.port_edges(i, config)
            points.append(right)
            points.extend(self.chain(i, config)[:-1])
        return points

    def deck_region(self, config: TrackConfig = DEFAULT) -> list[np.ndarray]:
        """The deck slab's footprint: the outline itself.

        An earlier version inset this by half a rail thickness so the deck
        would never present a vertical face coplanar with a rail's. That traded
        one degeneracy for a worse one. The inset boundary lands *inside* the
        rail's port-cap face rather than on its edge, so the solver has to split
        that face and recompute the point from a different expression. On the X
        and T the two answers agree bit for bit; on the Y, whose coordinates are
        irrational, they differed by 4e-7 mm and the union came out with sliver
        triangles that only surfaced after the float32 STL round trip.

        Using the outline directly means the deck and the rails share the very
        same `chain()` output, so their coincident vertices are identical
        floats, not merely close ones. Coplanar faces built from identical
        vertices are the easy case for a boolean; near-coincident vertices are
        the hard one. Prefer exact coincidence to near-miss every time.
        """
        return self.outline(config)

    def rail_regions(self, config: TrackConfig = DEFAULT
                     ) -> list[list[np.ndarray]]:
        """One closed strip per chain: the rail, capped at both port faces."""
        thickness = config.body.rail_thickness
        regions = []
        for i in range(len(self.arms)):
            chain = self.chain(i, config)
            inner = offset_polyline(chain, thickness)
            regions.append(list(chain) + list(reversed(inner)))
        return regions

    # -- solids ----------------------------------------------------------

    def solids(self, config: TrackConfig = DEFAULT) -> list[MeshData]:
        """The prismatic slabs, to be unioned. §5.3.

        Rail prisms span the **full** height rather than only above and below
        the deck, so their overlap with the deck prism is volumetric rather
        than face to face.
        """
        config.validate()
        self.validate(config)

        body = config.body
        out = [prism(_as_pairs(self.deck_region(config)),
                     -body.half_deck, body.half_deck)]
        for region in self.rail_regions(config):
            out.append(prism(_as_pairs(region),
                             -body.half_height, body.half_height))
        return out

    def piece(self, name: str, config: TrackConfig = DEFAULT) -> Piece:
        return Piece(name=name, solids=tuple(self.solids(config)))

    def expected_volume(self, config: TrackConfig = DEFAULT) -> float:
        """Analytic volume of the unioned hub, mm³. §9.16.

        No polygon boolean is needed. Below the deck surface the slabs union to
        exactly the outline: the deck is inset half a rail thickness along the
        chains and the rails cover a full rail thickness inward from the same
        edges, so together they reach the boundary, and both reach the port
        planes untouched. Above and below the deck only the rails are present.
        """
        body = config.body
        outline = abs(polygon_area(self.outline(config)))
        rails = sum(abs(polygon_area(r)) for r in self.rail_regions(config))
        return (outline * body.deck_thickness
                + rails * (body.rail_height_total - body.deck_thickness))

    # -- construction helpers --------------------------------------------

    @staticmethod
    def auto(angles_deg: list[float], corner_radius: float = 0.0,
             config: TrackConfig = DEFAULT, margin: float = 2.0) -> "Hub":
        """Build a hub with each arm just long enough for its armpits.

        Port distance is derived, not guessed: an arm must reach past its two
        armpits, past any fillet tangent, and leave room for the joint.
        """
        angles = [math.radians(a) % TAU for a in angles_deg]
        angles.sort()
        provisional = Hub(tuple(Arm(a, 1.0) for a in angles), corner_radius)

        clearance = (config.connector.lap_length
                     + config.connector.fit_clearance + margin)
        reach = [0.0] * len(angles)
        for i in range(len(angles)):
            j = (i + 1) % len(angles)
            point = provisional._armpit(i, j, config.body.half_width)
            if point is None:
                continue
            extra = provisional._fillet_offset(i, j)
            for k in (i, j):
                along = float(np.dot(point, direction(angles[k]))) + extra
                reach[k] = max(reach[k], along)

        return Hub(tuple(Arm(a, r + clearance) for a, r in zip(angles, reach)),
                   corner_radius)


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


def _as_pairs(points: list[np.ndarray]) -> list[Pt2]:
    return [(float(p[0]), float(p[1])) for p in points]


def _arc(centre: np.ndarray, start: np.ndarray, end: np.ndarray,
         sag: float) -> list[np.ndarray]:
    """Sample the shorter arc from ``start`` to ``end`` about ``centre``."""
    radius = float(np.linalg.norm(start - centre))
    a0 = math.atan2(*(start - centre)[::-1])
    a1 = math.atan2(*(end - centre)[::-1])
    sweep = (a1 - a0 + math.pi) % TAU - math.pi

    if sag >= radius:
        step = math.pi
    else:
        step = 2.0 * math.acos(1.0 - sag / radius)
    n = max(1, math.ceil(abs(sweep) / step))
    return [centre + radius * np.array([math.cos(a0 + sweep * k / n),
                                        math.sin(a0 + sweep * k / n)])
            for k in range(n + 1)]


def offset_polyline(points: list[np.ndarray], distance: float
                    ) -> list[np.ndarray]:
    """Offset a polyline to its left by ``distance``, mitring the corners.

    The outline is walked CCW, so the material is on the left and this offsets
    inward.
    """
    if len(points) < 2:
        raise ValueError("need at least two points to offset")

    dirs, lefts = [], []
    for a, b in zip(points, points[1:]):
        d = b - a
        length = float(np.linalg.norm(d))
        if length < 1e-12:
            raise ValueError("repeated point in polyline")
        d = d / length
        dirs.append(d)
        lefts.append(np.array([-d[1], d[0]]))

    out = [points[0] + distance * lefts[0]]
    for i in range(1, len(points) - 1):
        p1 = points[i] + distance * lefts[i - 1]
        p2 = points[i] + distance * lefts[i]
        d1, d2 = dirs[i - 1], dirs[i]
        cross = float(d1[0] * d2[1] - d1[1] * d2[0])
        if abs(cross) < 1e-9:
            out.append(p1)
            continue
        delta = p2 - p1
        s = float(delta[0] * d2[1] - delta[1] * d2[0]) / cross
        out.append(p1 + s * d1)
    out.append(points[-1] + distance * lefts[-1])
    return out


def polygon_area(points: list[np.ndarray]) -> float:
    return shoelace(_as_pairs(points))
