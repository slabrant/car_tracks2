"""Path primitives and their concatenation. docs/SPEC.md §4.1.

Pure Python + numpy. Never imports bpy.

Paths are built from primitives, not fitted splines. A physical track set needs
pieces whose ends sit at exact repeatable angles so they tile; a fitted spline
gives irrational end tangents and loops that never close.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Protocol, Sequence

import numpy as np

from .config import Body, Connector, Tolerances
from .mesh import rotation_z, translation

Vec3 = np.ndarray

DEFAULT_MIN_RADIUS = Tolerances().min_radius(Body())
"""18.0 mm at default dimensions. Below this a sweep folds through itself."""

BANK_RAMP_FRACTION = 0.1
"""Bank eases in and out over this fraction of the arc at each end."""

DEFAULT_PORT_CLEAR = (Connector().lap_length + Connector().fit_clearance + 2.0)
"""How much of each end must stay flat, mm. 5.15 at default dimensions.

**The cross-section must not roll inside a lap zone.** The connector's cut tools
are flat boxes aligned to the port frame (§6.6); if the section has rolled by
the time they reach in, they slice it at the wrong height on each rail and the
diagonal split comes apart. A banked 90° curve failed exactly this way — the
bank had reached six degrees where the notches bite, and the result came out
genus 3.

Horizontal curvature is fine and needs no clearance: it moves the section
sideways, not in z, and a notch removes everything below the lap plane whatever
its lateral position. It is roll and vertical curvature that must stay out.
"""


class PathTooTightError(ValueError):
    """The path curves tighter than the profile can be swept around, §4.1."""


class PathDiscontinuous(ValueError):
    """Two primitives do not meet in position or tangent, §4.1."""


def _smoothstep(t: float) -> float:
    t = min(max(t, 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n == 0.0:
        raise ValueError("cannot normalise a zero vector")
    return v / n


class Primitive(Protocol):
    """One segment, in its own local frame: starts at the origin heading +Y."""

    length: float

    def point(self, s: float) -> Vec3: ...
    def tangent(self, s: float) -> Vec3: ...
    def roll(self, s: float) -> float: ...
    def curvature(self, s: float) -> float: ...
    def min_radius_of_curvature(self) -> float: ...
    def stations(self, sag: float) -> list[float]: ...
    def end_transform(self) -> np.ndarray: ...


# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Line:
    """A straight run.

    ``roll_offset`` is a constant orientation offset, in radians, applied on
    top of the rotation-minimising frame. It exists for one job: a `Loop`
    twists the frame by `drift / radius` on its way round, and the straight
    leaving the loop has to carry that same offset, or its section comes out
    lying at that angle. Constant, so it is not a bank — nothing rolls *along*
    a Line, and a lap zone on one is as flat as any other.
    """

    length: float
    roll_offset: float = 0.0

    def __post_init__(self) -> None:
        if self.length <= 0:
            raise ValueError("Line length must be positive")

    def point(self, s: float) -> Vec3:
        return np.array([0.0, s, 0.0])

    def tangent(self, s: float) -> Vec3:
        return np.array([0.0, 1.0, 0.0])

    def roll(self, s: float) -> float:
        return self.roll_offset

    def curvature(self, s: float) -> float:
        return 0.0

    def min_radius_of_curvature(self) -> float:
        return math.inf

    def stations(self, sag: float) -> list[float]:
        return [0.0, self.length]

    def end_transform(self) -> np.ndarray:
        return translation(0.0, self.length, 0.0)


@dataclass(frozen=True)
class Arc:
    """A horizontal circular arc. ``angle`` > 0 turns left, §4.1.

    Left is `-X`: with forward `+Y` and up `+Z`, right is forward × up = `+X`.
    """

    radius: float
    angle: float
    bank: float = 0.0
    min_radius: float = DEFAULT_MIN_RADIUS
    bank_clear: float = DEFAULT_PORT_CLEAR

    def __post_init__(self) -> None:
        if self.radius <= 0:
            raise ValueError("Arc radius must be positive")
        if self.angle == 0:
            raise ValueError("Arc angle must be non-zero")
        if self.radius < self.min_radius:
            raise PathTooTightError(
                f"Arc(radius={self.radius}) is below the minimum {self.min_radius} "
                f"mm; the inner rail would turn inside out"
            )
        if self.bank != 0.0 and self.length <= 2.0 * self.bank_clear:
            raise ValueError(
                f"a banked arc must be longer than {2.0 * self.bank_clear:.1f} mm "
                f"so both lap zones stay flat; this one is {self.length:.1f} mm"
            )

    @property
    def length(self) -> float:
        return self.radius * abs(self.angle)

    @property
    def _turn(self) -> float:
        return 1.0 if self.angle > 0 else -1.0

    def _u(self, s: float) -> float:
        return s / self.radius

    def point(self, s: float) -> Vec3:
        k, r, u = self._turn, self.radius, self._u(s)
        return np.array([k * r * (math.cos(u) - 1.0), r * math.sin(u), 0.0])

    def tangent(self, s: float) -> Vec3:
        k, u = self._turn, self._u(s)
        return np.array([-k * math.sin(u), math.cos(u), 0.0])

    def roll(self, s: float) -> float:
        """Zero over each lap zone, then eased in. See DEFAULT_PORT_CLEAR."""
        if self.bank == 0.0:
            return 0.0
        length, clear = self.length, self.bank_clear
        if s <= clear or s >= length - clear:
            return 0.0
        run = min(BANK_RAMP_FRACTION * length, length / 2.0 - clear)
        if s < clear + run:
            return self.bank * _smoothstep((s - clear) / run)
        if s > length - clear - run:
            return self.bank * _smoothstep((length - clear - s) / run)
        return self.bank

    def curvature(self, s: float) -> float:
        return 1.0 / self.radius

    def min_radius_of_curvature(self) -> float:
        return self.radius

    def stations(self, sag: float) -> list[float]:
        if sag >= self.radius:
            step_angle = math.pi
        else:
            step_angle = 2.0 * math.acos(1.0 - sag / self.radius)
        n = max(1, math.ceil(abs(self.angle) / step_angle))
        if self.bank != 0.0:
            # the bank stays flat over each lap zone and then eases in; give
            # both the flat run and the ramp enough stations to be smooth
            n = max(n, math.ceil(12 / BANK_RAMP_FRACTION))
            edges = [self.bank_clear, self.length - self.bank_clear]
            return sorted(set([self.length * i / n for i in range(n + 1)] + edges))
        return [self.length * i / n for i in range(n + 1)]

    def end_transform(self) -> np.ndarray:
        return translation(*self.point(self.length)) @ rotation_z(self.angle)


@dataclass(frozen=True)
class Ramp:
    """A vertical S-curve: the bridge and slope primitive, §4.1.

    ``run`` is the horizontal distance; ``length`` is true arc length and is
    slightly greater. The vertical profile is a smoothstep, so the tangent is
    horizontal at both ends and a Ramp concatenates with a Line without a kink.
    """

    run: float
    rise: float
    min_radius: float = DEFAULT_MIN_RADIUS
    samples: int = 2001

    _u_table: np.ndarray = field(default=None, repr=False, compare=False)
    _s_table: np.ndarray = field(default=None, repr=False, compare=False)
    _length: float = field(default=0.0, repr=False, compare=False)

    def __post_init__(self) -> None:
        if self.run <= 0:
            raise ValueError("Ramp run must be positive")
        if self.rise == 0:
            raise ValueError("Ramp rise must be non-zero; use a Line instead")

        u = np.linspace(0.0, 1.0, self.samples)
        y = self.run * u
        z = self.rise * (3.0 * u**2 - 2.0 * u**3)
        seg = np.hypot(np.diff(y), np.diff(z))
        s = np.concatenate([[0.0], np.cumsum(seg)])
        object.__setattr__(self, "_u_table", u)
        object.__setattr__(self, "_s_table", s)
        object.__setattr__(self, "_length", float(s[-1]))

        r = self.min_radius_of_curvature()
        if r < self.min_radius:
            raise PathTooTightError(
                f"Ramp(run={self.run}, rise={self.rise}) bends to a radius of "
                f"{r:.2f} mm, below the minimum {self.min_radius} mm"
            )

    @property
    def length(self) -> float:
        return self._length

    def _u(self, s: float) -> float:
        return float(np.interp(s, self._s_table, self._u_table))

    def point(self, s: float) -> Vec3:
        u = self._u(s)
        return np.array([0.0, self.run * u,
                         self.rise * (3.0 * u**2 - 2.0 * u**3)])

    def _slope(self, u: float) -> float:
        return self.rise * (6.0 * u - 6.0 * u * u) / self.run

    def tangent(self, s: float) -> Vec3:
        return _unit(np.array([0.0, 1.0, self._slope(self._u(s))]))

    def roll(self, s: float) -> float:
        return 0.0

    def curvature(self, s: float) -> float:
        u = self._u(s)
        d1 = self._slope(u)
        d2 = self.rise * (6.0 - 12.0 * u) / (self.run * self.run)
        return abs(d2) / (1.0 + d1 * d1) ** 1.5

    def min_radius_of_curvature(self) -> float:
        # curvature peaks at the ends, where the slope is zero
        peak = abs(self.rise) * 6.0 / (self.run * self.run)
        return math.inf if peak == 0.0 else 1.0 / peak

    def stations(self, sag: float) -> list[float]:
        r = self.min_radius_of_curvature()
        if math.isinf(r):
            return [0.0, self.length]
        step = 2.0 * math.sqrt(max(2.0 * r * sag, 1e-12))
        n = max(1, math.ceil(self.length / step))
        return [self.length * i / n for i in range(n + 1)]

    def end_transform(self) -> np.ndarray:
        return translation(0.0, self.run, self.rise)


DEFAULT_LOOP_DRIFT = Body().width_outer + 2.0
"""How far a loop steps sideways over its turn, mm. 26.0 at default dimensions.

A vertical circle ends where it began. Swept as a solid that is not a joint,
it is a piece passing through itself at the bottom, and no amount of care in
the mesh code makes it printable. The loop therefore drifts **across** the
direction of travel as it goes round, by more than the track is wide, so the
run coming out passes beside the run going in with air between them.

Which is what a real loop does too, and for the same reason. Two millimetres
of that clearance is air; the rest is track.
"""


@dataclass(frozen=True)
class Loop:
    """A vertical loop: one full turn in the plane of travel, §4.1.

    The car goes over the top upside down, held there by speed rather than by
    the rails — which is a fact about the car, not about this geometry. What
    the geometry has to get right is that the channel faces **inward** the whole
    way round, and it does so for free: the rotation-minimising frame carries
    `up` with the tangent, so at the top of the loop `up` points at the floor
    and the deck is over the car's roof.

    Two things make it a helix rather than a circle.

    The **drift**: a closed circle would come back to its own start, so the
    piece would pass through itself where it crosses. See `DEFAULT_LOOP_DRIFT`.

    The **easing**: the drift follows a smoothstep in turn angle rather than
    growing linearly, so its lateral rate is zero at both ends. That is what
    lets the loop leave and rejoin heading exactly `+Y`, with no kink against
    the straights either side of it. Grown linearly the piece would enter at an
    angle — atan(drift / 2*pi*radius), about five degrees at the defaults —
    and `Path` would refuse it, rightly.

    `end_transform` is a pure sideways translation: a loop advances the track
    not at all along its own direction, and moves it one drift across. A layout
    that goes through a loop comes out travelling the way it went in, offset.
    """

    radius: float
    drift: float = DEFAULT_LOOP_DRIFT
    min_radius: float = DEFAULT_MIN_RADIUS
    min_drift: float = Body().width_outer
    samples: int = 2001

    _u_table: np.ndarray = field(default=None, repr=False, compare=False)
    _s_table: np.ndarray = field(default=None, repr=False, compare=False)
    _twist_table: np.ndarray = field(default=None, repr=False, compare=False)
    _length: float = field(default=0.0, repr=False, compare=False)

    def __post_init__(self) -> None:
        if self.radius <= 0:
            raise ValueError("Loop radius must be positive")
        if self.drift <= self.min_drift:
            raise ValueError(
                f"a loop drifting {self.drift:.1f} mm across a track "
                f"{self.min_drift:.1f} mm wide would pass through itself where "
                f"it crosses at the bottom"
            )

        u = np.linspace(0.0, 2.0 * math.pi, self.samples)
        p = self._point_at(u)
        seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
        table = np.concatenate([[0.0], np.cumsum(seg)])
        object.__setattr__(self, "_u_table", u)
        object.__setattr__(self, "_s_table", table)
        object.__setattr__(self, "_length", float(table[-1]))
        object.__setattr__(self, "_twist_table", self._accumulate_twist(u))

        r = self.min_radius_of_curvature()
        if r < self.min_radius:
            raise PathTooTightError(
                f"Loop(radius={self.radius}, drift={self.drift}) bends to a "
                f"radius of {r:.2f} mm, below the minimum {self.min_radius} mm"
            )

    # -- the curve, in turn angle ----------------------------------------
    #
    # Turn angle `u` runs 0 to 2*pi. In the loop's own plane the curve is the
    # circle (radius * sin u, radius * (1 - cos u)), which starts at the origin
    # heading +Y, is inverted at u = pi, and closes at u = 2*pi. Across the
    # plane it steps `drift * smoothstep(u / 2*pi)`.

    def _drift_at(self, u):
        t = u / (2.0 * math.pi)
        return self.drift * t * t * (3.0 - 2.0 * t)

    def _point_at(self, u):
        return np.stack([self._drift_at(u),
                         self.radius * np.sin(u),
                         self.radius * (1.0 - np.cos(u))], axis=-1)

    def _d1(self, u):
        """dP/du. The smoothstep's derivative is zero at both ends, which is
        what keeps the end tangents exactly +Y."""
        t = u / (2.0 * math.pi)
        dx = self.drift * 6.0 * t * (1.0 - t) / (2.0 * math.pi)
        return np.stack([dx,
                         self.radius * np.cos(u),
                         self.radius * np.sin(u)], axis=-1)

    def _d2(self, u):
        t = u / (2.0 * math.pi)
        ddx = self.drift * (6.0 - 12.0 * t) / (2.0 * math.pi) ** 2
        return np.stack([ddx,
                         -self.radius * np.sin(u),
                         self.radius * np.cos(u)], axis=-1)

    # -- the twist -------------------------------------------------------

    def _desired_up(self, u):
        """Where the channel must face: straight at the loop's own centre.

        In the loop's plane the inward radial direction is
        `(-sin u, cos u)`, which is `+Z` at both ends and `-Z` over the top.
        Square to the plane, so the drift is not banked into — the section
        stays level where the loop is level, which is what lets the exit port
        mate with an ordinary straight.
        """
        return np.stack([np.zeros_like(u), -np.sin(u), np.cos(u)], axis=-1)

    def _accumulate_twist(self, u):
        """How far to roll the frame at each station, radians.

        A drifting loop is a helix, and a helix has torsion. The frame in
        `frames.py` is rotation-*minimising*, not torsion-following: it carries
        the section round without ever turning it about the tangent, so by the
        time the track is level again the section is lying at an angle. That
        angle is not small — it comes to almost exactly `drift / radius`, 31
        degrees at the defaults, and it would be where the exit port sits: a
        port no other piece in the set can mate with.

        So the roll is measured, not derived: build the same frame `frames.py`
        will build, and record the signed angle from it to `_desired_up` at
        every station. Rolling by that lands the section where it belongs the
        whole way round, level ends included.

        The import is deferred because `frames` imports this module. That is
        the honest shape of the dependency: a primitive that has to undo what
        the frame builder does needs to know what it does.
        """
        from .frames import rotation_minimising

        points, tangents = self._point_at(u), self._d1(u)
        tangents = tangents / np.linalg.norm(tangents, axis=-1)[:, None]
        _across, up = rotation_minimising(points, tangents)

        want = self._desired_up(u)
        # signed angle about the tangent, from the frame's up to the wanted one
        sin = (np.cross(up, want) * tangents).sum(axis=-1)
        cos = (up * want).sum(axis=-1)
        return np.unwrap(np.arctan2(sin, cos))

    @property
    def twist(self) -> float:
        """Total roll the loop hands to whatever follows it, radians.

        Very nearly `drift / radius`: a loop that steps one track-width
        sideways over a radius of two track-widths twists the section by about
        half a radian, and there is no arranging the drift to avoid it. What
        follows the loop has to carry this, which is what `Line.roll_offset`
        is for.
        """
        return float(self._twist_table[-1])

    # -- Primitive -------------------------------------------------------

    @property
    def length(self) -> float:
        return self._length

    def _u(self, s: float) -> float:
        return float(np.interp(s, self._s_table, self._u_table))

    def point(self, s: float) -> Vec3:
        return self._point_at(np.float64(self._u(s)))

    def tangent(self, s: float) -> Vec3:
        return _unit(self._d1(np.float64(self._u(s))))

    def roll(self, s: float) -> float:
        """The torsion the rotation-minimising frame did not follow.

        Note what is *not* here: going upside down at the top. That is the
        path turning over and carrying the section with it, which the frame
        does on its own. This is only the sideways twist the drift adds; see
        `_accumulate_twist`.
        """
        return float(np.interp(s, self._s_table, self._twist_table))

    def curvature(self, s: float) -> float:
        return float(self._curvature_at(np.float64(self._u(s))))

    def _curvature_at(self, u):
        d1, d2 = self._d1(u), self._d2(u)
        return (np.linalg.norm(np.cross(d1, d2), axis=-1)
                / np.linalg.norm(d1, axis=-1) ** 3)

    def min_radius_of_curvature(self) -> float:
        peak = float(np.max(self._curvature_at(self._u_table)))
        return math.inf if peak == 0.0 else 1.0 / peak

    def stations(self, sag: float) -> list[float]:
        r = self.min_radius_of_curvature()
        step = 2.0 * math.sqrt(max(2.0 * r * sag, 1e-12))
        n = max(1, math.ceil(self.length / step))
        return [self.length * i / n for i in range(n + 1)]

    def end_transform(self) -> np.ndarray:
        return translation(self.drift, 0.0, 0.0)


# --------------------------------------------------------------------------


class Path:
    """Primitives laid end to end, queried in world coordinates by arc length."""

    C0_TOL = 1e-9
    C1_TOL = 1e-12

    def __init__(self, primitives: Sequence[Primitive]) -> None:
        if not primitives:
            raise ValueError("a Path needs at least one primitive")
        self.primitives = list(primitives)
        self.transforms: list[np.ndarray] = []
        self.starts: list[float] = []

        matrix = np.eye(4)
        travelled = 0.0
        for prim in self.primitives:
            self.transforms.append(matrix)
            self.starts.append(travelled)
            travelled += prim.length
            matrix = matrix @ prim.end_transform()
        self.length = travelled
        self.end_transform = matrix

        self._check_continuity()

    @classmethod
    def chain(cls, *primitives: Primitive) -> "Path":
        return cls(primitives)

    # -- queries ---------------------------------------------------------

    def _locate(self, s: float) -> tuple[int, float]:
        s = min(max(s, 0.0), self.length)
        for i in range(len(self.primitives) - 1, -1, -1):
            if s >= self.starts[i] - 1e-12:
                local = min(s - self.starts[i], self.primitives[i].length)
                return i, local
        return 0, s

    def point(self, s: float) -> Vec3:
        i, local = self._locate(s)
        p = self.primitives[i].point(local)
        return (self.transforms[i] @ np.array([p[0], p[1], p[2], 1.0]))[:3]

    def tangent(self, s: float) -> Vec3:
        i, local = self._locate(s)
        t = self.primitives[i].tangent(local)
        return _unit(self.transforms[i][:3, :3] @ t)

    def roll(self, s: float) -> float:
        i, local = self._locate(s)
        return self.primitives[i].roll(local)

    def curvature(self, s: float) -> float:
        i, local = self._locate(s)
        return self.primitives[i].curvature(local)

    def min_radius_of_curvature(self) -> float:
        return min(p.min_radius_of_curvature() for p in self.primitives)

    # -- stations --------------------------------------------------------

    def stations(self, sag: float) -> list[float]:
        """Arc-length positions to sample, §4.2.

        Straight runs get their endpoints only. Every primitive boundary and
        both path ends always get a station.
        """
        if sag <= 0:
            raise ValueError("chord sag tolerance must be positive")
        out: list[float] = []
        for prim, start in zip(self.primitives, self.starts):
            out.extend(start + local for local in prim.stations(sag))
        out.append(self.length)

        out.sort()
        deduped = [out[0]]
        for s in out[1:]:
            if s - deduped[-1] > 1e-9:
                deduped.append(s)
        return deduped

    # -- validation ------------------------------------------------------

    def _check_continuity(self) -> None:
        for i in range(len(self.primitives) - 1):
            s = self.starts[i + 1]
            before_p = self._world_point(i, self.primitives[i].length)
            after_p = self._world_point(i + 1, 0.0)
            gap = float(np.linalg.norm(before_p - after_p))
            if gap > self.C0_TOL:
                raise PathDiscontinuous(
                    f"primitives {i} and {i + 1} are {gap:.3g} mm apart at s={s:.3f}"
                )

            before_t = self._world_tangent(i, self.primitives[i].length)
            after_t = self._world_tangent(i + 1, 0.0)
            dot = float(np.dot(before_t, after_t))
            if dot < 1.0 - self.C1_TOL:
                raise PathDiscontinuous(
                    f"primitives {i} and {i + 1} kink at s={s:.3f}: "
                    f"tangent dot {dot:.12f}"
                )

            roll_gap = abs(self.primitives[i].roll(self.primitives[i].length)
                           - self.primitives[i + 1].roll(0.0))
            if roll_gap > 1e-12:
                raise PathDiscontinuous(
                    f"primitives {i} and {i + 1} step in roll by {roll_gap:.3g} rad"
                )

    def _world_point(self, i: int, local: float) -> Vec3:
        p = self.primitives[i].point(local)
        return (self.transforms[i] @ np.array([p[0], p[1], p[2], 1.0]))[:3]

    def _world_tangent(self, i: int, local: float) -> Vec3:
        return _unit(self.transforms[i][:3, :3] @ self.primitives[i].tangent(local))

    def check_curvature(self, min_radius: float) -> None:
        """Re-check against the profile actually being swept, §4.1."""
        for i, prim in enumerate(self.primitives):
            r = prim.min_radius_of_curvature()
            if r < min_radius:
                raise PathTooTightError(
                    f"primitive {i} ({type(prim).__name__}) bends to {r:.2f} mm, "
                    f"below the minimum {min_radius:.2f} mm"
                )
