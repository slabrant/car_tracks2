"""Rotation-minimising frames. docs/SPEC.md §4.2.

Pure Python + numpy. Never imports bpy.

**Do not use Frenet frames.** The Frenet normal is undefined where curvature is
zero, which is most of this track, and flips direction at inflections. It
produces a 180° twist at every straight-to-curve transition. The double
reflection method below (Wang, Jüttler, Zheng & Liu, 2008) is stable, needs no
curvature, and is about ten lines.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .path import Path

UP = np.array([0.0, 0.0, 1.0])
VERTICAL_TOL = 1e-6


class DegenerateFrame(ValueError):
    """The starting tangent is vertical, so world up cannot seed the frame."""


@dataclass(frozen=True)
class Frames:
    """One orthonormal frame per station, already rolled.

    ``across`` is the profile's `+X` direction and ``up`` its `+Z`. Swapping or
    negating that pair lays the track on its side.
    """

    s: np.ndarray        # (N,)
    points: np.ndarray   # (N, 3)
    tangent: np.ndarray  # (N, 3)
    across: np.ndarray   # (N, 3)
    up: np.ndarray       # (N, 3)


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)


def seed_across(tangent: np.ndarray) -> np.ndarray:
    """World up, projected orthogonal to the tangent, normalised.

    Raises rather than silently substituting another axis: a vertical start
    tangent means the caller has asked for something this system does not do.
    """
    residual = UP - np.dot(UP, tangent) * tangent
    if np.linalg.norm(residual) < VERTICAL_TOL:
        raise DegenerateFrame(
            "starting tangent is vertical; world up cannot seed the frame"
        )
    # right-handed like (X, Y, Z): across x tangent = up, so tangent x up = across
    return _unit(np.cross(tangent, _unit(residual)))


def rotation_minimising(points: np.ndarray,
                        tangents: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Propagate a frame along the path by double reflection.

    Returns ``(across, up)`` per station, before any roll is applied.
    """
    n = len(points)
    across = np.zeros((n, 3))
    up = np.zeros((n, 3))

    across[0] = seed_across(tangents[0])
    up[0] = _unit(np.cross(across[0], tangents[0]))

    for i in range(n - 1):
        v1 = points[i + 1] - points[i]
        c1 = float(np.dot(v1, v1))
        if c1 <= 0.0:
            across[i + 1] = across[i]
        else:
            r_l = across[i] - (2.0 / c1) * np.dot(v1, across[i]) * v1
            t_l = tangents[i] - (2.0 / c1) * np.dot(v1, tangents[i]) * v1
            v2 = tangents[i + 1] - t_l
            c2 = float(np.dot(v2, v2))
            if c2 <= 0.0:
                across[i + 1] = r_l
            else:
                across[i + 1] = r_l - (2.0 / c2) * np.dot(v2, r_l) * v2
        across[i + 1] = _unit(across[i + 1])
        up[i + 1] = _unit(np.cross(across[i + 1], tangents[i + 1]))

    return across, up


def _roll_about(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation matrix about a unit axis."""
    if angle == 0.0:
        return np.eye(3)
    k = np.array([[0.0, -axis[2], axis[1]],
                  [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k)


def build(path: Path, sag: float) -> Frames:
    """Sample the path and build a rolled, rotation-minimising frame per station."""
    s = np.array(path.stations(sag), dtype=np.float64)
    points = np.array([path.point(float(v)) for v in s])
    tangents = np.array([path.tangent(float(v)) for v in s])

    across, up = rotation_minimising(points, tangents)

    # roll is applied about the tangent *after* the RMF, never folded into it
    for i, value in enumerate(s):
        angle = path.roll(float(value))
        if angle == 0.0:
            continue
        rot = _roll_about(tangents[i], angle)
        across[i] = rot @ across[i]
        up[i] = rot @ up[i]

    return Frames(s=s, points=points, tangent=tangents, across=across, up=up)
