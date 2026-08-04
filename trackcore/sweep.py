"""Construction A: sweep the profile along a path. docs/SPEC.md §4.3.

Pure Python + numpy. Never imports bpy.

**No booleans.** The mesh is manifold by construction: N stations by a fixed
12-vertex profile, quad strips between consecutive rings, and two end caps.
Every edge is used by exactly two faces because the topology says so, not
because a solver got it right. The only way this can fail is self-intersection,
which the curvature guard in path.py forbids.
"""

from __future__ import annotations

import numpy as np

from .config import DEFAULT, TrackConfig
from .connector import port_matrix
from .edge_unit import PROFILE_VERTS, profile
from .frames import Frames, build
from .mesh import MeshData
from .path import Path


def rings(frames: Frames, section: list[tuple[float, float]]) -> np.ndarray:
    """Place the profile at every station. Returns (N, M, 3).

    Profile `+X` maps to the frame's `across`, profile `+Z` to its `up` (§4.2).
    """
    px = np.array([p[0] for p in section])
    pz = np.array([p[1] for p in section])
    return (frames.points[:, None, :]
            + px[None, :, None] * frames.across[:, None, :]
            + pz[None, :, None] * frames.up[:, None, :])


def sweep(path: Path, config: TrackConfig = DEFAULT) -> MeshData:
    """Sweep the track profile along ``path``.

    Ends are flat and square to the path. Connectors are Phase 3.
    """
    config.validate()
    path.check_curvature(config.min_radius)

    section = profile(config.body)
    frames = build(path, config.tolerances.chord_sag)
    grid = rings(frames, section)

    n_stations, n_profile, _ = grid.shape
    if n_profile != PROFILE_VERTS:
        raise AssertionError(f"expected {PROFILE_VERTS} profile vertices")
    if n_stations < 2:
        raise AssertionError("a sweep needs at least two stations")

    verts = grid.reshape(-1, 3)
    faces: list[list[int]] = []

    # side quads
    for i in range(n_stations - 1):
        base, nxt = i * n_profile, (i + 1) * n_profile
        for j in range(n_profile):
            k = (j + 1) % n_profile
            faces.append([base + j, base + k, nxt + k, nxt + j])

    # end caps. The profile reads CCW from +Y, so at the last station its own
    # order already points outward along the tangent; the first is reversed.
    last = (n_stations - 1) * n_profile
    faces.append(list(range(n_profile - 1, -1, -1)))
    faces.append([last + j for j in range(n_profile)])

    return MeshData(verts=verts, faces=faces)


def port_matrices(path: Path, config: TrackConfig = DEFAULT) -> list:
    """The two port frames of a swept piece, §6.

    Both point **out** of the piece, so the near one faces backwards along the
    path. That is what makes two pieces laid end to end present frames related
    by `connector.MATE`, which is the whole point of a genderless port.
    """
    frames = build(path, config.tolerances.chord_sag)
    return [
        port_matrix(frames.points[0], -frames.tangent[0], frames.up[0]),
        port_matrix(frames.points[-1], frames.tangent[-1], frames.up[-1]),
    ]


def expected_volume(path: Path, config: TrackConfig = DEFAULT) -> float:
    """Analytic volume of the swept solid, mm³.

    By Pappus, a profile whose centroid lies on the path sweeps out exactly
    area × path length. The track profile is symmetric about both its axes, so
    its centroid is on the centreline and this is exact for the ideal surface.
    The faceted mesh comes in a little under it.
    """
    from .edge_unit import profile_area

    return profile_area(config.body) * path.length
