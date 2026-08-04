"""Minimal mesh primitives for the Phase 0 coupon.

Pure Python + numpy. Never imports bpy.

Conventions follow docs/SPEC.md §1:
  +X across the track, +Y along the track, +Z up, right-handed.
  Faces are wound CCW seen from outside, so normals point outward.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

Vec3 = tuple[float, float, float]
Pt2 = tuple[float, float]


@dataclass(frozen=True)
class MeshData:
    """An indexed polygon mesh. Plain data; knows nothing about Blender."""

    verts: np.ndarray  # (V, 3) float64, mm
    faces: list[list[int]]  # CCW, outward

    def transformed(self, matrix: np.ndarray) -> "MeshData":
        """Apply a 4x4 homogeneous transform."""
        homo = np.hstack([self.verts, np.ones((len(self.verts), 1))])
        out = (homo @ matrix.T)[:, :3]
        return MeshData(verts=out, faces=[list(f) for f in self.faces])

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        return self.verts.min(axis=0), self.verts.max(axis=0)


# --------------------------------------------------------------------------
# transforms
# --------------------------------------------------------------------------


def translation(dx: float, dy: float, dz: float) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = (dx, dy, dz)
    return m


def rotation_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    m = np.eye(4)
    m[0, 0], m[0, 1] = c, -s
    m[1, 0], m[1, 1] = s, c
    return m


def rotation_y(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    m = np.eye(4)
    m[0, 0], m[0, 2] = c, s
    m[2, 0], m[2, 2] = -s, c
    return m


# --------------------------------------------------------------------------
# primitives
# --------------------------------------------------------------------------


def box(lo: Vec3, hi: Vec3) -> MeshData:
    """Axis-aligned box. 8 verts, 6 quads, outward normals."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    if not (x1 > x0 and y1 > y0 and z1 > z0):
        raise ValueError(f"degenerate box {lo} -> {hi}")

    verts = np.array(
        [
            (x0, y0, z0),
            (x1, y0, z0),
            (x1, y1, z0),
            (x0, y1, z0),
            (x0, y0, z1),
            (x1, y0, z1),
            (x1, y1, z1),
            (x0, y1, z1),
        ],
        dtype=np.float64,
    )
    faces = [
        [0, 3, 2, 1],  # -Z
        [4, 5, 6, 7],  # +Z
        [0, 1, 5, 4],  # -Y
        [3, 7, 6, 2],  # +Y
        [0, 4, 7, 3],  # -X
        [1, 2, 6, 5],  # +X
    ]
    return MeshData(verts=verts, faces=faces)


def prism_yz(poly: Sequence[Pt2], x0: float, x1: float) -> MeshData:
    """Extrude a polygon given in the YZ plane along X.

    ``poly`` must be CCW in (y, z) parameter order, which by Y x Z = X gives it
    an outward normal of +X. That makes the cap at ``x1`` use the given order
    and the cap at ``x0`` its reverse.
    """
    if x1 <= x0:
        raise ValueError(f"degenerate extrusion {x0} -> {x1}")
    n = len(poly)
    if n < 3:
        raise ValueError("polygon needs at least 3 points")
    if _shoelace(poly) <= 0:
        raise ValueError("polygon must be CCW in (y, z)")

    lo = np.array([(x0, y, z) for (y, z) in poly], dtype=np.float64)
    hi = np.array([(x1, y, z) for (y, z) in poly], dtype=np.float64)
    verts = np.vstack([lo, hi])

    faces: list[list[int]] = []
    faces.append(list(range(n - 1, -1, -1)))  # cap at x0, normal -X
    faces.append([n + i for i in range(n)])  # cap at x1, normal +X
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j, n + i])
    return MeshData(verts=verts, faces=faces)


def prism_xz(poly: Sequence[Pt2], y0: float, y1: float) -> MeshData:
    """Extrude a cross-section given in the XZ plane along Y.

    ``poly`` must be ordered so it reads CCW when viewed from +Y, which is the
    convention docs/SPEC.md §2.1 uses for the track profile. In raw (x, z)
    parameter order that is clockwise, i.e. negative shoelace.
    """
    if y1 <= y0:
        raise ValueError(f"degenerate extrusion {y0} -> {y1}")
    n = len(poly)
    if n < 3:
        raise ValueError("polygon needs at least 3 points")
    if _shoelace(poly) >= 0:
        raise ValueError("profile must be CW in (x, z), i.e. CCW viewed from +Y")

    lo = np.array([(x, y0, z) for (x, z) in poly], dtype=np.float64)
    hi = np.array([(x, y1, z) for (x, z) in poly], dtype=np.float64)
    verts = np.vstack([lo, hi])

    faces: list[list[int]] = []
    faces.append(list(range(n - 1, -1, -1)))  # cap at y0, normal -Y
    faces.append([n + i for i in range(n)])  # cap at y1, normal +Y
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j, n + i])
    return MeshData(verts=verts, faces=faces)


def _shoelace(poly: Sequence[Pt2]) -> float:
    pts = np.asarray(poly, dtype=np.float64)
    a, b = pts, np.roll(pts, -1, axis=0)
    return float(np.sum(a[:, 0] * b[:, 1] - b[:, 0] * a[:, 1]) / 2.0)


# --------------------------------------------------------------------------
# binary STL, for validating what actually came out of Blender
# --------------------------------------------------------------------------


def read_stl(path: str, weld_tol: float = 1e-5) -> MeshData:
    """Read a binary STL and weld coincident vertices into an indexed mesh."""
    with open(path, "rb") as fh:
        data = fh.read()
    if len(data) < 84:
        raise ValueError(f"{path}: too short to be a binary STL")
    (count,) = struct.unpack_from("<I", data, 80)
    expected = 84 + count * 50
    if len(data) != expected:
        raise ValueError(
            f"{path}: expected {expected} bytes for {count} triangles, got {len(data)}"
        )

    raw = np.frombuffer(data, dtype=np.uint8, offset=84).reshape(count, 50)
    coords = raw[:, 12:48].copy().view(np.float32).reshape(count, 3, 3)
    pts = coords.reshape(-1, 3).astype(np.float64)

    quantised = np.round(pts / weld_tol).astype(np.int64)
    _, first, inverse = np.unique(quantised, axis=0, return_index=True, return_inverse=True)
    verts = pts[first]
    tris = inverse.reshape(count, 3)
    return MeshData(verts=verts, faces=[list(map(int, t)) for t in tris])


def write_stl(mesh: MeshData, path: str, name: str = "coupon") -> None:
    """Write a binary STL, fan-triangulating any polygon faces."""
    tris: list[np.ndarray] = []
    for face in mesh.faces:
        for k in range(1, len(face) - 1):
            tris.append(mesh.verts[[face[0], face[k], face[k + 1]]])
    header = name.encode("ascii", "replace")[:80].ljust(80, b"\0")
    with open(path, "wb") as fh:
        fh.write(header)
        fh.write(struct.pack("<I", len(tris)))
        for tri in tris:
            normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            length = np.linalg.norm(normal)
            normal = normal / length if length > 0 else np.zeros(3)
            fh.write(struct.pack("<3f", *normal.astype(np.float32)))
            for point in tri:
                fh.write(struct.pack("<3f", *point.astype(np.float32)))
            fh.write(struct.pack("<H", 0))


def merge(meshes: Iterable[MeshData]) -> MeshData:
    """Concatenate meshes without any boolean. Used only for laying out plates."""
    verts: list[np.ndarray] = []
    faces: list[list[int]] = []
    offset = 0
    for mesh in meshes:
        verts.append(mesh.verts)
        faces.extend([[i + offset for i in f] for f in mesh.faces])
        offset += len(mesh.verts)
    return MeshData(verts=np.vstack(verts), faces=faces)
