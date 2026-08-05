"""MeshData and the transforms that move it. docs/SPEC.md §4.3.

Pure Python + numpy. Never imports bpy.

MeshData is plain data. It knows nothing about Blender, and the Blender layer
converts it at the boundary.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

Pt2 = tuple[float, float]
Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class Piece:
    """One printable part, as the solids that make it up.

    Construction A yields a single solid and needs no boolean. Construction B
    and C yield several overlapping solids to be unioned, under §5.3's
    obligation to re-run every §7 check on the result.
    """

    name: str
    solids: tuple["MeshData", ...]
    cuts: tuple["MeshData", ...] = ()
    additions: tuple["MeshData", ...] = ()

    @property
    def needs_boolean(self) -> bool:
        return len(self.solids) > 1 or bool(self.cuts) or bool(self.additions)

    def stages(self) -> list[tuple[str, tuple["MeshData", ...]]]:
        """The boolean programme, in order.

        Cuts run before additions. The tabs added at a port sit exactly against
        the slot and notch boundaries the cuts define, so adding first would
        leave a cut tool coincident with a face it must not touch.
        """
        return [("UNION", self.solids[1:]),
                ("DIFFERENCE", self.cuts),
                ("UNION", self.additions)]

    def every_solid(self):
        return (*self.solids, *self.cuts, *self.additions)


@dataclass(frozen=True)
class MeshData:
    """An indexed polygon mesh, in millimetres, wound CCW seen from outside."""

    verts: np.ndarray  # (V, 3) float64
    faces: list[list[int]]

    def transformed(self, matrix: np.ndarray) -> "MeshData":
        homo = np.hstack([self.verts, np.ones((len(self.verts), 1))])
        return MeshData(verts=(homo @ matrix.T)[:, :3],
                        faces=[list(f) for f in self.faces])

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        return self.verts.min(axis=0), self.verts.max(axis=0)

    def size(self) -> np.ndarray:
        lo, hi = self.bounds()
        return hi - lo


# --------------------------------------------------------------------------
# transforms
# --------------------------------------------------------------------------


def identity() -> np.ndarray:
    return np.eye(4)


def translation(dx: float, dy: float, dz: float) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = (dx, dy, dz)
    return m


def rotation_x(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    m = np.eye(4)
    m[1, 1], m[1, 2] = c, -s
    m[2, 1], m[2, 2] = s, c
    return m


def rotation_y(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    m = np.eye(4)
    m[0, 0], m[0, 2] = c, s
    m[2, 0], m[2, 2] = -s, c
    return m


def rotation_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    m = np.eye(4)
    m[0, 0], m[0, 1] = c, -s
    m[1, 0], m[1, 1] = s, c
    return m


def apply(matrix: np.ndarray, point) -> np.ndarray:
    """Transform a single 3D point."""
    return (matrix @ np.array([point[0], point[1], point[2], 1.0]))[:3]


def apply_direction(matrix: np.ndarray, vector) -> np.ndarray:
    """Transform a direction, ignoring translation."""
    return matrix[:3, :3] @ np.asarray(vector, dtype=np.float64)


# --------------------------------------------------------------------------
# 2D helpers
# --------------------------------------------------------------------------


def shoelace(poly: Sequence[Pt2]) -> float:
    """Signed area in the polygon's own parameter order."""
    pts = np.asarray(poly, dtype=np.float64)
    a, b = pts, np.roll(pts, -1, axis=0)
    return float(np.sum(a[:, 0] * b[:, 1] - b[:, 0] * a[:, 1]) / 2.0)


def is_simple(poly: Sequence[Pt2], tol: float = 1e-12) -> bool:
    """True when no two non-adjacent edges cross. O(n^2), and n is 12."""
    pts = [np.asarray(p, dtype=np.float64) for p in poly]
    n = len(pts)

    def cross(o, a, b) -> float:
        return float((a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]))

    for i in range(n):
        a1, a2 = pts[i], pts[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i or (j + 1) % n == i or (i + 1) % n == j:
                continue
            b1, b2 = pts[j], pts[(j + 1) % n]
            d1 = cross(a1, a2, b1)
            d2 = cross(a1, a2, b2)
            d3 = cross(b1, b2, a1)
            d4 = cross(b1, b2, a2)
            if ((d1 > tol) != (d2 > tol)) and ((d3 > tol) != (d4 > tol)):
                return False
    return True


def box(lo: Vec3, hi: Vec3) -> MeshData:
    """Axis-aligned box. 8 verts, 6 quads, outward normals."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    if not (x1 > x0 and y1 > y0 and z1 > z0):
        raise ValueError(f"degenerate box {lo} -> {hi}")

    verts = np.array([(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                      (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)],
                     dtype=np.float64)
    faces = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4],
             [3, 7, 6, 2], [0, 4, 7, 3], [1, 2, 6, 5]]
    return MeshData(verts=verts, faces=faces)


def prism_yz(poly: Sequence[Pt2], x0: float, x1: float) -> MeshData:
    """Extrude a polygon given in the YZ plane along X.

    ``poly`` must be CCW in (y, z) parameter order, which by Y x Z = X gives it
    an outward normal of +X.
    """
    if x1 <= x0:
        raise ValueError(f"degenerate extrusion {x0} -> {x1}")
    n = len(poly)
    if n < 3:
        raise ValueError("polygon needs at least 3 points")
    if shoelace(poly) <= 0:
        raise ValueError("polygon must be CCW in (y, z)")

    lo = np.array([(x0, y, z) for (y, z) in poly], dtype=np.float64)
    hi = np.array([(x1, y, z) for (y, z) in poly], dtype=np.float64)

    faces: list[list[int]] = [list(range(n - 1, -1, -1)),
                              [n + i for i in range(n)]]
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j, n + i])
    return MeshData(verts=np.vstack([lo, hi]), faces=faces)


def prism(poly: Sequence[Pt2], z0: float, z1: float) -> MeshData:
    """Extrude a CCW polygon in the XY plane between two heights.

    Manifold by construction: side quads plus two n-gon caps. §5.3 builds the
    junction slabs out of these.
    """
    if z1 <= z0:
        raise ValueError(f"degenerate extrusion {z0} -> {z1}")
    n = len(poly)
    if n < 3:
        raise ValueError("a prism needs at least 3 points")
    if shoelace(poly) <= 0:
        raise ValueError("prism polygon must be CCW in (x, y)")

    lo = np.array([(x, y, z0) for (x, y) in poly], dtype=np.float64)
    hi = np.array([(x, y, z1) for (x, y) in poly], dtype=np.float64)

    faces: list[list[int]] = [list(range(n - 1, -1, -1)),          # -Z cap
                              [n + i for i in range(n)]]           # +Z cap
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j, n + i])
    return MeshData(verts=np.vstack([lo, hi]), faces=faces)


# --------------------------------------------------------------------------
# triangulation
# --------------------------------------------------------------------------


def newell_normal(points: np.ndarray) -> np.ndarray:
    """Unit normal of a planar polygon. Exact whether convex or not."""
    normal = np.zeros(3)
    n = len(points)
    for i in range(n):
        a, b = points[i], points[(i + 1) % n]
        normal[0] += (a[1] - b[1]) * (a[2] + b[2])
        normal[1] += (a[2] - b[2]) * (a[0] + b[0])
        normal[2] += (a[0] - b[0]) * (a[1] + b[1])
    length = float(np.linalg.norm(normal))
    if length == 0.0:
        raise ValueError("degenerate polygon has no normal")
    return normal / length


def triangulate(verts: np.ndarray, face: Sequence[int]) -> list[tuple[int, int, int]]:
    """Ear-clip a planar polygon face into triangles, preserving its winding.

    A fan from one vertex is *not* good enough here. The track profile is an
    I-beam and therefore non-convex: a fan from its top-left corner emits
    triangles that cross the open channel between the rails, and two that are
    exactly collinear. Both are wrong in an exported mesh even though the
    signed sums used for area and volume happen to survive them.
    """
    if len(face) < 3:
        return []
    if len(face) == 3:
        return [(face[0], face[1], face[2])]

    points = verts[list(face)]
    normal = newell_normal(points)

    # an orthonormal basis in the polygon's plane, right-handed about `normal`,
    # so CCW in these coordinates means CCW seen from outside
    seed = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(seed, normal)) > 0.9:
        seed = np.array([0.0, 1.0, 0.0])
    u = seed - np.dot(seed, normal) * normal
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    flat = np.column_stack([points @ u, points @ v])

    def cross2(o, a, b) -> float:
        return float((a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]))

    def inside(p, a, b, c) -> bool:
        d1, d2, d3 = cross2(a, b, p), cross2(b, c, p), cross2(c, a, p)
        return not ((d1 < -1e-12 or d2 < -1e-12 or d3 < -1e-12)
                    and (d1 > 1e-12 or d2 > 1e-12 or d3 > 1e-12))

    remaining = list(range(len(face)))
    tris: list[tuple[int, int, int]] = []
    guard = 0
    while len(remaining) > 3 and guard < len(face) * len(face) + 16:
        guard += 1
        clipped = False
        for k in range(len(remaining)):
            i0 = remaining[k - 1]
            i1 = remaining[k]
            i2 = remaining[(k + 1) % len(remaining)]
            a, b, c = flat[i0], flat[i1], flat[i2]
            area = cross2(a, b, c)
            if area <= 1e-12:
                # reflex, or collinear. A collinear vertex is not an ear but it
                # also carries no area, so drop it rather than emit a sliver.
                if abs(area) <= 1e-12:
                    remaining.pop(k)
                    clipped = True
                    break
                continue
            if any(inside(flat[j], a, b, c)
                   for j in remaining if j not in (i0, i1, i2)):
                continue
            tris.append((face[i0], face[i1], face[i2]))
            remaining.pop(k)
            clipped = True
            break
        if not clipped:
            break

    for k in range(1, len(remaining) - 1):
        tris.append((face[remaining[0]], face[remaining[k]],
                     face[remaining[k + 1]]))
    return tris


def triangulated_faces(mesh: MeshData) -> list[tuple[int, int, int]]:
    out: list[tuple[int, int, int]] = []
    for face in mesh.faces:
        out.extend(triangulate(mesh.verts, face))
    return out


# --------------------------------------------------------------------------
# cross-sections
# --------------------------------------------------------------------------


def cross_section_area(mesh: MeshData, point, normal, tol: float = 1e-9,
                       within: float | None = None) -> float:
    """Area of the material where a plane cuts a closed mesh, mm².

    §7 proves a mesh is *manifold*. It cannot tell you it is the mesh you
    wanted: a ramp once shipped with a slot cut clean through its deck and
    passed every rule, because a hole with walls is still watertight. Measuring
    the section is how you check the shape rather than the topology.

    No loop assembly. Each triangle that crosses the plane contributes one
    directed segment, oriented by `normal × face_normal` so that material lies
    to its left; the shoelace sum over all of them is the total enclosed area,
    however many loops there are and whichever order they arrive in.

    ``within`` bounds how far from ``point`` a segment may lie and still count.
    A plane is infinite, so one cut square to a curve's tangent will also slice
    the far side of the same curve and return two sections added together. Pass
    a radius comfortably larger than the section and smaller than the distance
    to anything else — it must not fall *through* a loop, only between loops.
    """
    point = np.asarray(point, dtype=np.float64)
    normal = np.asarray(normal, dtype=np.float64)
    normal = normal / np.linalg.norm(normal)

    seed = np.array([1.0, 0.0, 0.0])
    if abs(float(seed @ normal)) > 0.9:
        seed = np.array([0.0, 1.0, 0.0])
    u = seed - float(seed @ normal) * normal
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)

    total = 0.0
    for face in mesh.faces:
        for tri in triangulate(mesh.verts, face):
            pts = mesh.verts[list(tri)]
            height = (pts - point) @ normal
            # `>= 0` is a tie-break, not a tolerance, and it matters. Treating
            # a vertex *on* the plane as a crossing makes every triangle that
            # touches it contribute a segment, so a cut landing exactly on a
            # station ring counts the section twice — which it does whenever
            # the ring count is even and you sample the middle. Classifying
            # each vertex to one side leaves every triangle with 0 or 2
            # crossings and no way to double count.
            above = height >= 0.0
            crossing = []
            for i in range(3):
                a, b = i, (i + 1) % 3
                if above[a] != above[b]:
                    t = height[a] / (height[a] - height[b])
                    crossing.append(pts[a] + t * (pts[b] - pts[a]))
            if len(crossing) != 2:
                continue
            start, end = crossing
            if np.linalg.norm(end - start) <= tol:
                continue

            face_normal = np.cross(pts[1] - pts[0], pts[2] - pts[0])
            length = float(np.linalg.norm(face_normal))
            if length <= tol:
                continue
            along = np.cross(normal, face_normal / length)
            if float((end - start) @ along) < 0.0:
                start, end = end, start

            x1, y1 = float((start - point) @ u), float((start - point) @ v)
            x2, y2 = float((end - point) @ u), float((end - point) @ v)
            if within is not None:
                mid = math.hypot((x1 + x2) / 2.0, (y1 + y2) / 2.0)
                if mid > within:
                    continue
            total += x1 * y2 - x2 * y1
    return abs(total) / 2.0


# --------------------------------------------------------------------------
# binary STL
# --------------------------------------------------------------------------


def write_stl(mesh: MeshData, path: str, name: str = "trackcore") -> None:
    """Write a binary STL, ear-clipping any polygon faces."""
    tris = [mesh.verts[list(t)] for t in triangulated_faces(mesh)]
    with open(path, "wb") as fh:
        fh.write(name.encode("ascii", "replace")[:80].ljust(80, b"\0"))
        fh.write(struct.pack("<I", len(tris)))
        for tri in tris:
            normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            length = float(np.linalg.norm(normal))
            normal = normal / length if length > 0 else np.zeros(3)
            fh.write(struct.pack("<3f", *normal.astype(np.float32)))
            for point in tri:
                fh.write(struct.pack("<3f", *point.astype(np.float32)))
            fh.write(struct.pack("<H", 0))


def read_stl(path: str, weld_tol: float = 1e-5) -> MeshData:
    """Read a binary STL and weld coincident vertices into an indexed mesh."""
    with open(path, "rb") as fh:
        data = fh.read()
    if len(data) < 84:
        raise ValueError(f"{path}: too short to be a binary STL")
    (count,) = struct.unpack_from("<I", data, 80)
    if len(data) != 84 + count * 50:
        raise ValueError(f"{path}: truncated; expected {count} triangles")

    raw = np.frombuffer(data, dtype=np.uint8, offset=84).reshape(count, 50)
    coords = raw[:, 12:48].copy().view(np.float32).reshape(count, 3, 3)
    pts = coords.reshape(-1, 3).astype(np.float64)

    quantised = np.round(pts / weld_tol).astype(np.int64)
    _, first, inverse = np.unique(quantised, axis=0, return_index=True,
                                  return_inverse=True)
    return MeshData(verts=pts[first],
                    faces=[list(map(int, t)) for t in inverse.reshape(count, 3)])


def merge(meshes: Iterable[MeshData]) -> MeshData:
    """Concatenate meshes. No boolean; only for laying several solids out."""
    verts: list[np.ndarray] = []
    faces: list[list[int]] = []
    offset = 0
    for mesh in meshes:
        verts.append(mesh.verts)
        faces.extend([[i + offset for i in f] for f in mesh.faces])
        offset += len(mesh.verts)
    return MeshData(verts=np.vstack(verts), faces=faces)
