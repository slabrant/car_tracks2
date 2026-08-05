"""Mesh validation, docs/SPEC.md §7.

Pure Python + numpy. Never imports bpy.

Every check here is one of the six required by the spec, and each corresponds to
a way v1 shipped unprintable geometry. Failures raise; they do not log. Applies
equally to Construction A output and to Construction B's post-union result.
"""

from __future__ import annotations

from collections import defaultdict

import numpy as np

from .mesh import MeshData


class MeshInvalid(Exception):
    """A mesh failed a §7 check and must never be exported."""


def _triangulate(face: list[int]) -> list[tuple[int, int, int]]:
    """Fan triangulation. Valid for signed area and volume even when the
    polygon is non-convex, provided it is planar and simple."""
    return [(face[0], face[k], face[k + 1]) for k in range(1, len(face) - 1)]


def face_normal_area(verts: np.ndarray, face: list[int]) -> tuple[np.ndarray, float]:
    """Newell's method. Exact for any planar simple polygon, convex or not."""
    normal = np.zeros(3)
    n = len(face)
    for i in range(n):
        a = verts[face[i]]
        b = verts[face[(i + 1) % n]]
        normal[0] += (a[1] - b[1]) * (a[2] + b[2])
        normal[1] += (a[2] - b[2]) * (a[0] + b[0])
        normal[2] += (a[0] - b[0]) * (a[1] + b[1])
    length = float(np.linalg.norm(normal))
    if length == 0.0:
        return np.zeros(3), 0.0
    return normal / length, length / 2.0


def signed_volume(mesh: MeshData) -> float:
    """Divergence theorem. Positive when normals point outward."""
    total = 0.0
    for face in mesh.faces:
        for i, j, k in _triangulate(face):
            a, b, c = mesh.verts[i], mesh.verts[j], mesh.verts[k]
            total += float(np.dot(a, np.cross(b, c)))
    return total / 6.0


def count_components(vertices, edges) -> int:
    """Number of connected components, by union-find over the edge set.

    A plate holding several separate solids is legitimate; it just has an Euler
    characteristic of 2 per solid rather than 2 overall.

    ``vertices`` is the set of indices that some face actually uses. Loose
    vertices are reported separately and must not be counted here.
    """
    parent = {index: index for index in vertices}

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b in edges:
        if a not in parent or b not in parent:
            continue
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    return len({find(i) for i in parent})


def _shell_report(mesh: MeshData, vertices, edges, limit: int = 3) -> str:
    """Locate and size the smallest shells, so rule 7 points at the culprit.

    "5 separate solids" sends you looking. "1.1 x 1.1 x 0.5 mm at (11.5, 2.5,
    0.1)" is the detent rib, by name, and you are done. The biggest shell is
    assumed to be the part and is left out.
    """
    parent = {index: index for index in vertices}

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b in edges:
        if a in parent and b in parent:
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb

    shells: defaultdict[int, set[int]] = defaultdict(set)
    for face in mesh.faces:
        shells[find(face[0])].update(face)

    # by bounding volume, not by vertex count: two boxes have eight corners
    # each however different their sizes, and the whole point is to name the
    # small one.
    measured = []
    for group in shells.values():
        points = mesh.verts[sorted(group)]
        low, high = points.min(axis=0), points.max(axis=0)
        measured.append((float(np.prod(np.maximum(high - low, 1e-6))),
                         low, high))

    # by the size key alone; tuples would fall through to comparing the numpy
    # corners when two shells happen to share a bounding volume
    ordered = sorted(measured, key=lambda shell: shell[0])[:-1]  # drop largest
    described = []
    for _, low, high in ordered[:limit]:
        size = high - low
        mid = (high + low) / 2.0
        described.append(
            f"{size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} mm at "
            f"({mid[0]:.2f}, {mid[1]:.2f}, {mid[2]:.2f})"
        )
    if len(ordered) > limit:
        described.append(f"and {len(ordered) - limit} more")
    return "; ".join(described)


def check(mesh: MeshData, *, name: str = "mesh", area_tol: float = 1e-9,
          weld_tol: float = 1e-9,
          components: int | None = 1) -> dict[str, float]:
    """Run all seven §7 checks. Raises MeshInvalid on the first failure.

    ``components`` is how many separate solids the caller expects, and it
    defaults to one because a *part* is one part. Pass ``None`` to accept any
    number — only a deliberately multi-body mesh, such as a full build plate,
    should do that.

    Rule 7 was added late and at some cost. §7 originally proved a mesh
    manifold, watertight, correctly wound and genus 0 *per component*, which a
    part shattered into five pieces satisfies perfectly: five closed spheres
    are five legal solids. The Phase 0 comb shipped with all seventy-two of its
    detent ribs floating free in the air beside the tab they belong to, and
    every rule passed, and the build printed OK. It was caught by eye. Counting
    the components was already being done, one line above; nothing was looking
    at the answer.

    Returns a small dict of measured quantities for reporting.
    """
    problems: list[str] = []

    # 4. no degenerate faces
    degenerate = []
    for idx, face in enumerate(mesh.faces):
        if len(face) < 3 or len(set(face)) != len(face):
            degenerate.append(idx)
            continue
        _, area = face_normal_area(mesh.verts, face)
        if area <= area_tol:
            degenerate.append(idx)
    if degenerate:
        problems.append(
            f"{len(degenerate)} degenerate face(s), first at index {degenerate[0]}"
        )

    # 5. no duplicate vertices
    quantised = np.round(mesh.verts / weld_tol).astype(np.int64)
    unique = np.unique(quantised, axis=0)
    duplicates = len(mesh.verts) - len(unique)
    if duplicates:
        problems.append(f"{duplicates} duplicate vertex/vertices within {weld_tol}")

    # 1 and 2. edge manifoldness and consistent orientation
    directed: defaultdict[tuple[int, int], int] = defaultdict(int)
    undirected: defaultdict[tuple[int, int], int] = defaultdict(int)
    for face in mesh.faces:
        n = len(face)
        for i in range(n):
            a, b = face[i], face[(i + 1) % n]
            directed[(a, b)] += 1
            undirected[(min(a, b), max(a, b))] += 1

    non_manifold = [e for e, c in undirected.items() if c != 2]
    if non_manifold:
        problems.append(
            f"{len(non_manifold)} non-manifold edge(s) (not used by exactly 2 faces), "
            f"first {non_manifold[0]}"
        )

    inconsistent = [e for e, c in directed.items() if c != 1]
    if inconsistent:
        problems.append(
            f"{len(inconsistent)} inconsistently wound edge(s), first {inconsistent[0]}"
        )

    # 6. watertight, genus 0 per connected component
    referenced = {index for face in mesh.faces for index in face}
    loose = len(mesh.verts) - len(referenced)
    if loose:
        problems.append(
            f"{loose} vertex/vertices belong to no face"
        )

    # counted over the *face graph*, not over every stored vertex. A vertex no
    # face uses is junk, and counting it as its own component turns a stray
    # point into a baffling Euler error instead of the plain statement above.
    v = len(referenced)
    e = len(undirected)
    f = len(mesh.faces)
    euler = v - e + f
    found = count_components(referenced, undirected.keys())
    if euler != 2 * found:
        problems.append(
            f"Euler characteristic V-E+F = {euler}, expected {2 * found} "
            f"for {found} genus-0 component(s)"
        )

    # 7. one part, one solid
    if components is not None and found != components:
        loose_shells = _shell_report(mesh, referenced, undirected.keys())
        problems.append(
            f"{found} separate solid(s), expected {components}"
            + (f"; the stray one(s) are {loose_shells}" if loose_shells else "")
        )

    # 3. outward normals
    volume = signed_volume(mesh)
    if volume <= 0:
        problems.append(f"signed volume {volume:.6g} <= 0, normals point inward")

    if problems:
        raise MeshInvalid(f"{name}: " + "; ".join(problems))

    return {
        "verts": float(v),
        "edges": float(e),
        "faces": float(f),
        "euler": float(euler),
        "components": float(found),
        "volume_mm3": volume,
    }
