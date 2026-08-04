"""Mesh validation, docs/SPEC.md §7.

Pure Python + numpy. Never imports bpy.

This is the seed of trackcore/validate.py. Every check here is one of the six
required by the spec, and each corresponds to a way v1 shipped unprintable
geometry. Failures raise; they do not log.
"""

from __future__ import annotations

from collections import defaultdict

import numpy as np

from geom import MeshData


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


def count_components(n_verts: int, edges) -> int:
    """Number of connected components, by union-find over the edge set.

    A plate holding several separate solids is legitimate; it just has an Euler
    characteristic of 2 per solid rather than 2 overall.
    """
    parent = list(range(n_verts))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b in edges:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    return len({find(i) for i in range(n_verts)})


def check(mesh: MeshData, *, name: str = "mesh", area_tol: float = 1e-9,
          weld_tol: float = 1e-9) -> dict[str, float]:
    """Run all six §7 checks. Raises MeshInvalid on the first failure.

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
    v = len(mesh.verts)
    e = len(undirected)
    f = len(mesh.faces)
    euler = v - e + f
    components = count_components(v, undirected.keys())
    if euler != 2 * components:
        problems.append(
            f"Euler characteristic V-E+F = {euler}, expected {2 * components} "
            f"for {components} genus-0 component(s)"
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
        "components": float(components),
        "volume_mm3": volume,
    }
