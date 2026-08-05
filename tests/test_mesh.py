"""Triangulation of the non-convex profile.

Regression cover for a real defect: the end caps are the I-beam profile, which
is non-convex, and a fan triangulation from one corner emits triangles that
cross the open channel between the rails plus two that are exactly collinear.
The signed sums used for area and volume survive that; an exported STL does not.
"""

from __future__ import annotations

import numpy as np
import pytest

from parts import curve, straight
from trackcore import DEFAULT, profile_area, sweep
from trackcore.edge_unit import PROFILE_VERTS
from trackcore.mesh import (MeshData, newell_normal, triangulate,
                            triangulated_faces)


def _cap_faces(mesh: MeshData) -> list[list[int]]:
    return [f for f in mesh.faces if len(f) == PROFILE_VERTS]


def test_the_profile_cap_is_non_convex():
    """If it were convex a fan would have been fine and this file pointless."""
    mesh = sweep(straight(50.0))
    face = _cap_faces(mesh)[0]
    pts = mesh.verts[face]
    normal = newell_normal(pts)

    signs = []
    for i in range(len(pts)):
        a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
        signs.append(np.dot(np.cross(b - a, c - b), normal))
    assert min(signs) < 0, "profile should have reflex corners"


def test_triangulating_a_cap_conserves_its_area():
    mesh = sweep(straight(50.0))
    face = _cap_faces(mesh)[0]
    tris = triangulate(mesh.verts, face)

    total = 0.0
    for tri in tris:
        a, b, c = mesh.verts[list(tri)]
        total += 0.5 * float(np.linalg.norm(np.cross(b - a, c - a)))

    assert total == pytest.approx(profile_area(DEFAULT.body), rel=1e-9)


def test_no_triangle_is_degenerate_or_wound_backwards():
    mesh = sweep(curve(radius=100.0, angle_deg=90.0))
    for tri in triangulated_faces(mesh):
        a, b, c = mesh.verts[list(tri)]
        cross = np.cross(b - a, c - a)
        assert float(np.linalg.norm(cross)) > 1e-9, f"degenerate triangle {tri}"

    caps = _cap_faces(mesh)
    for face in caps:
        normal = newell_normal(mesh.verts[face])
        for tri in triangulate(mesh.verts, face):
            a, b, c = mesh.verts[list(tri)]
            assert np.dot(np.cross(b - a, c - a), normal) > 0, (
                "cap triangle wound against the face normal"
            )


def test_a_fan_would_have_failed_this():
    """Pin the actual defect, so nobody reintroduces the cheap version.

    On the U the fan covers 2.8x the true area, sweeping straight across the
    open channel. The old I-section additionally produced two exactly collinear
    triangles; this section does not, so the assertion is about coverage alone.
    """
    mesh = sweep(straight(50.0))
    face = _cap_faces(mesh)[0]
    fan = [(face[0], face[k], face[k + 1]) for k in range(1, len(face) - 1)]

    areas = []
    for tri in fan:
        a, b, c = mesh.verts[list(tri)]
        areas.append(0.5 * float(np.linalg.norm(np.cross(b - a, c - a))))
    assert sum(areas) > profile_area(DEFAULT.body) * 1.5, (
        "expected the fan to cover area well outside the profile"
    )


def test_triangulate_passes_triangles_straight_through():
    verts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    assert triangulate(verts, [0, 1, 2]) == [(0, 1, 2)]
