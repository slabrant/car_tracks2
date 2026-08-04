"""Mesh repair after a boolean. docs/SPEC.md §8.

An exact boolean solver is entitled to leave sub-tolerance artifacts: a vertex
inserted on an edge it happens to touch, a triangle whose three corners are
collinear to within a nanometre. They carry no volume and no printed part can
express them, but they fail §7's degenerate-face rule, and rightly — the rule
cannot tell a harmless sliver from a real one.

Chasing each one back to its cause is endless and, worse, it tempts you to
loosen §7. Repair them instead, at a tolerance far below anything a printer can
resolve, and then validate strictly. §8 assigns exactly this job to Blender.

The tolerance is the load-bearing decision: it must be small enough that it can
only ever remove artifacts, never geometry. See `WELD_TOLERANCE`.
"""

from __future__ import annotations

import bmesh
import bpy

WELD_TOLERANCE = 0.005
"""Millimetres. 5 µm.

Twenty times finer than a 0.1 mm layer and thirty times finer than the smallest
real feature in the design, which is the 0.15 mm fit clearance. Nothing a
printer or a child can detect lives below this, so a weld at this distance can
only be removing solver noise. Raising it toward the clearance would start
destroying the joint, so it is capped by an assertion in `clean`.
"""


def clean(obj, tolerance: float = WELD_TOLERANCE, smallest_feature: float = 0.15):
    """Weld near-coincident vertices and dissolve degenerate faces."""
    if tolerance >= smallest_feature / 10.0:
        raise ValueError(
            f"weld tolerance {tolerance} is within an order of magnitude of the "
            f"smallest real feature ({smallest_feature} mm); it would start "
            f"removing geometry rather than noise"
        )

    mesh = bmesh.new()
    mesh.from_mesh(obj.data)
    bmesh.ops.remove_doubles(mesh, verts=mesh.verts[:], dist=tolerance)
    bmesh.ops.dissolve_degenerate(mesh, dist=tolerance, edges=mesh.edges[:])
    bmesh.ops.recalc_face_normals(mesh, faces=mesh.faces[:])
    mesh.to_mesh(obj.data)
    mesh.free()
    obj.data.update()
    return obj
