"""The one sanctioned boolean. docs/SPEC.md §5.3.

Junctions are the only place in the system where a boolean is permitted, and it
carries a mandatory obligation: the union result is re-checked against every
item in §7 before it may be exported. `union` does not do that check itself —
the caller must, and `blender/run.py` does.

The inputs are built so the solver has an easy job: rail prisms span the full
height so their overlap with the deck prism is volumetric rather than face to
face, and the deck is inset half a rail thickness on rail-covered edges so no
vertical faces are coplanar either. That condition is what generated v1's
non-manifold output.
"""

from __future__ import annotations

import bpy


def apply_boolean(target, tool, operation: str):
    """Apply one boolean with the MANIFOLD solver and bake the result."""
    modifier = target.modifiers.new(name=operation.lower(), type="BOOLEAN")
    modifier.operation = operation
    modifier.object = tool
    try:
        modifier.solver = "MANIFOLD"
    except TypeError:
        modifier.solver = "EXACT"

    depsgraph = bpy.context.evaluated_depsgraph_get()
    baked = bpy.data.meshes.new_from_object(target.evaluated_get(depsgraph))
    previous = target.data
    target.data = baked
    target.modifiers.clear()
    bpy.data.meshes.remove(previous)
    bpy.data.objects.remove(tool, do_unlink=True)
    return target


def union(objects, name: str | None = None):
    """Union a list of Blender objects into the first."""
    if not objects:
        raise ValueError("nothing to union")
    target, tools = objects[0], objects[1:]
    for tool in tools:
        apply_boolean(target, tool, "UNION")
    if name:
        target.name = name
    return target
