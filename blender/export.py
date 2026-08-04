"""STL and 3MF export. docs/SPEC.md §8.

Blender units are millimetres throughout, so global_scale is always 1.0.
"""

from __future__ import annotations

import os

import bpy


def _select(objects) -> None:
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]


def export(objects, path: str) -> str:
    """Write the given objects to ``path``. Format follows the extension."""
    if not objects:
        raise ValueError("nothing to export")
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    _select(objects)

    suffix = os.path.splitext(path)[1].lower()
    if suffix == ".stl":
        try:
            bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True,
                                  global_scale=1.0)
        except AttributeError:
            bpy.ops.export_mesh.stl(filepath=path, use_selection=True,
                                    global_scale=1.0)
    elif suffix == ".3mf":
        bpy.ops.export_mesh.threemf(filepath=path, use_selection=True)
    else:
        raise ValueError(f"unsupported export format {suffix!r}")
    return path
