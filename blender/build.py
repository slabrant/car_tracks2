"""MeshData -> bmesh -> Blender object. docs/SPEC.md §8.

One of the few files allowed to import bpy. It converts at the boundary and
does nothing else; all geometry decisions were already made in trackcore.
"""

from __future__ import annotations

import bpy
import numpy as np

from trackcore.mesh import MeshData

COLLECTION = "Tracks"


def scene_collection(name: str = COLLECTION):
    if name in bpy.data.collections:
        return bpy.data.collections[name]
    collection = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(collection)
    return collection


def reset_scene(name: str = COLLECTION):
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    for mesh in list(bpy.data.meshes):
        bpy.data.meshes.remove(mesh)
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.length_unit = "MILLIMETERS"
    return scene_collection(name)


def to_object(mesh_data: MeshData, name: str, collection=None):
    """Build a Blender object. Blender units are treated as millimetres."""
    mesh = bpy.data.meshes.new(f"{name}_mesh")
    mesh.from_pydata([tuple(v) for v in mesh_data.verts], [], mesh_data.faces)
    mesh.validate(verbose=False)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    (collection or scene_collection()).objects.link(obj)
    return obj


def from_object(obj) -> MeshData:
    """Read a Blender object back out, so trackcore.validate can check it."""
    mesh = obj.data
    verts = np.array([tuple(v.co) for v in mesh.vertices], dtype=np.float64)
    return MeshData(verts=verts,
                    faces=[list(p.vertices) for p in mesh.polygons])
