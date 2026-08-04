"""Render the built parts, so a human can look before anything is printed.

    blender --background --python blender/preview.py -- --out out/parts.png
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import bpy

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from blender.build import reset_scene  # noqa: E402
from blender.run import build_part  # noqa: E402
from parts import CATALOGUE  # noqa: E402

COLOURS = [
    (0.85, 0.36, 0.16, 1.0),
    (0.20, 0.45, 0.80, 1.0),
    (0.30, 0.62, 0.35, 1.0),
    (0.72, 0.60, 0.20, 1.0),
]


def add_camera(target, distance: float, azimuth: float, elevation: float,
               ortho_scale: float):
    from mathutils import Euler, Vector

    data = bpy.data.cameras.new("cam")
    data.type = "ORTHO"
    data.ortho_scale = ortho_scale
    cam = bpy.data.objects.new("cam", data)
    bpy.context.scene.collection.objects.link(cam)

    az, el = math.radians(azimuth), math.radians(elevation)
    direction = Vector((math.cos(el) * math.cos(az),
                        math.cos(el) * math.sin(az),
                        math.sin(el)))
    cam.location = Vector(target) + direction * distance
    cam.rotation_euler = Euler((math.pi / 2 - el, 0.0, az + math.pi / 2), "XYZ")
    bpy.context.scene.camera = cam
    return cam


def render(path: str, resolution=(1500, 950)) -> None:
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.render.resolution_x, scene.render.resolution_y = resolution
    scene.render.filepath = path
    shading = scene.display.shading
    shading.light = "STUDIO"
    shading.color_type = "OBJECT"
    shading.show_cavity = True
    shading.cavity_type = "BOTH"
    bpy.ops.render.render(write_still=True)


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="out/parts.png")
    parser.add_argument("--azimuth", type=float, default=-60.0)
    parser.add_argument("--elevation", type=float, default=42.0)
    parser.add_argument("--scale", type=float, default=380.0)
    parser.add_argument("--only", default=None,
                        help="comma-separated part names")
    parser.add_argument("--spacing", type=float, default=90.0)
    args = parser.parse_args(argv)

    from mathutils import Matrix

    names = args.only.split(",") if args.only else CATALOGUE
    collection = reset_scene()
    spacing = args.spacing
    span = spacing * (len(names) - 1)
    for i, name in enumerate(names):
        obj = build_part(name, collection)
        obj.matrix_world = Matrix.Translation((i * spacing - span / 2.0,
                                               0.0, 0.0))
        obj.color = COLOURS[i % len(COLOURS)]

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    add_camera((0.0, 40.0, 0.0), 500.0, args.azimuth, args.elevation,
               args.scale)
    render(args.out)
    print(f"wrote {os.path.abspath(args.out)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
