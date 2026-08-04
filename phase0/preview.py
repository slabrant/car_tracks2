"""Render a look at the Phase 0 coupon joint.

    blender --background --python phase0/preview.py -- --out phase0/out/joint.png

Renders two coupons mated at their ports, plus one on its own, so the lap and
the detents can be eyeballed before anything is printed.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import bpy
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from build_coupons import build_coupon, reset_scene  # noqa: E402
from coupon import Config  # noqa: E402


def add_camera(target, distance: float, azimuth: float, elevation: float,
               ortho_scale: float):
    from mathutils import Euler, Vector

    cam_data = bpy.data.cameras.new("cam")
    cam_data.type = "ORTHO"
    cam_data.ortho_scale = ortho_scale
    cam = bpy.data.objects.new("cam", cam_data)
    bpy.context.scene.collection.objects.link(cam)

    az, el = math.radians(azimuth), math.radians(elevation)
    direction = Vector((
        math.cos(el) * math.cos(az),
        math.cos(el) * math.sin(az),
        math.sin(el),
    ))
    cam.location = Vector(target) + direction * distance
    cam.rotation_euler = Euler((math.pi / 2 - el, 0.0, az + math.pi / 2), "XYZ")
    bpy.context.scene.camera = cam
    return cam


def render(path: str, resolution=(1400, 900)) -> None:
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.render.resolution_x, scene.render.resolution_y = resolution
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = False
    scene.render.filepath = path
    shading = scene.display.shading
    shading.light = "STUDIO"
    shading.color_type = "OBJECT"
    shading.show_cavity = True
    shading.cavity_type = "BOTH"
    bpy.ops.render.render(write_still=True)


def colour(obj, rgba) -> None:
    obj.color = rgba


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="phase0/out/joint.png")
    parser.add_argument("--clearance", type=float, default=0.20)
    parser.add_argument("--azimuth", type=float, default=-55.0)
    parser.add_argument("--elevation", type=float, default=28.0)
    parser.add_argument("--scale", type=float, default=44.0)
    args = parser.parse_args(argv)

    from mathutils import Matrix

    cfg = Config(fit_clearance=args.clearance)
    collection = reset_scene()

    piece_a = build_coupon(cfg, tally=0, collection=collection, name="mated_a")
    piece_b = build_coupon(cfg, tally=0, collection=collection, name="mated_b")
    piece_b.matrix_world = Matrix.Rotation(np.pi, 4, "Z")
    colour(piece_a, (0.85, 0.36, 0.16, 1.0))
    colour(piece_b, (0.20, 0.45, 0.80, 1.0))

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    add_camera((0.0, 0.0, 0.0), 120.0, args.azimuth, args.elevation, args.scale)
    render(args.out)
    print(f"wrote {os.path.abspath(args.out)}")

    def reshoot(suffix: str, azimuth: float, elevation: float, scale: float,
                target=(0.0, 0.0, 0.0)) -> None:
        path = os.path.splitext(args.out)[0] + f"_{suffix}.png"
        for obj in list(bpy.data.objects):
            if obj.type == "CAMERA":
                bpy.data.objects.remove(obj, do_unlink=True)
        add_camera(target, 120.0, azimuth, elevation, scale)
        render(path)
        print(f"wrote {os.path.abspath(path)}")

    # looking down the track
    reshoot("end", -90.0, 8.0, 32.0)
    # square on to the +X rail: this is the §6.2 side view, showing the lap
    # step and both detents
    reshoot("rail", 0.0, 0.0, 13.0)
    # and the same joint from underneath, to confirm flip symmetry
    reshoot("under", -55.0, -28.0, args.scale)
    return 0


if __name__ == "__main__":
    sys.exit(main())
