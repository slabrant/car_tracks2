"""Stand a bridge up on legs made of ordinary track. docs/SPEC.md §5.5.

    blender --background --python blender/bridge.py -- --out out/bridge.png

Nothing here knows what a support is. Every piece is placed by the one rule
that two ports mate when one frame equals the other times MATE — including the
foot, which is a support turned over.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from blender.build import from_object, reset_scene  # noqa: E402
from blender.preview import COLOURS, add_camera, render  # noqa: E402
from blender.run import build_part  # noqa: E402
from parts import GRID, port_frames  # noqa: E402
from trackcore import DEFAULT, MATE  # noqa: E402
from trackcore.mesh import rotation_y, translation  # noqa: E402


def place(name, collection, world, colour):
    from mathutils import Matrix
    obj = build_part(name, collection)
    obj.matrix_world = Matrix([list(row) for row in world])
    obj.color = colour
    return obj, [world @ f for f in port_frames(name)]


def attach(name, port, target, collection, colour):
    world = target @ MATE @ np.linalg.inv(port_frames(name)[port])
    return place(name, collection, world, colour)


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="out/bridge.png")
    parser.add_argument("--azimuth", type=float, default=-62.0)
    parser.add_argument("--elevation", type=float, default=18.0)
    parser.add_argument("--scale", type=float, default=210.0)
    args = parser.parse_args(argv)

    collection = reset_scene()
    half = DEFAULT.body.half_height

    # a foot is a support turned over: 180 degrees about its long axis, then
    # set down on its own rails
    foot_world = translation(0.0, 0.0, half) @ rotation_y(np.pi)
    _foot, foot_ports = place("support", collection, foot_world, COLOURS[0])

    _leg, leg_ports = attach("straight_quarter", 0, foot_ports[2],
                             collection, COLOURS[1])
    support, sup_ports = attach("support", 2, leg_ports[1],
                                collection, COLOURS[0])
    attach("straight_full", 0, sup_ports[1], collection, COLOURS[2])
    attach("straight_full", 0, sup_ports[0], collection, COLOURS[2])

    deck = float(sup_ports[0][2, 3])
    expected = half + GRID.deck_height
    print(f"  bridge deck mid-plane at {deck:.2f} mm above the ground")
    print(f"  the ramp delivers        {expected:.2f} mm")
    print(f"  difference               {abs(deck - expected):.6f} mm")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    add_camera((0.0, 24.0, 26.0), 500.0, args.azimuth, args.elevation, args.scale)
    render(args.out, (1400, 900))
    print(f"wrote {os.path.abspath(args.out)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
