"""Place parts at each other's ports and render the result.

    blender --background --python blender/assemble.py -- --hub x_rounded

Nothing here knows about connectors. It places pieces purely by the rule that
two ports mate when one frame equals the other times `MATE`, which is what
makes the geometry in §6 worth having.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from blender.build import reset_scene  # noqa: E402
from blender.preview import COLOURS, add_camera, render  # noqa: E402
from blender.run import build_part  # noqa: E402
from parts import port_frames  # noqa: E402
from trackcore import MATE  # noqa: E402


def attach(name: str, port: int, target_frame, collection, index: int):
    """Build ``name`` and place its ``port`` against ``target_frame``."""
    from mathutils import Matrix

    obj = build_part(name, collection)
    frames = port_frames(name)
    placement = target_frame @ MATE @ np.linalg.inv(frames[port])
    obj.matrix_world = Matrix([list(row) for row in placement])
    obj.color = COLOURS[index % len(COLOURS)]
    return obj, [placement @ f for f in frames]


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--hub", default="x_rounded")
    parser.add_argument("--spokes", default="straight,curve,ramp,s_bend")
    parser.add_argument("--out", default="out/assembly.png")
    parser.add_argument("--azimuth", type=float, default=-55.0)
    parser.add_argument("--elevation", type=float, default=48.0)
    parser.add_argument("--scale", type=float, default=430.0)
    args = parser.parse_args(argv)

    collection = reset_scene()
    hub = build_part(args.hub, collection)
    hub.color = COLOURS[0]
    frames = port_frames(args.hub)

    for index, name in enumerate(args.spokes.split(",")):
        if index >= len(frames):
            break
        attach(name, 0, frames[index], collection, index + 1)

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    add_camera((0.0, 0.0, 0.0), 500.0, args.azimuth, args.elevation, args.scale)
    render(args.out)
    print(f"wrote {os.path.abspath(args.out)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
